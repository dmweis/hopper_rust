use async_openai::{
    config::OpenAIConfig,
    types::{
        ChatCompletionFunctions, ChatCompletionFunctionsArgs, ChatCompletionRequestMessage,
        ChatCompletionRequestMessageArgs, CreateChatCompletionRequest,
        CreateChatCompletionRequestArgs, FinishReason, FunctionCall, Role,
    },
    Client,
};
use async_trait::async_trait;
use futures::StreamExt;
use schemars::{gen::SchemaSettings, JsonSchema};
use std::{collections::HashMap, sync::Arc};
use tracing::info;

use super::OpenAiHistory;

fn get_schema_generator() -> schemars::gen::SchemaGenerator {
    let settings = SchemaSettings::draft07().with(|s| {
        s.inline_subschemas = true;
        s.meta_schema = None;
    });
    settings.into_generator()
}

pub enum OpenAiApiResponse {
    AssistantResponse(String),
    FunctionCallWithNoResponse,
}

#[async_trait]
pub trait AsyncCallback: Send + Sync {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value>;
}

#[derive(Clone)]
pub struct ChatGptConversation {
    history: Vec<ChatCompletionRequestMessage>,
    functions: Vec<ChatCompletionFunctions>,
    temperature: Option<f32>,
    top_p: Option<f32>,
    model_name: String,
    function_table: HashMap<String, Arc<dyn AsyncCallback>>,
}

impl ChatGptConversation {
    pub fn new(system_prompt: &str, model_name: &str) -> Self {
        let history = vec![ChatCompletionRequestMessageArgs::default()
            .content(system_prompt)
            .role(Role::System)
            .build()
            // can this fail?
            .expect("Failed to build system prompt message")];
        Self {
            history,
            functions: vec![],
            temperature: None,
            top_p: None,
            model_name: model_name.to_string(),
            function_table: HashMap::new(),
        }
    }

    pub fn add_function<T: ?Sized + JsonSchema>(
        &mut self,
        function_name: &str,
        function_description: &str,
        func: Arc<dyn AsyncCallback>,
    ) -> anyhow::Result<()> {
        let schema = get_schema_generator().into_root_schema_for::<T>();
        let schema_json = serde_json::to_value(&schema)?;
        let new_function = ChatCompletionFunctionsArgs::default()
            .name(function_name)
            .description(function_description)
            .parameters(schema_json)
            .build()?;

        self.functions.push(new_function);

        self.function_table.insert(function_name.to_string(), func);
        Ok(())
    }

    async fn call_function(&self, name: &str, args: &str) -> anyhow::Result<serde_json::Value> {
        info!("Calling function {:?} with args {:?}", name, args);
        let function = self
            .function_table
            .get(name)
            .ok_or_else(|| anyhow::anyhow!("Function {} not found", name))?;
        function.call(args).await
    }

    /// build request message
    fn build_request_message(&self) -> anyhow::Result<CreateChatCompletionRequest> {
        // request builder setup is a bit more complicated because of the optional parameters
        let mut request_builder = CreateChatCompletionRequestArgs::default();

        request_builder
            .model(self.model_name.clone())
            .messages(self.history.clone())
            .functions(self.functions.clone())
            .function_call("auto");

        if let Some(temperature) = self.temperature {
            request_builder.temperature(temperature);
        }

        if let Some(top_p) = self.top_p {
            request_builder.top_p(top_p);
        }

        Ok(request_builder.build()?)
    }

    /// stream next message
    pub async fn next_message_stream(
        &mut self,
        message_text: Option<&str>,
        client: &Client<OpenAIConfig>,
    ) -> anyhow::Result<OpenAiApiResponse> {
        if let Some(message_text) = message_text {
            let user_message = ChatCompletionRequestMessageArgs::default()
                .content(message_text)
                .role(Role::User)
                .build()?;

            self.history.push(user_message);
        }

        let request = self.build_request_message()?;

        let mut stream = client.chat().create_stream(request).await?;

        let mut response_role = None;
        let mut response_content_buffer = String::new();
        let mut fn_name = String::new();
        let mut fn_args = String::new();

        // For reasons not documented in OpenAI docs / OpenAPI spec, the response of streaming call is different and doesn't include all the same fields.
        while let Some(result) = stream.next().await {
            let response = result?;

            // assert that we only get one response
            if response.choices.len() != 1 {
                return Err(anyhow::anyhow!(
                    "expected 1 response from OpenAI, got {}",
                    response.choices.len()
                ));
            }
            let choice = response
                .choices
                .first()
                .expect("Failed to get first choice from response");

            // take response role
            if let Some(role) = choice.delta.role {
                response_role = Some(role);
            }

            // take function call
            if let Some(fn_call) = &choice.delta.function_call {
                if let Some(name) = &fn_call.name {
                    fn_name = name.clone();
                }
                if let Some(args) = &fn_call.arguments {
                    fn_args.push_str(args);
                }
            }

            // take response content
            if let Some(delta_content) = &choice.delta.content {
                response_content_buffer.push_str(delta_content);
                // process chunk (print it?)
            }

            // check if response is end
            if let Some(finish_reason) = &choice.finish_reason {
                // figure out why the conversation ended
                if matches!(finish_reason, FinishReason::FunctionCall) {
                    // function call

                    // add function call to history
                    let function_call_request = ChatCompletionRequestMessageArgs::default()
                        .role(Role::Assistant)
                        .function_call(FunctionCall {
                            name: fn_name.clone(),
                            arguments: fn_args.clone(),
                        })
                        .build()?;
                    self.history.push(function_call_request);

                    // call function
                    let result = self.call_function(&fn_name, &fn_args).await?;

                    // add function call result to history
                    let function_call_result = ChatCompletionRequestMessageArgs::default()
                        .role(Role::Function)
                        .content(result.to_string())
                        .name(fn_name.clone())
                        .build()?;
                    self.history.push(function_call_result);

                    if !response_content_buffer.is_empty() {
                        // function calls can also include a response

                        let added_response = ChatCompletionRequestMessageArgs::default()
                            .content(&response_content_buffer)
                            .role(response_role.unwrap_or(Role::Assistant))
                            .build()?;

                        self.history.push(added_response);
                        return Ok(OpenAiApiResponse::AssistantResponse(
                            response_content_buffer,
                        ));
                    } else {
                        return Ok(OpenAiApiResponse::FunctionCallWithNoResponse);
                    }
                } else {
                    // other reasons ass message from assistant
                    let added_response = ChatCompletionRequestMessageArgs::default()
                        .content(&response_content_buffer)
                        .role(response_role.unwrap_or(Role::Assistant))
                        .build()?;

                    self.history.push(added_response);
                    return Ok(OpenAiApiResponse::AssistantResponse(
                        response_content_buffer,
                    ));
                }
            }
        }

        // return text anyway even if we don't get an end reason
        Ok(OpenAiApiResponse::AssistantResponse(
            response_content_buffer,
        ))
    }

    pub fn get_history(&self) -> String {
        let history = OpenAiHistory {
            history: self.history.clone(),
            timestamp: chrono::Utc::now(),
        };
        serde_json::to_string_pretty(&history).expect("Failed to serialize chat history")
    }
}
