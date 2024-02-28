use anyhow::Context;
use async_openai::{
    config::OpenAIConfig,
    types::{
        ChatCompletionMessageToolCall, ChatCompletionRequestAssistantMessageArgs,
        ChatCompletionRequestMessage, ChatCompletionRequestSystemMessageArgs,
        ChatCompletionRequestToolMessageArgs, ChatCompletionRequestUserMessageArgs,
        ChatCompletionTool, ChatCompletionToolArgs, ChatCompletionToolChoiceOption,
        ChatCompletionToolType, CreateChatCompletionRequest, CreateChatCompletionRequestArgs,
        FunctionCall, FunctionObject, FunctionObjectArgs,
    },
    Client,
};
use async_trait::async_trait;
use futures::StreamExt;
use schemars::{gen::SchemaSettings, JsonSchema};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, sync::Arc};
use tracing::{info, instrument};

#[derive(Serialize, Deserialize, Debug)]
pub struct OpenAiHistory {
    history: Vec<ChatCompletionRequestMessage>,
    functions: Vec<FunctionObject>,
    tools: Vec<ChatCompletionTool>,
    timestamp: chrono::DateTime<chrono::Utc>,
}

fn get_schema_generator() -> schemars::gen::SchemaGenerator {
    let settings = SchemaSettings::draft07().with(|s| {
        s.inline_subschemas = true;
        s.meta_schema = None;
    });
    settings.into_generator()
}

pub fn json_schema_for_func_args<T: ?Sized + JsonSchema>() -> serde_json::Value {
    let mut schema = get_schema_generator().into_root_schema_for::<T>();
    // remove title from schema
    schema.schema.metadata().title = None;
    serde_json::to_value(&schema).expect("Failed to serialize schema to json value")
}

pub enum OpenAiApiResponse {
    AssistantResponse(String),
    FunctionCallWithNoResponse,
}

#[async_trait]
pub trait ChatGptFunction: Send + Sync {
    fn name(&self) -> String;
    fn description(&self) -> String;
    fn parameters_schema(&self) -> serde_json::Value;

    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value>;
}

#[derive(Clone)]
pub struct ChatGptConversation {
    history: Vec<ChatCompletionRequestMessage>,
    tools: Vec<ChatCompletionTool>,
    temperature: Option<f32>,
    top_p: Option<f32>,
    model_name: String,
    function_table: HashMap<String, Arc<dyn ChatGptFunction>>,
}

impl ChatGptConversation {
    pub fn new(system_prompt: &str, model_name: &str) -> Self {
        let history = vec![ChatCompletionRequestSystemMessageArgs::default()
            .content(system_prompt)
            .build()
            // can this fail?
            .expect("Failed to build system prompt message")
            .into()];
        Self {
            history,
            tools: vec![],
            temperature: None,
            top_p: None,
            model_name: model_name.to_string(),
            function_table: HashMap::new(),
        }
    }

    pub fn add_function(&mut self, func: Arc<dyn ChatGptFunction>) -> anyhow::Result<()> {
        let new_function = FunctionObjectArgs::default()
            .name(func.name())
            .description(func.description())
            .parameters(func.parameters_schema())
            .build()?;

        let tool = ChatCompletionToolArgs::default()
            .r#type(ChatCompletionToolType::Function)
            .function(new_function)
            .build()?;

        self.tools.push(tool);
        self.function_table.insert(func.name(), func);
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
            .tools(self.tools.clone())
            .tool_choice(ChatCompletionToolChoiceOption::Auto);

        if let Some(temperature) = self.temperature {
            request_builder.temperature(temperature);
        }

        if let Some(top_p) = self.top_p {
            request_builder.top_p(top_p);
        }

        Ok(request_builder.build()?)
    }

    /// stream next message
    #[instrument(skip(self, client))]
    pub async fn next_message_stream(
        &mut self,
        message_text: Option<&str>,
        client: &Client<OpenAIConfig>,
    ) -> anyhow::Result<OpenAiApiResponse> {
        if let Some(message_text) = message_text {
            let user_message = ChatCompletionRequestUserMessageArgs::default()
                .content(message_text)
                .build()?
                .into();

            self.history.push(user_message);
        }

        let request = self.build_request_message()?;

        info!("starting stream");
        let mut stream = client.chat().create_stream(request).await?;

        // use this table to collect function call info
        let mut tool_call_map: HashMap<i32, async_openai::types::ChatCompletionMessageToolCall> =
            HashMap::new();
        let mut response_content_buffer = String::new();

        // handle stream collection
        while let Some(chunk) = stream.next().await {
            let chunk = chunk?;
            let choice = chunk
                .choices
                .first()
                .context("Failed to get first choice")?;

            if let Some(content) = &choice.delta.content {
                // TODO(David): Stream content here
                response_content_buffer.push_str(content);
            }

            // The openAI tool call streams are pretty messy. This code basically collects all of the tool calls into "buffers"
            // and executes them afterwards
            if let Some(tool_calls) = &choice.delta.tool_calls {
                for tool_call in tool_calls {
                    let index = tool_call.index;
                    if let Some(call) = &tool_call.function {
                        // some of these might be empty but that's fine because we are appending them
                        let id = tool_call.id.clone().unwrap_or_default();
                        let name = call.name.clone().unwrap_or_default();
                        let arguments = call.arguments.clone().unwrap_or_default();

                        let tool_call_ref = tool_call_map.entry(index).or_insert_with(|| {
                            // default empty
                            ChatCompletionMessageToolCall {
                                id: String::new(),
                                r#type: ChatCompletionToolType::Function,
                                function: FunctionCall {
                                    name: String::new(),
                                    arguments: String::new(),
                                },
                            }
                        });
                        tool_call_ref.id.push_str(&id);
                        tool_call_ref.function.name.push_str(&name);
                        tool_call_ref.function.arguments.push_str(&arguments);
                    }
                }
            }
        }
        info!(?tool_call_map, "Finished collecting stream");

        // execute tools calls
        // Make sure calls are sorted by index
        let mut tool_calls: Vec<_> = tool_call_map.into_iter().collect();
        tool_calls.sort_by_key(|a| a.0);
        let tool_calls: Vec<_> = tool_calls.into_iter().map(|(_key, call)| call).collect();

        // add tool calls to history
        if !tool_calls.is_empty() {
            let tool_call_request = ChatCompletionRequestAssistantMessageArgs::default()
                .tool_calls(tool_calls.clone())
                .build()?
                .into();
            self.history.push(tool_call_request);
        }

        for tool_call in tool_calls {
            if !matches!(tool_call.r#type, ChatCompletionToolType::Function) {
                tracing::error!("Tool call type is not function {:?}", tool_call.r#type);
            }
            let name = tool_call.function.name.clone();
            let args = tool_call.function.arguments.clone();
            let id = tool_call.id;
            let func_call_response = self
                .call_function(&name, &args)
                .await
                .context("Error in ChatGPT invoked function")?;

            // add response to history
            let tool_response = ChatCompletionRequestToolMessageArgs::default()
                .content(func_call_response.to_string())
                .tool_call_id(id)
                .build()
                .context("Failed to build tool response")?
                .into();
            self.history.push(tool_response);
        }

        if !response_content_buffer.is_empty() {
            let added_response = ChatCompletionRequestAssistantMessageArgs::default()
                .content(&response_content_buffer)
                .build()?
                .into();

            self.history.push(added_response);
            return Ok(OpenAiApiResponse::AssistantResponse(
                response_content_buffer,
            ));
        }

        Ok(OpenAiApiResponse::FunctionCallWithNoResponse)
    }

    pub fn get_history(&self) -> String {
        let history = OpenAiHistory {
            history: self.history.clone(),
            functions: vec![],
            tools: self.tools.clone(),
            timestamp: chrono::Utc::now(),
        };
        serde_json::to_string_pretty(&history).expect("Failed to serialize chat history")
    }
}
