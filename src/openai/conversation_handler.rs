use anyhow::Context;
use async_openai::{
    config::OpenAIConfig,
    types::{
        ChatCompletionFunctions, ChatCompletionFunctionsArgs,
        ChatCompletionRequestAssistantMessageArgs, ChatCompletionRequestMessage,
        ChatCompletionRequestSystemMessageArgs, ChatCompletionRequestToolMessageArgs,
        ChatCompletionRequestUserMessageArgs, ChatCompletionTool, ChatCompletionToolArgs,
        ChatCompletionToolChoiceOption, ChatCompletionToolType, CreateChatCompletionRequest,
        CreateChatCompletionRequestArgs,
    },
    Client,
};
use async_trait::async_trait;
use schemars::{gen::SchemaSettings, JsonSchema};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, sync::Arc};
use tracing::info;

#[derive(Serialize, Deserialize, Debug)]
pub struct OpenAiHistory {
    history: Vec<ChatCompletionRequestMessage>,
    functions: Vec<ChatCompletionFunctions>,
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

fn json_schema_for_func_args<T: ?Sized + JsonSchema>() -> anyhow::Result<serde_json::Value> {
    let mut schema = get_schema_generator().into_root_schema_for::<T>();
    // remove title from schema
    schema.schema.metadata().title = None;
    let schema_json = serde_json::to_value(&schema)?;
    Ok(schema_json)
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
    tools: Vec<ChatCompletionTool>,
    temperature: Option<f32>,
    top_p: Option<f32>,
    model_name: String,
    function_table: HashMap<String, Arc<dyn AsyncCallback>>,
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

    pub fn add_function<T: ?Sized + JsonSchema>(
        &mut self,
        function_name: &str,
        function_description: &str,
        func: Arc<dyn AsyncCallback>,
    ) -> anyhow::Result<()> {
        let schema_json = json_schema_for_func_args::<T>()?;
        let new_function = ChatCompletionFunctionsArgs::default()
            .name(function_name)
            .description(function_description)
            .parameters(schema_json)
            .build()?;

        let tool = ChatCompletionToolArgs::default()
            .r#type(ChatCompletionToolType::Function)
            .function(new_function)
            .build()?;

        self.tools.push(tool);
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

        let response_message = client
            .chat()
            .create(request)
            .await?
            .choices
            .get(0)
            .context("Failed to get first choice on OpenAI api response")?
            .message
            .clone();

        // execute tools calls

        if let Some(tool_calls) = response_message.tool_calls {
            // add tool calls to history
            let tool_call_request = ChatCompletionRequestAssistantMessageArgs::default()
                .tool_calls(tool_calls.clone())
                .build()?
                .into();
            self.history.push(tool_call_request);

            for tool_call in tool_calls {
                if !matches!(tool_call.r#type, ChatCompletionToolType::Function) {
                    tracing::error!("Tool call type is not function {:?}", tool_call.r#type);
                }
                let name = tool_call.function.name.clone();
                let args = tool_call.function.arguments.clone();
                let id = tool_call.id;
                let func_call_response = self.call_function(&name, &args).await?;

                // add response to history
                let tool_response = ChatCompletionRequestToolMessageArgs::default()
                    .content(func_call_response.to_string())
                    .tool_call_id(id)
                    .build()
                    .context("Failed to build tool response")?
                    .into();
                self.history.push(tool_response);
            }
        }

        if let Some(content) = response_message.content {
            let added_response = ChatCompletionRequestAssistantMessageArgs::default()
                .content(&content)
                .build()?
                .into();

            self.history.push(added_response);
            return Ok(OpenAiApiResponse::AssistantResponse(content));
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
