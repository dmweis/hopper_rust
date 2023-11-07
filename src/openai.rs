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
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::{collections::HashMap, sync::Arc};
use tokio::{select, sync::Mutex};
use tracing::info;
use zenoh::prelude::r#async::*;

use crate::{
    error::HopperError,
    high_five::HighFiveServiceController,
    ioc_container::IocContainer,
    lidar::LidarServiceController,
    motion_controller::{DanceMove, DanceService},
    speech::SpeechService,
    zenoh_consts::{
        HOPPER_OPENAI_COMMAND_SUBSCRIBER, OPENAI_DIAGNOSTICS_HISTORY, STANCE_SUBSCRIBER,
    },
};

// cheap but dumb
// const MODEL_NAME: &str = "gpt-3.5-turbo-0613";
// expensive but no rate limit
const MODEL_NAME: &str = "gpt-4-0613";
// best and cheapest but there is a rate limit
// const MODEL_NAME: &str = "gpt-4-1106-preview";

const SYSTEM_PROMPT: &str = "You are a hexapod pet robot. Your name is Hopper. \
You can perform physical actions such as dance, sit, stand up by calling functions. \
You have an extremely sarcastic. While you comply to most requests to best of your abilities. \
You usually give annoyed answers as if the users request was the most annoying thing anyone has asked you to do in years. \
You should think of your robot body as an extension of yourself. \
Perform actions such as waving hello naturally when people greet you. \
Take time to think about what physical action best fits each message. \
Do not narrate your actions. \
Give short and concise verbal answers. 
";

// Add to make robot speak slovak
// You can also speak slovak.

#[derive(Clone)]
pub struct OpenAiService {
    sender: tokio::sync::mpsc::Sender<String>,
}

impl OpenAiService {
    pub async fn send_command(&self, command: &str) -> anyhow::Result<()> {
        self.sender.send(command.to_string()).await?;
        Ok(())
    }
}

pub async fn start_openai_controller(
    openai_api_key: &str,
    zenoh_session: Arc<zenoh::Session>,
) -> anyhow::Result<OpenAiService> {
    let config = OpenAIConfig::new().with_api_key(openai_api_key);
    let client = Client::with_config(config);

    let mut chat_gpt_conversation = ChatGptConversation::new(SYSTEM_PROMPT, MODEL_NAME);

    chat_gpt_conversation.add_function::<HopperBodyPoseFuncArgs>(
        "set_body_pose",
        "set the body pose of the hopper hexapod robot",
        Arc::new(HopperBodyPoseFuncCallback {
            zenoh_session: zenoh_session.clone(),
        }),
    )?;

    chat_gpt_conversation.add_function::<HopperDanceFuncArgs>(
        "execute_hopper_dance",
        "perform a dance move with your body. Can be useful to express emotion or react to what user is saying",
        Arc::new(HopperDanceFuncCallback),
    )?;

    chat_gpt_conversation.add_function::<HopperHighFiveFuncArgs>(
        "enable_high_fives",
        "Enable automated high fives with the user. This mode will also enable the lidar sensor to detect the user.",
        Arc::new(HopperHighFiveFuncCallback),
    )?;

    let simple_text_command_subscriber = zenoh_session
        .declare_subscriber(HOPPER_OPENAI_COMMAND_SUBSCRIBER)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let (sender, mut receiver) = tokio::sync::mpsc::channel::<String>(10);

    tokio::spawn(async move {
        loop {
            let res: anyhow::Result<()> = async {
                select! {
                    text_command_msg = simple_text_command_subscriber.recv_async() => {
                        info!("Received new zenoh text command");
                        let text_command_msg = text_command_msg?;
                        let text_command: String = text_command_msg.value.try_into()?;
                        process_simple_text_command(&text_command, chat_gpt_conversation.clone(), client.clone(), zenoh_session.clone()).await?;
                        }
                    text_command = receiver.recv() => {
                        if let Some(text_command) = text_command {
                            info!("Received new text command");
                            process_simple_text_command(&text_command, chat_gpt_conversation.clone(), client.clone(), zenoh_session.clone()).await?;
                        }
                    }
                }
                Ok(())
            }
            .await;
            if let Err(e) = res {
                tracing::error!("Error in speech controller: {}", e);
            }
        }
    });

    let open_ai_service = OpenAiService { sender };

    Ok(open_ai_service)
}

async fn process_simple_text_command(
    text_command: &str,
    mut conversation: ChatGptConversation,
    open_ai_client: Client<OpenAIConfig>,
    zenoh_session: Arc<zenoh::Session>,
) -> anyhow::Result<()> {
    info!("Received hopper command {:?}", text_command);

    let mut command = Some(text_command);
    // get responses

    loop {
        match conversation
            .next_message_stream(command.take(), &open_ai_client)
            .await?
        {
            OpenAiApiResponse::AssistantResponse(response) => {
                info!("Assistant response form ChatGPT: {:?}", response);

                tokio::spawn(async move {
                    if let Err(err) = speak_with_face_animation(&response).await {
                        tracing::error!("Failed to speak with face animation: {}", err);
                    }
                });

                break;
            }
            OpenAiApiResponse::FunctionCallWithNoResponse => {
                // nothing to do here
            }
        }
    }

    let history = conversation.get_history();

    zenoh_session
        .put(OPENAI_DIAGNOSTICS_HISTORY, history)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    Ok(())
}

async fn speak_with_face_animation(message: &str) -> anyhow::Result<()> {
    IocContainer::global_instance()
        .service::<Mutex<SpeechService>>()?
        .lock()
        .await
        .say_azure_with_style(message, crate::speech::AzureVoiceStyle::Cheerful)
        .await?;

    IocContainer::global_instance()
        .service::<crate::face::FaceController>()?
        .speaking(crate::face::driver::CYAN)?;

    IocContainer::global_instance()
        .service::<Mutex<SpeechService>>()?
        .lock()
        .await
        .wait_until_sound_ends()
        .await;

    IocContainer::global_instance()
        .service::<crate::face::FaceController>()?
        .larson_scanner(crate::face::driver::PURPLE)?;

    Ok(())
}

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
        serde_json::to_string_pretty(&self.history).expect("Failed to serialize chat history")
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct HopperBodyPoseFuncArgs {
    /// body pose
    pub body_pose: HopperBodyPose,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum HopperBodyPose {
    Folded,
    Standing,
    Sitting,
}

struct HopperBodyPoseFuncCallback {
    zenoh_session: Arc<zenoh::Session>,
}

#[async_trait]
impl AsyncCallback for HopperBodyPoseFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let hopper_body_pose_func: HopperBodyPoseFuncArgs = serde_json::from_str(args)?;

        let message = match hopper_body_pose_func.body_pose {
            HopperBodyPose::Folded => "folded",
            HopperBodyPose::Standing => "stand",
            HopperBodyPose::Sitting => "ground",
        };

        self.zenoh_session
            .put(STANCE_SUBSCRIBER, message)
            .res()
            .await
            .map_err(HopperError::ZenohError)?;

        let result = json!({
            "success": true
        });
        Ok(result)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct HopperDanceFuncArgs {
    /// dance move to perform
    pub dance_move: DanceMove,
}

struct HopperDanceFuncCallback;

#[async_trait]
impl AsyncCallback for HopperDanceFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let hopper_dance_move: HopperDanceFuncArgs = serde_json::from_str(args)?;

        IocContainer::global_instance()
            .service::<DanceService>()?
            .start_sequence(hopper_dance_move.dance_move);

        let result = json!({
            "success": true
        });
        Ok(result)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct HopperHighFiveFuncArgs {
    /// respond to high fives
    pub enable_high_fives: bool,
}

struct HopperHighFiveFuncCallback;

#[async_trait]
impl AsyncCallback for HopperHighFiveFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let high_five_args: HopperHighFiveFuncArgs = serde_json::from_str(args)?;

        IocContainer::global_instance()
            .service::<HighFiveServiceController>()?
            .set_active(high_five_args.enable_high_fives);

        IocContainer::global_instance()
            .service::<LidarServiceController>()?
            .set_active(high_five_args.enable_high_fives);

        let result = json!({
            "high_fives_enabled": high_five_args.enable_high_fives
        });
        Ok(result)
    }
}
