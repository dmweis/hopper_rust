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
use std::{
    collections::HashMap,
    sync::{atomic::AtomicU8, Arc, Mutex},
};
use tokio::select;
use tracing::info;
use zenoh::prelude::r#async::*;

use crate::{
    error::HopperError,
    face::animations::Animation,
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
// const MODEL_NAME: &str = "gpt-4-0613";
// best and cheapest but there is a rate limit
const MODEL_NAME: &str = "gpt-4-1106-preview";

const SYSTEM_PROMPT: &str = "You are a hexapod pet robot. Your name is Hopper. \
You can perform physical actions such as dance, sit, stand up by calling functions. \
You have an extremely sarcastic personality. While you comply to all requests to the best of your abilities \
you usually give annoyed answers as if the users request was the most annoying thing anyone has asked you to do in years. 
You should think of your robot body as an extension of yourself. \
Perform actions such as waving hello naturally when people greet you. \
Take time to think about what physical action best fits each message. \
Do not narrate your actions. \
Give answers should be short and to the point with just a bit of sass. 
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

    chat_gpt_conversation.add_function::<FaceDisplayFuncArgs>(
        "set_face_animation",
        "Control the face panel on the Hopper robot. You can set the color and animation.",
        Arc::new(FaceDisplayFuncCallback),
    )?;

    let voice_provider_arc = Arc::new(Mutex::new(VoiceProvider::Default));

    chat_gpt_conversation.add_function::<SwitchVoiceFuncArgs>(
        "switch_voice_provider",
        "Switch voice provider",
        Arc::new(SwitchVoiceFuncCallback {
            voice_provider: voice_provider_arc.clone(),
        }),
    )?;

    let simple_text_command_subscriber = zenoh_session
        .declare_subscriber(HOPPER_OPENAI_COMMAND_SUBSCRIBER)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let wake_word_transcript_subscriber = zenoh_session
        .declare_subscriber("wakeword/event/transcript")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let wake_word_detection_subscriber = zenoh_session
        .declare_subscriber("wakeword/event/wake_word_detection")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let wake_word_detection_end_subscriber = zenoh_session
        .declare_subscriber("wakeword/event/wake_word_detection_end")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let voice_probability_subscriber = zenoh_session
        .declare_subscriber("wakeword/telemetry/voice_probability")
        .best_effort()
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let voice_probability_val = Arc::new(AtomicU8::new(0));

    let (sender, mut receiver) = tokio::sync::mpsc::channel::<String>(10);

    tokio::spawn(async move {
        loop {
            let res: anyhow::Result<()> = async {
                select! {
                    text_command_msg = simple_text_command_subscriber.recv_async() => {
                        info!("Received new zenoh text command");
                        let text_command_msg = text_command_msg?;
                        let text_command: String = text_command_msg.value.try_into()?;
                        let voice_provider = *voice_provider_arc.lock().unwrap();

                        process_simple_text_command(&text_command, chat_gpt_conversation.clone(), client.clone(), zenoh_session.clone(), voice_provider).await?;
                        }
                    text_command = receiver.recv() => {
                        if let Some(text_command) = text_command {
                            info!("Received new text command");
                            let voice_provider = *voice_provider_arc.lock().unwrap();
                            process_simple_text_command(&text_command, chat_gpt_conversation.clone(), client.clone(), zenoh_session.clone(), voice_provider).await?;
                        }
                    }
                    wake_word_detection = wake_word_detection_subscriber.recv_async() => {
                        info!("Received wakeword detection");
                        let wake_word_detection: String = wake_word_detection?.value.try_into()?;
                        let wake_word_detection = serde_json::from_str::<WakeWordDetection>(&wake_word_detection)?;
                        if wake_word_detection.wake_word.to_lowercase().contains("hopper") {
                            IocContainer::global_instance()
                                .service::<crate::face::FaceController>()?
                                .set_temporary_animation(Animation::Speaking(crate::face::driver::RED, voice_probability_val.clone()))?;
                        }
                    }
                    wake_word_detection_end = wake_word_detection_end_subscriber.recv_async() => {
                        info!("Received wakeword detection end");
                        let wake_word_detection_end: String = wake_word_detection_end?.value.try_into()?;
                        let wake_word_detection_end = serde_json::from_str::<WakeWordDetection>(&wake_word_detection_end)?;
                        if wake_word_detection_end.wake_word.to_lowercase().contains("hopper") {
                            IocContainer::global_instance()
                                .service::<crate::face::FaceController>()?
                                .clear_temporary_animation()?;
                        }
                    }
                    voice_probability_msg = voice_probability_subscriber.recv_async() => {
                        let voice_probability_msg: String = voice_probability_msg?.value.try_into()?;
                        let voice_probability = serde_json::from_str::<VoiceProbability>(&voice_probability_msg)?;
                        voice_probability_val.store((voice_probability.probability * 255.0) as u8, std::sync::atomic::Ordering::Relaxed);
                    }
                    wake_word_transcript = wake_word_transcript_subscriber.recv_async() => {
                        let wake_word_transcript: String = wake_word_transcript?.value.try_into()?;
                        let wake_word_transcript: AudioTranscript = serde_json::from_str(&wake_word_transcript)?;
                        if wake_word_transcript.wake_word.to_lowercase().contains("hopper") {
                            info!("Received new text command");
                            let voice_provider = *voice_provider_arc.lock().unwrap();
                            process_simple_text_command(&wake_word_transcript.transcript, chat_gpt_conversation.clone(), client.clone(), zenoh_session.clone(), voice_provider).await?;
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
    voice_provider: VoiceProvider,
) -> anyhow::Result<()> {
    info!("Received hopper command {:?}", text_command);

    let mut command = Some(text_command);
    // get responses

    loop {
        IocContainer::global_instance()
            .service::<crate::face::FaceController>()?
            .set_temporary_animation(Animation::CountDownBasic)?;

        let next_response = conversation
            .next_message_stream(command.take(), &open_ai_client)
            .await?;

        IocContainer::global_instance()
            .service::<crate::face::FaceController>()?
            .clear_temporary_animation()?;

        match next_response {
            OpenAiApiResponse::AssistantResponse(response) => {
                info!("Assistant response form ChatGPT: {:?}", response);

                tokio::spawn(async move {
                    if let Err(err) = speak_with_face_animation(&response, voice_provider).await {
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

async fn speak_with_face_animation(
    message: &str,
    voice_provider: VoiceProvider,
) -> anyhow::Result<()> {
    match voice_provider {
        VoiceProvider::Default => {
            IocContainer::global_instance()
                .service::<SpeechService>()?
                .say_azure_with_style(message, crate::speech::AzureVoiceStyle::Cheerful)
                .await?;
        }
        VoiceProvider::Expensive => {
            IocContainer::global_instance()
                .service::<SpeechService>()?
                .say_eleven_with_default_voice(message)
                .await?;
        }
    }

    IocContainer::global_instance()
        .service::<crate::face::FaceController>()?
        .set_temporary_animation(Animation::SpeakingRandom(crate::face::driver::CYAN))?;

    IocContainer::global_instance()
        .service::<SpeechService>()?
        .wait_until_sound_ends()
        .await;

    IocContainer::global_instance()
        .service::<crate::face::FaceController>()?
        .clear_temporary_animation()?;

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
        let history = OpenAiHistory {
            history: self.history.clone(),
            timestamp: chrono::Utc::now(),
        };
        serde_json::to_string_pretty(&history).expect("Failed to serialize chat history")
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct OpenAiHistory {
    history: Vec<ChatCompletionRequestMessage>,
    timestamp: chrono::DateTime<chrono::Utc>,
}

#[derive(Serialize, Deserialize, Debug)]
struct WakeWordDetection {
    wake_word: String,
    timestamp: chrono::DateTime<chrono::Utc>,
}

#[derive(Serialize, Deserialize, Debug)]
struct WakeWordDetectionEnd {
    wake_word: String,
    timestamp: chrono::DateTime<chrono::Utc>,
}

#[derive(Serialize, Deserialize, Debug)]
struct AudioTranscript {
    wake_word: String,
    timestamp: chrono::DateTime<chrono::Utc>,
    transcript: String,
}

#[derive(Serialize, Deserialize, Debug)]
struct VoiceProbability {
    probability: f32,
    timestamp: chrono::DateTime<chrono::Utc>,
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

        // stop high fives in case we are sitting down or folding
        match hopper_body_pose_func.body_pose {
            HopperBodyPose::Folded | HopperBodyPose::Sitting => {
                IocContainer::global_instance()
                    .service::<HighFiveServiceController>()?
                    .set_active(false);

                IocContainer::global_instance()
                    .service::<LidarServiceController>()?
                    .set_active(false);
            }
            _ => (),
        }

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

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct FaceDisplayFuncArgs {
    /// Animation that is currently displayed on face display
    pub animation: FaceAnimation,
    /// Optional color for animations
    /// Not all animations require a color
    /// If no color is provided, the default color for the animation will be used
    pub color: Option<FaceColor>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum FaceColor {
    Red,
    Green,
    Blue,
    Yellow,
    Purple,
    Cyan,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum FaceAnimation {
    /// Larson scanner
    /// Similar to KITT from the show Knight Rider
    LarsonScanner,
    /// Speaking animation
    /// Looks like a wave form of a human voice
    SpeakingAnimation,
    /// Breathing animation
    /// Pulsating light
    BreathingAnimation,
    /// Solid color
    SolidColor,
    /// Lights off
    /// This animation doesn't need color
    Off,
    CountDownAnimation,
}

struct FaceDisplayFuncCallback;

#[async_trait]
impl AsyncCallback for FaceDisplayFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let face_display_args: FaceDisplayFuncArgs = serde_json::from_str(args)?;

        let color = match face_display_args.color {
            Some(color) => match color {
                FaceColor::Red => crate::face::driver::RED,
                FaceColor::Green => crate::face::driver::GREEN,
                FaceColor::Blue => crate::face::driver::BLUE,
                FaceColor::Yellow => crate::face::driver::YELLOW,
                FaceColor::Purple => crate::face::driver::PURPLE,
                FaceColor::Cyan => crate::face::driver::CYAN,
            },
            None => crate::face::driver::PURPLE,
        };

        let face_controller =
            IocContainer::global_instance().service::<crate::face::FaceController>()?;

        match face_display_args.animation {
            FaceAnimation::LarsonScanner => face_controller.larson_scanner(color)?,
            FaceAnimation::SpeakingAnimation => face_controller.speaking(color)?,
            FaceAnimation::BreathingAnimation => face_controller.breathing(color)?,
            FaceAnimation::SolidColor => face_controller.solid_color(color)?,
            FaceAnimation::Off => face_controller.off()?,
            FaceAnimation::CountDownAnimation => face_controller.count_down_basic()?,
        }

        let result = json!({});
        Ok(result)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct SwitchVoiceFuncArgs {
    /// TTS voice provider
    pub voice_provider: VoiceProvider,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, JsonSchema, Default)]
#[serde(rename_all = "snake_case")]
pub enum VoiceProvider {
    /// default voice provider Microsoft Azure
    #[default]
    Default,
    /// expensive voice provider form Eleven Labs
    /// Should be used carefully
    Expensive,
}

struct SwitchVoiceFuncCallback {
    voice_provider: Arc<Mutex<VoiceProvider>>,
}

#[async_trait]
impl AsyncCallback for SwitchVoiceFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let switch_voice: SwitchVoiceFuncArgs = serde_json::from_str(args)?;

        let mut voice_provider = self.voice_provider.lock().unwrap();

        *voice_provider = switch_voice.voice_provider;

        let result = json!({});
        Ok(result)
    }
}
