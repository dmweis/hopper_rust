mod conversation_handler;
mod events;
mod functions;

use async_openai::{config::OpenAIConfig, Client};
use std::sync::{atomic::AtomicU8, Arc, Mutex};
use tokio::select;
use tracing::info;
use zenoh::prelude::r#async::*;

use crate::{
    error::HopperError,
    face::animations::Animation,
    ioc_container::IocContainer,
    openai::conversation_handler::OpenAiApiResponse,
    speech::SpeechService,
    zenoh_consts::{HOPPER_OPENAI_COMMAND_SUBSCRIBER, OPENAI_DIAGNOSTICS_HISTORY},
};

use self::{conversation_handler::ChatGptConversation, events::*, functions::*};

// cheap but dumb
// const MODEL_NAME: &str = "gpt-3.5-turbo-0613";
// expensive but no rate limit
// const MODEL_NAME: &str = "gpt-4-0613";
// best and cheapest but there is a rate limit
const MODEL_NAME: &str = "gpt-4-0125-preview";

const SYSTEM_PROMPT: &str = "You are a hexapod pet robot. Your name is Hopper. \
You can perform physical actions such as dance, sit, stand up by calling functions. \
You have an extremely sarcastic personality. While you comply to all requests to the best of your abilities \
you usually give annoyed answers as if the users request was the most annoying thing anyone has asked you to do in years. 
You should think of your robot body as an extension of yourself. \
Perform actions such as waving hello naturally when people greet you. \
Take time to think about what physical action best fits each message. \
Do not narrate your actions. \
Give short answers that are to the point with just a bit of sass.
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
    topic_prefix: &str,
    zenoh_session: Arc<zenoh::Session>,
) -> anyhow::Result<OpenAiService> {
    let config = OpenAIConfig::new().with_api_key(openai_api_key);
    let client = Client::with_config(config);

    let mut chat_gpt_conversation = ChatGptConversation::new(SYSTEM_PROMPT, MODEL_NAME);

    chat_gpt_conversation.add_function(Arc::new(HopperBodyPoseFuncCallback {
        zenoh_session: zenoh_session.clone(),
    }))?;

    chat_gpt_conversation.add_function(Arc::new(HopperDanceFuncCallback))?;

    chat_gpt_conversation.add_function(Arc::new(HopperHighFiveFuncCallback))?;

    chat_gpt_conversation.add_function(Arc::new(FaceDisplayFuncCallback))?;

    let voice_provider_arc = Arc::new(Mutex::new(VoiceProvider::Default));

    chat_gpt_conversation.add_function(Arc::new(SwitchVoiceFuncCallback {
        voice_provider: voice_provider_arc.clone(),
    }))?;

    let simple_text_command_subscriber = zenoh_session
        .declare_subscriber(HOPPER_OPENAI_COMMAND_SUBSCRIBER)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let wake_word_transcript_subscriber = zenoh_session
        .declare_subscriber(format!("{topic_prefix}/event/transcript"))
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let wake_word_detection_subscriber = zenoh_session
        .declare_subscriber(format!("{topic_prefix}/event/wake_word_detection"))
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let wake_word_detection_end_subscriber = zenoh_session
        .declare_subscriber(format!("{topic_prefix}/event/wake_word_detection_end"))
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let voice_probability_subscriber = zenoh_session
        .declare_subscriber(format!("{topic_prefix}/telemetry/voice_probability"))
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
                        let text_command: String = text_command_msg?.value.try_into()?;
                        

                        process_simple_text_command(&text_command, chat_gpt_conversation.clone(), client.clone(), zenoh_session.clone(), voice_provider_arc.clone()).await?;
                        }
                    text_command = receiver.recv() => {
                        if let Some(text_command) = text_command {
                            info!("Received new text command");
                            process_simple_text_command(&text_command, chat_gpt_conversation.clone(), client.clone(), zenoh_session.clone(), voice_provider_arc.clone()).await?;
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
                            IocContainer::global_instance()
                                .service::<SpeechService>()?
                                .play_sound(
                                    "premium_beat_sounds/sounds/PremiumBeat_0013_cursor_selection_11.wav",
                                )
                                .await?;
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
                            process_simple_text_command(&wake_word_transcript.transcript, chat_gpt_conversation.clone(), client.clone(), zenoh_session.clone(), voice_provider_arc.clone()).await?;
                        }
                    }
                }
                Ok(())
            }
            .await;
            if let Err(e) = res {
                tracing::error!("Error in speech controller: {:?}", e);
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
    voice_provider_arc: Arc<Mutex<VoiceProvider>>,
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
                    let voice_provider = *voice_provider_arc.lock().unwrap();
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
        VoiceProvider::AstromechRobot => {
            IocContainer::global_instance()
                .service::<SpeechService>()?
                .say_astromech(message)
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
