use anyhow::Context;
use async_openai::{config::OpenAIConfig, types::CreateTranscriptionRequestArgs, Client};
use base64::{engine::general_purpose, Engine};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tempdir::TempDir;
use tokio::select;
use zenoh::prelude::r#async::*;

use crate::{
    error::HopperError,
    zenoh_remotes::topic_consts::{HOPPER_OPENAI_VOICE_COMMAND_SUBSCRIBER, OPENAI_DIAGNOSTICS_TRANSCRIPT}, openai::OpenAiService, ioc_container::IocContainer,
};

const VOICE_TO_TEXT_TRANSCRIBE_MODEL: &str = "whisper-1";
const VOICE_TO_TEXT_TRANSCRIBE_MODEL_ENGLISH_LANGUAGE: &str = "en";

pub async fn start_audio_transcribe_service(
    openai_api_key: &str,
    zenoh_session: Arc<zenoh::Session>,
) -> anyhow::Result<()> {
    let config = OpenAIConfig::new().with_api_key(openai_api_key);
    let client = Client::with_config(config);

    let audio_command_subscriber = zenoh_session
        .declare_subscriber(HOPPER_OPENAI_VOICE_COMMAND_SUBSCRIBER)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    

    // this is because of the transcribe future not being send
    tokio::spawn(async move  {
        loop {
            let res: anyhow::Result<()> = async {
                    select! {
                        audio_command_msg = audio_command_subscriber.recv_async() => {
                            let audio_command_msg = audio_command_msg?;
                            let audio_command_msg_json: String = audio_command_msg.value.try_into()?;
                            let encoded_audio_command: Base64AudioMessage = serde_json::from_str(&audio_command_msg_json)?;
                            let audio_command: DecodedAudioMessage = encoded_audio_command.try_into()?;
                            let text = transcribe(audio_command, "Audio command for a hexapod robot called Hopper", &client).await?;
                            
                             IocContainer::global_instance()
                                .service::<OpenAiService>()?
                                .send_command(&text)
                                .await?;

                                zenoh_session.put(
                                    OPENAI_DIAGNOSTICS_TRANSCRIPT,
                                    text,
                                ).res().await.map_err(HopperError::ZenohError)?;

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

    Ok(())
}

pub async fn transcribe(
    audio: DecodedAudioMessage,
    prompt: &str,
    open_ai_client: &Client<OpenAIConfig>,
) -> anyhow::Result<String> {
    let temp_dir = TempDir::new("audio_message_temp_dir")?;
    let temp_auido_file = temp_dir
        .path()
        .join(format!("recorded.{}", audio.format_extension));

    tokio::fs::write(&temp_auido_file, &audio.data).await?;

    let request = CreateTranscriptionRequestArgs::default()
        .file(temp_auido_file)
        .model(VOICE_TO_TEXT_TRANSCRIBE_MODEL)
        .language(VOICE_TO_TEXT_TRANSCRIBE_MODEL_ENGLISH_LANGUAGE)
        .prompt(prompt)
        .build()?;

    let response = open_ai_client.audio().transcribe(request).await?;
    Ok(response.text)
}

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct Base64AudioMessage {
    pub data: String,
    pub format: String,
}

pub struct DecodedAudioMessage {
    pub data: Vec<u8>,
    /// .wav, .mp3, etc
    pub format_extension: String,
}

impl DecodedAudioMessage {
    pub fn new(data: Vec<u8>, format_extension: &str) -> Self {
        Self {
            data,
            format_extension: format_extension.to_string(),
        }
    }
}

impl TryFrom<Base64AudioMessage> for DecodedAudioMessage {
    type Error = anyhow::Error;

    fn try_from(audio_message: Base64AudioMessage) -> anyhow::Result<Self> {
        let data = base64_to_binary(&audio_message.data)?;
        let format_extension = audio_message.format;
        Ok(Self {
            data,
            format_extension,
        })
    }
}

pub fn base64_to_binary(base64: &str) -> anyhow::Result<Vec<u8>> {
    let decoded_file = general_purpose::STANDARD
        .decode(base64)
        .context("Failed to parse base64")?;
    Ok(decoded_file)
}
