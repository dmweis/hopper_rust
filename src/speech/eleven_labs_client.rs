use anyhow::{Context, Result};
use base64::prelude::*;
use bytes::Bytes;
use futures::{stream::SplitSink, SinkExt};
use futures_util::StreamExt;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use tokio::{net::TcpStream, sync::mpsc::Receiver};
use tokio_tungstenite::{connect_async, tungstenite::Message, MaybeTlsStream, WebSocketStream};
use tracing::info;

const VOICE_MODEL: &str = "eleven_multilingual_v2";

#[derive(Debug, Serialize, Deserialize)]
pub struct TtsRequest {
    /// Identifier of the model that will be used.
    /// Defaults to "eleven_monolingual_v1"
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_id: Option<String>,
    /// The text that will get converted into speech.
    pub text: String,
    /// Voice settings overriding stored setttings for the given voice.
    /// They are applied only on the given TTS request.
    /// Defaults to None
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_settings: Option<VoiceSettings>,
}

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone)]
pub struct VoiceSettings {
    pub similarity_boost: f64,
    pub stability: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub style: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub use_speaker_boost: Option<bool>,
}

impl Default for VoiceSettings {
    fn default() -> Self {
        Self {
            similarity_boost: 0.5,
            stability: 0.5,
            style: None,
            use_speaker_boost: None,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Voices {
    pub voices: Vec<Voice>,
}

impl Voices {
    pub fn name_to_id_table(&self) -> HashMap<String, String> {
        let mut table = HashMap::new();
        for voice in &self.voices {
            table.insert(voice.name.clone(), voice.voice_id.clone());
        }
        table
    }
}

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone)]
pub struct Voice {
    pub voice_id: String,
    pub name: String,
    pub samples: Option<Vec<VoiceSample>>,
    pub category: Option<String>,
    pub labels: Option<HashMap<String, String>>,
    pub description: Option<String>,
    pub preview_url: Option<String>,
    pub settings: Option<VoiceSettings>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct VoiceSample {
    pub sample_id: String,
    file_name: String,
    mime_type: String,
    size_bytes: Option<i64>,
    hash: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Subscription {
    tier: String,
    pub character_count: i64,
    pub character_limit: i64,
    can_extend_character_limit: bool,
    allowed_to_extend_character_limit: bool,
    next_character_count_reset_unix: i64,
    voice_limit: i64,
    professional_voice_limit: i64,
    can_extend_voice_limit: bool,
    can_use_instant_voice_cloning: bool,
    can_use_professional_voice_cloning: bool,
    currency: Option<String>,
    status: String,
    next_invoice: Option<Invoice>,
}

impl Subscription {
    #[allow(dead_code)]
    pub fn character_left(&self) -> i64 {
        self.character_limit - self.character_count
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Invoice {
    amount_due_cents: i64,
    next_payment_attempt_unix: i64,
}

#[derive(Default, Debug, Clone, Serialize)]
pub struct StreamingInputText {
    pub text: String,
    pub voice_settings: Option<VoiceSettings>,
    pub generation_config: Option<GenerationConfig>,
    pub xi_api_key: Option<String>,
    pub authorization: Option<String>,
}

#[derive(Default, Debug, Clone, Serialize)]
pub struct GenerationConfig {
    pub chunk_length_schedule: Vec<i64>,
}

#[derive(Default, Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct StreamingOutputAudio {
    pub audio: Option<String>,
    pub is_final: Option<bool>,
    pub normalized_alignment: Option<Alignment>,
    pub alignment: Option<Alignment>,
}

#[derive(Default, Debug, Clone, Deserialize)]
pub struct Alignment {
    #[serde(default)]
    pub char_start_times_ms: Vec<u32>,
    #[serde(default)]
    pub chars_durations_ms: Vec<u32>,
    #[serde(default)]
    pub chars: Vec<char>,
}

#[derive(Debug, Clone)]
pub struct ElevenLabsTtsClient {
    client: reqwest::Client,
    api_key: String,
}

impl ElevenLabsTtsClient {
    pub fn new(api_key: String) -> Self {
        ElevenLabsTtsClient {
            client: reqwest::Client::new(),
            api_key,
        }
    }

    pub async fn tts(&self, text: &str, voice_id: &str) -> Result<Bytes> {
        let url = format!("https://api.elevenlabs.io/v1/text-to-speech/{}", voice_id);

        let body = TtsRequest {
            text: text.to_owned(),
            // switch to better model that is slower
            // model_id: Some(String::from("eleven_multilingual_v2")),
            model_id: Some(VOICE_MODEL.to_owned()),
            voice_settings: Some(VoiceSettings::default()),
        };

        let resp = self
            .client
            .post(url)
            .header("xi-api-key", self.api_key.clone())
            .header("accept", "audio/mpeg")
            .header("Content-Type", "application/json")
            .json(&body)
            .send()
            .await?;
        resp.error_for_status_ref()
            .context("Request failed with status")?;
        let data = resp.bytes().await?;
        Ok(data)
    }

    pub async fn voices(&self) -> Result<Voices> {
        let resp = self
            .client
            .get("https://api.elevenlabs.io/v1/voices")
            .header("xi-api-key", self.api_key.clone())
            .header("accept", "application/json")
            .send()
            .await?;
        resp.error_for_status_ref()
            .context("Request failed with status")?;
        let data = resp.json::<Voices>().await?;

        Ok(data)
    }

    #[allow(dead_code)]
    pub async fn get_subscription_info(&self) -> Result<Subscription> {
        let resp = self
            .client
            .get("https://api.elevenlabs.io/v1/user/subscription")
            .header("xi-api-key", self.api_key.clone())
            .header("accept", "application/json")
            .send()
            .await?;
        resp.error_for_status_ref()
            .context("Request failed with status")?;
        let data = resp.json::<Subscription>().await?;

        Ok(data)
    }

    pub async fn start_streaming_session(
        &self,
        voice_id: &str,
    ) -> Result<(StreamingSession, Receiver<Vec<u8>>)> {
        let url = format!(
            "wss://api.elevenlabs.io/v1/text-to-speech/{voice_id}/stream-input?model_id={VOICE_MODEL}"
        );

        let (ws_stream, response) = connect_async(url).await.context("Failed to connect")?;

        if response.status().is_client_error() || response.status().is_server_error() {
            anyhow::bail!("Request failed {:?}", response.status());
        }

        let (writer, mut read) = ws_stream.split();

        let streaming_session = StreamingSession {
            api_key: Some(self.api_key.clone()),
            voice_settings: Some(VoiceSettings::default()),
            writer,
        };

        let (sender, receiver) = tokio::sync::mpsc::channel(10);

        tokio::spawn(async move {
            while let Some(message) = read.next().await {
                //TODO(David): these unwraps are not great
                let message = message.unwrap();
                match message {
                    Message::Text(text) => {
                        let parsed: StreamingOutputAudio = serde_json::from_str(&text)
                            .context("Failed to parse json")
                            .unwrap();
                        if let Some(audio) = &parsed.audio {
                            let decoded = BASE64_STANDARD
                                .decode(audio)
                                .context("Failed to parse base64")
                                .unwrap();

                            if !decoded.is_empty() {
                                sender.send(decoded).await.unwrap();
                                if let Some(alignment) = parsed.alignment {
                                    let contents: String = alignment.chars.iter().collect();
                                    info!("Playing audio chunk with contents: {:?}", contents);
                                }
                            }
                        }
                        if let Some(is_final) = parsed.is_final {
                            if is_final {
                                break;
                            }
                        }
                    }
                    other => {
                        tracing::warn!("Received other message from eleven labs {:?}", other);
                        // ignore other messages
                    }
                }
            }
        });

        Ok((streaming_session, receiver))
    }
}

pub struct StreamingSession {
    api_key: Option<String>,
    voice_settings: Option<VoiceSettings>,
    writer: SplitSink<WebSocketStream<MaybeTlsStream<TcpStream>>, Message>,
}

impl StreamingSession {
    pub async fn send_chunk(&mut self, text: &str) -> anyhow::Result<()> {
        if text.is_empty() {
            // do not send empty chunks because that will terminate the connection
            return Ok(());
        }
        let input = StreamingInputText {
            text: text.to_owned(),
            voice_settings: self.voice_settings.take(),
            generation_config: None,
            xi_api_key: self.api_key.take(),
            authorization: None,
        };

        let json = serde_json::to_string(&input)?;
        let message = Message::text(json);
        self.writer.send(message).await?;

        Ok(())
    }

    pub async fn finish(&mut self) -> anyhow::Result<()> {
        let input = StreamingInputText {
            text: String::from(""),
            voice_settings: self.voice_settings.take(),
            generation_config: None,
            xi_api_key: self.api_key.take(),
            authorization: None,
        };

        let json = serde_json::to_string(&input)?;
        let message = Message::text(json);
        self.writer.send(message).await?;

        Ok(())
    }
}
