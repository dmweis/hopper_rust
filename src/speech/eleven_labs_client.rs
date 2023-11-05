use anyhow::Context;
use anyhow::Result;
use bytes::Bytes;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

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
            model_id: Some(String::from("eleven_multilingual_v1")),
            voice_settings: Some(VoiceSettings {
                similarity_boost: 0.5,
                stability: 0.5,
                style: None,
                use_speaker_boost: None,
            }),
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
}
