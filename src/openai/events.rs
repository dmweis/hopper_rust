use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct WakeWordDetection {
    pub wake_word: String,
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct WakeWordDetectionEnd {
    pub wake_word: String,
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct AudioTranscript {
    pub wake_word: String,
    pub timestamp: chrono::DateTime<chrono::Utc>,
    pub transcript: String,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VoiceProbability {
    pub probability: f32,
    pub timestamp: chrono::DateTime<chrono::Utc>,
}
