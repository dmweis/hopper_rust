// Handle compiling without alsa for cross compilation

#[cfg(feature = "audio")]
mod audio_cache;
#[cfg(feature = "audio")]
mod audio_repository;
#[cfg(feature = "audio")]
mod speech_service;

#[cfg(feature = "audio")]
mod eleven_labs_client;

#[cfg(feature = "audio")]
pub use speech_service::SpeechService;

#[cfg(not(feature = "audio"))]
mod fake_speech_service;

#[cfg(not(feature = "audio"))]
pub use fake_speech_service::SpeechService;

// These are styles that apply to en-US-SaraNeural
// since that's the most used voice
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize, Default)]
pub enum AzureVoiceStyle {
    #[default]
    Plain,
    Angry,
    Cheerful,
    Sad,
}
