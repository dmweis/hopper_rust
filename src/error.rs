use dynamixel_driver::DynamixelDriverError;
use std::result::Result;
use thiserror::Error;

pub type HopperResult<T> = Result<T, HopperError>;

#[derive(Error, Debug)]
pub enum HopperError {
    #[error("Generic IK error")]
    GenericIkError,
    #[error("Dynamixel driver error")]
    DynamixelDriverError(#[from] DynamixelDriverError),
    #[error("IO error")]
    IoError(#[from] std::io::Error),
    #[error("Toml serde error")]
    TomlError(#[from] toml::ser::Error),
    #[error("Json serde error")]
    JsonError(#[from] serde_json::Error),
    #[error("Lidar error")]
    LidarError(#[from] rplidar_driver::RposError),
    #[error("Audio cache dir error")]
    AudioCacheDirError,
    #[error("Failed to decode audio file")]
    FailedToDecodeAudioFile,
    #[error("Failed to create audio output stream")]
    FailedToCreateAudioOutputStream,
    #[error("Failed to create audio sink")]
    FailedToCreateAudioSink,
    #[cfg(feature = "audio")]
    #[error("Text to speech error")]
    TtsError(#[from] azure_tts::TtsError),
}

impl HopperError {
    pub fn is_recoverable_driver_error(&self) -> bool {
        if let HopperError::DynamixelDriverError(error) = self {
            error.is_recoverable()
        } else {
            false
        }
    }
}
