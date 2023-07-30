use dynamixel_driver::DynamixelDriverError;
use std::result::Result;
use thiserror::Error;

pub type HopperResult<T> = Result<T, HopperError>;

#[derive(Error, Debug)]
pub enum HopperError {
    #[error("Generic IK error")]
    GenericIkError,
    #[error("Dynamixel sync write error {0:?}")]
    DynamixelSyncWriteError(#[source] DynamixelDriverError),
    #[error("Dynamixel driver error {0} {1:?}")]
    DynamixelDriverError(u8, #[source] DynamixelDriverError),
    #[error("IO error {0:?}")]
    IoError(#[from] std::io::Error),
    #[error("Toml serde error {0:?}")]
    TomlError(#[from] toml::ser::Error),
    #[error("Json serde error {0:?}")]
    JsonError(#[from] serde_json::Error),
    #[error("Lidar error {0:?}")]
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
    #[error("Text to speech error {0:?}")]
    TtsError(#[from] azure_tts::TtsError),
    #[error("Zenoh error {0:?}")]
    ZenohError(#[from] zenoh::Error),
}

impl HopperError {
    pub fn is_recoverable_driver_error(&self) -> bool {
        match self {
            HopperError::DynamixelSyncWriteError(error) => error.is_recoverable(),
            HopperError::DynamixelDriverError(_id, error) => error.is_recoverable(),
            _ => false,
        }
    }
}
