use config::Config;
use serde::Deserialize;
use std::{path::PathBuf, str};
use tracing::*;
use zenoh::config::Config as ZenohConfig;

use crate::error::HopperError;

/// Use default config if no path is provided
pub fn get_configuration(config: &Option<PathBuf>) -> Result<HopperConfig, anyhow::Error> {
    let settings = if let Some(config) = config {
        info!("Using configuration from {:?}", config);
        Config::builder()
            .add_source(config::Environment::with_prefix("APP"))
            .add_source(config::File::with_name(
                config
                    .to_str()
                    .ok_or_else(|| anyhow::anyhow!("Failed to convert path"))?,
            ))
            .build()?
    } else {
        info!("Using dev configuration");
        Config::builder()
            .add_source(config::Environment::with_prefix("APP"))
            .add_source(config::File::with_name("config/settings"))
            .add_source(config::File::with_name("config/dev_settings"))
            .build()?
    };

    Ok(settings.try_deserialize()?)
}

#[derive(Deserialize, Debug, Clone)]
pub struct HopperConfig {
    pub base: BaseConfig,
    pub tts_service_config: TtsServiceConfig,
    pub lidar: LidarConfig,
    pub zenoh: HopperZenohConfig,
    pub camera: CameraConfig,
    pub openai: HopperOpenAiConfig,
}

#[derive(Deserialize, Debug, Clone)]
pub struct BaseConfig {
    pub dynamixel_port: String,
    pub face_port: String,
}

#[derive(Deserialize, Debug, Clone)]
pub struct TtsServiceConfig {
    pub azure_api_key: String,
    pub eleven_labs_api_key: String,
    pub cache_dir_path: Option<String>,
    pub audio_repository_path: Option<String>,
}

/// Named like this because OpenAiConfig is already a type in the openai crate
#[derive(Deserialize, Debug, Clone)]
pub struct HopperOpenAiConfig {
    pub api_key: String,
}

#[derive(Deserialize, Debug, Clone)]
pub struct LidarConfig {
    pub serial_port: String,
    pub state_topic: String,
    pub point_cloud_topic: String,
    pub start_state_on: bool,
}

#[derive(Deserialize, Debug, Clone)]
pub struct HopperZenohConfig {
    pub connect: Vec<zenoh_config::EndPoint>,
    pub listen: Vec<zenoh_config::EndPoint>,
    pub config_path: Option<String>,
}

impl HopperZenohConfig {
    pub fn get_zenoh_config(&self) -> anyhow::Result<ZenohConfig> {
        let mut config = if let Some(conf_file) = &self.config_path {
            ZenohConfig::from_file(conf_file).map_err(HopperError::ZenohError)?
        } else {
            ZenohConfig::default()
        };
        if !self.connect.is_empty() {
            config.connect.endpoints = self.connect.clone();
        }
        if !self.listen.is_empty() {
            config.listen.endpoints = self.listen.clone();
        }
        Ok(config)
    }
}

#[derive(Deserialize, Debug, Clone)]
pub struct CameraConfig {
    pub id: usize,
    pub image_topic: String,
    pub image_width: u32,
    pub image_height: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    static DEFAULT_CONFIG: &str = include_str!("../config/settings.yaml");

    #[test]
    fn test_config() {
        let builder = Config::builder()
            .add_source(config::File::from_str(
                DEFAULT_CONFIG,
                config::FileFormat::Yaml,
            ))
            .build()
            .unwrap();
        builder.try_deserialize::<HopperConfig>().unwrap();
    }
}
