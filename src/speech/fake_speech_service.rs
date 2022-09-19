use super::AzureVoiceStyle;
use crate::error::HopperResult;

pub struct SpeechService {}

impl SpeechService {
    pub fn new(
        _azure_subscription_key: String,
        _cache_dir_path: Option<String>,
        _audio_repository_path: Option<String>,
    ) -> HopperResult<SpeechService> {
        Ok(SpeechService {})
    }

    pub async fn play_sound(&mut self, _sound_name: &str) -> HopperResult<()> {
        Ok(())
    }

    pub async fn say_astromech(&mut self, _phrase: &str) -> HopperResult<()> {
        Ok(())
    }

    pub async fn random_astromech_noise(&mut self, _length: u32) -> HopperResult<()> {
        Ok(())
    }

    pub async fn say_azure(&mut self, _text: &str) -> HopperResult<()> {
        Ok(())
    }

    pub async fn say_azure_with_style(
        &mut self,
        _text: &str,
        _style: AzureVoiceStyle,
    ) -> HopperResult<()> {
        Ok(())
    }

    pub fn pause(&self) {}

    pub fn resume(&self) {}

    pub fn stop(&self) {}

    pub fn volume(&self, _volume: f32) {}
}
