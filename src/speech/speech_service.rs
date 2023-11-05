use super::audio_cache::AudioCache;
use super::audio_repository::AudioRepository;
use super::AzureVoiceStyle;
use crate::error::{HopperError, HopperResult};
use sha2::{Digest, Sha256};
use std::{fs::File, io::Cursor, thread};
use tokio::sync::mpsc::{channel, Receiver, Sender};
use tracing::*;

use rodio::cpal::traits::{DeviceTrait, HostTrait};

// Used to invalidate old cache
const AZURE_FORMAT_VERSION: u32 = 3;

fn hash_azure_tts(
    text: &str,
    voice: &azure_tts::VoiceSettings,
    format: azure_tts::AudioFormat,
    style: AzureVoiceStyle,
) -> String {
    let mut hasher = Sha256::new();
    hasher.update(text);
    hasher.update(&voice.name);
    hasher.update(&voice.language);
    hasher.update(format.as_string());
    hasher.update([style as u8]);
    hasher.update(AZURE_FORMAT_VERSION.to_be_bytes());
    // Turning it into json to hash is a hack.
    // TODO: hash the type not the json
    hasher.update(serde_json::to_string(&voice.gender).unwrap());
    let hashed = hasher.finalize();
    format!("{}-{:x}", voice.name, hashed)
}

enum AudioPlayerCommand {
    Play(Box<dyn Playable>),
    Pause,
    Resume,
    Stop,
    Volume(f32),
}

/// Select the first audio output device that contains "CARD=Device" in its name
/// This is a hack to select the USB audio device on the raspberry pi
/// Based on https://github.com/RustAudio/rodio/blob/4973f330e07be8480c35f145c9da84dc60e2184c/src/stream.rs#L57
fn select_output_device() -> anyhow::Result<(rodio::OutputStream, rodio::OutputStreamHandle)> {
    let host_ids = rodio::cpal::available_hosts();
    for host_id in host_ids {
        let host = rodio::cpal::host_from_id(host_id).unwrap();
        info!("Found audio host {}", host_id.name());
        let output_devices = host.devices().unwrap();
        for device in output_devices {
            let device_name = device.name().unwrap_or_default();
            if device_name.contains("CARD=Device") {
                let default_stream = rodio::OutputStream::try_from_device(&device);

                return Ok(default_stream.or_else(|original_err| {
                    // default device didn't work, try other ones
                    let mut devices = match rodio::cpal::default_host().output_devices() {
                        Ok(d) => d,
                        Err(_) => return Err(original_err),
                    };

                    devices
                        .find_map(|d| rodio::OutputStream::try_from_device(&d).ok())
                        .ok_or(original_err)
                })?);
            }
        }
    }
    anyhow::bail!("No audio output device found");
}

fn audio_player_loop(receiver: &mut Receiver<AudioPlayerCommand>) -> HopperResult<()> {
    // This is the default output stream, but it doesn't ALWAYS work on the raspberry pi
    // let (_output_stream, output_stream_handle) = rodio::OutputStream::try_default()
    //     .map_err(|_| HopperError::FailedToCreateAudioOutputStream)?;

    let (_output_stream, output_stream_handle) =
        select_output_device().map_err(|_| HopperError::FailedToCreateAudioOutputStream)?;

    let sink = rodio::Sink::try_new(&output_stream_handle)
        .map_err(|_| HopperError::FailedToCreateAudioSink)?;
    while let Some(command) = receiver.blocking_recv() {
        match command {
            AudioPlayerCommand::Play(sound) => {
                sink.append(
                    rodio::Decoder::new(sound).map_err(|_| HopperError::FailedToDecodeAudioFile)?,
                );
            }
            AudioPlayerCommand::Pause => {
                info!("Pausing audio");
                sink.pause()
            }
            AudioPlayerCommand::Resume => {
                info!("Resuming audio");
                sink.play()
            }
            AudioPlayerCommand::Stop => {
                info!("Stopping audio");
                warn!("Ignoring stop because it destroys the sink");
                // sink.stop()
            }
            AudioPlayerCommand::Volume(volume) => {
                info!("Settings volume to {}", volume);
                sink.set_volume(volume)
            }
        }
    }
    warn!("Audio player loop exiting");

    Ok(())
}

fn create_player() -> Sender<AudioPlayerCommand> {
    let (sender, receiver) = channel(100);
    thread::spawn(move || {
        let mut receiver = receiver;
        loop {
            // This may miss on sender being dead. But if sender is dead we have bigger issues
            if let Err(e) = audio_player_loop(&mut receiver) {
                error!("Audio player loop failed with {}", e);
            }
        }
    });
    sender
}

pub struct SpeechService {
    azure_speech_client: azure_tts::VoiceService,
    audio_cache: Option<AudioCache>,
    audio_repository: Option<AudioRepository>,
    azure_voice: azure_tts::VoiceSettings,
    azure_audio_format: azure_tts::AudioFormat,
    audio_sender: Sender<AudioPlayerCommand>,
}

pub trait Playable: std::io::Read + std::io::Seek + Send + Sync {}

impl Playable for Cursor<Vec<u8>> {}
impl Playable for File {}

impl SpeechService {
    pub fn new(
        azure_subscription_key: String,
        cache_dir_path: Option<String>,
        audio_repository_path: Option<String>,
    ) -> HopperResult<SpeechService> {
        let azure_speech_client =
            azure_tts::VoiceService::new(&azure_subscription_key, azure_tts::Region::uksouth);

        let audio_cache = match cache_dir_path {
            Some(path) => Some(AudioCache::new(path)?),
            None => None,
        };

        let audio_sender = create_player();

        let audio_repository = match audio_repository_path {
            Some(path) => Some(AudioRepository::new(path)?),
            None => None,
        };

        Ok(SpeechService {
            azure_speech_client,
            audio_cache,
            audio_repository,
            azure_voice: azure_tts::EnUsVoices::SaraNeural.to_voice_settings(),
            azure_audio_format: azure_tts::AudioFormat::Audio48khz192kbitrateMonoMp3,
            audio_sender,
        })
    }

    async fn play(&mut self, data: Box<dyn Playable>) {
        self.audio_sender
            .send(AudioPlayerCommand::Play(data))
            .await
            .unwrap();
    }

    async fn say_azure_with_voice(
        &mut self,
        text: &str,
        voice: &azure_tts::VoiceSettings,
        style: AzureVoiceStyle,
    ) -> HopperResult<()> {
        info!("Using {:?} style", &style);
        let mut segments = vec![
            azure_tts::VoiceSegment::silence(
                azure_tts::SilenceAttributeType::Sentenceboundary,
                "50ms".to_owned(),
            ),
            azure_tts::VoiceSegment::silence(
                azure_tts::SilenceAttributeType::Tailing,
                "25ms".to_owned(),
            ),
            azure_tts::VoiceSegment::silence(
                azure_tts::SilenceAttributeType::Leading,
                "25ms".to_owned(),
            ),
        ];
        let contents = match style {
            AzureVoiceStyle::Plain => azure_tts::VoiceSegment::plain(text),
            AzureVoiceStyle::Angry => {
                azure_tts::VoiceSegment::with_expression(text, azure_tts::Style::Angry)
            }
            AzureVoiceStyle::Sad => {
                azure_tts::VoiceSegment::with_expression(text, azure_tts::Style::Sad)
            }
            AzureVoiceStyle::Cheerful => {
                azure_tts::VoiceSegment::with_expression(text, azure_tts::Style::Cheerful)
            }
        };
        segments.push(contents);

        let sound: Box<dyn Playable> = if let Some(ref audio_cache) = self.audio_cache {
            let file_key = hash_azure_tts(text, voice, self.azure_audio_format, style);
            if let Some(file) = audio_cache.get(&file_key) {
                info!("Using cached value with key {}", file_key);
                file
            } else {
                info!("Writing new file with key {}", file_key);
                let data = self
                    .azure_speech_client
                    .synthesize_segments(segments, voice, self.azure_audio_format)
                    .await?;
                audio_cache.set(&file_key, data.clone())?;
                Box::new(Cursor::new(data))
            }
        } else {
            let data = self
                .azure_speech_client
                .synthesize_segments(segments, voice, self.azure_audio_format)
                .await?;
            Box::new(Cursor::new(data))
        };
        self.play(sound).await;
        Ok(())
    }

    pub async fn play_sound(&mut self, sound_name: &str) -> HopperResult<()> {
        info!("Playing sound {}", sound_name);
        if let Some(ref audio_repository) = self.audio_repository {
            if let Some(data) = audio_repository.load(sound_name) {
                self.play(data).await;
            } else {
                error!("No sound found with name {}", sound_name);
            }
        } else {
            error!("No audio repository configured");
        }
        Ok(())
    }

    pub async fn play_random_sound(&mut self) -> HopperResult<()> {
        if let Some(ref audio_repository) = self.audio_repository {
            if let Some((sound, path)) = audio_repository.random_file_recursive() {
                info!("Playing random sound {:?}", path);
                self.play(sound).await;
            } else {
                error!("No sounds found");
            }
        } else {
            error!("No audio repository configured");
        }
        Ok(())
    }

    pub async fn say_astromech(&mut self, phrase: &str) -> HopperResult<()> {
        let mut sounds = vec![];
        if let Some(ref audio_repository) = self.audio_repository {
            for letter in phrase.to_ascii_lowercase().chars() {
                if letter.is_ascii_alphabetic() {
                    let lookup = format!("astromech/{}.wav", letter);
                    if let Some(data) = audio_repository.load(&lookup) {
                        sounds.push(data);
                    } else {
                        error!("No sound found with name {}", lookup);
                    }
                }
            }
        } else {
            error!("No audio repository configured");
        }
        for sound in sounds {
            // send manually to prevent dropping
            self.play(sound).await;
        }
        Ok(())
    }

    pub async fn random_astromech_noise(&mut self, length: u32) -> HopperResult<()> {
        let mut sounds = vec![];
        if let Some(ref audio_repository) = self.audio_repository {
            for _ in 0..length {
                if let Some(sound) = audio_repository.random_file_from_dir("astromech") {
                    sounds.push(sound);
                }
            }
        }

        for sound in sounds {
            self.play(sound).await;
        }
        Ok(())
    }

    pub async fn say_azure(&mut self, text: &str) -> HopperResult<()> {
        // This cloning here is lame...
        self.say_azure_with_voice(text, &self.azure_voice.clone(), AzureVoiceStyle::Plain)
            .await
    }

    pub async fn say_azure_with_style(
        &mut self,
        text: &str,
        style: AzureVoiceStyle,
    ) -> HopperResult<()> {
        // This cloning here is lame...
        self.say_azure_with_voice(text, &self.azure_voice.clone(), style)
            .await
    }

    pub async fn pause(&self) {
        self.audio_sender
            .send(AudioPlayerCommand::Pause)
            .await
            .unwrap();
    }

    pub async fn resume(&self) {
        self.audio_sender
            .send(AudioPlayerCommand::Resume)
            .await
            .unwrap();
    }

    pub async fn stop(&self) {
        self.audio_sender
            .send(AudioPlayerCommand::Stop)
            .await
            .unwrap();
    }

    pub async fn volume(&self, volume: f32) {
        self.audio_sender
            .send(AudioPlayerCommand::Volume(volume))
            .await
            .unwrap();
    }
}
