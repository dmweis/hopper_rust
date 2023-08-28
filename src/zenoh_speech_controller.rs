use crate::error::HopperError;
use crate::speech::SpeechService;
use std::sync::Arc;
use tokio::{select, sync::Mutex};
use tracing::*;
use zenoh::prelude::r#async::*;
use zenoh::Session;

pub async fn start_speech_controller(
    speech_service: Arc<Mutex<SpeechService>>,
    zenoh_session: Arc<Session>,
) -> anyhow::Result<()> {
    let say_command_subscriber = zenoh_session
        .declare_subscriber("hopper/command/speech/say")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let say_astromech_command_subscriber = zenoh_session
        .declare_subscriber("hopper/command/speech/astromech")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let random_astromech_command = zenoh_session
        .declare_subscriber("hopper/command/speech/astromech/random")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let play_sound_command_subscriber = zenoh_session
        .declare_subscriber("hopper/command/speech/play_sound")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let random_sound_subscriber = zenoh_session
        .declare_subscriber("hopper/command/speech/play_sound/random")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    tokio::spawn(async move {
        loop {
            let res: anyhow::Result<()> = async {
                select! {
                    say_msg = say_command_subscriber.recv_async() => {
                        let say_msg = say_msg?;
                        let say_msg: String = say_msg.value.try_into()?;
                        info!("Received speech say command {}", say_msg);
                        speech_service.lock().await.say_azure(&say_msg).await?;
                    }
                    say_astromech_msg = say_astromech_command_subscriber.recv_async() => {
                        let say_astromech_msg = say_astromech_msg?;
                        let say_astromech_msg: String = say_astromech_msg.value.try_into()?;
                        info!("Received speech say astromech command {}", say_astromech_msg);
                        speech_service.lock().await.say_astromech(&say_astromech_msg).await?;
                    }
                    random_astromech_msg = random_astromech_command.recv_async() => {
                        let random_astromech_msg = random_astromech_msg?;
                        let count = if let Ok(random_astromech_msg) = <zenoh::prelude::Value as TryInto<String>>::try_into(random_astromech_msg.value) {
                            random_astromech_msg.parse::<u32>().unwrap_or(20)
                        } else {
                            20
                        };
                        info!("Received play random astromech command");
                        speech_service.lock().await.random_astromech_noise(count).await?;
                    }
                    play_sound_msg = play_sound_command_subscriber.recv_async() => {
                        let play_sound_msg = play_sound_msg?;
                        let play_sound_msg: String = play_sound_msg.value.try_into()?;
                        info!("Received speech play sound command {}", play_sound_msg);
                        speech_service.lock().await.play_sound(&play_sound_msg).await?;
                    }
                    _random_sound_msg = random_sound_subscriber.recv_async() => {
                        info!("Received play random sound command");
                        speech_service.lock().await.play_random_sound().await?;
                    }
                }
                Ok(())
            }
            .await;
            if let Err(e) = res {
                error!("Error in speech controller: {}", e);
            }
        }
    });
    Ok(())
}
