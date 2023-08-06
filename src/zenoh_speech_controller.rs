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

    let play_sound_command_subscriber = zenoh_session
        .declare_subscriber("hopper/command/speech/play_sound")
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
                    play_sound_msg = play_sound_command_subscriber.recv_async() => {
                        let play_sound_msg = play_sound_msg?;
                        let play_sound_msg: String = play_sound_msg.value.try_into()?;
                        info!("Received speech play sound command {}", play_sound_msg);
                        speech_service.lock().await.play_sound(&play_sound_msg).await?;
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
