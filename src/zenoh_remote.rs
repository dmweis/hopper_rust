use crate::error::HopperError;
use crate::motion_controller;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use tracing::*;
use zenoh::prelude::r#async::*;

pub async fn simple_zenoh_controller(
    controller: &mut motion_controller::MotionController,
    zenoh_session: Arc<zenoh::Session>,
) -> anyhow::Result<()> {
    let keep_running = Arc::new(AtomicBool::new(true));

    let keep_running_copy = keep_running.clone();
    ctrlc::set_handler(move || {
        info!("Received interrupt");
        keep_running_copy.store(false, Ordering::SeqCst);
    })
    .expect("Failed to set Ctrl-C handler");

    let subscriber = zenoh_session
        .declare_subscriber("/hopper/command/simple/stance")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    while keep_running.load(Ordering::SeqCst) {
        let sample = subscriber.recv_async().await?;

        trace!("got new message");

        let command: String = sample.value.try_into()?;

        if &command.to_lowercase() == "stand" {
            controller.set_body_state(motion_controller::BodyState::Standing);
        } else if &command.to_lowercase() == "ground" {
            controller.set_body_state(motion_controller::BodyState::Grounded);
        } else if &command.to_lowercase() == "happy" {
            controller.start_sequence(motion_controller::DanceMove::HappyDance);
        } else if &command.to_lowercase() == "wave" {
            controller.start_sequence(motion_controller::DanceMove::WaveHi);
        } else {
            error!("Unknown command {}", command);
        }
    }
    info!("Exiting control loop");
    Ok(())
}
