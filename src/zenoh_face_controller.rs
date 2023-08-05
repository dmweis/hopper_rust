use crate::error::HopperError;
use hopper_face::FaceController;
use std::sync::Arc;
use tokio::select;
use tracing::*;
use zenoh::prelude::r#async::*;
use zenoh::Session;

pub async fn start_face_controller(
    face_controller: FaceController,
    zenoh_session: Arc<Session>,
) -> anyhow::Result<()> {
    let face_color_subscriber = zenoh_session
        .declare_subscriber("hopper/command/face/color")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let face_animation_subscriber = zenoh_session
        .declare_subscriber("hopper/command/face/animation")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let mut selected_color = hopper_face::driver::OFF;

    loop {
        select! {
            color = face_color_subscriber.recv_async() => {
                let color = color?;
                let color: String = color.value.try_into()?;
                match color.to_lowercase().as_str() {
                    "red" => selected_color = hopper_face::driver::RED,
                    "green" => selected_color = hopper_face::driver::GREEN,
                    "blue" => selected_color = hopper_face::driver::BLUE,
                    "yellow" => selected_color = hopper_face::driver::YELLOW,
                    "purple" => selected_color = hopper_face::driver::PURPLE,
                    "off" => selected_color = hopper_face::driver::OFF,
                    _ => error!("Unknown color {}", color),
                }
            }
            animation = face_animation_subscriber.recv_async() => {
                let animation = animation?;
                let animation: String = animation.value.try_into()?;
                match animation.to_lowercase().as_str() {
                    "larson_scanner" => face_controller.larson_scanner(selected_color)?,
                    _ => error!("Unknown animation {}", animation),
                }
            }
        }
    }
}
