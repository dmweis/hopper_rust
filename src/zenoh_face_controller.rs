use crate::error::{HopperError, HopperResult};
use hopper_face::FaceController;
use rand::seq::SliceRandom;
use rand::Rng;
use std::sync::Arc;
use tokio::select;
use tracing::*;
use zenoh::prelude::r#async::*;
use zenoh::Session;

pub async fn start_face_controller(
    face_controller: Arc<FaceController>,
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

    let random_face_animation_subscriber = zenoh_session
        .declare_subscriber("hopper/command/face/random")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    tokio::spawn(async move {
        let mut selected_color = hopper_face::driver::PURPLE;
        let mut selected_animation = String::from("larson_scanner");

        loop {
            let res: anyhow::Result<()> = async {
                select! {
                    color = face_color_subscriber.recv_async() => {
                        let color = color?;
                        let color: String = color.value.try_into()?;
                        info!("Received face color command {}", color);
                        match color.to_lowercase().as_str() {
                            "red" => selected_color = hopper_face::driver::RED,
                            "green" => selected_color = hopper_face::driver::GREEN,
                            "blue" => selected_color = hopper_face::driver::BLUE,
                            "yellow" => selected_color = hopper_face::driver::YELLOW,
                            "purple" => selected_color = hopper_face::driver::PURPLE,
                            "off" => selected_color = hopper_face::driver::OFF,
                            _ => error!("Unknown color {}", color),
                        }
                        set_animation(&face_controller, &selected_animation, selected_color)?;
                    }
                    animation = face_animation_subscriber.recv_async() => {
                        let animation = animation?;
                        let animation: String = animation.value.try_into()?;
                        selected_animation = animation;
                        info!("Received face animation command {}", selected_animation);
                        set_animation(&face_controller, &selected_animation, selected_color)?;
                    }
                    _ = random_face_animation_subscriber.recv_async() => {
                        info!("Received random face animation command");
                        random_face(&face_controller)?;
                    }
                }
                Ok(())
            }
            .await;
            if let Err(e) = res {
                error!("Error in face controller: {}", e);
            }
        }
    });
    Ok(())
}

fn set_animation(
    face_controller: &FaceController,
    animation: &str,
    color: hopper_face::driver::RGB,
) -> anyhow::Result<()> {
    match animation.to_lowercase().as_str() {
        "larson_scanner" => face_controller.larson_scanner(color)?,
        "run_animation" => face_controller.run_animation(color)?,
        "off" => face_controller.off()?,
        "cycle_all_colors" => face_controller.cycle_all_colors()?,
        "cycle_bright_colors" => face_controller.cycle_bright_colors()?,
        "cycle_normal_colors" => face_controller.cycle_normal_colors()?,
        "count_down_basic" => face_controller.count_down_basic()?,
        "breathing" => face_controller.breathing(color)?,
        "solid_color" => face_controller.solid_color(color)?,
        _ => error!("Unknown animation {}", animation),
    }
    Ok(())
}

fn random_face(face_controller: &Arc<FaceController>) -> HopperResult<()> {
    let mut rng = rand::thread_rng();
    let random_color = hopper_face::driver::ALL_COLORS.choose(&mut rng).unwrap();

    let choice: u8 = rng.gen_range(0..8);

    match choice {
        0 => face_controller.larson_scanner(*random_color)?,
        1 => face_controller.run_animation(*random_color)?,
        2 => face_controller.cycle_all_colors()?,
        3 => face_controller.cycle_bright_colors()?,
        4 => face_controller.cycle_normal_colors()?,
        5 => face_controller.count_down_basic()?,
        6 => face_controller.breathing(*random_color)?,
        7 => face_controller.solid_color(*random_color)?,
        _ => unreachable!(),
    }

    Ok(())
}
