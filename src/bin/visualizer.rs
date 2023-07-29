use anyhow::Result;
use clap::Parser;
use gilrs::Gilrs;
use hopper_rust::{hopper_body_config, motion_controller, utilities};
use motion_controller::{visualizer::GroundType, walking::MoveCommand};
use nalgebra::Vector2;
use std::path::Path;
use std::{thread::sleep, time::Duration};
use tracing::*;

/// Visualize Hopper
#[derive(Parser)]
#[command(author, version)]
struct Args {
    /// Sets path to body config file (.yaml)
    /// If unset uses default value.
    #[arg(long)]
    body_config: Option<String>,
    /// type of floor to draw in visualizer
    #[arg(short, long, default_value = "ChessBoard")]
    ground: GroundType,
    /// Sets the level of verbosity
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    utilities::setup_tracing(args.verbose);
    info!("Started main visualizer");

    let _config = args
        .body_config
        .map(|path| hopper_body_config::HopperConfig::load(Path::new(&path)))
        .unwrap_or_else(|| Ok(hopper_body_config::HopperConfig::default()))?;

    let visualizer = motion_controller::visualizer::HopperVisualizer::new(args.ground);
    let mut motion_controller =
        motion_controller::MotionController::new(Box::new(visualizer)).await?;

    // gamepad
    let mut gilrs = Gilrs::new().unwrap();
    loop {
        trace!("Read gamepad");
        while gilrs.next_event().is_some() {}
        if let Some((_, gamepad)) = gilrs.gamepads().next() {
            if gamepad.is_connected() {
                let x = gamepad.value(gilrs::Axis::LeftStickY);
                let x = if x.abs() > 0.2 { x } else { 0.0 };
                // y direction needs to be inverted
                let y = -gamepad.value(gilrs::Axis::LeftStickX);
                let y = if y.abs() > 0.2 { y } else { 0.0 };

                let yaw = -gamepad.value(gilrs::Axis::RightStickX);
                let yaw = if yaw.abs() > 0.2 { yaw } else { 0.0 };

                if gamepad.is_pressed(gilrs::Button::South) {
                    info!("Settings desired state to Standing");
                    motion_controller.set_body_state(motion_controller::BodyState::Standing);
                }
                if gamepad.is_pressed(gilrs::Button::East) {
                    info!("Settings desired state to Grounded");
                    motion_controller.set_body_state(motion_controller::BodyState::Grounded);
                }

                let move_command =
                    MoveCommand::new(Vector2::new(0.06 * x, 0.06 * y), 10_f32.to_radians() * yaw);
                trace!("{:?}", move_command);
                motion_controller.set_command(move_command);
            }
        }
        sleep(Duration::from_millis(20));
    }
}
