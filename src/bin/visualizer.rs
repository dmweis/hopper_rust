use anyhow::Result;
use clap::Clap;
use hopper_rust::{hopper_config, motion_controller, utilities};
use log::*;
use motion_controller::{visualizer::GroundType, walking::MoveCommand};
use nalgebra::Vector2;
use std::path::Path;
use std::{thread::sleep, time::Duration};

use gilrs::Gilrs;

/// Visualize Hopper
#[derive(Clap)]
#[clap(version = "0.0.2", author = "David Weis <dweis7@gmail.com>")]
struct Args {
    /// Sets path to body config file (.yaml)
    /// If unset uses default value.
    #[clap(long)]
    body_config: Option<String>,
    /// type of floor to draw in visualizer
    #[clap(short, long, default_value = "ChessBoard")]
    ground: GroundType,
    /// Sets the level of verbosity
    #[clap(short, parse(from_occurrences))]
    verbose: u8,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    utilities::start_loggers(None, args.verbose)?;
    info!("Started main visualizer");

    let _config = args
        .body_config
        .map(|path| hopper_config::HopperConfig::load(Path::new(&path)))
        .unwrap_or_else(|| Ok(hopper_config::HopperConfig::default()))?;

    let visualizer = motion_controller::visualizer::HopperVisualizer::new(args.ground);
    let mut motion_controller = motion_controller::MotionController::new(Box::new(visualizer));

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

                let yaw = gamepad.value(gilrs::Axis::RightStickX);
                let yaw = if yaw.abs() > 0.2 { yaw } else { 0.0 };

                if gamepad.is_pressed(gilrs::Button::South) {
                    info!("You pressed A. That does nothing");
                }

                let move_command =
                    MoveCommand::new(Vector2::new(0.06 * x, 0.06 * y), 10_f32.to_radians() * yaw);
                info!("{:?}", move_command);
                motion_controller.set_command(move_command);
            }
        }
        sleep(Duration::from_millis(20));
    }
}
