use anyhow::Result;
use clap::Clap;
use hopper_rust::{hopper_config, motion_controller, utilities};
use log::*;
use motion_controller::{visualizer::GroundType, walking::MoveCommand};
use nalgebra::Vector2;
use std::io;
use std::path::Path;

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
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    utilities::start_loggers(None, 1)?;
    info!("Started main visualizer");

    let _config = args
        .body_config
        .map(|path| hopper_config::HopperConfig::load(Path::new(&path)))
        .unwrap_or_else(|| Ok(hopper_config::HopperConfig::default()))?;

    let visualizer = motion_controller::visualizer::HopperVisualizer::new(args.ground);
    let mut motion_controller = motion_controller::MotionController::new(Box::new(visualizer));
    motion_controller.set_command(MoveCommand::new(
        Vector2::new(0.04, 0.04),
        10_f32.to_radians(),
    ));
    info!("Press enter to exit");
    let mut buffer = String::new();
    io::stdin().read_line(&mut buffer)?;
    Ok(())
}
