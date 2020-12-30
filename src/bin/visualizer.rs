use anyhow::Result;
use clap::Clap;
use hopper_rust::{hopper_config, motion_controller, utilities};
use log::*;
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

    let visualizer = motion_controller::visualizer::HopperVisualizer::default();
    let _motion_controller =
        motion_controller::MotionController::start_as_task(Box::new(visualizer));
    info!("Press enter to exit");
    let mut buffer = String::new();
    io::stdin().read_line(&mut buffer)?;
    Ok(())
}
