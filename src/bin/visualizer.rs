use anyhow::Result;
use clap::Clap;
use log::*;
use std::io::{self, Read};

use hopper_rust::{motion_controller, utilities};

/// Visualize Hopper
#[derive(Clap)]
#[clap(version = "0.0.2", author = "David Weis <dweis7@gmail.com>")]
struct Args {
    /// Sets path to body config file (.yaml)
    /// If unset uses default value.
    #[clap(long)]
    _body_config: Option<String>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let _args: Args = Args::parse();
    utilities::start_loggers(None, 1)?;
    info!("Started main visualizer");

    // let hopper_config = args
    //     .body_config
    //     .map(|path| hopper_config::HopperConfig::load(Path::new(&path)))
    //     .unwrap_or_else(|| Ok(hopper_config::HopperConfig::default()))?;

    let _visualizer = motion_controller::visualizer::HopperVisualizer::default();
    let mut buffer = String::new();
    io::stdin().read_to_string(&mut buffer)?;
    Ok(())
}
