mod body_controller;
mod hopper_config;
mod ik_controller;
mod udp_adaptor;
mod utilities;

use anyhow::Result;
use clap::Clap;
use log::*;
use std::path::Path;

/// Hopper body controller
/// Controls body of Hopper
#[derive(Clap)]
#[clap(version = "0.0.2", author = "David Weis <dweis7@gmail.com>")]
struct Args {
    /// Sets path to body config file (.yaml)
    /// If unset uses default value.
    #[clap(long)]
    body_config: Option<String>,
    /// Path for external log file.
    /// If no give will only log to out
    #[clap(long)]
    log_path: Option<String>,
    /// Serial port name of the dynamixel port
    #[clap(short, long)]
    dynamixel_port: String,
    /// Sets the level of verbosity
    #[clap(short, parse(from_occurrences))]
    verbose: u8,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    utilities::start_loggers(args.log_path, args.verbose)?;
    info!("Started main controller");

    let hopper_config = match args.body_config {
        Some(path) => {
            let path = Path::new(&path);
            hopper_config::HopperConfig::load(path)?
        }
        None => hopper_config::HopperConfig::default(),
    };

    let body_controller =
        body_controller::AsyncBodyController::new(&args.dynamixel_port, hopper_config.legs.clone())
            .unwrap();

    let ik_controller = ik_controller::IkController::new(Box::new(body_controller), hopper_config);

    udp_adaptor::udp_ik_commander(ik_controller).await.unwrap();
    Ok(())
}
