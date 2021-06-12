use anyhow::Result;
use clap::Clap;
use log::*;
use std::{path::Path, time::Duration};

use hopper_rust::{
    body_controller, body_controller::BodyController, hopper_config, ik_controller,
    motion_controller, udp_adaptor, utilities,
};

/// Hopper body controller
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
    /// prometheus port
    #[clap(long, default_value = "8686")]
    prometheus_port: u16,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    utilities::start_loggers(args.log_path, args.verbose)?;
    utilities::start_prometheus_exporter(args.prometheus_port)?;
    info!("Started main controller");

    let hopper_config = args
        .body_config
        .map(|path| hopper_config::HopperConfig::load(Path::new(&path)))
        .unwrap_or_else(|| Ok(hopper_config::HopperConfig::default()))?;

    let body_controller =
        body_controller::AsyncBodyController::new(&args.dynamixel_port, hopper_config.legs.clone())
            .unwrap();

    let mut ik_controller =
        ik_controller::IkController::new(Box::new(body_controller), hopper_config);

    // TODO (David): Move to some settings system
    // also maybe tune...
    ik_controller.set_compliance(64).await?;
    ik_controller.set_speed(1023).await?;

    let mut motion_controller = motion_controller::MotionController::new(ik_controller).await?;

    udp_adaptor::udp_controller_handler(&mut motion_controller)
        .await
        .unwrap();

    motion_controller.set_body_state(motion_controller::BodyState::Grounded);
    tokio::time::sleep(Duration::from_secs_f32(2.0)).await;
    drop(motion_controller);
    Ok(())
}
