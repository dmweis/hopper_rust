mod body_controller;
mod hopper_config;
mod ik_controller;
mod udp_adaptor;
mod utilities;

use anyhow::Result;
use clap::{App, Arg};
use log::*;
use std::path::Path;

#[tokio::main]
async fn main() -> Result<()> {
    let matches = App::new("Hopper body controller")
        .version("0.0.1")
        .author("David Weis <dweis7@gmail.com>")
        .about("Controls body of Hopper")
        .arg(
            Arg::with_name("body_config")
                .long("body_config")
                .help("Sets path to body config file (.yaml)\nIf unset uses default value.")
                .required(false)
                .takes_value(true),
        )
        .arg(
            Arg::with_name("mqtt_host")
                .long("mqtt_host")
                .help("MQTT hostname")
                .required(true)
                .takes_value(true),
        )
        .arg(
            Arg::with_name("log_path")
                .long("log_path")
                .help("Path for external log file.\nIf no give will only log to out")
                .required(false)
                .takes_value(true),
        )
        .arg(
            Arg::with_name("dynamixel_port")
                .long("dynamixel_port")
                .help("Serial port name of the dynamixel port")
                .required(true)
                .takes_value(true),
        )
        .arg(
            Arg::with_name("v")
                .short("v")
                .multiple(true)
                .help("Sets the level of verbosity"),
        )
        .get_matches();
    utilities::start_loggers(matches.value_of("log_path"), matches.occurrences_of("v"))?;
    info!("Started main controller");

    let dynamixel_port = matches
        .value_of("dynamixel_port")
        .expect("Dynamixel port has to be provided");

    let hopper_config = match matches.value_of("body_config") {
        Some(path) => {
            let path = Path::new(path);
            hopper_config::HopperConfig::load(path)?
        }
        None => hopper_config::HopperConfig::default(),
    };

    let body_controller =
        body_controller::AsyncBodyController::new(&dynamixel_port, hopper_config.legs.clone())
            .unwrap();

    let ik_controller = ik_controller::IkController::new(Box::new(body_controller), hopper_config);

    udp_adaptor::udp_ik_commander(ik_controller).await.unwrap();
    Ok(())
}
