mod hopper_config;
mod mqtt_adaptor;
mod body_controller;

use clap::{App, Arg};
use std::error::Error;
use std::path::Path;

use log::*;
use simplelog::*;
use std::fs::OpenOptions;

use std::io::{self, Read};

fn main() -> Result<(), Box<dyn Error>> {
    let matches = App::new("Hopper body controller")
        .version("0.0.1")
        .author("David Weis <dweis7@gmail.com>")
        .about("Controls body of Hopper")
        .arg(
            Arg::with_name("body_config")
                .long("body_config")
                .help("Sets path to body config file (.yaml)")
                .required(true)
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
                .required(false)
                .takes_value(true),
        )
        .get_matches();
    
    //
    if let Some(path) = matches.value_of("log_path") {
        let log_file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(path)
            .expect("Failed to open log file");
        CombinedLogger::init(vec![
            TermLogger::new(LevelFilter::Info, Config::default(), TerminalMode::Mixed)
                .expect("Failed to initialize logger"),
            WriteLogger::new(LevelFilter::Info, Config::default(), log_file),
        ])
        .expect("Failed to initialize logger");
    } else {
        CombinedLogger::init(vec![TermLogger::new(
            LevelFilter::Info,
            Config::default(),
            TerminalMode::Mixed,
        )
        .expect("Failed to initialize logger")])
        .expect("Failed to initialize logger");
    }

    let body_config_path = Path::new(
        matches
            .value_of("body_config")
            .ok_or("body_config must be specified")?,
    );

    let mqtt_host = matches.value_of("mqtt_host").unwrap_or("mqtt.local");
    let dynamixel_port = matches.value_of("dynamixel_port").unwrap();

    let hopper_config = hopper_config::HopperConfig::load(body_config_path)?;

    let mqtt = mqtt_adaptor::MqttAdaptor::new(&mqtt_host);

    let _body_controller = body_controller::BodyController::new(&dynamixel_port, hopper_config.legs.clone(), mqtt);

    let mut buffer = String::new();
    println!("Press Enter to exit");
    io::stdin().read_line(&mut buffer)?;
    Ok(())
}
