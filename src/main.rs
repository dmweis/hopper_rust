mod body_controller;
mod hopper_config;
mod mqtt_adaptor;
mod utilities;
mod udp_adaptor;

use clap::{App, Arg};
use std::error::Error;
use std::path::Path;
use looprate::RateTimer;

use log::*;


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
    utilities::start_loggers(matches.value_of("log_path"))?;
    trace!("Started main controller");

    let body_config_path = Path::new(
        matches
            .value_of("body_config")
            .ok_or("body_config must be specified")?,
    );

    let mqtt_host = matches.value_of("mqtt_host").unwrap_or("mqtt.local");
    let dynamixel_port = matches
        .value_of("dynamixel_port")
        .expect("Dynamixel port has to be provided");

    let hopper_config = hopper_config::HopperConfig::load(body_config_path)?;

    let mqtt = mqtt_adaptor::MqttAdaptor::new(&mqtt_host);

    let mut body_controller =
        body_controller::BodyController::new(&dynamixel_port, hopper_config.legs.clone(), mqtt);

    let mut udp_command_receiver = udp_adaptor::UdpCommandReceiver::new();

    let mut rate_timer = RateTimer::new();
    info!("Starting main body loop");
    loop {
        let new_postition = udp_command_receiver.get_command();
        body_controller.move_to_position(new_postition);
        rate_timer.tick();
    }
}
