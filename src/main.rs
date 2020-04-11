mod hopper_config;

use std::error::Error;
use clap::{Arg, App};
use std::path::Path;

fn main() -> Result<(), Box<dyn Error>> {
    let matches = App::new("Hopper body controller")
                          .version("0.0.1")
                          .author("David Weis <dweis7@gmail.com>")
                          .about("Controls body of Hopper")
                          .arg(Arg::with_name("body_config")
                               .long("body_config")
                               .help("Sets path to body config file (.yaml)")
                               .required(true)
                               .takes_value(true))
                          .get_matches();
    let body_config_path = Path::new(matches
        .value_of("body_config")
        .ok_or("body_config must be specified")?);

    let config = hopper_config::HopperConfig::load(body_config_path)?;
    println!("{:?}", config);
    Ok(())
}
