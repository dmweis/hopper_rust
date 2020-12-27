use anyhow::Result;
use log::*;
use simplelog::*;
use std::fs::OpenOptions;

/// Initializes loggers
///
/// By default tries to initialize colored logger.
/// If this fails (systemd services can't) will default to simple logger
/// If log_file is provided can fail on creating it
///
/// Log level is Trace for terminal logger and Warn for file
///
/// # Arguments
///
/// * `log_file` - Path to the optional file to log into
pub fn start_loggers(log_file: Option<&str>, verbosity_level: u64) -> Result<()> {
    let config = ConfigBuilder::new()
        .add_filter_allow_str("hopper_rust")
        .add_filter_allow_str("dynamixel_driver")
        .build();
    let filter = match verbosity_level {
        0 => LevelFilter::Info,
        _ => LevelFilter::max(),
    };
    let mut loggers: Vec<Box<dyn SharedLogger>> = vec![];
    if let Some(path) = log_file {
        let log_file = OpenOptions::new().append(true).create(true).open(path)?;
        let file_logger = WriteLogger::new(filter, config.clone(), log_file);
        loggers.push(file_logger);
    }
    loggers.push(TermLogger::new(filter, config, TerminalMode::Mixed));
    CombinedLogger::init(loggers)?;
    Ok(())
}
