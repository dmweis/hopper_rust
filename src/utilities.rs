use log::*;
use simplelog::*;
use std::error::Error;
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
pub fn start_loggers(log_file: Option<&str>) -> Result<(), Box<dyn Error>> {
    let config = ConfigBuilder::new().add_filter_allow_str("hopper_rust::*").build();
    let mut loggers: Vec<Box<dyn SharedLogger>> = vec![];
    if let Some(path) = log_file {
        let log_file = OpenOptions::new().append(true).create(true).open(path)?;
        let file_logger = WriteLogger::new(LevelFilter::Info, config.clone(), log_file);
        loggers.push(file_logger);
    }
    match TermLogger::new(LevelFilter::Trace, config.clone(), TerminalMode::Mixed) {
        Some(logger) => loggers.push(logger as Box<dyn SharedLogger>),
        None => {
            loggers.push(SimpleLogger::new(LevelFilter::Trace, config.clone()));
            warn!("Failed to create Term logger, creating simple logger instead");
        },
    }
    CombinedLogger::init(loggers)?;
    Ok(())
}
