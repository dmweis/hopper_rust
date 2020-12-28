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
pub fn start_loggers(log_file: Option<String>, verbosity_level: u8) -> Result<()> {
    let config = Config::default();
    let filter = match verbosity_level {
        0 => LevelFilter::Info,
        _ => LevelFilter::max(),
    };
    let mut loggers: Vec<Box<dyn SharedLogger>> = vec![];
    if let Some(path) = log_file {
        let log_file = OpenOptions::new().append(true).create(true).open(path)?;
        let file_logger = WriteLogger::new(filter, config, log_file);
        loggers.push(file_logger);
    }
    // loggers.push(TermLogger::new(filter, config, TerminalMode::Mixed));
    loggers.push(TermLogger::new(
        filter,
        Config::default(),
        TerminalMode::Mixed,
    ));
    CombinedLogger::init(loggers)?;
    Ok(())
}
