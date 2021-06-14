use anyhow::Result;
use log::*;
use prometheus::{Encoder, TextEncoder};
use simplelog::*;
use std::{
    fs::OpenOptions,
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::mpsc,
};
use warp::Filter;

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
    let config = ConfigBuilder::new()
        .add_filter_allow_str("hopper_rust")
        .build();
    let filter = match verbosity_level {
        0 => LevelFilter::Warn,
        1 => LevelFilter::Info,
        2 => LevelFilter::Debug,
        3 => LevelFilter::Trace,
        _ => LevelFilter::max(),
    };
    let mut loggers: Vec<Box<dyn SharedLogger>> = vec![];
    if let Some(path) = log_file {
        let log_file = OpenOptions::new().append(true).create(true).open(path)?;
        let file_logger = WriteLogger::new(filter, config.clone(), log_file);
        loggers.push(file_logger);
    }
    loggers.push(TermLogger::new(
        filter,
        config,
        TerminalMode::Mixed,
        ColorChoice::Auto,
    ));
    CombinedLogger::init(loggers)?;
    info!("Logging level set to {}", filter);
    Ok(())
}

// example histogram
// lazy_static! {
//     static ref HISTOGRAM_TEST: HistogramVec = register_histogram_vec!(
//         "example_histogram_test",
//         "Some speedy latencies",
//         &["handler"]
//     )
//     .unwrap();
// }

pub fn start_prometheus_exporter(port: u16) -> Result<()> {
    let address = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0)), port);
    let prometheus_endpoint = warp::path("metrics").map(|| -> Box<dyn warp::reply::Reply> {
        let metric_families = prometheus::gather();
        let mut buffer = vec![];
        let encoder = TextEncoder::new();
        encoder
            .encode(&metric_families, &mut buffer)
            .expect("Failed to encode prometheus metrics");

        Box::new(warp::reply::with_header(
            buffer,
            "Content-Type",
            encoder.format_type(),
        ))
    });
    info!("Starting prometheus exporter");
    tokio::task::spawn(async move {
        warp::serve(prometheus_endpoint).run(address).await;
    });
    Ok(())
}

pub trait MpscChannelHelper<T> {
    fn try_recv_optional(&mut self) -> std::result::Result<Option<T>, mpsc::TryRecvError>;
}

impl<T> MpscChannelHelper<T> for mpsc::Receiver<T> {
    fn try_recv_optional(&mut self) -> std::result::Result<Option<T>, mpsc::TryRecvError> {
        match self.try_recv() {
            Ok(value) => Ok(Some(value)),
            Err(error) => match error {
                mpsc::TryRecvError::Empty => Ok(None),
                mpsc::TryRecvError::Disconnected => Err(error),
            },
        }
    }
}
