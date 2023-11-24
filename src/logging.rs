use std::sync::Arc;
use tracing_subscriber::{fmt, prelude::*, EnvFilter};
use zenoh::{prelude::r#async::*, Session as ZenohSession};

use crate::error::HopperError;
use crate::ioc_container::IocContainer;
use crate::zenoh_consts::HOPPER_TRACING_FULL;

pub fn setup_tracing(verbosity_level: u8) {
    let filter = match verbosity_level {
        0 => tracing::level_filters::LevelFilter::INFO,
        1 => tracing::level_filters::LevelFilter::DEBUG,
        2 => tracing::level_filters::LevelFilter::TRACE,
        _ => tracing::level_filters::LevelFilter::TRACE,
    };

    let zenoh_log_writer = start_log_writer();

    let stderr_writer = fmt::Layer::default()
        .with_thread_names(true)
        .with_writer(std::io::stderr);

    let zenoh_writer = fmt::Layer::default()
        .with_thread_names(true)
        .with_thread_ids(true)
        .with_file(true)
        .with_line_number(true)
        .with_writer(move || zenoh_log_writer.clone());

    let subscriber = fmt::Subscriber::builder()
        // subscriber configuration
        .with_env_filter(EnvFilter::from_default_env())
        .with_max_level(filter)
        .finish()
        // add additional writers
        .with(stderr_writer)
        .with(zenoh_writer);

    tracing::subscriber::set_global_default(subscriber).expect("unable to set global subscriber");
}

#[derive(Clone)]
struct LogWriter {
    sender: tokio::sync::mpsc::Sender<Vec<u8>>,
}

impl std::io::Write for LogWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let data = buf.to_vec();
        // ignore errors
        _ = self.sender.try_send(data);
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

fn start_log_writer() -> LogWriter {
    let (sender, receiver) = tokio::sync::mpsc::channel(100);
    let log_writer = LogWriter { sender };

    tokio::spawn(async move { repeat_loop(receiver).await });

    log_writer
}

async fn repeat_loop(mut receiver: tokio::sync::mpsc::Receiver<Vec<u8>>) {
    loop {
        if let Err(err) = publisher_loop(&mut receiver).await {
            tracing::error!("Log publisher loop failed {:?}", err);
        }
    }
}

async fn publisher_loop(receiver: &mut tokio::sync::mpsc::Receiver<Vec<u8>>) -> anyhow::Result<()> {
    if let Some(zenoh_session) = IocContainer::global_instance().get::<Arc<ZenohSession>>() {
        let publisher = zenoh_session
            .declare_publisher(HOPPER_TRACING_FULL)
            .congestion_control(CongestionControl::Drop)
            .priority(Priority::DataLow)
            .res()
            .await
            .map_err(HopperError::ZenohError)?;

        while let Some(data) = receiver.recv().await {
            publisher
                .put(data)
                .res()
                .await
                .map_err(HopperError::ZenohError)?;
        }
    } else {
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
    }

    Ok(())
}
