use std::{
    sync::mpsc,
    time::{Duration, Instant},
};

use serde::Serialize;
use zenoh::prelude::r#async::*;
use zenoh::publication::Publisher;

use crate::error::{HopperError, HopperResult};

pub fn setup_tracing(verbosity_level: u8) {
    let filter = match verbosity_level {
        0 => tracing::level_filters::LevelFilter::INFO,
        1 => tracing::level_filters::LevelFilter::INFO,
        2 => tracing::level_filters::LevelFilter::DEBUG,
        3 => tracing::level_filters::LevelFilter::TRACE,
        _ => tracing::level_filters::LevelFilter::TRACE,
    };

    tracing_subscriber::fmt()
        .with_thread_names(true)
        .with_max_level(filter)
        .init();
}

pub trait MpscChannelHelper<T> {
    fn try_recv_optional(&self) -> std::result::Result<Option<T>, mpsc::TryRecvError>;
}

impl<T> MpscChannelHelper<T> for mpsc::Receiver<T> {
    fn try_recv_optional(&self) -> std::result::Result<Option<T>, mpsc::TryRecvError> {
        match self.try_recv() {
            Ok(value) => Ok(Some(value)),
            Err(error) => match error {
                mpsc::TryRecvError::Empty => Ok(None),
                mpsc::TryRecvError::Disconnected => Err(error),
            },
        }
    }
}

pub struct RateTracker {
    last_tick: Instant,
    last_report: Instant,
    report_rate: Duration,
    timer_buffer: Vec<Duration>,
    publisher: Publisher<'static>,
}

impl RateTracker {
    pub fn new(report_rate: Duration, publisher: Publisher<'static>) -> Self {
        let now = Instant::now();
        Self {
            last_tick: now,
            last_report: now,
            report_rate,
            timer_buffer: Vec::new(),
            publisher,
        }
    }

    pub fn tick(&mut self) {
        let now = Instant::now();
        let elapsed = now - self.last_tick;
        self.last_tick = now;
        self.timer_buffer.push(elapsed);
    }

    pub async fn report(&mut self) -> HopperResult<Option<RateReport>> {
        if self.timer_buffer.is_empty() {
            return Ok(None);
        }
        if self.last_report.elapsed() > self.report_rate {
            let ns: Vec<_> = self
                .timer_buffer
                .iter()
                .map(|d| d.as_nanos() as u64)
                .collect();
            self.timer_buffer.clear();
            let mean_ns = ns.iter().sum::<u64>() / ns.len() as u64;
            let max_ns = *ns.iter().max().unwrap();
            let min_ns = *ns.iter().min().unwrap();
            let fps = ns.len() as f32 / self.last_report.elapsed().as_secs() as f32;
            self.last_report = Instant::now();
            let report = RateReport {
                fps,
                mean_ns,
                max_ns,
                min_ns,
            };
            let json = serde_json::to_string(&report)?;
            self.publisher
                .put(json)
                .res()
                .await
                .map_err(HopperError::ZenohError)?;

            Ok(Some(report))
        } else {
            Ok(None)
        }
    }
}

#[derive(Debug, Serialize, Clone, Copy)]
pub struct RateReport {
    pub fps: f32,
    pub mean_ns: u64,
    pub max_ns: u64,
    pub min_ns: u64,
}
