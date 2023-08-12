use std::{
    sync::mpsc,
    time::{Duration, Instant},
};

use serde::Serialize;

pub fn setup_tracing(verbosity_level: u8) {
    let filter = match verbosity_level {
        // 0 => tracing::level_filters::LevelFilter::WARN,
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
}

impl RateTracker {
    pub fn new(report_rate: Duration) -> Self {
        let now = Instant::now();
        Self {
            last_tick: now,
            last_report: now,
            report_rate,
            timer_buffer: Vec::new(),
        }
    }

    pub fn tick(&mut self) {
        let now = Instant::now();
        let elapsed = now - self.last_tick;
        self.last_tick = now;
        self.timer_buffer.push(elapsed);
    }

    pub fn report(&mut self) -> Option<RateReport> {
        if self.timer_buffer.is_empty() {
            return None;
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
            Some(RateReport {
                mean_ns,
                max_ns,
                min_ns,
            })
        } else {
            None
        }
    }
}

#[derive(Debug, Serialize, Clone, Copy)]
pub struct RateReport {
    pub mean_ns: u64,
    pub max_ns: u64,
    pub min_ns: u64,
}
