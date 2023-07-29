use std::sync::mpsc;

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
