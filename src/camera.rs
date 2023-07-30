use std::sync::Arc;

use prost::Message;
use prost_types::Timestamp;
use tracing::info;
use v4l::buffer::Type;
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use v4l::video::Capture;
use v4l::Device;
use v4l::FourCC;
use zenoh::prelude::r#async::*;
use zenoh::Session;

use crate::error::HopperError;
use crate::foxglove::CompressedImage;

pub fn scan_camera() -> anyhow::Result<()> {
    let dev = Device::new(0).expect("Failed to open device");

    let caps = dev.query_caps()?;
    info!("Device capabilities:\n{}", caps);

    let controls = dev.query_controls()?;
    info!("Device controls:");
    let mut max_name_len = 0;
    for ctrl in &controls {
        if ctrl.name.len() > max_name_len {
            max_name_len = ctrl.name.len();
        }
    }
    for ctrl in controls {
        info!(
            "{:indent$} : [{}, {}]",
            ctrl.name,
            ctrl.minimum,
            ctrl.maximum,
            indent = max_name_len
        );
    }

    Ok(())
}

pub async fn start_camera(zenoh_session: Arc<Session>) -> anyhow::Result<()> {
    let dev = Device::new(0).expect("Failed to open device");

    let image_publisher = zenoh_session
        .declare_publisher("hopper/camera/image")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let mut fmt = dev.format().expect("Failed to read format");
    fmt.width = 320;
    fmt.height = 240;
    fmt.fourcc = FourCC::new(b"MJPG");
    let fmt = dev.set_format(&fmt).expect("Failed to write format");
    info!("Format in use:\n{}", fmt);

    let mut stream =
        Stream::with_buffers(&dev, Type::VideoCapture, 4).expect("Failed to create buffer stream");

    let mut compressed_image = CompressedImage {
        frame_id: "camera".to_string(),
        format: "jpeg".to_string(),
        data: vec![],
        timestamp: Some(proto_timestamp_now()),
    };

    loop {
        let (buf, meta) = stream.next().unwrap();
        println!(
            "Buffer size: {}, seq: {}, timestamp: {}",
            buf.len(),
            meta.sequence,
            meta.timestamp
        );

        compressed_image.data = buf.to_vec();
        compressed_image.timestamp = Some(proto_timestamp_from_v4l(meta.timestamp));
        image_publisher
            .put(compressed_image.encode_to_vec())
            .res()
            .await
            .map_err(HopperError::ZenohError)?;
    }
}

fn proto_timestamp_from_v4l(ts: v4l::Timestamp) -> Timestamp {
    Timestamp {
        seconds: ts.sec as i64,
        nanos: ts.usec as i32 * 1000,
    }
}

fn proto_timestamp_now() -> Timestamp {
    let now = std::time::SystemTime::now();
    let duration = now.duration_since(std::time::UNIX_EPOCH).unwrap();
    Timestamp {
        seconds: duration.as_secs() as i64,
        nanos: duration.subsec_nanos() as i32,
    }
}
