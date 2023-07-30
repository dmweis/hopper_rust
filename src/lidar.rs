use crate::foxglove;
use crate::{configuration::LidarConfig, error::HopperError};
use prost::Message;
use prost_types::Timestamp;
use rplidar_driver::{utils::sort_scan, RplidarDevice, RposError, ScanOptions, ScanPoint};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tokio::sync::mpsc::{channel, Receiver, Sender};
use tracing::{error, info, log::warn};
use zenoh::prelude::r#async::*;

pub async fn start_lidar_driver(
    zenoh_session: Arc<Session>,
    config: &LidarConfig,
) -> anyhow::Result<()> {
    let (mut scan_receiver, should_lidar_run) =
        start_lidar_driver_internal(&config.serial_port, config.start_state_on)?;

    let subscriber = zenoh_session
        .declare_subscriber(&config.state_topic)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let point_cloud_publisher = zenoh_session
        .declare_publisher(config.point_cloud_topic.to_owned())
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let pose = foxglove::Pose {
        position: Some(foxglove::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }),
        orientation: Some(foxglove::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        }),
    };

    //                      x   y   dis ang quality
    let point_stride = 4 + 4 + 4 + 4 + 1;
    let point_cloud_fields = vec![
        foxglove::PackedElementField {
            name: "x".to_string(),
            offset: 0,
            r#type: foxglove::packed_element_field::NumericType::Float32 as i32,
        },
        foxglove::PackedElementField {
            name: "y".to_string(),
            offset: 4,
            r#type: foxglove::packed_element_field::NumericType::Float32 as i32,
        },
        foxglove::PackedElementField {
            name: "distance".to_string(),
            offset: 8,
            r#type: foxglove::packed_element_field::NumericType::Float32 as i32,
        },
        foxglove::PackedElementField {
            name: "angle".to_string(),
            offset: 12,
            r#type: foxglove::packed_element_field::NumericType::Float32 as i32,
        },
        foxglove::PackedElementField {
            name: "quality".to_string(),
            offset: 16,
            r#type: foxglove::packed_element_field::NumericType::Uint8 as i32,
        },
    ];

    tokio::spawn(async move {
        loop {
            if let Ok(sample) = subscriber.recv_async().await {
                info!("Received message: {}", sample);
                if let Ok(message) = TryInto::<String>::try_into(&sample.value) {
                    info!("Message: {}", message);
                    let lidar_command_on = message.to_lowercase().ends_with("on");
                    if lidar_command_on {
                        info!("Starting scan");
                        should_lidar_run.store(true, Ordering::Relaxed);
                    } else {
                        info!("Stopping scan");
                        should_lidar_run.store(false, Ordering::Relaxed);
                    }
                } else {
                    warn!("Failed to parse message: {:?}", sample.value);
                }
            }
        }
    });

    tokio::spawn(async move {
        let mut scan_counter = 0;
        while let Some(mut scan) = scan_receiver.recv().await {
            let capture_time = SystemTime::now();
            scan_counter += 1;

            if scan_counter % 80 == 0 {
                info!("Scan counter: {}", scan_counter);
            }

            sort_scan(&mut scan).unwrap();

            // point cloud
            let projected_scan = scan
                .iter()
                .filter(|scan| scan.is_valid())
                .map(|scan_point| {
                    let x = scan_point.distance() * (-scan_point.angle()).cos();
                    let y = scan_point.distance() * (-scan_point.angle()).sin();
                    let quality = scan_point.quality;
                    (x, y, scan_point.distance(), scan_point.angle(), quality)
                })
                .collect::<Vec<_>>();

            let point_cloud = foxglove::PointCloud {
                timestamp: Some(system_time_to_proto_time(&capture_time)),
                frame_id: String::from("hopper_lidar"),
                pose: Some(pose.clone()),
                point_stride,
                fields: point_cloud_fields.clone(),
                data: projected_scan
                    .iter()
                    .flat_map(|(x, y, distance, angle, quality)| {
                        // foxglove data is low endian
                        let mut data = x.to_le_bytes().to_vec();
                        data.extend(y.to_le_bytes());
                        data.extend(distance.to_le_bytes());
                        data.extend(angle.to_le_bytes());
                        data.extend(quality.to_le_bytes());
                        data
                    })
                    .collect(),
            };

            point_cloud_publisher
                .put(point_cloud.encode_to_vec())
                .res()
                .await
                .unwrap();
        }
    });

    Ok(())
}

fn system_time_to_proto_time(time: &SystemTime) -> Timestamp {
    let duration = time
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards");
    Timestamp {
        seconds: duration.as_secs() as i64,
        nanos: duration.subsec_nanos() as i32,
    }
}

fn start_lidar_driver_internal(
    port: &str,
    start_with_lidar_running: bool,
) -> anyhow::Result<(Receiver<Vec<ScanPoint>>, Arc<AtomicBool>)> {
    let (scan_sender, scan_receiver) = channel(10);
    let should_lidar_run = Arc::new(AtomicBool::new(start_with_lidar_running));

    thread::spawn({
        let port = port.to_owned();
        let should_lidar_run = Arc::clone(&should_lidar_run);
        move || loop {
            if let Err(err) = lidar_loop(&port, scan_sender.clone(), should_lidar_run.clone()) {
                error!("Lidar loop error: {}", err);
                thread::sleep(Duration::from_secs(1));
            }
        }
    });

    Ok((scan_receiver, should_lidar_run))
}

fn lidar_loop(
    port: &str,
    scan_sender: Sender<Vec<ScanPoint>>,
    should_lidar_run: Arc<AtomicBool>,
) -> anyhow::Result<()> {
    let mut lidar = RplidarDevice::open_port(port)?;
    // start with this flag opposite of desired so that we set the lidar to correct start
    let mut lidar_running = !should_lidar_run.load(Ordering::Relaxed);
    loop {
        match should_lidar_run.load(Ordering::Relaxed) {
            true => {
                if !lidar_running {
                    lidar.start_motor()?;
                    let scan_options = ScanOptions::with_mode(2);
                    let _ = lidar.start_scan_with_options(&scan_options)?;
                    lidar_running = true;
                }
                match lidar.grab_scan() {
                    Ok(scan) => {
                        scan_sender.blocking_send(scan)?;
                    }
                    Err(err) => match err {
                        RposError::OperationTimeout => continue,
                        _ => info!("Error: {:?}", err),
                    },
                }
            }
            false => match lidar_running {
                true => {
                    info!("Stopping lidar");
                    lidar.stop_motor()?;
                    lidar_running = false;
                }
                false => thread::sleep(std::time::Duration::from_millis(500)),
            },
        }
    }
}
