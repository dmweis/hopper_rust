//! Driver for the arduino based LSM9DS0 IMU
//!
//! The arduino streams newline delimited json messages
//! over serial at ~50 Hz
//!
//! Used to detect when hopper is picked up and held upside down

use std::{
    io::{BufRead, BufReader},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use tokio::sync::mpsc::{channel, Sender};
use tracing::*;
use zenoh::prelude::r#async::*;

use crate::{configuration::ImuConfig, error::HopperError};

const IMU_BAUD_RATE: u32 = 115200;
const SERIAL_READ_TIMEOUT: Duration = Duration::from_secs(1);
const RETRY_DELAY: Duration = Duration::from_secs(10);
/// publish telemetry every Nth reading (~2 Hz at 50 Hz input)
const TELEMETRY_DIVIDER: u32 = 25;
/// low pass filter constant for gravity estimation
const FILTER_ALPHA: f32 = 0.2;
/// readings with lower acceleration norm are ignored (free fall or garbage)
const MIN_ACCEL_NORM: f32 = 1.0;

/// Raw IMU reading from the arduino
///
/// accel is in m/s^2, gyro in deg/s, mag in gauss
#[derive(Deserialize, Serialize, Debug, Clone, Copy)]
pub struct ImuReading {
    pub accel: ImuVector,
    pub gyro: ImuVector,
    pub mag: ImuVector,
    pub temp: f32,
}

#[derive(Deserialize, Serialize, Debug, Clone, Copy)]
pub struct ImuVector {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl From<ImuVector> for Vector3<f32> {
    fn from(value: ImuVector) -> Self {
        Vector3::new(value.x, value.y, value.z)
    }
}

#[derive(Clone, Debug, Default)]
pub struct OrientationStatus {
    upside_down: Arc<AtomicBool>,
}

impl OrientationStatus {
    pub fn is_upside_down(&self) -> bool {
        self.upside_down.load(Ordering::Relaxed)
    }

    fn set_upside_down(&self, upside_down: bool) {
        self.upside_down.store(upside_down, Ordering::Relaxed)
    }
}

/// Detects orientation relative to gravity using accelerometer data
///
/// Low pass filters acceleration and compares the angle between
/// the filtered gravity vector and the configured up axis
/// with hysteresis between the two thresholds
struct TiltDetector {
    up_axis: Vector3<f32>,
    /// tilt in radians past which we are upside down
    upside_down_angle: f32,
    /// tilt in radians under which we are righted again
    righted_angle: f32,
    filtered_gravity: Option<Vector3<f32>>,
    upside_down: bool,
}

impl TiltDetector {
    fn new(config: &ImuConfig) -> Self {
        Self {
            up_axis: Vector3::from(config.up_axis).normalize(),
            upside_down_angle: config.upside_down_angle_degrees.to_radians(),
            righted_angle: config.righted_angle_degrees.to_radians(),
            filtered_gravity: None,
            upside_down: false,
        }
    }

    fn process(&mut self, accel: Vector3<f32>) -> bool {
        let filtered = match self.filtered_gravity {
            Some(previous) => previous.lerp(&accel, FILTER_ALPHA),
            None => accel,
        };
        self.filtered_gravity = Some(filtered);
        if filtered.norm() < MIN_ACCEL_NORM {
            return self.upside_down;
        }
        let tilt = filtered.angle(&self.up_axis);
        if self.upside_down {
            if tilt < self.righted_angle {
                self.upside_down = false;
            }
        } else if tilt > self.upside_down_angle {
            self.upside_down = true;
        }
        self.upside_down
    }

    fn tilt_degrees(&self) -> Option<f32> {
        self.filtered_gravity
            .filter(|gravity| gravity.norm() >= MIN_ACCEL_NORM)
            .map(|gravity| gravity.angle(&self.up_axis).to_degrees())
    }
}

pub async fn start_imu_driver(
    config: &ImuConfig,
    zenoh_session: Arc<Session>,
) -> anyhow::Result<OrientationStatus> {
    let orientation_status = OrientationStatus::default();

    let (reading_sender, mut reading_receiver) = channel::<ImuReading>(10);

    thread::spawn({
        let port_name = config.serial_port.clone();
        move || loop {
            if let Err(error) = imu_reader_loop(&port_name, &reading_sender) {
                error!("IMU reader error: {}", error);
                thread::sleep(RETRY_DELAY);
            }
        }
    });

    let telemetry_publisher = zenoh_session
        .declare_publisher(config.status_topic.clone())
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let mut detector = TiltDetector::new(config);

    tokio::spawn({
        let orientation_status = orientation_status.clone();
        async move {
            let mut reading_counter = 0_u32;
            while let Some(reading) = reading_receiver.recv().await {
                let upside_down = detector.process(reading.accel.into());
                if upside_down != orientation_status.is_upside_down() {
                    if upside_down {
                        warn!("IMU detected that hopper is upside down");
                    } else {
                        info!("IMU detected that hopper is righted");
                    }
                    orientation_status.set_upside_down(upside_down);
                }
                reading_counter = reading_counter.wrapping_add(1);
                if reading_counter.is_multiple_of(TELEMETRY_DIVIDER) {
                    let telemetry = serde_json::json!({
                        "tilt_degrees": detector.tilt_degrees(),
                        "upside_down": upside_down,
                        "reading": reading,
                    });
                    if let Err(error) = telemetry_publisher
                        .put(telemetry.to_string())
                        .res()
                        .await
                    {
                        error!("Failed to publish IMU telemetry: {}", error);
                    }
                }
            }
            warn!("IMU reading channel closed");
        }
    });

    Ok(orientation_status)
}

fn imu_reader_loop(port_name: &str, sender: &Sender<ImuReading>) -> anyhow::Result<()> {
    let port = serialport::new(port_name, IMU_BAUD_RATE)
        .timeout(SERIAL_READ_TIMEOUT)
        .open_native()?;
    info!("Connected to IMU on {}", port_name);
    let mut reader = BufReader::new(port);
    let mut line = String::new();
    loop {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => anyhow::bail!("IMU serial port closed"),
            Ok(_) => match serde_json::from_str::<ImuReading>(line.trim()) {
                Ok(reading) => {
                    if sender.blocking_send(reading).is_err() {
                        anyhow::bail!("IMU reading channel closed");
                    }
                }
                // corrupted lines are expected on connect
                Err(error) => debug!("Failed to parse IMU message {:?}: {}", line, error),
            },
            Err(error) if error.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(error) => return Err(error.into()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GRAVITY: f32 = 9.81;

    fn test_detector() -> TiltDetector {
        TiltDetector {
            up_axis: Vector3::z(),
            upside_down_angle: 125.0_f32.to_radians(),
            righted_angle: 100.0_f32.to_radians(),
            filtered_gravity: None,
            upside_down: false,
        }
    }

    fn feed(detector: &mut TiltDetector, accel: Vector3<f32>, samples: usize) -> bool {
        let mut result = detector.upside_down;
        for _ in 0..samples {
            result = detector.process(accel);
        }
        result
    }

    #[test]
    fn upright_is_not_upside_down() {
        let mut detector = test_detector();
        assert!(!feed(&mut detector, Vector3::new(0.0, 0.0, GRAVITY), 50));
    }

    #[test]
    fn flipped_is_upside_down() {
        let mut detector = test_detector();
        feed(&mut detector, Vector3::new(0.0, 0.0, GRAVITY), 50);
        assert!(feed(&mut detector, Vector3::new(0.0, 0.0, -GRAVITY), 50));
    }

    #[test]
    fn tilted_is_not_upside_down() {
        let mut detector = test_detector();
        // 90 degrees on the side is below the 125 degree threshold
        assert!(!feed(&mut detector, Vector3::new(GRAVITY, 0.0, 0.0), 50));
    }

    #[test]
    fn hysteresis_keeps_state_between_thresholds() {
        let mut detector = test_detector();
        feed(&mut detector, Vector3::new(0.0, 0.0, -GRAVITY), 50);
        assert!(detector.upside_down);
        // 110 degrees tilt is between righted (100) and upside down (125)
        let between = Vector3::new(GRAVITY * 110.0_f32.to_radians().sin(), 0.0, GRAVITY * 110.0_f32.to_radians().cos());
        assert!(feed(&mut detector, between, 50));
        // fully upright clears the state
        assert!(!feed(&mut detector, Vector3::new(0.0, 0.0, GRAVITY), 50));
    }

    #[test]
    fn filter_debounces_single_flipped_reading() {
        let mut detector = test_detector();
        feed(&mut detector, Vector3::new(0.0, 0.0, GRAVITY), 50);
        // single flipped sample should not trip the low pass filter
        assert!(!feed(&mut detector, Vector3::new(0.0, 0.0, -GRAVITY), 1));
        assert!(!feed(&mut detector, Vector3::new(0.0, 0.0, GRAVITY), 50));
    }

    #[test]
    fn parses_arduino_message() {
        let message = r#"{"accel":{"x":0.12,"y":-0.05,"z":9.78},"mag":{"x":0.1,"y":0.2,"z":0.3},"gyro":{"x":1.0,"y":2.0,"z":3.0},"temp":24.5}"#;
        let reading: ImuReading = serde_json::from_str(message).unwrap();
        assert_eq!(reading.accel.z, 9.78);
        assert_eq!(reading.temp, 24.5);
    }
}
