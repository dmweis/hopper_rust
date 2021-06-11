use anyhow::Result;
use clap::Clap;
use hopper_rust::utilities;
use log::*;
use serde::{Deserialize, Serialize};
use std::net::UdpSocket;
use std::{thread::sleep, time::Duration};

use gilrs::Gilrs;

/// Visualize Hopper
#[derive(Clap)]
#[clap(version = "0.0.2", author = "David Weis <dweis7@gmail.com>")]
struct Args {
    /// addr:port of target
    #[clap()]
    target: String,
}

#[derive(Debug, Deserialize, Serialize)]
struct ControllerData {
    x: f32,
    y: f32,
    yaw: f32,
    a_down: bool,
    b_down: bool,
}

impl ControllerData {
    fn new(x: f32, y: f32, yaw: f32, a_down: bool, b_down: bool) -> Self {
        ControllerData {
            x,
            y,
            yaw,
            a_down,
            b_down,
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    utilities::start_loggers(None, 1)?;
    info!("Started main visualizer");

    let socket = UdpSocket::bind("0.0.0.0:6666")?;
    socket.connect(args.target)?;

    // gamepad
    let mut gilrs = Gilrs::new().unwrap();
    loop {
        trace!("Read gamepad");
        while gilrs.next_event().is_some() {}
        if let Some((_, gamepad)) = gilrs.gamepads().next() {
            if gamepad.is_connected() {
                let x = gamepad.value(gilrs::Axis::LeftStickY);
                let x = if x.abs() > 0.2 { x } else { 0.0 };
                // y direction needs to be inverted
                let y = -gamepad.value(gilrs::Axis::LeftStickX);
                let y = if y.abs() > 0.2 { y } else { 0.0 };

                let yaw = gamepad.value(gilrs::Axis::RightStickX);
                let yaw = if yaw.abs() > 0.2 { yaw } else { 0.0 };

                let a_down = gamepad.is_pressed(gilrs::Button::South);
                let b_down = gamepad.is_pressed(gilrs::Button::East);

                let move_command = ControllerData::new(
                    0.06 * x,
                    0.06 * y,
                    10_f32.to_radians() * yaw,
                    a_down,
                    b_down,
                );

                trace!("{:?}", move_command);
                let payload = serde_json::to_string(&move_command)?;
                socket.send(payload.as_bytes())?;
            }
        }
        sleep(Duration::from_millis(20));
    }
}
