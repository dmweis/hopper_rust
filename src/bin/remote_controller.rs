use anyhow::Result;
use clap::Parser;
use gilrs::Gilrs;
use hopper_rust::logging;
use hopper_rust::udp_remote::ControllerData;
use std::net::UdpSocket;
use std::{thread::sleep, time::Duration};
use tracing::*;

/// Visualize Hopper
#[derive(Parser)]
#[command(author, version)]
struct Args {
    /// addr:port of target
    #[arg()]
    target: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    logging::setup_tracing(1);
    info!("Started remote controller");

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

                let z = -gamepad.value(gilrs::Axis::RightStickX);
                let z = if z.abs() > 0.2 { z } else { 0.0 };

                let height = -gamepad.value(gilrs::Axis::RightStickY);
                let height = if height.abs() > 0.2 { height } else { 0.0 };

                let a_down = gamepad.is_pressed(gilrs::Button::South);
                let b_down = gamepad.is_pressed(gilrs::Button::East);
                let x_down = gamepad.is_pressed(gilrs::Button::West);
                let y_down = gamepad.is_pressed(gilrs::Button::North);

                let lb_down = gamepad.is_pressed(gilrs::Button::LeftTrigger);
                let rb_down = gamepad.is_pressed(gilrs::Button::RightTrigger);

                let translation_mode = lb_down;
                let rotation_mode = rb_down;

                let command = if translation_mode {
                    ControllerData::with_translation(
                        0.05 * -x,
                        0.05 * -y,
                        0.04 * height,
                        a_down,
                        b_down,
                        x_down,
                        y_down,
                    )
                } else if rotation_mode {
                    ControllerData::with_rotation(
                        (10.0 * y).to_radians(),
                        -(10.0 * x).to_radians(),
                        -(10.0 * z).to_radians(),
                        a_down,
                        b_down,
                        x_down,
                        y_down,
                    )
                } else {
                    ControllerData::with_move(
                        0.06 * x,
                        0.06 * y,
                        15_f32.to_radians() * z,
                        a_down,
                        b_down,
                        x_down,
                        y_down,
                    )
                };
                trace!("{:?}", command);
                if socket.send(&command.to_json_bytes()?).is_err() {
                    error!("Failed to send datagram");
                }
            }
        }
        sleep(Duration::from_millis(20));
    }
}
