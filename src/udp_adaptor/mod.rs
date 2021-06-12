use crate::body_controller::{BodyController, BodyMotorPositions};
use crate::ik_controller::leg_positions::*;
use crate::ik_controller::IkControllable;
use crate::motion_controller;
use crate::motion_controller::walking::MoveCommand;
use anyhow::Result;
use log::*;
use nalgebra::Vector2;
use serde::Deserialize;
use std::net::UdpSocket;
use std::str::from_utf8;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

#[derive(Deserialize)]
enum Command {
    MoveMotorsTo(BodyMotorPositions),
    SetSpeed(u16),
    SetCompliance(u8),
    SetTorque(bool),
    ReadMotorPosition,
    MoveLegsTo(LegPositions),
    DisableMotors,
    ReadLegPosition,
}

#[derive(Deserialize)]
struct Message {
    command: Command,
}

#[allow(dead_code)]
pub async fn udp_motor_commander(mut controller: Box<dyn BodyController>) -> Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:6666")?;
    let mut buffer = [0; 1024];
    loop {
        if let Ok((amt, addr)) = socket.recv_from(&mut buffer) {
            trace!("got new message");
            let received = &mut buffer[..amt];
            let message: Result<Message, _> = serde_json::from_slice(&received);
            if let Ok(message) = message {
                match message.command {
                    Command::MoveMotorsTo(position) => {
                        trace!("Moving to pos");
                        controller.move_motors_to(&position).await?;
                    }
                    Command::SetSpeed(speed) => {
                        trace!("Setting speed");
                        controller.set_speed(speed).await?;
                    }
                    Command::SetCompliance(compliance) => {
                        trace!("Setting compliance");
                        controller.set_compliance(compliance).await?;
                    }
                    Command::SetTorque(torque) => {
                        trace!("Setting torque");
                        controller.set_torque(torque).await?;
                    }
                    Command::ReadMotorPosition => {
                        trace!("Reading position");
                        if let Ok(positions) = controller.read_motor_positions().await {
                            let json = serde_json::to_vec(&positions)?;
                            socket.send_to(&json, addr).unwrap();
                        } else {
                            socket.send_to(b"FATAL ERROR", addr).unwrap();
                        }
                    }
                    _ => {
                        error!("Command not supported by body controller");
                        panic!("Command not supported by body controller");
                    }
                }
            } else {
                error!("Got malformed message");
            }
        }
    }
}

pub async fn udp_ik_commander(mut controller: Box<dyn IkControllable>) -> Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:6666")?;
    let mut buffer = [0; 1024];
    loop {
        if let Ok((amt, addr)) = socket.recv_from(&mut buffer) {
            trace!("got new message");
            let received = &mut buffer[..amt];
            let message: Result<Message, _> = serde_json::from_slice(&received);
            if let Ok(message) = message {
                match message.command {
                    Command::MoveLegsTo(position) => {
                        trace!("Moving to pos");
                        if let Err(error) = controller.move_to_positions(&position).await {
                            warn!("Failed writing position {}", &error);
                        }
                    }
                    Command::DisableMotors => {
                        trace!("Disabling motors");
                        controller.disable_motors().await?;
                    }
                    Command::ReadLegPosition => {
                        trace!("Reading position");
                        if let Ok(positions) = controller.read_leg_positions().await {
                            let json = serde_json::to_vec(&positions)?;
                            socket.send_to(&json, addr).unwrap();
                        } else {
                            error!("Failed reading position");
                            socket.send_to(b"{\"value\": \"error\"}", addr).unwrap();
                        }
                    }
                    Command::MoveMotorsTo(position) => {
                        trace!("Moving to pos");
                        controller.move_motors_to(&position).await?;
                    }
                    Command::SetSpeed(speed) => {
                        trace!("Setting speed");
                        controller.set_speed(speed).await?;
                    }
                    Command::SetCompliance(compliance) => {
                        trace!("Setting compliance");
                        controller.set_compliance(compliance).await?;
                    }
                    Command::SetTorque(torque) => {
                        trace!("Setting torque");
                        controller.set_torque(torque).await?;
                    }
                    Command::ReadMotorPosition => {
                        trace!("Reading position");
                        if let Ok(positions) = controller.read_motor_positions().await {
                            let json = serde_json::to_vec(&positions)?;
                            socket.send_to(&json, addr).unwrap();
                        } else {
                            socket.send_to(b"FATAL ERROR", addr).unwrap();
                        }
                    }
                }
            } else {
                error!("Received malformed message {:?}", from_utf8(&received));
            }
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct ControllerData {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub a_down: bool,
    pub b_down: bool,
}

pub async fn udp_controller_handler(
    controller: &mut motion_controller::MotionController,
) -> Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:6666")?;
    let mut buffer = [0; 1024];

    let keep_running = Arc::new(AtomicBool::new(true));

    let keep_running_copy = keep_running.clone();
    ctrlc::set_handler(move || {
        info!("Received interrupt");
        keep_running_copy.store(false, Ordering::SeqCst);
    })
    .expect("Failed to set Ctrl-C handler");

    while keep_running.load(Ordering::SeqCst) {
        if let Ok((amt, _)) = socket.recv_from(&mut buffer) {
            trace!("got new message");
            let received = &mut buffer[..amt];
            let message: Result<ControllerData, _> = serde_json::from_slice(&received);
            if let Ok(message) = message {
                if message.a_down {
                    controller.set_body_state(motion_controller::BodyState::Standing);
                } else if message.b_down {
                    controller.set_body_state(motion_controller::BodyState::Grounded);
                }

                let move_command =
                    MoveCommand::new(Vector2::new(message.x, message.y), message.yaw);
                controller.set_command(move_command);
            } else {
                error!("Received malformed message {:?}", from_utf8(&received));
            }
        }
    }
    info!("Exiting control loop");
    Ok(())
}
