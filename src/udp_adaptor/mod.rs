use crate::body_controller::{BodyController, BodyMotorPositions};
use crate::ik_controller::leg_positions::*;
use crate::ik_controller::IkControllable;
use crate::motion_controller;
use crate::motion_controller::walking::MoveCommand;
use anyhow::Result;
use log::*;
use nalgebra::{UnitQuaternion, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::net::UdpSocket;
use std::str::from_utf8;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

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
            let message: Result<Message, _> = serde_json::from_slice(received);
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
            let message: Result<Message, _> = serde_json::from_slice(received);
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
                error!("Received malformed message {:?}", from_utf8(received));
            }
        }
    }
}

#[derive(Debug, Deserialize, Serialize, Default)]
pub struct ControllerData {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub translation_x: f32,
    pub translation_y: f32,
    pub translation_z: f32,
    pub body_roll: f32,
    pub body_pitch: f32,
    pub body_yaw: f32,
    pub a_down: bool,
    pub b_down: bool,
    pub x_down: bool,
    pub y_down: bool,
}

impl ControllerData {
    pub fn with_move(x: f32, y: f32, yaw: f32, a_down: bool, b_down: bool) -> Self {
        ControllerData {
            x,
            y,
            yaw,
            a_down,
            b_down,
            ..Default::default()
        }
    }

    pub fn with_translation(
        translation_x: f32,
        translation_y: f32,
        translation_z: f32,
        a_down: bool,
        b_down: bool,
    ) -> Self {
        ControllerData {
            translation_x,
            translation_y,
            translation_z,
            a_down,
            b_down,
            ..Default::default()
        }
    }

    pub fn with_rotation(
        body_roll: f32,
        body_pitch: f32,
        body_yaw: f32,
        a_down: bool,
        b_down: bool,
    ) -> Self {
        ControllerData {
            body_roll,
            body_pitch,
            body_yaw,
            a_down,
            b_down,
            ..Default::default()
        }
    }

    pub fn rotation_as_quaternion(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_euler_angles(self.body_roll, self.body_pitch, self.body_yaw)
    }

    pub fn to_json_bytes(&self) -> Result<Vec<u8>> {
        let json = serde_json::to_string(self)?;
        Ok(json.as_bytes().to_vec())
    }
}

pub async fn udp_controller_handler(
    controller: &mut motion_controller::MotionController,
) -> Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:6666")?;
    socket.set_read_timeout(Some(Duration::from_millis(500)))?;
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
            if amt == 0 {
                continue;
            }
            trace!("got new message");
            let received = &mut buffer[..amt];
            let message: Result<ControllerData, _> = serde_json::from_slice(received);
            if let Ok(message) = message {
                if message.a_down {
                    controller.set_body_state(motion_controller::BodyState::Standing);
                } else if message.b_down {
                    controller.set_body_state(motion_controller::BodyState::Grounded);
                }

                let move_command =
                    MoveCommand::new(Vector2::new(message.x, message.y), message.yaw);
                controller.set_command(move_command);

                let translation = Vector3::new(
                    message.translation_x,
                    message.translation_y,
                    message.translation_z,
                );
                let rotation = message.rotation_as_quaternion();
                controller.set_transformation(translation, rotation);
            } else {
                error!("Received malformed message {:?}", from_utf8(received));
            }
        }
    }
    info!("Exiting control loop");
    Ok(())
}
