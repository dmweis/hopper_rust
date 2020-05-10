use crate::body_controller::{ BodyMotorPositions, BodyController };
use crate::ik_controller::leg_positions::*;
use crate::ik_controller::IkControlable;
use log::*;
use std::error::Error;
use std::net::UdpSocket;

use serde::Deserialize;


#[derive(Deserialize)]
enum MotorCommand {
    MoveTo(BodyMotorPositions),
    SetSpeed(u16),
    SetCompliance(u8),
    SetTorque(bool),
    ReadPosition,
}

#[derive(Deserialize)]
struct MotorMessage {
    command: MotorCommand,
}

#[allow(dead_code)]
pub fn udp_motor_commander(mut controller: Box<dyn BodyController>) -> Result<(), Box<dyn Error>> {
    let socket = UdpSocket::bind("0.0.0.0:6666")?;
    let mut buffer = [0; 1024];
    loop {
        if let Ok((amt, addr)) = socket.recv_from(&mut buffer) {
            trace!("got new message");
            let received = &mut buffer[..amt];
            let message: Result<MotorMessage, _> = serde_json::from_slice(&received);
            if let Ok(message) = message {
                match message.command {
                    MotorCommand::MoveTo(position) => {
                        trace!("Moving to pos");
                        controller.move_to_position(position);
                    },
                    MotorCommand::SetSpeed(speed) => {
                        trace!("Setting speed");
                        controller.set_speed(speed);
                    },
                    MotorCommand::SetCompliance(compliance) => {
                        trace!("Setting compliance");
                        controller.set_compliance(compliance);
                    },
                    MotorCommand::SetTorque(torque) => {
                        trace!("Setting torque");
                        controller.set_torque(torque);
                    },
                    MotorCommand::ReadPosition => {
                        trace!("Reading position");
                        if let Ok(positions) = controller.read_position() {
                            let json = serde_json::to_vec(&positions)?;
                            socket.send_to(&json, addr).unwrap();
                        } else {
                            socket.send_to("FATAL ERROR".as_bytes(), addr).unwrap();
                        }
                    }
                }
            } else {
                error!("Got malformed message");
            }
        }
        
    }
}


#[derive(Deserialize)]
enum IkCommand {
    MoveTo(LegPositions),
    DisableMotors,
    ReadPosition,
}

#[derive(Deserialize)]
struct IKMessage {
    command: IkCommand,
}

pub(crate) fn udp_ik_commander(mut controller: Box<dyn IkControlable>) -> Result<(), Box<dyn Error>> {
    let socket = UdpSocket::bind("0.0.0.0:6666")?;
    let mut buffer = [0; 1024];
    loop {
        if let Ok((amt, addr)) = socket.recv_from(&mut buffer) {
            trace!("got new message");
            let received = &mut buffer[..amt];
            let message: Result<IKMessage, _> = serde_json::from_slice(&received);
            if let Ok(message) = message {
                match message.command {
                    IkCommand::MoveTo(position) => {
                        trace!("Moving to pos");
                        controller.move_to(position)?;
                    },
                    IkCommand::DisableMotors => {
                        trace!("Disabling motors");
                        controller.disable_motors();
                    },
                    IkCommand::ReadPosition => {
                        trace!("Reading position");
                        if let Ok(positions) = controller.read_positions() {
                            let json = serde_json::to_vec(&positions)?;
                            socket.send_to(&json, addr).unwrap();
                        } else {
                            error!("Failed reading position");
                            socket.send_to("{\"value\": \"error\"}".as_bytes(), addr).unwrap();
                        }
                    }
                }
            } else {
                error!("Got malformed message");
            }
        }
        
    }
}