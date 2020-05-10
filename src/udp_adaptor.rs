use crate::body_controller::{ BodyMotorPositions, BodyController };
use crate::ik_controller::leg_positions::*;
use crate::ik_controller::IkControlable;
use log::*;
use std::error::Error;
use std::net::UdpSocket;
use std::str::from_utf8;
use serde::Deserialize;


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
pub fn udp_motor_commander(mut controller: Box<dyn BodyController>) -> Result<(), Box<dyn Error>> {
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
                        controller.move_motors_to(position);
                    },
                    Command::SetSpeed(speed) => {
                        trace!("Setting speed");
                        controller.set_speed(speed);
                    },
                    Command::SetCompliance(compliance) => {
                        trace!("Setting compliance");
                        controller.set_compliance(compliance);
                    },
                    Command::SetTorque(torque) => {
                        trace!("Setting torque");
                        controller.set_torque(torque);
                    },
                    Command::ReadMotorPosition => {
                        trace!("Reading position");
                        if let Ok(positions) = controller.read_motor_positions() {
                            let json = serde_json::to_vec(&positions)?;
                            socket.send_to(&json, addr).unwrap();
                        } else {
                            socket.send_to("FATAL ERROR".as_bytes(), addr).unwrap();
                        }
                    },
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


pub(crate) fn udp_ik_commander(mut controller: Box<dyn IkControlable>) -> Result<(), Box<dyn Error>> {
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
                        controller.move_to_positions(position)?;
                    },
                    Command::DisableMotors => {
                        trace!("Disabling motors");
                        controller.disable_motors();
                    },
                    Command::ReadLegPosition => {
                        trace!("Reading position");
                        if let Ok(positions) = controller.read_leg_positions() {
                            let json = serde_json::to_vec(&positions)?;
                            socket.send_to(&json, addr).unwrap();
                        } else {
                            error!("Failed reading position");
                            socket.send_to("{\"value\": \"error\"}".as_bytes(), addr).unwrap();
                        }
                    },
                    Command::MoveMotorsTo(position) => {
                        trace!("Moving to pos");
                        controller.move_motors_to(position);
                    },
                    Command::SetSpeed(speed) => {
                        trace!("Setting speed");
                        controller.set_speed(speed);
                    },
                    Command::SetCompliance(compliance) => {
                        trace!("Setting compliance");
                        controller.set_compliance(compliance);
                    },
                    Command::SetTorque(torque) => {
                        trace!("Setting torque");
                        controller.set_torque(torque);
                    },
                    Command::ReadMotorPosition => {
                        trace!("Reading position");
                        if let Ok(positions) = controller.read_motor_positions() {
                            let json = serde_json::to_vec(&positions)?;
                            socket.send_to(&json, addr).unwrap();
                        } else {
                            socket.send_to("FATAL ERROR".as_bytes(), addr).unwrap();
                        }
                    },
                }
            } else {
                error!("Received malformed message {:?}", from_utf8(&received));
            }
        }
        
    }
}