use crate::body_controller::{ BodyMotorPositions, BodyController };
use log::*;
use std::error::Error;
use std::net::UdpSocket;
use std::sync::{
    mpsc::{channel, Receiver, Sender},
    Arc, Mutex,
};
use std::thread;
use std::time::Duration;
use serde::{Deserialize, Serialize};

pub struct UdpCommandReceiver {
    receiver: Receiver<BodyMotorPositions>,
    thread_handle: Option<thread::JoinHandle<()>>,
    keep_running: Arc<Mutex<bool>>,
}


#[derive(Deserialize)]
enum Command {
    MoveTo(BodyMotorPositions),
    SetSpeed(u16),
    SetCompliance(u8),
    SetTorque(bool),
    ReadPosition,
}

#[derive(Deserialize)]
struct Message {
    command: Command,
}


pub fn udp_command_loop(mut controller: Box<dyn BodyController>) -> Result<(), Box<dyn Error>> {
    let socket = UdpSocket::bind("0.0.0.0:6666")?;
    let mut buffer = [0; 1024];
    loop {
        if let Ok((amt, addr)) = socket.recv_from(&mut buffer) {
            trace!("got new message");
            let received = &mut buffer[..amt];
            let message: Result<Message, _> = serde_json::from_slice(&received);
            if let Ok(message) = message {
                match message.command {
                    Command::MoveTo(position) => {
                        trace!("Moving to pos");
                        controller.move_to_position(position);
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
                        trace!("Setting compliance");
                        controller.set_torque(torque);
                    },
                    Command::ReadPosition => {
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

fn start_receiver_loop(
    tx: Sender<BodyMotorPositions>,
    keep_running: Arc<Mutex<bool>>,
) -> Result<(), Box<dyn Error>> {
    let socket = UdpSocket::bind("0.0.0.0:6666")?;
    socket.set_read_timeout(Some(Duration::from_secs_f32(0.5)))?;
    let mut buffer = [0; 1024];
    loop {
        if let Ok(amt) = socket.recv(&mut buffer) {
            trace!("got new message");
            let received = &mut buffer[..amt];
            let motor_positions: BodyMotorPositions = serde_json::from_slice(&received)?;
            tx.send(motor_positions).unwrap();
        }
        let keep_running = keep_running.lock().unwrap();
        if !*keep_running {
            info!("UDP receiver signaled to close");
            break;
        }
    }
    Ok(())
}

impl UdpCommandReceiver {
    pub fn new() -> UdpCommandReceiver {
        let keep_running = Arc::new(Mutex::new(true));
        let keep_running_copy = Arc::clone(&keep_running);
        let (tx, rx) = channel();
        info!("Starting new UDP receiver");
        let thread_handle = thread::spawn(move || {
            start_receiver_loop(tx, keep_running_copy).unwrap();
        });
        UdpCommandReceiver {
            receiver: rx,
            thread_handle: Some(thread_handle),
            keep_running,
        }
    }

    /// Block until new command is available
    /// won't fail but will panic if the receiver thread panics
    pub fn get_command(&mut self) -> BodyMotorPositions {
        self.receiver.recv().unwrap()
    }
}

impl Drop for UdpCommandReceiver {
    fn drop(&mut self) {
        {
            let mut handle = self.keep_running.lock().unwrap();
            *handle = false;
        }
        match self.thread_handle.take() {
            Some(handle) => {
                if let Err(error) = handle.join() {
                    error!("Failed joining UDP adaptor thread with {:?}", error);
                } else {
                    info!("UDP receiver closed")
                }
            }
            None => error!("Missing join handle for UDP controller thread"),
        }
    }
}
