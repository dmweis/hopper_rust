pub mod motor_positions;
mod motor_controller;

pub use motor_positions::*;
use motor_controller::*;
use crate::hopper_config::BodyConfig;
use crate::mqtt_adaptor::MqttAdaptor;
use log::*;
use looprate;
use std::sync::{Arc, Mutex, mpsc::{channel, Sender, Receiver}};
use std::thread;
use std::error::Error;


enum BufferedCommand {
    SetCompliance(u8),
    SetSpeed(u16),
    SetTorque(bool),
    ReadPosition,
}

enum ImmediateCommand {
    MoveToPosition(BodyMotorPositions),
    Exit,
}

pub trait BodyController {
    fn move_to_position(&mut self, positions: BodyMotorPositions);
    fn set_compliance(&mut self, compliance: u8);
    fn set_speed(&mut self, speed: u16);
    fn set_torque(&mut self, torque: bool);
    fn read_position(&mut self) -> Result<BodyMotorPositions, Box<dyn Error>>;
}

pub struct AsyncBodyController {
    join_handle: Option<thread::JoinHandle<()>>,
    command: Arc<Mutex<Option<ImmediateCommand>>>,
    buffered_command_sender: Sender<BufferedCommand>,
    position_receiver: Receiver<Option<BodyMotorPositions>>,
}

impl AsyncBodyController {
    pub fn new(
        port: &str,
        body_config: BodyConfig,
        telemetry_sender: MqttAdaptor,
    ) -> Box<dyn BodyController> {
        let (command_tx, command_rx) = channel();
        let (position_tx, position_rx) = channel();
        let body_config = body_config.clone();
        let command = Arc::new(Mutex::new(None));
        let command_copy = Arc::clone(&command);
        let port_name = port.to_owned();
        let join_handle = thread::spawn(move || {
            let mut telemetry_rate = looprate::Rate::from_frequency(5.0);

            let mut loop_rate = looprate::Rate::from_frequency(100.0);

            let mut motor_controller = MotorController::new(&port_name, body_config).unwrap();

            loop {
                let command = command_copy.lock().unwrap().take();
                if let Some(command) = command {
                    match command {
                        ImmediateCommand::MoveToPosition(positions) => {
                            if let Err(error) = motor_controller.move_to_position(positions) {
                                warn!("Failed moving motors {}", error);
                            }
                        }
                        ImmediateCommand::Exit => return,
                    }
                }
                if telemetry_rate.check() {
                    match motor_controller.read_mean_voltage() {
                        Ok(mean) => telemetry_sender.send(&format!("{}", mean)),
                        Err(error) => warn!("Failed to read voltage {}", error),
                    }
                }
                if let Ok(buffered_command) = command_rx.try_recv() {
                    match buffered_command {
                        BufferedCommand::SetSpeed(speed) => {
                            if let Err(error) = motor_controller.set_speed(speed) {
                                warn!("failed setting speed {}", error);
                            }
                        },
                        BufferedCommand::SetCompliance(compliance) => {
                            if let Err(error) = motor_controller.set_compliance(compliance) {
                                warn!("Failed setting compliance {}", error);
                            }
                        },
                        BufferedCommand::ReadPosition => {
                            match motor_controller.read_positions() {
                                Ok(motor_positions) => position_tx.send(Some(motor_positions)).unwrap(),
                                Err(error) => {
                                    warn!("Failed reading motor positions {}", error);
                                    position_tx.send(None).unwrap()
                                }
                            }
                        },
                        BufferedCommand::SetTorque(torque) => {
                            if let Err(error) = motor_controller.set_torque(torque) {
                                warn!("Failed setting torque {}", error);
                            }
                        }
                    }
                }
                loop_rate.wait()
            }
        });
        Box::new(AsyncBodyController {
            join_handle: Some(join_handle),
            command: command,
            buffered_command_sender: command_tx,
            position_receiver: position_rx,
        })
    }
}

impl BodyController for AsyncBodyController {
    fn move_to_position(&mut self, positions: BodyMotorPositions) {
        let mut command = self.command.lock().expect("Body controller thread panicked");
        command.replace(ImmediateCommand::MoveToPosition(positions));
    }

    fn set_compliance(&mut self, compliance: u8) {
        self.buffered_command_sender
            .send(BufferedCommand::SetCompliance(compliance))
            .unwrap();
    }

    fn set_speed(&mut self, speed: u16) {
        self.buffered_command_sender
            .send(BufferedCommand::SetSpeed(speed))
            .unwrap();
    }

    fn read_position(&mut self) -> Result<BodyMotorPositions, Box<dyn Error>> {
        self.buffered_command_sender
            .send(BufferedCommand::ReadPosition)
            .unwrap();
        self.position_receiver.recv().unwrap().ok_or(Err("Positions couldn't be read")?)
    }

    fn set_torque(&mut self, torque: bool) {
        self.buffered_command_sender
            .send(BufferedCommand::SetTorque(torque))
            .unwrap();
    }
}

impl Drop for AsyncBodyController {
    fn drop(&mut self) {
        self.command
            .lock()
            .unwrap()
            .replace(ImmediateCommand::Exit);
        match self.join_handle.take() {
            Some(handle) => {
                if let Err(error) = handle.join() {
                    error!("Failed joining body controller thread with {:?}", error);
                } else {
                    info!("Body controller exited");
                }
            }
            None => error!("Missing join handle for body controller thread"),
        }
    }
}
