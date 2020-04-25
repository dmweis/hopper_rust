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

enum BufferedCommand {
    SetCompliance(u8),
    SetSpeed(u16),
    ReadPosition,
}

enum ImmediateCommand {
    MoveToPosition(BodyMotorPositions),
    Exit,
}

pub struct BodyController {
    join_handle: Option<thread::JoinHandle<()>>,
    command: Arc<Mutex<Option<ImmediateCommand>>>,
    buffered_command_sender: Sender<BufferedCommand>,
    position_receiver: Receiver<BodyMotorPositions>,
}

impl BodyController {
    pub fn new(
        port: &str,
        body_config: BodyConfig,
        telemetry_sender: MqttAdaptor,
    ) -> BodyController {
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
                            motor_controller.move_to(positions).unwrap();
                        }
                        ImmediateCommand::Exit => return,
                    }
                }
                if telemetry_rate.check() {
                    let mean = motor_controller.read_mean_voltage().unwrap();
                    telemetry_sender.send(&format!("{}", mean));
                }
                if let Ok(buffered_command) = command_rx.try_recv() {
                    match buffered_command {
                        BufferedCommand::SetSpeed(speed) => {
                            motor_controller.set_speed(speed).unwrap();
                        },
                        BufferedCommand::SetCompliance(compliance) => {
                            motor_controller.set_compliance(compliance).unwrap();
                        },
                        BufferedCommand::ReadPosition => {
                            let motor_positions = motor_controller.read_positions().unwrap();
                            position_tx.send(motor_positions).unwrap();
                        }
                    }
                }
                loop_rate.wait()
            }
        });
        BodyController {
            join_handle: Some(join_handle),
            command: command,
            buffered_command_sender: command_tx,
            position_receiver: position_rx,
        }
    }

    pub fn move_to_position(&mut self, positions: BodyMotorPositions) {
        let mut command = self.command.lock().expect("Body controller thread panicked");
        command.replace(ImmediateCommand::MoveToPosition(positions));
    }

    pub fn set_compliance(&mut self, compliance: u8) {
        self.buffered_command_sender
            .send(BufferedCommand::SetCompliance(compliance))
            .unwrap();
    }

    pub fn set_speed(&mut self, speed: u16) {
        self.buffered_command_sender
            .send(BufferedCommand::SetSpeed(speed))
            .unwrap();
    }

    pub fn read_position(&mut self) -> BodyMotorPositions {
        self.buffered_command_sender
            .send(BufferedCommand::ReadPosition)
            .unwrap();
        self.position_receiver.recv().unwrap()
    }
}

impl Drop for BodyController {
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

