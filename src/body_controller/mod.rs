mod motor_controller;
pub mod motor_positions;

use crate::hopper_config::BodyConfig;
use crate::mqtt_adaptor::MqttAdaptor;
use log::*;
use looprate::shared_timers::SharedTimer;
use motor_controller::*;
pub use motor_positions::*;
use std::sync::{
    mpsc::{channel, Receiver, Sender},
    Arc, Mutex,
};
use std::thread;
use tokio::runtime::Runtime;

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
    fn move_motors_to(&mut self, positions: BodyMotorPositions);
    fn set_compliance(&mut self, compliance: u8);
    fn set_speed(&mut self, speed: u16);
    fn set_torque(&mut self, torque: bool);
    fn read_motor_positions(&mut self) -> Result<BodyMotorPositions, &str>;
}

pub struct AsyncBodyController {
    join_handle: Option<thread::JoinHandle<anyhow::Result<()>>>,
    command: Arc<Mutex<Option<ImmediateCommand>>>,
    buffered_command_sender: Sender<BufferedCommand>,
    position_receiver: Receiver<Result<BodyMotorPositions, &'static str>>,
}

impl AsyncBodyController {
    pub fn new(
        port: &str,
        body_config: BodyConfig,
        telemetry_sender: MqttAdaptor,
        mut monitor_timer: SharedTimer,
    ) -> Box<Self> {
        let (command_tx, command_rx) = channel();
        let (position_tx, position_rx) = channel();
        let body_config = body_config;
        let command = Arc::new(Mutex::new(None));
        let command_copy = Arc::clone(&command);
        let port_name = port.to_owned();
        let join_handle = thread::spawn(move || -> anyhow::Result<()> {
            let mut rt = Runtime::new()?;
            rt.block_on(async {
                let mut telemetry_rate = looprate::Rate::from_frequency(5.0);

                let mut loop_rate = looprate::Rate::from_frequency(50.0);

                let mut motor_controller = MotorController::new(&port_name, body_config).unwrap();

                loop {
                    let command = command_copy.lock().unwrap().take();
                    monitor_timer.tick();
                    if let Some(command) = command {
                        match command {
                            ImmediateCommand::MoveToPosition(positions) => {
                                if let Err(error) =
                                    motor_controller.move_to_position(positions).await
                                {
                                    warn!("Failed moving motors {}", error);
                                }
                            }
                            ImmediateCommand::Exit => return,
                        }
                    }
                    if telemetry_rate.check() {
                        match motor_controller.read_mean_voltage().await {
                            Ok(mean) => telemetry_sender.send("voltage", &format!("{}", mean)),
                            Err(error) => warn!("Failed to read voltage {}", error),
                        }
                        telemetry_sender.send(
                            "body_controller_rate",
                            &monitor_timer.elapsed_hz().to_string(),
                        )
                    }
                    if let Ok(buffered_command) = command_rx.try_recv() {
                        match buffered_command {
                            BufferedCommand::SetSpeed(speed) => {
                                if let Err(error) = motor_controller.set_speed(speed).await {
                                    warn!("Failed setting speed {}", error);
                                }
                            }
                            BufferedCommand::SetCompliance(compliance) => {
                                if let Err(error) =
                                    motor_controller.set_compliance(compliance).await
                                {
                                    warn!("Failed setting compliance {}", error);
                                }
                            }
                            BufferedCommand::ReadPosition => {
                                match motor_controller.read_positions().await {
                                    Ok(motor_positions) => {
                                        position_tx.send(Ok(motor_positions)).unwrap()
                                    }
                                    Err(error) => {
                                        warn!("Failed reading motor positions {}", error);
                                        position_tx.send(Err("Failed to read position")).unwrap()
                                    }
                                }
                            }
                            BufferedCommand::SetTorque(torque) => {
                                if let Err(error) = motor_controller.set_torque(torque).await {
                                    warn!("Failed setting torque {}", error);
                                }
                            }
                        }
                    }
                    loop_rate.wait()
                }
            });
            Ok(())
        });
        Box::new(AsyncBodyController {
            join_handle: Some(join_handle),
            command,
            buffered_command_sender: command_tx,
            position_receiver: position_rx,
        })
    }
}

impl BodyController for AsyncBodyController {
    fn move_motors_to(&mut self, positions: BodyMotorPositions) {
        let mut command = self
            .command
            .lock()
            .expect("Body controller thread panicked");
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

    fn read_motor_positions(&mut self) -> Result<BodyMotorPositions, &str> {
        self.buffered_command_sender
            .send(BufferedCommand::ReadPosition)
            .unwrap();
        self.position_receiver.recv().unwrap()
    }

    fn set_torque(&mut self, torque: bool) {
        self.buffered_command_sender
            .send(BufferedCommand::SetTorque(torque))
            .unwrap();
    }
}

impl Drop for AsyncBodyController {
    fn drop(&mut self) {
        self.command.lock().unwrap().replace(ImmediateCommand::Exit);
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
