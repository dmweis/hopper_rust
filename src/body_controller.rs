use dynamixel_driver::*;
use std::thread;
use std::sync::{
    Mutex,
    Arc,
};
use log::*;
use crate::hopper_config::BodyConfig;
use crate::mqtt_adaptor::MqttAdaptor;
use ratelimit::Builder;

enum BodyControllerCommand {
    MoveToPosition(BodyMotorPositions),
    Exit
}

pub struct LegMotorPositions {
    coxa: f32,
    femur: f32,
    tibia: f32,
}

pub struct BodyMotorPositions {
    left_front: LegMotorPositions,
    left_middle: LegMotorPositions,
    left_rear: LegMotorPositions,
    right_front: LegMotorPositions,
    right_middle: LegMotorPositions,
    right_rear: LegMotorPositions,
}

pub struct BodyController {
    join_handle: Option<thread::JoinHandle<()>>,
    command: Arc<Mutex<Option<BodyControllerCommand>>>
}

impl BodyController {
    pub fn new(port: &str, body_config: BodyConfig, telemetry_sender: MqttAdaptor) -> BodyController {

        let body_config = body_config.clone();
        let command = Arc::new(Mutex::new(None));
        let command_copy = Arc::clone(&command);
        let port = port.to_owned();
        let join_handle = thread::spawn(move || {

            let mut telemetry_rate = Builder::new()
                .capacity(1)
                .quantum(1)
                .frequency(5)
                .build();

            let mut loop_rate = Builder::new()
                .capacity(1)
                .quantum(1)
                .frequency(50)
                .build();

            let mut dynamixel_driver = DynamixelDriver::new(&port).unwrap();

            loop {
                let command = command_copy.lock().unwrap().take();
                if let Some(command) = command {
                    match command {
                        BodyControllerCommand::MoveToPosition(_positions) => {
                            // write position to motors
                        },
                        BodyControllerCommand::Exit => return,
                    }
                }
                if let Ok(()) = telemetry_rate.try_wait() {
                    let mut voltages = vec![];
                    for id in &body_config.get_ids() {
                        let voltage = dynamixel_driver.read_voltage(*id).unwrap();
                        voltages.push(voltage);
                    }
                    let sum: f32 = voltages.iter().sum();
                    let mean = sum / 18.0;
                    telemetry_sender.send(&format!("{}", mean));
                }
                loop_rate.wait()
            }
        });
        BodyController {
            join_handle: Some(join_handle),
            command: command,
        }
    }
}

impl Drop for BodyController {
    fn drop(&mut self) {
        self.command.lock().unwrap().replace(BodyControllerCommand::Exit);
        match self.join_handle.take() {
            Some(handle) => {
                    if let Err(error) = handle.join() {
                        error!("Failed joining body controller thread with {:?}", error);
                    }
                },
            None => error!("Missing join handle for body controller thread"),
        }
    }
}