use super::motor_positions::*;
use crate::hopper_config::{BodyConfig, LegConfig};
use anyhow::Result;
use async_trait::async_trait;
use dynamixel_driver::*;
use lazy_static::lazy_static;
use log::*;
use prometheus::{exponential_buckets, register_histogram_vec, HistogramVec};
use std::collections::VecDeque;

lazy_static! {
    static ref BODY_CONTROLLER_TIMER: HistogramVec = register_histogram_vec!(
        "hopper_body_controller_action_duration_seconds",
        "Duration of command send to dynamixel motors in seconds.",
        &["method"],
        exponential_buckets(0.005, 2.0, 10).unwrap()
    )
    .unwrap();
}

#[async_trait]
pub trait BodyController: Send + Sync {
    async fn move_motors_to(&mut self, positions: &BodyMotorPositions) -> Result<()>;
    async fn set_compliance(&mut self, compliance: u8) -> Result<()>;
    async fn set_speed(&mut self, speed: u16) -> Result<()>;
    async fn set_torque(&mut self, torque: bool) -> Result<()>;
    async fn read_motor_positions(&mut self) -> Result<BodyMotorPositions>;
    async fn read_mean_voltage(&mut self) -> Result<f32>;
}

pub struct AsyncBodyController {
    driver: DynamixelDriver,
    body_config: BodyConfig,
    last_read_voltage: usize,
    last_voltages: VecDeque<f32>,
}

impl AsyncBodyController {
    pub fn new(port_name: &str, body_config: BodyConfig) -> Result<Self> {
        let dynamixel_driver = DynamixelDriver::new(port_name)?;
        Ok(Self {
            driver: dynamixel_driver,
            body_config,
            last_read_voltage: 0,
            last_voltages: VecDeque::new(),
        })
    }
}

#[async_trait]
impl BodyController for AsyncBodyController {
    async fn move_motors_to(&mut self, positions: &BodyMotorPositions) -> Result<()> {
        let _timer = BODY_CONTROLLER_TIMER
            .with_label_values(&["move_motors_to"])
            .start_timer();
        let commands = create_commands_for_body(&self.body_config, positions);
        self.driver.sync_write_position_rad(commands).await?;
        Ok(())
    }

    async fn set_speed(&mut self, speed: u16) -> Result<()> {
        let _timer = BODY_CONTROLLER_TIMER
            .with_label_values(&["set_speed"])
            .start_timer();
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| SyncCommand::new(*id, speed as u32))
            .collect::<Vec<_>>();
        self.driver.sync_write_moving_speed(commands).await?;
        Ok(())
    }

    async fn set_compliance(&mut self, compliance: u8) -> Result<()> {
        let _timer = BODY_CONTROLLER_TIMER
            .with_label_values(&["set_compliance"])
            .start_timer();
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| SyncCommand::new(*id, compliance as u32))
            .collect::<Vec<_>>();
        self.driver.sync_write_compliance_both(commands).await?;
        Ok(())
    }

    async fn set_torque(&mut self, torque: bool) -> Result<()> {
        let _timer = BODY_CONTROLLER_TIMER
            .with_label_values(&["set_torque"])
            .start_timer();
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| (*id, torque))
            .collect::<Vec<_>>();
        self.driver.sync_write_torque(commands).await?;
        Ok(())
    }

    async fn read_mean_voltage(&mut self) -> Result<f32> {
        let _timer = BODY_CONTROLLER_TIMER
            .with_label_values(&["read_mean_voltage"])
            .start_timer();
        let ids = self.body_config.get_ids();
        // Load all on first call
        if self.last_voltages.is_empty() {
            info!("Read all voltages");
            for id in &ids {
                let voltage = self.driver.read_voltage(*id).await?;
                self.last_voltages.push_front(voltage);
            }
            self.last_read_voltage = ids.len() - 1;
        }
        self.last_read_voltage += 1;
        if self.last_read_voltage >= ids.len() {
            self.last_read_voltage = 0;
        }
        let id = ids[self.last_read_voltage];
        let voltage = self.driver.read_voltage(id).await?;
        self.last_voltages.push_front(voltage);
        self.last_voltages.pop_back();
        let sum: f32 = self.last_voltages.iter().sum();
        let mean = sum / ids.len() as f32;
        Ok(mean)
    }

    async fn read_motor_positions(&mut self) -> Result<BodyMotorPositions> {
        let _timer = BODY_CONTROLLER_TIMER
            .with_label_values(&["read_motor_positions"])
            .start_timer();
        async fn read_leg_positions(
            driver: &mut DynamixelDriver,
            leg_config: &LegConfig,
        ) -> Result<LegMotorPositions> {
            let coxa = driver.read_position_rad(leg_config.coxa_id).await?;
            let femur = driver.read_position_rad(leg_config.femur_id).await?;
            let tibia = driver.read_position_rad(leg_config.tibia_id).await?;
            Ok(LegMotorPositions::new(coxa, femur, tibia))
        }
        let left_front =
            read_leg_positions(&mut self.driver, &self.body_config.left_front()).await?;
        let left_middle =
            read_leg_positions(&mut self.driver, &self.body_config.left_middle()).await?;
        let left_rear = read_leg_positions(&mut self.driver, &self.body_config.left_rear()).await?;
        let right_front =
            read_leg_positions(&mut self.driver, &self.body_config.right_front()).await?;
        let right_middle =
            read_leg_positions(&mut self.driver, &self.body_config.right_middle()).await?;
        let right_rear =
            read_leg_positions(&mut self.driver, &self.body_config.right_rear()).await?;
        Ok(BodyMotorPositions::new(
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear,
        ))
    }
}
