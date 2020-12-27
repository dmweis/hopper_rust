use super::motor_positions::*;
use crate::hopper_config::{BodyConfig, LegConfig};
use anyhow::Result;
use dynamixel_driver::*;
use log::*;
use std::collections::VecDeque;

pub struct MotorController {
    driver: DynamixelDriver,
    body_config: BodyConfig,
    last_read_voltage: usize,
    last_voltages: VecDeque<f32>,
}

impl MotorController {
    pub fn new(port_name: &str, body_config: BodyConfig) -> Result<MotorController> {
        let dynamixel_driver = DynamixelDriver::new(port_name)?;
        Ok(MotorController {
            driver: dynamixel_driver,
            body_config: body_config,
            last_read_voltage: 0,
            last_voltages: VecDeque::new(),
        })
    }

    pub async fn move_to_position(&mut self, positions: BodyMotorPositions) -> Result<()> {
        let commands = create_commands_for_body(&self.body_config, &positions);
        self.driver.sync_write_position_rad(commands).await?;
        Ok(())
    }

    pub async fn set_speed(&mut self, speed: u16) -> Result<()> {
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| SyncCommand::new(*id, speed as u32))
            .collect::<Vec<_>>();
        self.driver.sync_write_moving_speed(commands).await?;
        Ok(())
    }

    pub async fn set_compliance(&mut self, compliance: u8) -> Result<()> {
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| SyncCommand::new(*id, compliance as u32))
            .collect::<Vec<_>>();
        self.driver.sync_write_compliance_both(commands).await?;
        Ok(())
    }

    pub async fn set_torque(&mut self, torque: bool) -> Result<()> {
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| (*id, torque))
            .collect::<Vec<_>>();
        self.driver.sync_write_torque(commands).await?;
        Ok(())
    }

    pub async fn read_mean_voltage(&mut self) -> Result<f32> {
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

    pub async fn read_positions(&mut self) -> Result<BodyMotorPositions> {
        async fn read_leg_positions(
            driver: &mut DynamixelDriver,
            leg_config: &LegConfig,
        ) -> Result<LegMotorPositions> {
            let coxa = driver.read_position_rad(leg_config.coxa_id).await?;
            let femur = driver.read_position_rad(leg_config.femur_id).await?;
            let tibia = driver.read_position_rad(leg_config.tibia_id).await?;
            Ok(LegMotorPositions::new(coxa, femur, tibia))
        }
        Ok(BodyMotorPositions::new(
            read_leg_positions(&mut self.driver, &self.body_config.left_front).await?,
            read_leg_positions(&mut self.driver, &self.body_config.left_middle).await?,
            read_leg_positions(&mut self.driver, &self.body_config.left_rear).await?,
            read_leg_positions(&mut self.driver, &self.body_config.right_front).await?,
            read_leg_positions(&mut self.driver, &self.body_config.right_middle).await?,
            read_leg_positions(&mut self.driver, &self.body_config.right_rear).await?,
        ))
    }
}
