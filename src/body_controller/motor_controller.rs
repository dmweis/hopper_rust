use super::motor_positions::*;
use crate::{
    error::{HopperError, HopperResult},
    hopper_body_config::{BodyConfig, LegConfig},
};
use async_trait::async_trait;
use dynamixel_driver::*;
use std::collections::VecDeque;
use tracing::*;

#[async_trait]
pub trait BodyController: Send + Sync {
    async fn move_motors_to(&mut self, positions: &BodyMotorPositions) -> HopperResult<()>;
    async fn set_compliance_slope(&mut self, compliance: u8) -> HopperResult<()>;
    async fn set_speed(&mut self, speed: u16) -> HopperResult<()>;
    async fn set_torque(&mut self, torque: bool) -> HopperResult<()>;
    async fn read_motor_positions(&mut self) -> HopperResult<BodyMotorPositions>;
    async fn read_mean_voltage(&mut self) -> HopperResult<f32>;
    async fn scan_motors(&mut self) -> HopperResult<()>;
    async fn flush_and_clear_motors(&mut self) -> HopperResult<()>;
}

pub struct AsyncBodyController {
    driver: DynamixelDriver,
    body_config: BodyConfig,
    last_read_voltage: usize,
    last_voltages: VecDeque<f32>,
}

impl AsyncBodyController {
    pub fn new(port_name: &str, body_config: BodyConfig) -> HopperResult<Self> {
        let dynamixel_driver =
            DynamixelDriver::new(port_name).map_err(HopperError::DynamixelSyncWriteError)?;
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
    async fn move_motors_to(&mut self, positions: &BodyMotorPositions) -> HopperResult<()> {
        let commands = create_commands_for_body(&self.body_config, positions);
        self.driver
            .sync_write_position_rad(commands)
            .await
            .map_err(HopperError::DynamixelSyncWriteError)?;
        Ok(())
    }

    async fn set_speed(&mut self, speed: u16) -> HopperResult<()> {
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| SyncCommand::new(*id, speed as u32))
            .collect::<Vec<_>>();
        self.driver
            .sync_write_moving_speed(commands)
            .await
            .map_err(HopperError::DynamixelSyncWriteError)?;
        Ok(())
    }

    async fn set_compliance_slope(&mut self, compliance: u8) -> HopperResult<()> {
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| SyncCommand::new(*id, compliance as u32))
            .collect::<Vec<_>>();
        self.driver
            .sync_write_compliance_slope_both(commands)
            .await
            .map_err(HopperError::DynamixelSyncWriteError)?;
        Ok(())
    }

    async fn set_torque(&mut self, torque: bool) -> HopperResult<()> {
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| (*id, torque))
            .collect::<Vec<_>>();
        self.driver
            .sync_write_torque(commands)
            .await
            .map_err(HopperError::DynamixelSyncWriteError)?;
        Ok(())
    }

    async fn read_mean_voltage(&mut self) -> HopperResult<f32> {
        let ids = self.body_config.get_ids();
        // Load all on first call
        if self.last_voltages.is_empty() {
            info!("Read all voltages");
            for id in &ids {
                let voltage = self
                    .driver
                    .read_voltage(*id)
                    .await
                    .map_err(|err| HopperError::DynamixelDriverError(*id, err))?;
                self.last_voltages.push_front(voltage);
            }
            self.last_read_voltage = ids.len() - 1;
        }
        self.last_read_voltage += 1;
        if self.last_read_voltage >= ids.len() {
            self.last_read_voltage = 0;
        }
        let id = ids[self.last_read_voltage];
        let voltage = self
            .driver
            .read_voltage(id)
            .await
            .map_err(|err| HopperError::DynamixelDriverError(id, err))?;
        self.last_voltages.push_front(voltage);
        self.last_voltages.pop_back();
        let sum: f32 = self.last_voltages.iter().sum();
        let mean = sum / ids.len() as f32;
        Ok(mean)
    }

    async fn read_motor_positions(&mut self) -> HopperResult<BodyMotorPositions> {
        async fn read_leg_positions(
            driver: &mut DynamixelDriver,
            leg_config: &LegConfig,
        ) -> HopperResult<LegMotorPositions> {
            let coxa = driver
                .read_position_rad(leg_config.coxa_id)
                .await
                .map_err(|err| HopperError::DynamixelDriverError(leg_config.coxa_id, err))?;
            let femur = driver
                .read_position_rad(leg_config.femur_id)
                .await
                .map_err(|err| HopperError::DynamixelDriverError(leg_config.femur_id, err))?;
            let tibia = driver
                .read_position_rad(leg_config.tibia_id)
                .await
                .map_err(|err| HopperError::DynamixelDriverError(leg_config.tibia_id, err))?;
            Ok(LegMotorPositions::new(coxa, femur, tibia))
        }
        let left_front =
            read_leg_positions(&mut self.driver, self.body_config.left_front()).await?;
        let left_middle =
            read_leg_positions(&mut self.driver, self.body_config.left_middle()).await?;
        let left_rear = read_leg_positions(&mut self.driver, self.body_config.left_rear()).await?;
        let right_front =
            read_leg_positions(&mut self.driver, self.body_config.right_front()).await?;
        let right_middle =
            read_leg_positions(&mut self.driver, self.body_config.right_middle()).await?;
        let right_rear =
            read_leg_positions(&mut self.driver, self.body_config.right_rear()).await?;
        Ok(BodyMotorPositions::new(
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear,
        ))
    }

    async fn scan_motors(&mut self) -> HopperResult<()> {
        let ids = self.body_config.get_ids();
        for id in ids {
            match self.driver.ping(id).await {
                Ok(()) => info!("Motor {} works", id),
                Err(err) => error!("Motor {} failing with {:?}", id, err),
            }
        }
        Ok(())
    }

    async fn flush_and_clear_motors(&mut self) -> HopperResult<()> {
        self.driver
            .flush_and_clear()
            .await
            .map_err(HopperError::DynamixelSyncWriteError)?;
        Ok(())
    }
}
