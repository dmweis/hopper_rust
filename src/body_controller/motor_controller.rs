use super::motor_positions::*;

use crate::{
    error::{HopperError, HopperResult},
    hexapod::{HexapodTypes, ToSyncCommand, TripodLegType},
    hopper_body_config::{BodyConfig, LegConfig},
    utilities::RateTracker,
};
use async_trait::async_trait;
use dynamixel_driver::*;
use std::{collections::VecDeque, time::Duration};
use tracing::*;
use zenoh::publication::Publisher;

#[async_trait]
pub trait BodyController: Send + Sync {
    async fn move_motors_to(&mut self, positions: &BodyMotorPositions) -> HopperResult<()>;
    async fn move_optional_motors_to(
        &mut self,
        positions: &OptionalBodyMotorPositions,
    ) -> HopperResult<()>;
    async fn set_compliance_slope(&mut self, compliance: u8) -> HopperResult<()>;
    async fn set_body_compliance_slope(
        &mut self,
        compliance: HexapodCompliance,
    ) -> HopperResult<()>;
    async fn set_motor_speed(&mut self, speed: u16) -> HopperResult<()>;
    async fn set_body_motor_speed(&mut self, speed: HexapodMotorSpeed) -> HopperResult<()>;
    async fn set_torque(&mut self, torque: bool) -> HopperResult<()>;
    async fn read_motor_positions(&mut self) -> HopperResult<BodyMotorPositions>;
    async fn read_mean_voltage(&mut self) -> HopperResult<f32>;
    async fn scan_motors(&mut self) -> HopperResult<()>;
    async fn clear_serial_io_buffers(&mut self) -> HopperResult<()>;
}

pub type HexapodCompliance = HexapodTypes<TripodLegType<u8>>;

pub type HexapodMotorSpeed = HexapodTypes<TripodLegType<u16>>;

pub struct AsyncBodyController {
    driver: DynamixelDriver,
    body_config: BodyConfig,
    last_read_voltage: usize,
    last_voltages: VecDeque<f32>,
    motor_move_rate_tracker: RateTracker,
}

impl AsyncBodyController {
    pub fn new(
        port_name: &str,
        body_config: BodyConfig,
        motor_move_rate_publisher: Publisher<'static>,
    ) -> HopperResult<Self> {
        let dynamixel_driver =
            DynamixelDriver::new(port_name).map_err(HopperError::DynamixelSyncWriteError)?;
        Ok(Self {
            driver: dynamixel_driver,
            body_config,
            last_read_voltage: 0,
            last_voltages: VecDeque::new(),
            motor_move_rate_tracker: RateTracker::new(
                Duration::from_secs(1),
                motor_move_rate_publisher,
            ),
        })
    }
}

#[async_trait]
impl BodyController for AsyncBodyController {
    async fn move_motors_to(&mut self, positions: &BodyMotorPositions) -> HopperResult<()> {
        let commands = positions.create_command(&self.body_config);
        self.driver
            .sync_write_position_rad(commands)
            .await
            .map_err(HopperError::DynamixelSyncWriteError)?;
        // report rate
        self.motor_move_rate_tracker.tick();
        if let Some(report) = self.motor_move_rate_tracker.report().await? {
            debug!(?report, "motor move rate");
        }
        Ok(())
    }

    async fn move_optional_motors_to(
        &mut self,
        positions: &OptionalBodyMotorPositions,
    ) -> HopperResult<()> {
        let commands = positions.create_command(&self.body_config);
        self.driver
            .sync_write_position_rad(commands)
            .await
            .map_err(HopperError::DynamixelSyncWriteError)?;
        // report rate
        self.motor_move_rate_tracker.tick();
        if let Some(report) = self.motor_move_rate_tracker.report().await? {
            debug!(?report, "motor move rate");
        }
        Ok(())
    }

    async fn set_motor_speed(&mut self, speed: u16) -> HopperResult<()> {
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

    async fn set_body_motor_speed(&mut self, speed: HexapodMotorSpeed) -> HopperResult<()> {
        let commands = speed.cerate_sync_command(&self.body_config);
        self.driver
            .sync_write_compliance_slope_both(commands)
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

    async fn set_body_compliance_slope(
        &mut self,
        compliance: HexapodCompliance,
    ) -> HopperResult<()> {
        let commands = compliance.cerate_sync_command(&self.body_config);
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
        let mut current_read_voltage_id = self.last_read_voltage + 1;
        if current_read_voltage_id >= ids.len() {
            current_read_voltage_id = 0;
        }
        let id = ids[current_read_voltage_id];
        let voltage = self
            .driver
            .read_voltage(id)
            .await
            .map_err(|err| HopperError::DynamixelDriverError(id, err))?;
        self.last_voltages.push_front(voltage);
        self.last_voltages.pop_back();
        let sum: f32 = self.last_voltages.iter().sum();
        let mean = sum / ids.len() as f32;
        // only bump read voltage index if we were successful
        self.last_read_voltage = current_read_voltage_id;
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

    async fn clear_serial_io_buffers(&mut self) -> HopperResult<()> {
        tokio::time::sleep(Duration::from_millis(100)).await;
        self.driver
            .clear_io_buffers()
            .await
            .map_err(HopperError::DynamixelSyncWriteError)?;
        Ok(())
    }
}
