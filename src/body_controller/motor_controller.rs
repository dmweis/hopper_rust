
use dynamixel_driver::*;
use std::error::Error;
use crate::hopper_config::{ BodyConfig, LegConfig };
use super::motor_positions::*;


pub struct MotorController {
    driver: DynamixelDriver,
    body_config: BodyConfig,
}

impl MotorController {
    pub fn new(port_name: &str, body_config: BodyConfig) -> Result<MotorController, Box<dyn Error>> {
        let dynamixel_driver = DynamixelDriver::new(port_name)?;
        Ok(MotorController {
            driver: dynamixel_driver,
            body_config: body_config,
        })
    }

    pub fn move_to_position(&mut self, positions: BodyMotorPositions) -> Result<(), Box<dyn Error>> {
        let commands = create_commands_for_body(
            &self.body_config,
            &positions,
        );
        self.driver.sync_write_position_rad(commands)?;
        Ok(())
    }

    pub fn set_speed(&mut self, speed: u16) -> Result<(), Box<dyn Error>> {
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| SyncCommand::new(*id, speed as u32))
            .collect::<Vec<_>>();
        self.driver.sync_write_moving_speed(commands)?;
        Ok(())
    }

    pub fn set_compliance(&mut self, compliance: u8) -> Result<(), Box<dyn Error>> {
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| SyncCommand::new(*id, compliance as u32))
            .collect::<Vec<_>>();
        self.driver.sync_write_compliance_both(commands)?;
        Ok(())
    }

    pub fn set_torque(&mut self, torque: bool) -> Result<(), Box<dyn Error>> {
        let commands = self
            .body_config
            .get_ids()
            .iter()
            .map(|id| (*id, torque))
            .collect::<Vec<_>>();
        self.driver.sync_write_torque(commands)?;
        Ok(())
    }

    pub fn read_mean_voltage(&mut self) -> Result<f32, Box<dyn Error>> {
        let mut voltages = Vec::with_capacity(18);
        for id in &self.body_config.get_ids() {
            let voltage = self.driver.read_voltage(*id)?;
            voltages.push(voltage);
        }
        let sum: f32 = voltages.iter().sum();
        let mean = sum / 18.0;
        Ok(mean)
    }

    pub fn read_positions(&mut self) -> Result<BodyMotorPositions, Box<dyn Error>> {
        fn read_leg_positions(
            driver: &mut DynamixelDriver,
            leg_config: &LegConfig
        ) -> Result<LegMotorPositions, Box<dyn Error>> {
            let coxa = driver.read_position_rad(leg_config.coxa_id)?;
            let femur = driver.read_position_rad(leg_config.femur_id)?;
            let tibia = driver.read_position_rad(leg_config.tibia_id)?;
            Ok(LegMotorPositions::new(coxa, femur, tibia))
        }
        Ok(BodyMotorPositions::new(
            read_leg_positions(&mut self.driver, &self.body_config.left_front)?,
            read_leg_positions(&mut self.driver, &self.body_config.left_middle)?,
            read_leg_positions(&mut self.driver, &self.body_config.left_rear)?,
            read_leg_positions(&mut self.driver, &self.body_config.right_front)?,
            read_leg_positions(&mut self.driver, &self.body_config.right_middle)?,
            read_leg_positions(&mut self.driver, &self.body_config.right_rear)?,
        ))
    }
}