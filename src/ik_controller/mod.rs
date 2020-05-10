pub mod leg_positions;

use leg_positions::*;
use crate::body_controller::{ motor_positions::{LegMotorPositions, BodyMotorPositions}, BodyController };
use crate::hopper_config::{ LegConfig, HopperConfig };
use nalgebra::{Point3, Vector3};
use std::error::Error;
use log::*;


pub(crate) trait IkControlable : BodyController {
    fn move_to_positions(&mut self, positions: LegPositions) -> Result<(), Box<dyn Error>>;
    fn read_leg_positions(&mut self) -> Result<LegPositions, Box<dyn Error>>;
    fn disable_motors(&mut self);
}

pub(crate) struct IkController {
    body_controller: Box<dyn BodyController>,
    body_configuration: HopperConfig,
}

impl IkController {
    pub(crate) fn new(
        body_controller: Box<dyn BodyController>,
        body_configuration: HopperConfig
    ) -> Box<dyn IkControlable> {
        Box::new(IkController {
            body_controller,
            body_configuration
        })
    }
}

impl BodyController for IkController {
    fn move_motors_to(&mut self, positions: BodyMotorPositions) {
        self.body_controller.move_motors_to(positions);
    }

    fn set_compliance(&mut self, compliance: u8) {
        self.body_controller.set_compliance(compliance);
    }

    fn set_speed(&mut self, speed: u16) {
        self.body_controller.set_speed(speed);
    }

    fn set_torque(&mut self, torque: bool) {
        self.body_controller.set_torque(torque);
    }

    fn read_motor_positions(&mut self) -> Result<BodyMotorPositions, &str> {
        self.body_controller.read_motor_positions()
    }
}

impl IkControlable for IkController {
    fn move_to_positions(&mut self, positions: LegPositions) -> Result<(), Box<dyn Error>> {
        let motor_positions = calculate_ik(&positions, &self.body_configuration)?;
        self.body_controller.move_motors_to(motor_positions);
        Ok(())
    }
    
    fn read_leg_positions(&mut self) -> Result<LegPositions, Box<dyn Error>> {
        let motor_positions = self.body_controller.read_motor_positions()?;
        let leg_positions = calculate_fk(&motor_positions, &self.body_configuration);
        Ok(leg_positions)
    }

    fn disable_motors(&mut self) {
        self.body_controller.set_torque(false);
    }
}

fn calculate_ik(
    positions: &LegPositions,
    body_config: &HopperConfig
) -> Result<BodyMotorPositions, Box<dyn Error>> {
    let left_front = calculate_ik_for_leg(&positions.left_front, &body_config, &body_config.legs.left_front)?;
    let right_front = calculate_ik_for_leg(&positions.right_front, &body_config, &body_config.legs.right_front)?;
    let left_middle = calculate_ik_for_leg(&positions.left_middle, &body_config, &body_config.legs.left_middle)?;
    let right_middle = calculate_ik_for_leg(&positions.right_middle, &body_config, &body_config.legs.right_middle)?;
    let left_rear = calculate_ik_for_leg(&positions.left_rear, &body_config, &body_config.legs.left_rear)?;
    let right_rear = calculate_ik_for_leg(&positions.right_rear, &body_config, &body_config.legs.right_rear)?;
    Ok(BodyMotorPositions::new(
        left_front,
        left_middle,
        left_rear,
        right_front,
        right_middle,
        right_rear
    ))
}

fn calculate_fk(
    motor_positions: &BodyMotorPositions,
    body_config: &HopperConfig
) -> LegPositions {
    let left_front = calculate_fk_for_leg(&motor_positions.left_front, &body_config, &body_config.legs.left_front);
    let right_front = calculate_fk_for_leg(&motor_positions.right_front, &body_config, &body_config.legs.right_front);
    let left_middle = calculate_fk_for_leg(&motor_positions.left_middle, &body_config, &body_config.legs.left_middle);
    let right_middle = calculate_fk_for_leg(&motor_positions.right_middle, &body_config, &body_config.legs.right_middle);
    let left_rear = calculate_fk_for_leg(&motor_positions.left_rear, &body_config, &body_config.legs.left_rear);
    let right_rear = calculate_fk_for_leg(&motor_positions.right_rear, &body_config, &body_config.legs.right_rear);
    LegPositions::new(
        left_front,
        left_middle,
        left_rear,
        right_front,
        right_middle,
        right_rear
    )
}

fn calculate_ik_for_leg(
    target: &Point3<f32>,
    body_config: &HopperConfig,
    leg_config: &LegConfig,
) -> Result<LegMotorPositions, Box<dyn Error>> {
    let coxa_position = leg_config.position;
    let relative_vector: Vector3<f32> = target - coxa_position;
    let target_angle = relative_vector.y.atan2(relative_vector.x) + leg_config.angle_offset;
    let horizontal_distance = (
        relative_vector.x.powi(2) + relative_vector.y.powi(2)
    ).sqrt() - body_config.coxa_length;
    let distance = (
        horizontal_distance.powi(2) + relative_vector.z.powi(2)
    ).sqrt();
    // use sss triangle solution to calculate angles
    // use law of cosinus to get angles in two corners
    let angle_by_tibia = get_alpha_angle(
        &distance,
        &body_config.femur_length,
        &body_config.tibia_length,
    );
    let angle_by_femur = get_alpha_angle(
        &body_config.tibia_length,
        &body_config.femur_length,
        &distance,
    );
    // we have angles of the SSS triangle. now we need angle for the servos
    let ground_target_angle = horizontal_distance.atan2(-relative_vector.z);
    if target_angle >= 90_f32.to_radians() || target_angle <= (-90_f32).to_radians() {
        warn!("Failed IK for leg: {} target: {}", leg_config.position, &target);
        Err(format!("sad math for {}", target_angle))?
    }
    let femur_angle = angle_by_femur + ground_target_angle;
    let corrected_femur = (leg_config.femur_correction + body_config.femur_offset + femur_angle).abs();
    let corrected_tibia = (leg_config.tibia_correction + body_config.tibia_offset + angle_by_tibia).abs();
    let corrected_coxa = 150_f32.to_radians() + target_angle;
    Ok(LegMotorPositions::new(corrected_coxa, corrected_femur, corrected_tibia))
}

fn calculate_fk_for_leg(
    motor_positions: &LegMotorPositions,
    body_config: &HopperConfig,
    leg_config: &LegConfig,) -> Point3<f32> {
        let femur_angle = (
            motor_positions.femur() -
            (
                leg_config.femur_correction +
                body_config.femur_offset
            ).abs()
        ).abs();
        let tibia_angle = (
            motor_positions.tibia() -
            (
                leg_config.tibia_correction +
                body_config.tibia_offset
            ).abs()
        ).abs();
        let coxa_angle =
            motor_positions.coxa() -
            150_f32.to_radians() -
            leg_config.angle_offset;
        let base_x = coxa_angle.cos();
        let base_y = coxa_angle.sin();
        let coxa_vector = Vector3::new(base_x, base_y, 0.0) * body_config.coxa_length;
        let femur_x = (femur_angle - 90_f32.to_radians()).sin() * body_config.femur_length;
        let femur_y = (femur_angle - 90_f32.to_radians()).cos() * body_config.femur_length;
        let femur_vector = Vector3::new(base_x * femur_y, base_y * femur_y, femur_x);
        // to calculate tibia we need angle between tibia and a vertical line
        // we get this by calculating the angles formed by a horizontal line from femur
        // femur and part of tibia by knowing that the sum of angles is 180
        // than we just remove this from teh tibia angle and done
        let angle_tibia_vector = tibia_angle - (90_f32.to_radians() - (femur_angle - 90_f32.to_radians()));
        let tibia_x = angle_tibia_vector.sin() * body_config.tibia_length;
        let tibia_y = angle_tibia_vector.cos() * body_config.tibia_length;
        let tibia_vector = Vector3::new(base_x * tibia_x, base_y * tibia_x, -tibia_y);
        let coxa_position = leg_config.position;
        coxa_position + coxa_vector + femur_vector + tibia_vector
}

fn get_alpha_angle(a: &f32, b: &f32, c: &f32) -> f32 {
    let upper = b.powi(2) + c.powi(2) - a.powi(2);
    let bottom = 2.0 * b * c;
    let divident: f32 = (-1_f32).max((1_f32).min(upper / bottom));
    divident.acos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::*;
    use nalgebra::distance;

    #[test]
    fn get_angle_equilateral_triangle() {
        let angle = get_alpha_angle(&1.0, &1.0, &1.0);
        assert_approx_eq!(60_f32.to_radians(), angle);
    }

    #[test]
    fn get_angle_by_a_right_angled() {
        let b = 2_f32;
        let c = 2_f32;
        let a = (b.powi(2) + c.powi(2)).sqrt();
        let angle = get_alpha_angle(&a, &b, &c);
        assert_approx_eq!(90_f32.to_radians(), angle);
    }

    #[test]
    fn basic_ik_left_front() {
        let hopper_config = HopperConfig::default();
        let target = Point3::new(0.18, 0.15, -0.09);
        let motor_positions = calculate_ik_for_leg(
            &target,
            &hopper_config,
            &hopper_config.legs.left_front
        ).unwrap();
        assert_approx_eq!(motor_positions.coxa().to_degrees(), 113.28124);
        assert_approx_eq!(motor_positions.femur().to_degrees(), 112.15929);
        assert_approx_eq!(motor_positions.tibia().to_degrees(), 196.29994);
    }

    #[test]
    fn basic_ik_right_front() {
        let hopper_config = HopperConfig::default();
        let target = Point3::new(0.18, -0.15, -0.09);
        let motor_positions = calculate_ik_for_leg(
            &target,
            &hopper_config,
            &hopper_config.legs.right_front
        ).unwrap();
        assert_approx_eq!(motor_positions.coxa().to_degrees(), 186.71875);
        assert_approx_eq!(motor_positions.femur().to_degrees(), 188.0706);
        assert_approx_eq!(motor_positions.tibia().to_degrees(), 103.929955);
    }

    #[test]
    fn full_ik_resting_pose_magic_number() {
        // This is a magic number test for hoppers standard resting position
        // if it fails it's a good indicator something went wrong
        // but not a good indicator of what went wrong
        let hopper_config = HopperConfig::default();
        let pose = LegPositions::new(
            Point3::new(0.18, 0.15, -0.09),
            Point3::new(0.0, 0.22, -0.09),
            Point3::new(-0.18, 0.15, -0.09),
            Point3::new(0.18, -0.15, -0.09),
            Point3::new(0.0, -0.22, -0.09),
            Point3::new(-0.18, -0.15, -0.09),
        );
        let motor_positions = calculate_ik(&pose, &hopper_config).unwrap();
        let expected_motor_positions = BodyMotorPositions::new(
            LegMotorPositions::new(1.9771307, 1.9575489, 3.4260802),
            LegMotorPositions::new(2.6187901, 1.9678993, 3.3529518),
            LegMotorPositions::new(3.2604494, 1.9575489, 3.4260802),
            LegMotorPositions::new(3.258857, 3.2824512, 1.8139199),
            LegMotorPositions::new(2.6171975, 3.2721004, 1.8870485),
            LegMotorPositions::new(1.9755381, 3.2824512, 1.8139199),
        );
        assert_eq!(motor_positions, expected_motor_positions);
    }

    #[test]
    fn test_fk_against_ik_left_front(){
        let hopper_config = HopperConfig::default();
        let target = Point3::new(0.18, 0.15, -0.09);
        let motor_positions = calculate_ik_for_leg(
            &target,
            &hopper_config,
            &hopper_config.legs.left_front
        ).unwrap();
        let fk_calculated = calculate_fk_for_leg(
            &motor_positions, 
            &hopper_config,
            &hopper_config.legs.left_front
        );
        let result_distance = distance(&target, &fk_calculated);
        assert!(result_distance < 0.0000001);
    }

    #[test]
    fn test_fk_against_ik_right_front(){
        let hopper_config = HopperConfig::default();
        let target = Point3::new(0.18, -0.15, -0.09);
        let motor_positions = calculate_ik_for_leg(
            &target,
            &hopper_config,
            &hopper_config.legs.right_front
        ).unwrap();
        let fk_calculated = calculate_fk_for_leg(
            &motor_positions, 
            &hopper_config,
            &hopper_config.legs.right_front
        );
        let result_distance = distance(&target, &fk_calculated);
        assert!(result_distance < 0.0000001);
    }

    #[test]
    fn test_full_fk_against_ik() {
        let hopper_config = HopperConfig::default();
        let origin = LegPositions::new(
            Point3::new(0.18, 0.15, -0.09),
            Point3::new(0.0, 0.22, -0.09),
            Point3::new(-0.18, 0.15, -0.09),
            Point3::new(0.18, -0.15, -0.09),
            Point3::new(0.0, -0.22, -0.09),
            Point3::new(-0.18, -0.15, -0.09),
        );
        let motor_positions = calculate_ik(&origin, &hopper_config).unwrap();
        let result = calculate_fk(&motor_positions, &hopper_config);
        assert!(distance(&origin.left_front, &result.left_front) < 0.0000001);
        assert!(distance(&origin.left_middle, &result.left_middle) < 0.0000001);
        assert!(distance(&origin.left_rear, &result.left_rear) < 0.0000001);
        assert!(distance(&origin.right_front, &result.right_front) < 0.0000001);
        assert!(distance(&origin.right_middle, &result.right_middle) < 0.0000001);
        assert!(distance(&origin.right_rear, &result.right_rear) < 0.0000001);
    }
}
