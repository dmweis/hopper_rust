use crate::body_controller::motor_positions::LegMotorPositions;
use crate::hopper_config::{ LegConfig, HopperConfig };
use nalgebra::{Point3, Vector3};
use std::error::Error;

pub(crate) struct LegPositions {
    pub left_front: Point3<f32>,
    pub left_middle: Point3<f32>,
    pub left_rear: Point3<f32>,
    pub right_front: Point3<f32>,
    pub right_middle: Point3<f32>,
    pub right_rear: Point3<f32>,
}

impl LegPositions {
    fn new(
        left_front: Point3<f32>,
        left_middle: Point3<f32>,
        left_rear: Point3<f32>,
        right_front: Point3<f32>,
        right_middle: Point3<f32>,
        right_rear: Point3<f32>,
    ) -> LegPositions {
        LegPositions {
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear,
        }
    }
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
        Err(format!("sad math for {}", target_angle))?
    }
    let femur_angle = angle_by_femur + ground_target_angle;
    let corrected_femur = (leg_config.femur_correction + body_config.femur_offset + femur_angle).abs();
    let corrected_tibia = (leg_config.tibia_correction + body_config.tibia_offset + angle_by_tibia).abs();
    let corrected_coxa = 150_f32.to_radians() + target_angle;
    Ok(LegMotorPositions::new(corrected_coxa, corrected_femur, corrected_tibia))
}

fn get_alpha_angle(a: &f32, b: &f32, c: &f32) -> f32 {
    let upper = b.powi(2) + c.powi(2) - a.powi(2);
    let bottom = 2.0 * b * c;
    let divident: f32 = (-1_f32).max((1_f32).min(upper / bottom));
    divident.acos()
}

pub(crate) struct OptionalLegPositions {
    pub left_front: Option<Point3<f32>>,
    pub left_middle: Option<Point3<f32>>,
    pub left_rear: Option<Point3<f32>>,
    pub right_front: Option<Point3<f32>>,
    pub right_middle: Option<Point3<f32>>,
    pub right_rear: Option<Point3<f32>>,
}


#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::*;

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
}
