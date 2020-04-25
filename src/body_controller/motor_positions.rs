use crate::hopper_config::{BodyConfig, LegConfig};
use dynamixel_driver::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct LegMotorPositions {
    pub(super) coxa: f32,
    pub(super) femur: f32,
    pub(super) tibia: f32,
}

impl LegMotorPositions {
    pub fn new(coxa: f32, femur: f32, tibia: f32) -> LegMotorPositions {
        LegMotorPositions { coxa, femur, tibia }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct BodyMotorPositions {
    pub(super) left_front: LegMotorPositions,
    pub(super) left_middle: LegMotorPositions,
    pub(super) left_rear: LegMotorPositions,
    pub(super) right_front: LegMotorPositions,
    pub(super) right_middle: LegMotorPositions,
    pub(super) right_rear: LegMotorPositions,
}

impl BodyMotorPositions {
    pub fn new(
        left_front: LegMotorPositions,
        left_middle: LegMotorPositions,
        left_rear: LegMotorPositions,
        right_front: LegMotorPositions,
        right_middle: LegMotorPositions,
        right_rear: LegMotorPositions,
    ) -> BodyMotorPositions {
        BodyMotorPositions {
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear,
        }
    }
}

fn create_commands_for_leg(
    config: &LegConfig,
    positions: &LegMotorPositions,
) -> [SyncCommandFloat; 3] {
    [
        SyncCommandFloat::new(config.coxa_id, positions.coxa),
        SyncCommandFloat::new(config.femur_id, positions.femur),
        SyncCommandFloat::new(config.tibia_id, positions.tibia),
    ]
}

pub fn create_commands_for_body(
    config: &BodyConfig,
    positions: &BodyMotorPositions,
) -> Vec<SyncCommandFloat> {
    let mut commands = Vec::with_capacity(18);
    commands.extend_from_slice(&create_commands_for_leg(
        &config.left_front,
        &positions.left_front,
    ));
    commands.extend_from_slice(&create_commands_for_leg(
        &config.right_front,
        &positions.right_front,
    ));
    commands.extend_from_slice(&create_commands_for_leg(
        &config.left_middle,
        &positions.left_middle,
    ));
    commands.extend_from_slice(&create_commands_for_leg(
        &config.right_middle,
        &positions.right_middle,
    ));
    commands.extend_from_slice(&create_commands_for_leg(
        &config.left_rear,
        &positions.left_rear,
    ));
    commands.extend_from_slice(&create_commands_for_leg(
        &config.right_rear,
        &positions.right_rear,
    ));
    commands
}

#[cfg(test)]
mod tests {
    use super::*;
    use mint::Point3;

    #[test]
    fn creating_commands_for_legs_aligns() {
        let config = LegConfig {
            coxa_id: 1,
            femur_id: 2,
            tibia_id: 3,
            angle_offset: 0.0,
            position: Point3::from([0.0, 0.0, 0.0]),
            femur_correction: 0.0,
            tibia_correction: 0.0,
        };
        let positions = LegMotorPositions {
            coxa: 10.0,
            femur: 20.0,
            tibia: 30.0,
        };
        let commands = create_commands_for_leg(&config, &positions);
        let expected = [
            SyncCommandFloat::new(1, 10.0),
            SyncCommandFloat::new(2, 20.0),
            SyncCommandFloat::new(3, 30.0),
        ];
        assert_eq!(commands, expected);
    }
}
