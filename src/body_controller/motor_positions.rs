use crate::{
    hexapod::{HexapodTypes, TripodLegType},
    hopper_body_config::{BodyConfig, LegConfig},
};
use dynamixel_driver::*;

pub type LegMotorPositions = TripodLegType<f32>;
pub type BodyMotorPositions = HexapodTypes<LegMotorPositions>;

impl LegMotorPositions {
    pub fn create_command(&self, leg_config: &LegConfig) -> [SyncCommandFloat; 3] {
        let pairs = self.pair_with_id(leg_config);
        [
            SyncCommandFloat::new(pairs[0].0, pairs[0].1),
            SyncCommandFloat::new(pairs[1].0, pairs[1].1),
            SyncCommandFloat::new(pairs[2].0, pairs[2].1),
        ]
    }
}

impl BodyMotorPositions {
    pub fn create_command(&self, body_config: &BodyConfig) -> Vec<SyncCommandFloat> {
        let mut commands = Vec::with_capacity(18);
        commands.extend_from_slice(&self.left_front().create_command(body_config.left_front()));
        commands.extend_from_slice(&self.left_middle().create_command(body_config.left_middle()));
        commands.extend_from_slice(&self.left_rear().create_command(body_config.left_rear()));
        commands.extend_from_slice(&self.right_front().create_command(body_config.right_front()));
        commands.extend_from_slice(
            &self
                .right_middle()
                .create_command(body_config.right_middle()),
        );
        commands.extend_from_slice(&self.right_rear().create_command(body_config.right_rear()));
        commands
    }
}

pub type OptionalLegMotorPositions = TripodLegType<Option<f32>>;
pub type OptionalBodyMotorPositions = HexapodTypes<OptionalLegMotorPositions>;

impl Default for OptionalLegMotorPositions {
    fn default() -> Self {
        Self::new(None, None, None)
    }
}

impl Default for OptionalBodyMotorPositions {
    fn default() -> Self {
        Self::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
        )
    }
}

impl OptionalLegMotorPositions {
    pub fn create_command(&self, leg_config: &LegConfig) -> Vec<SyncCommandFloat> {
        let pairs = self.pair_with_id(leg_config);
        pairs
            .into_iter()
            .filter_map(|(id, position)| {
                position.map(|position| SyncCommandFloat::new(id, position))
            })
            .collect()
    }

    pub fn is_none(&self) -> bool {
        self.coxa().is_none() && self.femur().is_none() && self.tibia().is_none()
    }

    pub fn merge_with_none_optional(&self, leg_positions: &LegMotorPositions) -> LegMotorPositions {
        LegMotorPositions::new(
            self.coxa().unwrap_or(leg_positions.coxa()),
            self.femur().unwrap_or(leg_positions.femur()),
            self.tibia().unwrap_or(leg_positions.tibia()),
        )
    }
}

impl OptionalBodyMotorPositions {
    pub fn create_command(&self, body_config: &BodyConfig) -> Vec<SyncCommandFloat> {
        let mut commands = Vec::with_capacity(18);
        commands.extend_from_slice(&self.left_front().create_command(body_config.left_front()));
        commands.extend_from_slice(&self.left_middle().create_command(body_config.left_middle()));
        commands.extend_from_slice(&self.left_rear().create_command(body_config.left_rear()));
        commands.extend_from_slice(&self.right_front().create_command(body_config.right_front()));
        commands.extend_from_slice(
            &self
                .right_middle()
                .create_command(body_config.right_middle()),
        );
        commands.extend_from_slice(&self.right_rear().create_command(body_config.right_rear()));
        commands
    }

    pub fn is_none(&self) -> bool {
        self.left_front().is_none()
            && self.left_middle().is_none()
            && self.left_rear().is_none()
            && self.right_front().is_none()
            && self.right_middle().is_none()
            && self.right_rear().is_none()
    }

    pub fn merge_with_none_optional(
        &self,
        body_positions: &BodyMotorPositions,
    ) -> BodyMotorPositions {
        BodyMotorPositions::new(
            self.left_front()
                .merge_with_none_optional(body_positions.left_front()),
            self.left_middle()
                .merge_with_none_optional(body_positions.left_middle()),
            self.left_rear()
                .merge_with_none_optional(body_positions.left_rear()),
            self.right_front()
                .merge_with_none_optional(body_positions.right_front()),
            self.right_middle()
                .merge_with_none_optional(body_positions.right_middle()),
            self.right_rear()
                .merge_with_none_optional(body_positions.right_rear()),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn creating_commands_for_legs_aligns() {
        let config = LegConfig {
            coxa_id: 1,
            femur_id: 2,
            tibia_id: 3,
            angle_offset: 0.0,
            position: Point3::new(0.0, 0.0, 0.0),
            femur_correction: 0.0,
            tibia_correction: 0.0,
        };
        let positions = LegMotorPositions::new(10.0, 20.0, 30.0);
        let commands = positions.create_command(&config);
        let expected = [
            SyncCommandFloat::new(1, 10.0),
            SyncCommandFloat::new(2, 20.0),
            SyncCommandFloat::new(3, 30.0),
        ];
        assert_eq!(commands, expected);
    }
}
