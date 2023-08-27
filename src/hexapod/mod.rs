use bitflags::bitflags;
use dynamixel_driver::SyncCommand;
use serde::{Deserialize, Serialize};

use crate::{
    error::{HopperError, HopperResult},
    hopper_body_config::{BodyConfig, LegConfig},
};

#[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
pub struct HexapodTypes<T: Clone> {
    left_front: T,
    left_middle: T,
    left_rear: T,
    right_front: T,
    right_middle: T,
    right_rear: T,
}

impl<T: Clone> HexapodTypes<T> {
    pub fn new(
        left_front: T,
        left_middle: T,
        left_rear: T,
        right_front: T,
        right_middle: T,
        right_rear: T,
    ) -> Self {
        Self {
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear,
        }
    }

    pub fn left_front(&self) -> &T {
        &self.left_front
    }

    pub fn left_middle(&self) -> &T {
        &self.left_middle
    }

    pub fn left_rear(&self) -> &T {
        &self.left_rear
    }

    pub fn right_front(&self) -> &T {
        &self.right_front
    }

    pub fn right_middle(&self) -> &T {
        &self.right_middle
    }

    pub fn right_rear(&self) -> &T {
        &self.right_rear
    }

    pub fn updated_left_front(&self, change: T) -> Self {
        let mut new = (*self).clone();
        new.left_front = change;
        new
    }

    pub fn updated_left_middle(&self, change: T) -> Self {
        let mut new = (*self).clone();
        new.left_middle = change;
        new
    }

    pub fn updated_left_rear(&self, change: T) -> Self {
        let mut new = (*self).clone();
        new.left_rear = change;
        new
    }

    pub fn updated_right_front(&self, change: T) -> Self {
        let mut new = (*self).clone();
        new.right_front = change;
        new
    }

    pub fn updated_right_middle(&self, change: T) -> Self {
        let mut new = (*self).clone();
        new.right_middle = change;
        new
    }

    pub fn updated_right_rear(&self, change: T) -> Self {
        let mut new = (*self).clone();
        new.right_rear = change;
        new
    }

    pub fn from_legs(legs: [&T; 6]) -> Self {
        Self {
            left_front: legs[0].clone(),
            right_front: legs[1].clone(),
            left_middle: legs[2].clone(),
            right_middle: legs[3].clone(),
            left_rear: legs[4].clone(),
            right_rear: legs[5].clone(),
        }
    }

    pub fn as_legs(&self) -> [&T; 6] {
        [
            &self.left_front,
            &self.right_front,
            &self.left_middle,
            &self.right_middle,
            &self.left_rear,
            &self.right_rear,
        ]
    }

    pub fn selected_legs(&self, legs: LegFlags) -> Vec<&T> {
        let mut selected = Vec::with_capacity(6);
        if legs.contains(LegFlags::LEFT_FRONT) {
            selected.push(&self.left_front);
        }
        if legs.contains(LegFlags::RIGHT_FRONT) {
            selected.push(&self.right_front);
        }
        if legs.contains(LegFlags::LEFT_MIDDLE) {
            selected.push(&self.left_middle);
        }
        if legs.contains(LegFlags::RIGHT_MIDDLE) {
            selected.push(&self.right_middle);
        }
        if legs.contains(LegFlags::LEFT_REAR) {
            selected.push(&self.left_rear);
        }
        if legs.contains(LegFlags::RIGHT_REAR) {
            selected.push(&self.right_rear);
        }
        selected
    }

    pub fn updated_from_selected_legs(&mut self, values: &[T], legs: LegFlags) -> HopperResult<()> {
        if values.len() != legs.bits().count_ones() as usize {
            return Err(HopperError::WrongNumberOfLogs(
                values.len(),
                legs.bits().count_ones() as usize,
            ));
        }
        let mut iterator = values.iter();
        if legs.contains(LegFlags::LEFT_FRONT) {
            self.left_front = iterator.next().unwrap().clone();
        }
        if legs.contains(LegFlags::RIGHT_FRONT) {
            self.right_front = iterator.next().unwrap().clone();
        }
        if legs.contains(LegFlags::LEFT_MIDDLE) {
            self.left_middle = iterator.next().unwrap().clone();
        }
        if legs.contains(LegFlags::RIGHT_MIDDLE) {
            self.right_middle = iterator.next().unwrap().clone();
        }
        if legs.contains(LegFlags::LEFT_REAR) {
            self.left_rear = iterator.next().unwrap().clone();
        }
        if legs.contains(LegFlags::RIGHT_REAR) {
            self.right_rear = iterator.next().unwrap().clone();
        }
        Ok(())
    }

    pub fn merge_with(&self, other: &Self, legs: LegFlags) -> Self {
        let left_front = if LegFlags::LEFT_FRONT & legs == LegFlags::LEFT_FRONT {
            other.left_front()
        } else {
            self.left_front()
        };
        let left_middle = if LegFlags::LEFT_MIDDLE & legs == LegFlags::LEFT_MIDDLE {
            other.left_middle()
        } else {
            self.left_middle()
        };
        let left_rear = if LegFlags::LEFT_REAR & legs == LegFlags::LEFT_REAR {
            other.left_rear()
        } else {
            self.left_rear()
        };
        let right_front = if LegFlags::RIGHT_FRONT & legs == LegFlags::RIGHT_FRONT {
            other.right_front()
        } else {
            self.right_front()
        };
        let right_middle = if LegFlags::RIGHT_MIDDLE & legs == LegFlags::RIGHT_MIDDLE {
            other.right_middle()
        } else {
            self.right_middle()
        };
        let right_rear = if LegFlags::RIGHT_REAR & legs == LegFlags::RIGHT_REAR {
            other.right_rear()
        } else {
            self.right_rear()
        };
        Self::new(
            left_front.clone(),
            left_middle.clone(),
            left_rear.clone(),
            right_front.clone(),
            right_middle.clone(),
            right_rear.clone(),
        )
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub struct LegFlags: u32 {
        const LEFT_FRONT = 0b00000001;
        const LEFT_MIDDLE = 0b00000010;
        const LEFT_REAR = 0b00000100;
        const RIGHT_FRONT = 0b00001000;
        const RIGHT_MIDDLE = 0b00010000;
        const RIGHT_REAR = 0b00100000;
        const LRL_TRIPOD = Self::LEFT_FRONT.bits() | Self::RIGHT_MIDDLE.bits() | Self::LEFT_REAR.bits();
        const RLR_TRIPOD = Self::RIGHT_FRONT.bits() | Self::LEFT_MIDDLE.bits() | Self::RIGHT_REAR.bits();
        const RIGHT = Self::RIGHT_FRONT.bits() | Self::RIGHT_MIDDLE.bits() | Self::RIGHT_REAR.bits();
        const LEFT = Self::LEFT_FRONT.bits() | Self::LEFT_MIDDLE.bits() | Self::LEFT_REAR.bits();
        const MIDDLE = Self::RIGHT_MIDDLE.bits() | Self::LEFT_MIDDLE.bits();
        const FRONT = Self::LEFT_FRONT.bits() | Self::RIGHT_FRONT.bits();
        const REAR = Self::LEFT_REAR.bits() | Self::RIGHT_REAR.bits();
        const ALL = Self::LEFT.bits() | Self::RIGHT.bits();
    }
}

impl<T: Clone + Copy> Copy for HexapodTypes<T> {}

#[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
pub struct TripodLegType<T: Clone> {
    coxa: T,
    femur: T,
    tibia: T,
}

impl<T: Clone> TripodLegType<T> {
    pub fn new(coxa: T, femur: T, tibia: T) -> Self {
        Self { coxa, femur, tibia }
    }

    pub fn pair_with_id(&self, leg_config: &LegConfig) -> [(u8, T); 3] {
        [
            (leg_config.coxa_id, self.coxa.clone()),
            (leg_config.femur_id, self.femur.clone()),
            (leg_config.tibia_id, self.tibia.clone()),
        ]
    }
}

impl<T: Clone + Copy> Copy for TripodLegType<T> {}

impl<T: Clone + Copy> TripodLegType<T> {
    pub fn coxa(&self) -> T {
        self.coxa
    }

    pub fn femur(&self) -> T {
        self.femur
    }

    pub fn tibia(&self) -> T {
        self.tibia
    }
}

pub trait ToSyncCommand {
    fn cerate_sync_command(&self, config: &BodyConfig) -> Vec<SyncCommand>;
}

impl<T> ToSyncCommand for HexapodTypes<TripodLegType<T>>
where
    T: Clone + Into<u32>,
{
    fn cerate_sync_command(&self, config: &BodyConfig) -> Vec<SyncCommand> {
        let mut commands = Vec::with_capacity(18);
        commands.extend_from_slice(&self.left_front().pair_with_id(config.left_front()));
        commands.extend_from_slice(&self.left_middle().pair_with_id(config.left_middle()));
        commands.extend_from_slice(&self.left_rear().pair_with_id(config.left_rear()));
        commands.extend_from_slice(&self.right_front().pair_with_id(config.right_front()));
        commands.extend_from_slice(&self.right_middle().pair_with_id(config.right_middle()));
        commands.extend_from_slice(&self.right_rear().pair_with_id(config.right_rear()));
        commands
            .into_iter()
            .map(|(id, value)| SyncCommand::new(id, value.into()))
            .collect()
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    type TestingHexapodType = HexapodTypes<usize>;

    #[test]
    fn from_legs_and_as_legs_are_in_the_same_order() {
        let testing_hexapod_type = TestingHexapodType::new(0, 1, 2, 3, 4, 5);

        let after_conversion = TestingHexapodType::from_legs(testing_hexapod_type.as_legs());
        assert_eq!(testing_hexapod_type, after_conversion);

        assert_eq!(
            testing_hexapod_type.as_legs().to_vec(),
            testing_hexapod_type.selected_legs(LegFlags::ALL),
        );

        #[allow(clippy::clone_on_copy)]
        let mut testing_hexapod_type_clone = testing_hexapod_type.clone();

        let collected = testing_hexapod_type.selected_legs(LegFlags::LRL_TRIPOD);

        testing_hexapod_type_clone
            .updated_from_selected_legs(
                collected
                    .into_iter()
                    .cloned()
                    .collect::<Vec<_>>()
                    .as_slice(),
                LegFlags::LRL_TRIPOD,
            )
            .unwrap();
        assert_eq!(testing_hexapod_type, testing_hexapod_type_clone);
    }
}
