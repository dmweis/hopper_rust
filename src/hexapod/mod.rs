use bitflags::bitflags;
use dynamixel_driver::SyncCommand;
use serde::{Deserialize, Serialize};

use crate::hopper_body_config::{BodyConfig, LegConfig};

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

    pub fn all_legs(&self) -> [&T; 6] {
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
