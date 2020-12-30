use bitflags::bitflags;
use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct HexapodTypes<T> {
    left_front: T,
    left_middle: T,
    left_rear: T,
    right_front: T,
    right_middle: T,
    right_rear: T,
}

impl<T> HexapodTypes<T> {
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
}

bitflags! {
    struct Legs: u32 {
        const LEFT_FRONT = 0b00000001;
        const LEFT_MIDDLE = 0b00000010;
        const LEFT_REAR = 0b00000100;
        const RIGHT_FRONT = 0b00001000;
        const RIGHT_MIDDLE = 0b00010000;
        const RIGHT_REAR = 0b00100000;
        const LRL_TRIPOD = Self::LEFT_FRONT.bits | Self::RIGHT_MIDDLE.bits | Self::LEFT_REAR.bits;
        const RLR_TRIPOD = Self::RIGHT_FRONT.bits | Self::LEFT_MIDDLE.bits | Self::RIGHT_REAR.bits;
        const RIGHT = Self::RIGHT_FRONT.bits | Self::RIGHT_MIDDLE.bits | Self::RIGHT_REAR.bits;
        const LEFT = Self::LEFT_FRONT.bits | Self::LEFT_MIDDLE.bits | Self::LEFT_REAR.bits;
        const MIDDLE = Self::RIGHT_MIDDLE.bits | Self::LEFT_MIDDLE.bits;
        const FRONT = Self::LEFT_FRONT.bits | Self::RIGHT_FRONT.bits;
        const REAR = Self::LEFT_REAR.bits | Self::RIGHT_REAR.bits;
    }
}
