use nalgebra::{distance, Point3};
use serde::{Deserialize, Serialize};
use std::error::Error;

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct LegPositions {
    left_front: Point3<f32>,
    left_middle: Point3<f32>,
    left_rear: Point3<f32>,
    right_front: Point3<f32>,
    right_middle: Point3<f32>,
    right_rear: Point3<f32>,
}

impl LegPositions {
    pub(crate) fn new(
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

    pub fn left_front(&self) -> &Point3<f32> {
        &self.left_front
    }

    pub fn left_middle(&self) -> &Point3<f32> {
        &self.left_middle
    }

    pub fn left_rear(&self) -> &Point3<f32> {
        &self.left_rear
    }

    pub fn right_front(&self) -> &Point3<f32> {
        &self.right_front
    }

    pub fn right_middle(&self) -> &Point3<f32> {
        &self.right_middle
    }

    pub fn right_rear(&self) -> &Point3<f32> {
        &self.right_rear
    }
}

pub trait MoveTowards {
    type Item;

    fn move_towards(&self, target: &Self, max_move: &f32) -> (Self::Item, bool);
}

impl MoveTowards for Point3<f32> {
    type Item = Point3<f32>;

    fn move_towards(&self, target: &Point3<f32>, max_move: &f32) -> (Point3<f32>, bool) {
        if self == target {
            return (*target, false);
        }
        let distance = distance(self, target);
        if &distance <= max_move {
            // TODO: Think about if this should be true or false
            return (*target, true);
        }
        let vector = target - self;
        (self + vector.normalize() * *max_move, true)
    }
}

impl MoveTowards for LegPositions {
    type Item = LegPositions;

    fn move_towards(&self, target: &LegPositions, max_move: &f32) -> (LegPositions, bool) {
        let (left_front, lf_moved) = self.left_front.move_towards(&target.left_front, max_move);
        let (left_middle, lm_moved) = self.left_middle.move_towards(&target.left_middle, max_move);
        let (left_rear, lr_moved) = self.left_rear.move_towards(&target.left_rear, max_move);
        let (right_front, rf_moved) = self.right_front.move_towards(&target.right_front, max_move);
        let (right_middle, rm_moved) = self
            .right_middle
            .move_towards(&target.right_middle, max_move);
        let (right_rear, rr_moved) = self.right_rear.move_towards(&target.right_rear, max_move);
        let leg_positions = LegPositions::new(
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear,
        );
        let moved = lf_moved || lm_moved || lr_moved || rf_moved || rm_moved || rr_moved;
        (leg_positions, moved)
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub(crate) struct OptionalLegPositions {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub left_front: Option<Point3<f32>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub left_middle: Option<Point3<f32>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub left_rear: Option<Point3<f32>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub right_front: Option<Point3<f32>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub right_middle: Option<Point3<f32>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub right_rear: Option<Point3<f32>>,
}

#[allow(dead_code)]
impl OptionalLegPositions {
    fn new(
        left_front: Option<Point3<f32>>,
        left_middle: Option<Point3<f32>>,
        left_rear: Option<Point3<f32>>,
        right_front: Option<Point3<f32>>,
        right_middle: Option<Point3<f32>>,
        right_rear: Option<Point3<f32>>,
    ) -> OptionalLegPositions {
        OptionalLegPositions {
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear,
        }
    }

    fn to_json(&self) -> Result<String, Box<dyn Error>> {
        Ok(serde_json::to_string(self)?)
    }

    fn from_json(json: &str) -> Result<OptionalLegPositions, Box<dyn Error>> {
        Ok(serde_json::from_str(json)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn move_point_towards_full_step() {
        let start = Point3::new(0_f32, 0_f32, 0_f32);
        let target = Point3::new(1_f32, 1_f32, 1_f32);
        let (new, moved) = start.move_towards(&target, &distance(&start, &target));
        assert!(moved);
        assert_eq!(new, target);
    }

    #[test]
    fn move_point_towards_half() {
        let start = Point3::new(0_f32, 0_f32, 0_f32);
        let target = Point3::new(1_f32, 0_f32, 0_f32);
        let (new, moved) = start.move_towards(&target, &(distance(&start, &target) / 2.0));
        assert!(moved);
        assert_eq!(new, Point3::new(0.5, 0.0, 0.0));
    }

    #[test]
    fn move_point_towards_not_move() {
        let start = Point3::new(1_f32, 0_f32, 0_f32);
        let target = Point3::new(1_f32, 0_f32, 0_f32);
        let (new, moved) = start.move_towards(&target, &10.0);
        assert!(!moved);
        assert_eq!(new, target);
    }

    #[test]
    fn move_legs_towards_full_step() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let from = LegPositions::new(a, a, a, a, a, a);
        let b = Point3::new(1.0, 0.0, 0.0);
        let to = LegPositions::new(b, b, b, b, b, b);
        let (new, moved) = from.move_towards(&to, &1.0);
        assert!(moved);
        assert_eq!(to, new);
    }

    #[test]
    fn move_legs_towards_multiple_steps() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(2.0, 0.0, 0.0);
        let start = LegPositions::new(a, a, a, a, a, a);
        let middle = LegPositions::new(b, b, b, b, b, b);
        let target = LegPositions::new(b, b, b, b, b, c);
        let (new, moved) = start.move_towards(&target, &1.0);
        assert!(moved);
        assert_eq!(middle, new);
        let (new, moved) = new.move_towards(&target, &1.0);
        assert!(moved);
        assert_eq!(target, new);
        let (new, moved) = new.move_towards(&target, &1.0);
        assert!(!moved);
        assert_eq!(target, new);
    }

    #[test]
    fn move_legs_towards_moves_correct_legs() {
        let start = LegPositions::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        );

        let target = LegPositions::new(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
        );
        let mut step = start;
        let mut keep_moving = true;
        let mut counter = 0;
        while keep_moving {
            let (new, moved) = step.move_towards(&target, &1.0);
            step = new;
            keep_moving = moved;
            counter += 1;
        }
        assert_eq!(counter, 7);
        assert_eq!(target, step);
    }

    #[test]
    fn optional_leg_positions_serializes_without_nones() {
        let positions = OptionalLegPositions::new(
            Some(Point3::new(0.15, 0.15, 0.15)),
            None,
            None,
            None,
            None,
            None,
        );
        let json = positions.to_json().unwrap();
        assert_eq!(json, "{\"left_front\":[0.15,0.15,0.15]}");
    }

    #[test]
    fn optional_leg_positions_deserializes_with_or_without_none() {
        let json = "{\"left_front\":[0.15,0.15,0.15],\"right_front\":null,\"right_middle\":null,\"right_rear\":null}";
        let positions = OptionalLegPositions::from_json(&json).unwrap();
        let expected = OptionalLegPositions::new(
            Some(Point3::new(0.15, 0.15, 0.15)),
            None,
            None,
            None,
            None,
            None,
        );
        assert_eq!(expected, positions);
    }
}
