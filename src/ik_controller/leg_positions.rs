use crate::hexapod::{HexapodTypes, LegFlags};
use nalgebra::{distance, Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use prost_types::Timestamp;
use std::error::Error;

pub type LegPositions = HexapodTypes<Point3<f32>>;

impl LegPositions {
    pub fn transform(
        &self,
        translation: Vector3<f32>,
        rotation: UnitQuaternion<f32>,
    ) -> LegPositions {
        let translation = Translation3::from(translation);
        let iso = Isometry3::from_parts(translation, rotation);
        LegPositions::new(
            iso.transform_point(self.left_front()),
            iso.transform_point(self.left_middle()),
            iso.transform_point(self.left_rear()),
            iso.transform_point(self.right_front()),
            iso.transform_point(self.right_middle()),
            iso.transform_point(self.right_rear()),
        )
    }

    pub fn transform_selected_legs(
        &self,
        translation: Vector3<f32>,
        rotation: UnitQuaternion<f32>,
        legs: LegFlags,
    ) -> LegPositions {
        let new_positions = self.transform(translation, rotation);
        self.merge_with(&new_positions, legs)
    }

    pub fn longest_distance(&self, other: &LegPositions) -> f32 {
        let self_legs = self.as_legs();
        let other_legs = other.as_legs();
        self_legs
            .iter()
            .zip(other_legs.iter())
            .map(|(my, other)| distance(my, other))
            .fold(std::f32::NAN, f32::max)
    }

    pub fn to_foxglove_frame_transport(self) -> crate::foxglove::FrameTransforms {
        fn to_foxglove_vector3(point: &Point3<f32>) -> crate::foxglove::Vector3 {
            crate::foxglove::Vector3 {
                x: point.x as f64,
                y: point.y as f64,
                z: point.z as f64,
            }
        }
        let now = proto_timestamp_now();
        let left_front = crate::foxglove::FrameTransform {
            timestamp: Some(now.clone()),
            parent_frame_id: "body".to_string(),
            child_frame_id: "left_front".to_string(),
            translation: Some(to_foxglove_vector3(self.left_front())),
            rotation: None,
        };
        let left_middle = crate::foxglove::FrameTransform {
            timestamp: Some(now.clone()),
            parent_frame_id: "body".to_string(),
            child_frame_id: "left_middle".to_string(),
            translation: Some(to_foxglove_vector3(self.left_middle())),
            rotation: None,
        };
        let left_rear = crate::foxglove::FrameTransform {
            timestamp: Some(now.clone()),
            parent_frame_id: "body".to_string(),
            child_frame_id: "left_rear".to_string(),
            translation: Some(to_foxglove_vector3(self.left_rear())),
            rotation: None,
        };
        let right_front = crate::foxglove::FrameTransform {
            timestamp: Some(now.clone()),
            parent_frame_id: "body".to_string(),
            child_frame_id: "right_front".to_string(),
            translation: Some(to_foxglove_vector3(self.right_front())),
            rotation: None,
        };
        let right_middle = crate::foxglove::FrameTransform {
            timestamp: Some(now.clone()),
            parent_frame_id: "body".to_string(),
            child_frame_id: "right_middle".to_string(),
            translation: Some(to_foxglove_vector3(self.right_middle())),
            rotation: None,
        };
        let right_rear = crate::foxglove::FrameTransform {
            timestamp: Some(now.clone()),
            parent_frame_id: "body".to_string(),
            child_frame_id: "right_rear".to_string(),
            translation: Some(to_foxglove_vector3(self.right_rear())),
            rotation: None,
        };
        let lidar = crate::foxglove::FrameTransform {
            timestamp: Some(now.clone()),
            parent_frame_id: "body".to_string(),
            child_frame_id: "hopper_lidar".to_string(),
            translation: Some(crate::foxglove::Vector3 {
                x: 0.035,
                y: 0.0,
                z: 0.112,
            }),
            rotation: None,
        };

        let camera = crate::foxglove::FrameTransform {
            timestamp: Some(now.clone()),
            parent_frame_id: "body".to_string(),
            child_frame_id: "hopper_camera".to_string(),
            translation: Some(crate::foxglove::Vector3 {
                x: 0.050,
                y: 0.0,
                z: 0.0,
            }),
            rotation: None,
        };

        crate::foxglove::FrameTransforms {
            transforms: vec![
                left_front,
                left_middle,
                left_rear,
                right_front,
                right_middle,
                right_rear,
                lidar,
                camera,
            ],
        }
    }
}

fn proto_timestamp_now() -> Timestamp {
    let now = std::time::SystemTime::now();
    let duration = now.duration_since(std::time::UNIX_EPOCH).unwrap();
    Timestamp {
        seconds: duration.as_secs() as i64,
        nanos: duration.subsec_nanos() as i32,
    }
}

pub struct MovingTowardsIterator<T> {
    target: T,
    max_move: f32,
    last_state: T,
}

impl Iterator for MovingTowardsIterator<Point3<f32>> {
    type Item = Point3<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.last_state == self.target {
            return None;
        }
        let distance = distance(&self.last_state, &self.target);
        if distance <= self.max_move {
            return Some(self.target);
        }
        let vector = self.target - self.last_state;
        let new_state = self.last_state + vector.normalize() * self.max_move;
        self.last_state = new_state;
        Some(self.last_state)
    }
}

impl Iterator for MovingTowardsIterator<LegPositions> {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        let (left_front, lf_moved) = self
            .last_state
            .left_front()
            .move_towards(self.target.left_front(), &self.max_move);
        let (left_middle, lm_moved) = self
            .last_state
            .left_middle()
            .move_towards(self.target.left_middle(), &self.max_move);
        let (left_rear, lr_moved) = self
            .last_state
            .left_rear()
            .move_towards(self.target.left_rear(), &self.max_move);
        let (right_front, rf_moved) = self
            .last_state
            .right_front()
            .move_towards(self.target.right_front(), &self.max_move);
        let (right_middle, rm_moved) = self
            .last_state
            .right_middle()
            .move_towards(self.target.right_middle(), &self.max_move);
        let (right_rear, rr_moved) = self
            .last_state
            .right_rear()
            .move_towards(self.target.right_rear(), &self.max_move);

        let moved = lf_moved || lm_moved || lr_moved || rf_moved || rm_moved || rr_moved;
        if moved {
            let leg_positions = LegPositions::new(
                left_front,
                left_middle,
                left_rear,
                right_front,
                right_middle,
                right_rear,
            );
            self.last_state = leg_positions;
            Some(leg_positions)
        } else {
            None
        }
    }
}

pub trait MoveTowards {
    type Item;

    fn move_towards(&self, target: &Self, max_move: &f32) -> (Self::Item, bool);
    fn to_move_towards_iter(
        &self,
        target: &Self,
        max_move: f32,
    ) -> MovingTowardsIterator<Self::Item>;
}

impl MoveTowards for Point3<f32> {
    type Item = Point3<f32>;

    fn move_towards(&self, target: &Point3<f32>, max_move: &f32) -> (Point3<f32>, bool) {
        if self == target {
            return (*target, false);
        }
        let distance = distance(self, target);
        if &distance <= max_move {
            return (*target, true);
        }
        let vector = target - self;
        (self + vector.normalize() * *max_move, true)
    }

    fn to_move_towards_iter(
        &self,
        target: &Self,
        max_move: f32,
    ) -> MovingTowardsIterator<Self::Item> {
        MovingTowardsIterator::<Point3<f32>> {
            target: *target,
            max_move,
            last_state: *self,
        }
    }
}

impl MoveTowards for Vector3<f32> {
    type Item = Vector3<f32>;

    fn move_towards(&self, target: &Vector3<f32>, max_move: &f32) -> (Vector3<f32>, bool) {
        if self == target {
            return (*target, false);
        }
        let distance = (self - target).magnitude();
        if &distance <= max_move {
            return (*target, true);
        }
        let vector = target - self;
        (self + vector.normalize() * *max_move, true)
    }

    fn to_move_towards_iter(
        &self,
        target: &Self,
        max_move: f32,
    ) -> MovingTowardsIterator<Self::Item> {
        MovingTowardsIterator::<Vector3<f32>> {
            target: *target,
            max_move,
            last_state: *self,
        }
    }
}

impl MoveTowards for LegPositions {
    type Item = LegPositions;

    fn move_towards(&self, target: &LegPositions, max_move: &f32) -> (LegPositions, bool) {
        let (left_front, lf_moved) = self
            .left_front()
            .move_towards(target.left_front(), max_move);
        let (left_middle, lm_moved) = self
            .left_middle()
            .move_towards(target.left_middle(), max_move);
        let (left_rear, lr_moved) = self.left_rear().move_towards(target.left_rear(), max_move);
        let (right_front, rf_moved) = self
            .right_front()
            .move_towards(target.right_front(), max_move);
        let (right_middle, rm_moved) = self
            .right_middle()
            .move_towards(target.right_middle(), max_move);
        let (right_rear, rr_moved) = self
            .right_rear()
            .move_towards(target.right_rear(), max_move);
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

    fn to_move_towards_iter(
        &self,
        target: &Self,
        max_move: f32,
    ) -> MovingTowardsIterator<Self::Item> {
        MovingTowardsIterator::<LegPositions> {
            target: *target,
            max_move,
            last_state: *self,
        }
    }
}

pub type OptionalLegPositions = HexapodTypes<Option<Point3<f32>>>;

#[allow(dead_code)]
impl OptionalLegPositions {
    fn to_json(self) -> Result<String, Box<dyn Error>> {
        Ok(serde_json::to_string(&self)?)
    }

    fn from_json(json: &str) -> Result<OptionalLegPositions, Box<dyn Error>> {
        Ok(serde_json::from_str(json)?)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::Vector3;

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
    fn move_point_towards_full_step_iter() {
        let start = Point3::new(0_f32, 0_f32, 0_f32);
        let target = Point3::new(1_f32, 1_f32, 1_f32);
        let new = start
            .to_move_towards_iter(&target, distance(&start, &target))
            .next()
            .unwrap();
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
    fn move_point_towards_half_iter() {
        let start = Point3::new(0_f32, 0_f32, 0_f32);
        let target = Point3::new(1_f32, 0_f32, 0_f32);
        let new = start
            .to_move_towards_iter(&target, distance(&start, &target) / 2.0)
            .next()
            .unwrap();
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
    fn move_point_towards_not_move_iter() {
        let start = Point3::new(1_f32, 0_f32, 0_f32);
        let target = Point3::new(1_f32, 0_f32, 0_f32);
        assert!(start.to_move_towards_iter(&target, 10.0).next().is_none())
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
    fn move_legs_towards_multiple_steps_iter() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(2.0, 0.0, 0.0);
        let start = LegPositions::new(a, a, a, a, a, a);
        let middle = LegPositions::new(b, b, b, b, b, b);
        let target = LegPositions::new(b, b, b, b, b, c);
        let mut iterator = start.to_move_towards_iter(&target, 1.0);
        let new = iterator.next().unwrap();
        assert_eq!(middle, new);
        let new = iterator.next().unwrap();
        assert_eq!(target, new);
        assert!(iterator.next().is_none());
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
    fn move_legs_towards_moves_correct_legs_iter() {
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
        let mut counter = 0;
        for new in step.to_move_towards_iter(&target, 1.0) {
            step = new;
            counter += 1;
        }
        assert_eq!(counter, 6);
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
        assert_eq!(json, "{\"left_front\":[0.15,0.15,0.15],\"left_middle\":null,\"left_rear\":null,\"right_front\":null,\"right_middle\":null,\"right_rear\":null}");
    }

    #[test]
    fn optional_leg_positions_deserializes_with_or_without_none() {
        let json = "{\"left_front\":[0.15,0.15,0.15],\"right_front\":null,\"right_middle\":null,\"right_rear\":null}";
        let positions = OptionalLegPositions::from_json(json).unwrap();
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

    #[test]
    fn merging_leg_positions() {
        let point_a = Point3::new(0.0, 0.0, 0.0);
        let a = LegPositions::new(point_a, point_a, point_a, point_a, point_a, point_a);
        let point_b = Point3::new(1.0, 1.0, 1.0);
        let b = LegPositions::new(point_b, point_b, point_b, point_b, point_b, point_b);
        let merged = a.merge_with(&b, LegFlags::LEFT_FRONT);
        assert_eq!(merged.left_front(), b.left_front());
        assert_eq!(merged.left_middle(), a.left_middle());
        assert_eq!(merged.left_rear(), a.left_rear());
        assert_eq!(merged.right_front(), a.right_front());
        assert_eq!(merged.right_middle(), a.right_middle());
        assert_eq!(merged.right_rear(), a.right_rear());
    }

    #[test]
    fn merging_leg_positions_tripod() {
        let point_a = Point3::new(0.0, 0.0, 0.0);
        let a = LegPositions::new(point_a, point_a, point_a, point_a, point_a, point_a);
        let point_b = Point3::new(1.0, 1.0, 1.0);
        let b = LegPositions::new(point_b, point_b, point_b, point_b, point_b, point_b);
        let merged = a.merge_with(&b, LegFlags::LRL_TRIPOD);
        assert_eq!(merged.left_front(), b.left_front());
        assert_eq!(merged.right_middle(), b.right_middle());
        assert_eq!(merged.left_rear(), b.left_rear());
        assert_eq!(merged.right_front(), a.right_front());
        assert_eq!(merged.left_middle(), a.left_middle());
        assert_eq!(merged.right_rear(), a.right_rear());
    }

    #[test]
    fn apply_transformation_to_leg_positions() {
        let point_a = Point3::new(0.0, 0.0, 0.0);
        let a = LegPositions::new(point_a, point_a, point_a, point_a, point_a, point_a);
        let point_b = Point3::new(1.0, 1.0, 1.0);
        let b = LegPositions::new(point_b, point_b, point_b, point_b, point_b, point_b);

        let transformed_a = a.transform(Vector3::new(1.0, 1.0, 1.0), UnitQuaternion::identity());
        assert_relative_eq!(transformed_a.left_front(), b.left_front());
        assert_relative_eq!(transformed_a.right_middle(), b.right_middle());
        assert_relative_eq!(transformed_a.left_rear(), b.left_rear());
        assert_relative_eq!(transformed_a.right_front(), b.right_front());
        assert_relative_eq!(transformed_a.left_middle(), b.left_middle());
        assert_relative_eq!(transformed_a.right_rear(), b.right_rear());
    }
}
