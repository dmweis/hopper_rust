use crate::hexapod::LegFlags;
use crate::ik_controller::leg_positions::LegPositions;
use nalgebra::{distance, Point3, Rotation3, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::f32;

#[derive(Debug, Serialize, Deserialize, Copy, Clone, PartialEq)]
pub struct MoveCommand {
    direction: Vector2<f32>,
    rotation: f32,
}

impl MoveCommand {
    pub fn new(direction: Vector2<f32>, rotation: f32) -> Self {
        Self {
            direction,
            rotation,
        }
    }

    pub fn direction(&self) -> Vector2<f32> {
        self.direction
    }

    pub fn rotation(&self) -> f32 {
        self.rotation
    }
}

#[derive(Debug, Clone)]
pub(crate) enum Tripod {
    LRL,
    RLR,
}

impl Tripod {
    pub(crate) fn invert(&mut self) {
        (*self) = match self {
            Tripod::LRL => Tripod::RLR,
            Tripod::RLR => Tripod::LRL,
        };
    }

    pub(crate) fn as_flag(&self) -> LegFlags {
        match self {
            Tripod::LRL => LegFlags::LRL_TRIPOD,
            Tripod::RLR => LegFlags::RLR_TRIPOD,
        }
    }
}

pub(crate) struct StepIterator {
    start: LegPositions,
    current: LegPositions,
    target: LegPositions,
    max_move: f32,
    step_height: f32,
    tripod: Tripod,
}

impl StepIterator {
    pub(crate) fn step(
        start: LegPositions,
        target: LegPositions,
        max_move: f32,
        step_height: f32,
        tripod: Tripod,
    ) -> StepIterator {
        StepIterator {
            start: start.clone(),
            current: start,
            target,
            max_move,
            step_height,
            tripod,
        }
    }
}

impl Iterator for StepIterator {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        let (positions, moved) = match self.tripod {
            Tripod::LRL => {
                // lifted
                let (left_front, lf_moved) = step_lifted_leg(
                    self.start.left_front(),
                    self.current.left_front(),
                    self.target.left_front(),
                    self.max_move,
                    self.step_height,
                );
                let (right_middle, rm_moved) = step_lifted_leg(
                    self.start.right_middle(),
                    self.current.right_middle(),
                    self.target.right_middle(),
                    self.max_move,
                    self.step_height,
                );
                let (left_rear, lr_moved) = step_lifted_leg(
                    self.start.left_rear(),
                    self.current.left_rear(),
                    self.target.left_rear(),
                    self.max_move,
                    self.step_height,
                );
                // grounded
                let (right_front, rf_moved) = step_grounded_leg(
                    self.start.right_front(),
                    self.current.right_front(),
                    self.target.right_front(),
                    self.max_move,
                );
                let (left_middle, lm_moved) = step_grounded_leg(
                    self.start.left_middle(),
                    self.current.left_middle(),
                    self.target.left_middle(),
                    self.max_move,
                );
                let (right_rear, rr_moved) = step_grounded_leg(
                    self.start.right_rear(),
                    self.current.right_rear(),
                    self.target.right_rear(),
                    self.max_move,
                );
                let moved = lf_moved || lm_moved || lr_moved || rf_moved || rm_moved || rr_moved;
                let positions = LegPositions::new(
                    left_front,
                    left_middle,
                    left_rear,
                    right_front,
                    right_middle,
                    right_rear,
                );
                (positions, moved)
            }
            Tripod::RLR => {
                // grounded
                let (left_front, lf_moved) = step_grounded_leg(
                    self.start.left_front(),
                    self.current.left_front(),
                    self.target.left_front(),
                    self.max_move,
                );
                let (right_middle, rm_moved) = step_grounded_leg(
                    self.start.right_middle(),
                    self.current.right_middle(),
                    self.target.right_middle(),
                    self.max_move,
                );
                let (left_rear, lr_moved) = step_grounded_leg(
                    self.start.left_rear(),
                    self.current.left_rear(),
                    self.target.left_rear(),
                    self.max_move,
                );
                // lifted
                let (right_front, rf_moved) = step_lifted_leg(
                    self.start.right_front(),
                    self.current.right_front(),
                    self.target.right_front(),
                    self.max_move,
                    self.step_height,
                );
                let (left_middle, lm_moved) = step_lifted_leg(
                    self.start.left_middle(),
                    self.current.left_middle(),
                    self.target.left_middle(),
                    self.max_move,
                    self.step_height,
                );
                let (right_rear, rr_moved) = step_lifted_leg(
                    self.start.right_rear(),
                    self.current.right_rear(),
                    self.target.right_rear(),
                    self.max_move,
                    self.step_height,
                );
                let moved = lf_moved || lm_moved || lr_moved || rf_moved || rm_moved || rr_moved;
                let positions = LegPositions::new(
                    left_front,
                    left_middle,
                    left_rear,
                    right_front,
                    right_middle,
                    right_rear,
                );
                (positions, moved)
            }
        };
        if moved {
            self.current = positions;
            Some(self.current.clone())
        } else {
            None
        }
    }
}

pub(crate) fn step_lifted_leg(
    start: &Point3<f32>,
    last_written: &Point3<f32>,
    target: &Point3<f32>,
    max_move: f32,
    step_height: f32,
) -> (Point3<f32>, bool) {
    if last_written == target {
        return (*target, false);
    }
    if distance(&last_written.xy(), &target.xy()) <= max_move {
        return (*target, true);
    }
    let full_ground_translation = target.xy() - start.xy();
    let current_translation =
        (last_written.xy() - start.xy()) + full_ground_translation.normalize() * max_move;
    let ground_position = start.xy() + current_translation;
    let progress = distance(&ground_position, &target.xy()) / distance(&start.xy(), &target.xy());
    let height = (progress * f32::consts::PI).sin() * step_height + start.z;
    let new_position = Point3::new(ground_position.x, ground_position.y, height);
    (new_position, true)
}

pub(crate) fn step_grounded_leg(
    start: &Point3<f32>,
    last_written: &Point3<f32>,
    target: &Point3<f32>,
    max_move: f32,
) -> (Point3<f32>, bool) {
    if last_written == target {
        return (*target, false);
    }
    if distance(&last_written.xy(), &target.xy()) <= max_move {
        return (*target, true);
    }
    let full_translation = target - start;
    let new_position = last_written + full_translation.normalize() * max_move;
    (new_position, true)
}

#[allow(dead_code)]
pub(crate) fn step_transformation(
    start: &LegPositions,
    lifted_tripod: &Tripod,
    motion: Vector2<f32>,
) -> LegPositions {
    let linear_motion = motion.to_homogeneous();
    match lifted_tripod {
        Tripod::LRL => {
            // lifted
            let left_front = start.left_front() + linear_motion;
            let right_middle = start.right_middle() + linear_motion;
            let left_rear = start.left_rear() + linear_motion;
            // grounded
            let left_middle = start.left_middle() - linear_motion;
            let right_front = start.right_front() - linear_motion;
            let right_rear = start.right_rear() - linear_motion;
            LegPositions::new(
                left_front,
                left_middle,
                left_rear,
                right_front,
                right_middle,
                right_rear,
            )
        }
        Tripod::RLR => {
            // lifted
            let right_front = start.right_front() + linear_motion;
            let left_middle = start.left_middle() + linear_motion;
            let right_rear = start.right_rear() + linear_motion;
            // grounded
            let left_front = start.left_front() - linear_motion;
            let left_rear = start.left_rear() - linear_motion;
            let right_middle = start.right_middle() - linear_motion;
            LegPositions::new(
                left_front,
                left_middle,
                left_rear,
                right_front,
                right_middle,
                right_rear,
            )
        }
    }
}

pub(crate) fn step_with_relaxed_transformation(
    start: &LegPositions,
    relaxed: &LegPositions,
    lifted_tripod: &Tripod,
    command: MoveCommand,
) -> LegPositions {
    let linear_motion = command.direction().to_homogeneous();
    let rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), command.rotation() / 2.0);
    let inverse_rotation =
        Rotation3::from_axis_angle(&Vector3::z_axis(), -command.rotation() / 2.0);
    match lifted_tripod {
        Tripod::LRL => {
            // lifted
            let left_front = rotation * (relaxed.left_front() + linear_motion);
            let right_middle = rotation * (relaxed.right_middle() + linear_motion);
            let left_rear = rotation * (relaxed.left_rear() + linear_motion);
            // grounded
            let left_middle = inverse_rotation * (start.left_middle() - linear_motion);
            let right_front = inverse_rotation * (start.right_front() - linear_motion);
            let right_rear = inverse_rotation * (start.right_rear() - linear_motion);
            LegPositions::new(
                left_front,
                left_middle,
                left_rear,
                right_front,
                right_middle,
                right_rear,
            )
        }
        Tripod::RLR => {
            // lifted
            let right_front = rotation * (relaxed.right_front() + linear_motion);
            let left_middle = rotation * (relaxed.left_middle() + linear_motion);
            let right_rear = rotation * (relaxed.right_rear() + linear_motion);
            // grounded
            let left_front = inverse_rotation * (start.left_front() - linear_motion);
            let left_rear = inverse_rotation * (start.left_rear() - linear_motion);
            let right_middle = inverse_rotation * (start.right_middle() - linear_motion);
            LegPositions::new(
                left_front,
                left_middle,
                left_rear,
                right_front,
                right_middle,
                right_rear,
            )
        }
    }
}

fn max_distance(a: &LegPositions, b: &LegPositions) -> f32 {
    let left_front = distance(a.left_front(), b.left_front());
    let left_middle = distance(a.left_middle(), b.left_middle());
    let left_rear = distance(a.left_rear(), b.left_rear());
    let right_front = distance(a.right_front(), b.right_front());
    let right_middle = distance(a.right_middle(), b.right_middle());
    let right_rear = distance(a.right_rear(), b.right_rear());

    left_front
        .max(left_middle)
        .max(left_rear)
        .max(right_front)
        .max(right_middle)
        .max(right_rear)
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn stepping_with_relaxed() {
        let relaxed_point = Point3::new(0.0, 0.0, 0.0);
        let relaxed = LegPositions::new(
            relaxed_point,
            relaxed_point,
            relaxed_point,
            relaxed_point,
            relaxed_point,
            relaxed_point,
        );

        let start_point = Point3::new(-1.0, 0.0, 0.0);
        let start = LegPositions::new(
            start_point,
            start_point,
            start_point,
            start_point,
            start_point,
            start_point,
        );

        let after_step_lrl = step_with_relaxed_transformation(
            &start,
            &relaxed,
            &Tripod::LRL,
            MoveCommand::new(Vector2::new(1.0, 0.0), 0.0),
        );
        let after_step_rlr = step_with_relaxed_transformation(
            &start,
            &relaxed,
            &Tripod::RLR,
            MoveCommand::new(Vector2::new(1.0, 0.0), 0.0),
        );

        let expected_lifted_point = Point3::new(1.0, 0.0, 0.0);
        // lifted
        assert_relative_eq!(after_step_lrl.left_front(), &expected_lifted_point);
        assert_relative_eq!(after_step_lrl.right_middle(), &expected_lifted_point);
        assert_relative_eq!(after_step_lrl.left_rear(), &expected_lifted_point);
        assert_relative_eq!(after_step_rlr.right_front(), &expected_lifted_point);
        assert_relative_eq!(after_step_rlr.left_middle(), &expected_lifted_point);
        assert_relative_eq!(after_step_rlr.right_rear(), &expected_lifted_point);

        let expected_grounded_point = Point3::new(-2.0, 0.0, 0.0);
        // grounded
        assert_relative_eq!(after_step_lrl.right_front(), &expected_grounded_point);
        assert_relative_eq!(after_step_lrl.left_middle(), &expected_grounded_point);
        assert_relative_eq!(after_step_lrl.right_rear(), &expected_grounded_point);
        assert_relative_eq!(after_step_rlr.left_front(), &expected_grounded_point);
        assert_relative_eq!(after_step_rlr.right_middle(), &expected_grounded_point);
        assert_relative_eq!(after_step_rlr.left_rear(), &expected_grounded_point);
    }

    #[test]
    fn stepping_direct() {
        let start_point = Point3::new(0.0, 0.0, 0.0);
        let start = LegPositions::new(
            start_point,
            start_point,
            start_point,
            start_point,
            start_point,
            start_point,
        );

        let after_step_lrl = step_transformation(&start, &Tripod::LRL, Vector2::new(1.0, 0.0));
        let after_step_rlr = step_transformation(&start, &Tripod::RLR, Vector2::new(1.0, 0.0));

        let expected_lifted_point = Point3::new(1.0, 0.0, 0.0);
        // lifted
        assert_relative_eq!(after_step_lrl.left_front(), &expected_lifted_point);
        assert_relative_eq!(after_step_lrl.right_middle(), &expected_lifted_point);
        assert_relative_eq!(after_step_lrl.left_rear(), &expected_lifted_point);
        assert_relative_eq!(after_step_rlr.right_front(), &expected_lifted_point);
        assert_relative_eq!(after_step_rlr.left_middle(), &expected_lifted_point);
        assert_relative_eq!(after_step_rlr.right_rear(), &expected_lifted_point);

        let expected_grounded_point = Point3::new(-1.0, 0.0, 0.0);
        // grounded
        assert_relative_eq!(after_step_lrl.right_front(), &expected_grounded_point);
        assert_relative_eq!(after_step_lrl.left_middle(), &expected_grounded_point);
        assert_relative_eq!(after_step_lrl.right_rear(), &expected_grounded_point);
        assert_relative_eq!(after_step_rlr.left_front(), &expected_grounded_point);
        assert_relative_eq!(after_step_rlr.right_middle(), &expected_grounded_point);
        assert_relative_eq!(after_step_rlr.left_rear(), &expected_grounded_point);
    }

    #[test]
    fn step_iterator_lifts_correct_legs_rlr() {
        let start_point = Point3::new(0.0, 0.0, 0.0);
        let start = LegPositions::new(
            start_point,
            start_point,
            start_point,
            start_point,
            start_point,
            start_point,
        );

        let target_point = Point3::new(1.0, 0.0, 0.0);
        let target = LegPositions::new(
            target_point,
            target_point,
            target_point,
            target_point,
            target_point,
            target_point,
        );

        let mut right_front_lifted = false;
        let mut left_middle_lifted = false;
        let mut right_rear_lifted = false;

        let mut final_state = start.clone();
        for step in StepIterator::step(start, target.clone(), 0.001, 0.02, Tripod::RLR) {
            assert_relative_eq!(step.left_front().z, 0.0);
            assert_relative_eq!(step.right_middle().z, 0.0);
            assert_relative_eq!(step.left_rear().z, 0.0);

            if step.right_front().z > 0.01 {
                right_front_lifted = true;
            }
            if step.left_middle().z > 0.01 {
                left_middle_lifted = true;
            }
            if step.right_rear().z > 0.01 {
                right_rear_lifted = true;
            }
            final_state = step;
        }
        assert!(right_front_lifted);
        assert!(left_middle_lifted);
        assert!(right_rear_lifted);
        assert_eq!(final_state, target);
    }

    #[test]
    fn step_iterator_lifts_correct_legs_lrl() {
        let start_point = Point3::new(0.0, 0.0, 0.0);
        let start = LegPositions::new(
            start_point,
            start_point,
            start_point,
            start_point,
            start_point,
            start_point,
        );

        let target_point = Point3::new(1.0, 0.0, 0.0);
        let target = LegPositions::new(
            target_point,
            target_point,
            target_point,
            target_point,
            target_point,
            target_point,
        );

        let mut left_front_lifted = false;
        let mut right_middle_lifted = false;
        let mut left_rear_lifted = false;

        let mut final_state = start.clone();
        for step in StepIterator::step(start, target.clone(), 0.001, 0.02, Tripod::LRL) {
            assert_relative_eq!(step.right_front().z, 0.0);
            assert_relative_eq!(step.left_middle().z, 0.0);
            assert_relative_eq!(step.right_rear().z, 0.0);

            if step.left_front().z > 0.01 {
                left_front_lifted = true;
            }
            if step.right_middle().z > 0.01 {
                right_middle_lifted = true;
            }
            if step.left_rear().z > 0.01 {
                left_rear_lifted = true;
            }
            final_state = step;
        }
        assert!(left_front_lifted);
        assert!(right_middle_lifted);
        assert!(left_rear_lifted);
        assert_eq!(final_state, target);
    }

    #[test]
    fn max_distance_checks_correct_legs() {
        let a = LegPositions::new(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(20.0, 0.0, 0.0),
            Point3::origin(),
            Point3::origin(),
            Point3::origin(),
            Point3::origin(),
        );

        let b = LegPositions::new(
            Point3::origin(),
            Point3::new(20.0, 0.0, 0.0),
            Point3::origin(),
            Point3::origin(),
            Point3::origin(),
            Point3::origin(),
        );

        let max = max_distance(&a, &b);

        assert_relative_eq!(max, 1.0);
    }
}