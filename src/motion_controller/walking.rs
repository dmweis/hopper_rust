use crate::hexapod::LegFlags;
use crate::ik_controller::leg_positions::LegPositions;
use log::*;
use nalgebra::{distance, Point3, Rotation3, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::f32;
use std::time::{Duration, Instant};

#[derive(Debug, Serialize, Deserialize, Copy, Clone, PartialEq, Default)]
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

    pub fn should_move(&self) -> bool {
        let is_zero = self.rotation.abs() < std::f32::EPSILON
            && self.direction.x.abs() < std::f32::EPSILON
            && self.direction.y.abs() < std::f32::EPSILON;
        !is_zero
    }
}

#[derive(Debug, Clone, Copy)]
#[allow(clippy::upper_case_acronyms)]
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
        let full_distance = max_horizontal_distance(&self.start, &self.target);
        let current_distance = (max_horizontal_distance(&self.current, &self.start)
            + self.max_move)
            .min(full_distance);
        let mut progress = current_distance / full_distance;
        if !progress.is_finite() {
            progress = 0.0;
        }
        let (positions, moved) = match self.tripod {
            Tripod::LRL => {
                // lifted
                let (left_front, lf_moved) = step_lifted_leg(
                    self.start.left_front(),
                    self.current.left_front(),
                    self.target.left_front(),
                    self.step_height,
                    progress,
                );
                let (right_middle, rm_moved) = step_lifted_leg(
                    self.start.right_middle(),
                    self.current.right_middle(),
                    self.target.right_middle(),
                    self.step_height,
                    progress,
                );
                let (left_rear, lr_moved) = step_lifted_leg(
                    self.start.left_rear(),
                    self.current.left_rear(),
                    self.target.left_rear(),
                    self.step_height,
                    progress,
                );
                // grounded
                let (right_front, rf_moved) = step_grounded_leg(
                    self.start.right_front(),
                    self.current.right_front(),
                    self.target.right_front(),
                    progress,
                );
                let (left_middle, lm_moved) = step_grounded_leg(
                    self.start.left_middle(),
                    self.current.left_middle(),
                    self.target.left_middle(),
                    progress,
                );
                let (right_rear, rr_moved) = step_grounded_leg(
                    self.start.right_rear(),
                    self.current.right_rear(),
                    self.target.right_rear(),
                    progress,
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
                    progress,
                );
                let (right_middle, rm_moved) = step_grounded_leg(
                    self.start.right_middle(),
                    self.current.right_middle(),
                    self.target.right_middle(),
                    progress,
                );
                let (left_rear, lr_moved) = step_grounded_leg(
                    self.start.left_rear(),
                    self.current.left_rear(),
                    self.target.left_rear(),
                    progress,
                );
                // lifted
                let (right_front, rf_moved) = step_lifted_leg(
                    self.start.right_front(),
                    self.current.right_front(),
                    self.target.right_front(),
                    self.step_height,
                    progress,
                );
                let (left_middle, lm_moved) = step_lifted_leg(
                    self.start.left_middle(),
                    self.current.left_middle(),
                    self.target.left_middle(),
                    self.step_height,
                    progress,
                );
                let (right_rear, rr_moved) = step_lifted_leg(
                    self.start.right_rear(),
                    self.current.right_rear(),
                    self.target.right_rear(),
                    self.step_height,
                    progress,
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

pub(crate) struct TimedStepIterator {
    start: LegPositions,
    last: LegPositions,
    target: LegPositions,
    period: Duration,
    step_height: f32,
    tripod: Tripod,
    start_time: Instant,
}

impl TimedStepIterator {
    pub(crate) fn step(
        start: LegPositions,
        target: LegPositions,
        period: Duration,
        step_height: f32,
        tripod: Tripod,
    ) -> Self {
        Self {
            start: start.clone(),
            last: start,
            target,
            period,
            step_height,
            tripod,
            start_time: Instant::now(),
        }
    }
}

impl Iterator for TimedStepIterator {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        let step_portion = self.start_time.elapsed().as_secs_f32() / self.period.as_secs_f32();
        trace!("Step portion is {}", step_portion);
        let full_distance = max_horizontal_distance(&self.start, &self.target);
        let current_distance = (full_distance * step_portion).min(full_distance);
        let mut progress = current_distance / full_distance;
        if !progress.is_finite() {
            progress = 0.0;
        }
        let (positions, moved) = match self.tripod {
            Tripod::LRL => {
                // lifted
                let (left_front, lf_moved) = step_lifted_leg(
                    self.start.left_front(),
                    self.last.left_front(),
                    self.target.left_front(),
                    self.step_height,
                    progress,
                );
                let (right_middle, rm_moved) = step_lifted_leg(
                    self.start.right_middle(),
                    self.last.right_middle(),
                    self.target.right_middle(),
                    self.step_height,
                    progress,
                );
                let (left_rear, lr_moved) = step_lifted_leg(
                    self.start.left_rear(),
                    self.last.left_rear(),
                    self.target.left_rear(),
                    self.step_height,
                    progress,
                );
                // grounded
                let (right_front, rf_moved) = step_grounded_leg(
                    self.start.right_front(),
                    self.last.right_front(),
                    self.target.right_front(),
                    progress,
                );
                let (left_middle, lm_moved) = step_grounded_leg(
                    self.start.left_middle(),
                    self.last.left_middle(),
                    self.target.left_middle(),
                    progress,
                );
                let (right_rear, rr_moved) = step_grounded_leg(
                    self.start.right_rear(),
                    self.last.right_rear(),
                    self.target.right_rear(),
                    progress,
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
                    self.last.left_front(),
                    self.target.left_front(),
                    progress,
                );
                let (right_middle, rm_moved) = step_grounded_leg(
                    self.start.right_middle(),
                    self.last.right_middle(),
                    self.target.right_middle(),
                    progress,
                );
                let (left_rear, lr_moved) = step_grounded_leg(
                    self.start.left_rear(),
                    self.last.left_rear(),
                    self.target.left_rear(),
                    progress,
                );
                // lifted
                let (right_front, rf_moved) = step_lifted_leg(
                    self.start.right_front(),
                    self.last.right_front(),
                    self.target.right_front(),
                    self.step_height,
                    progress,
                );
                let (left_middle, lm_moved) = step_lifted_leg(
                    self.start.left_middle(),
                    self.last.left_middle(),
                    self.target.left_middle(),
                    self.step_height,
                    progress,
                );
                let (right_rear, rr_moved) = step_lifted_leg(
                    self.start.right_rear(),
                    self.last.right_rear(),
                    self.target.right_rear(),
                    self.step_height,
                    progress,
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
            self.last = positions;
            Some(self.last.clone())
        } else {
            None
        }
    }
}

pub(crate) fn step_lifted_leg(
    start: &Point3<f32>,
    last_written: &Point3<f32>,
    target: &Point3<f32>,
    step_height: f32,
    progress: f32,
) -> (Point3<f32>, bool) {
    if last_written == target {
        return (*target, false);
    }
    if progress >= 1.0 {
        return (*target, true);
    }
    let full_ground_translation = target.xy() - start.xy();
    let current_translation = start.xy() + full_ground_translation * progress;
    let height = (progress * f32::consts::PI).sin() * step_height + start.z;
    let new_position = Point3::new(current_translation.x, current_translation.y, height);
    (new_position, true)
}

pub(crate) fn step_grounded_leg(
    start: &Point3<f32>,
    last_written: &Point3<f32>,
    target: &Point3<f32>,
    progress: f32,
) -> (Point3<f32>, bool) {
    if last_written == target {
        return (*target, false);
    }
    if progress >= 1.0 {
        return (*target, true);
    }
    let full_translation = target - start;
    let new_position = start + full_translation * progress;
    (new_position, true)
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

// Calculate longest distance a leg has to travel
fn max_horizontal_distance(a: &LegPositions, b: &LegPositions) -> f32 {
    let left_front = distance(&a.left_front().xy(), &b.left_front().xy());
    let left_middle = distance(&a.left_middle().xy(), &b.left_middle().xy());
    let left_rear = distance(&a.left_rear().xy(), &b.left_rear().xy());
    let right_front = distance(&a.right_front().xy(), &b.right_front().xy());
    let right_middle = distance(&a.right_middle().xy(), &b.right_middle().xy());
    let right_rear = distance(&a.right_rear().xy(), &b.right_rear().xy());

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

    use crate::motion_controller::stance::relaxed_stance;

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

        let max = max_horizontal_distance(&a, &b);

        assert_relative_eq!(max, 1.0);
    }

    #[test]
    fn tripods_stay_same_height_when_walking() {
        const MAX_MOVE: f32 = 0.001;
        const STEP_HEIGHT: f32 = 0.03;
        let mut tripod = Tripod::LRL;
        let mut last_written = relaxed_stance().clone();
        for _ in 0..4 {
            tripod.invert();
            let step = step_with_relaxed_transformation(
                &last_written,
                relaxed_stance(),
                &tripod,
                MoveCommand::new(Vector2::new(0.00, 0.0), 10_f32.to_radians()),
            );
            for new_pose in StepIterator::step(
                last_written.clone(),
                step.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                tripod,
            ) {
                // LRL tripod
                assert_relative_eq!(new_pose.left_front().z, new_pose.right_middle().z);
                assert_relative_eq!(new_pose.left_rear().z, new_pose.right_middle().z);
                assert_relative_eq!(new_pose.left_rear().z, new_pose.left_front().z);
                // RLR tripod
                assert_relative_eq!(new_pose.right_front().z, new_pose.left_middle().z);
                assert_relative_eq!(new_pose.right_rear().z, new_pose.left_middle().z);
                assert_relative_eq!(new_pose.right_rear().z, new_pose.right_front().z);
                last_written = new_pose;
            }
        }
    }

    #[test]
    fn grounded_tripods_stay_same_distance_on_ground() {
        const MAX_MOVE: f32 = 0.001;
        const STEP_HEIGHT: f32 = 0.03;
        const MAX_RELATIVE_ERROR: f32 = 0.001;
        let mut tripod = Tripod::LRL;
        let initial_pose = relaxed_stance().clone();
        let mut last_written = initial_pose.clone();
        for _ in 0..4 {
            tripod.invert();
            let step = step_with_relaxed_transformation(
                &last_written,
                &initial_pose,
                &tripod,
                MoveCommand::new(Vector2::new(0.04, 0.04), 10_f32.to_radians()),
            );
            for new_pose in StepIterator::step(
                last_written.clone(),
                step.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                tripod,
            ) {
                match tripod {
                    Tripod::RLR => {
                        assert_relative_eq!(
                            distance(new_pose.left_front(), new_pose.right_middle()),
                            distance(initial_pose.left_front(), initial_pose.right_middle()),
                            max_relative = MAX_RELATIVE_ERROR
                        );
                        assert_relative_eq!(
                            distance(new_pose.left_rear(), new_pose.right_middle()),
                            distance(initial_pose.left_rear(), initial_pose.right_middle()),
                            max_relative = MAX_RELATIVE_ERROR
                        );
                        assert_relative_eq!(
                            distance(new_pose.left_rear(), new_pose.left_front()),
                            distance(initial_pose.left_rear(), initial_pose.left_front()),
                            max_relative = MAX_RELATIVE_ERROR
                        );
                    }
                    Tripod::LRL => {
                        assert_relative_eq!(
                            distance(new_pose.right_front(), new_pose.left_middle()),
                            distance(initial_pose.right_front(), initial_pose.left_middle()),
                            max_relative = MAX_RELATIVE_ERROR
                        );
                        assert_relative_eq!(
                            distance(new_pose.right_rear(), new_pose.left_middle()),
                            distance(initial_pose.right_rear(), initial_pose.left_middle()),
                            max_relative = MAX_RELATIVE_ERROR
                        );
                        assert_relative_eq!(
                            distance(new_pose.right_rear(), new_pose.right_front()),
                            distance(initial_pose.right_rear(), initial_pose.right_front()),
                            max_relative = MAX_RELATIVE_ERROR
                        );
                    }
                }

                last_written = new_pose;
            }
        }
    }

    /// this test tests both lifted and grounded triangles for changes
    /// this is technically speaking not guaranteed. But still good to know.
    /// This test could just fail after an update to the gait generation
    /// with no negative effects
    #[test]
    fn no_tripods_stay_same_distance_on_ground() {
        const MAX_MOVE: f32 = 0.001;
        const STEP_HEIGHT: f32 = 0.03;
        const MAX_RELATIVE_ERROR: f32 = 0.005;
        let mut tripod = Tripod::LRL;
        let initial_pose = relaxed_stance().clone();
        let mut last_written = initial_pose.clone();
        for _ in 0..4 {
            tripod.invert();
            let step = step_with_relaxed_transformation(
                &last_written,
                &initial_pose,
                &tripod,
                MoveCommand::new(Vector2::new(0.04, 0.04), 10_f32.to_radians()),
            );
            for new_pose in StepIterator::step(
                last_written.clone(),
                step.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                tripod,
            ) {
                // RLR tripod
                assert_relative_eq!(
                    distance(new_pose.right_front(), new_pose.left_middle()),
                    distance(initial_pose.right_front(), initial_pose.left_middle()),
                    max_relative = MAX_RELATIVE_ERROR
                );
                assert_relative_eq!(
                    distance(new_pose.right_rear(), new_pose.left_middle()),
                    distance(initial_pose.right_rear(), initial_pose.left_middle()),
                    max_relative = MAX_RELATIVE_ERROR
                );
                assert_relative_eq!(
                    distance(new_pose.right_rear(), new_pose.right_front()),
                    distance(initial_pose.right_rear(), initial_pose.right_front()),
                    max_relative = MAX_RELATIVE_ERROR
                );
                // LRL tripod
                assert_relative_eq!(
                    distance(new_pose.left_front(), new_pose.right_middle()),
                    distance(initial_pose.left_front(), initial_pose.right_middle()),
                    max_relative = MAX_RELATIVE_ERROR
                );
                assert_relative_eq!(
                    distance(new_pose.left_rear(), new_pose.right_middle()),
                    distance(initial_pose.left_rear(), initial_pose.right_middle()),
                    max_relative = MAX_RELATIVE_ERROR
                );
                assert_relative_eq!(
                    distance(new_pose.left_rear(), new_pose.left_front()),
                    distance(initial_pose.left_rear(), initial_pose.left_front()),
                    max_relative = MAX_RELATIVE_ERROR
                );
                last_written = new_pose;
            }
        }
    }

    #[test]
    fn move_command_should_not_move() {
        let move_command = MoveCommand::new(Vector2::new(0.0, 0.0), 0.0);
        assert!(!move_command.should_move());
    }

    #[test]
    fn move_command_should_move_rotation() {
        let move_command = MoveCommand::new(Vector2::new(0.0, 0.0), 0.1);
        assert!(move_command.should_move());
    }

    #[test]
    fn move_command_should_move_translation() {
        let move_command = MoveCommand::new(Vector2::new(0.1, 0.0), 0.0);
        assert!(move_command.should_move());
        let move_command = MoveCommand::new(Vector2::new(0.0, 0.1), 0.0);
        assert!(move_command.should_move());
    }
}
