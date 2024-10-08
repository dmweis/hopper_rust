use crate::hexapod::LegFlags;
use crate::ik_controller::leg_positions::LegPositions;
use nalgebra::{distance, Point3, Rotation2, Rotation3, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::f32;
use std::time::{Duration, Instant};
use tracing::*;

pub const DEFAULT_STEP_TIME: Duration = Duration::from_millis(600);
pub const DEFAULT_STEP_HEIGHT: f32 = 0.03;
pub const DEFAULT_STEP_DISTANCE: f32 = 0.025;

#[derive(Debug, Serialize, Deserialize, Copy, Clone, PartialEq)]
pub struct MoveCommand {
    direction: Vector2<f32>,
    rotation: f32,
    step_time: Duration,
    step_height: f32,
    aggressive_leg_lift: bool,
}

impl Default for MoveCommand {
    fn default() -> Self {
        Self {
            direction: Vector2::new(0.0, 0.0),
            rotation: 0.0,
            step_time: DEFAULT_STEP_TIME,
            step_height: DEFAULT_STEP_HEIGHT,
            aggressive_leg_lift: false,
        }
    }
}

impl MoveCommand {
    pub fn new(direction: Vector2<f32>, rotation: f32) -> Self {
        Self {
            direction,
            rotation,
            ..Default::default()
        }
    }

    pub fn with_optional_fields(
        direction: Vector2<f32>,
        rotation: f32,
        step_time: Duration,
        step_height: f32,
        aggressive_leg_lift: bool,
    ) -> Self {
        Self {
            direction,
            rotation,
            step_time,
            step_height,
            aggressive_leg_lift,
        }
    }

    pub fn direction(&self) -> Vector2<f32> {
        self.direction
    }

    pub fn rotation(&self) -> f32 {
        self.rotation
    }

    pub fn step_time(&self) -> Duration {
        self.step_time
    }

    pub fn step_height(&self) -> f32 {
        self.step_height
    }

    pub fn aggressive_leg_lift(&self) -> bool {
        self.aggressive_leg_lift
    }

    pub fn should_move(&self) -> bool {
        let is_zero = self.rotation.abs() < f32::EPSILON
            && self.direction.x.abs() < f32::EPSILON
            && self.direction.y.abs() < f32::EPSILON;
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
    last: LegPositions,
    target: LegPositions,
    max_move: f32,
    step_height: f32,
    grounded_leg_descent: f32,
    tripod: Tripod,
    /// In this mode the legs will immediately lift to full height
    /// this prevents dragging of feet but also results in a very choppy movement
    aggressive_leg_lift: bool,
}

impl StepIterator {
    pub(crate) fn step(
        start: LegPositions,
        target: LegPositions,
        max_move: f32,
        step_height: f32,
        tripod: Tripod,
        aggressive_leg_lift: bool,
    ) -> StepIterator {
        StepIterator {
            start,
            last: start,
            target,
            max_move,
            step_height,
            grounded_leg_descent: 0.0,
            tripod,
            aggressive_leg_lift,
        }
    }
}

impl Iterator for StepIterator {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        let full_distance = max_horizontal_distance(&self.start, &self.target);
        if full_distance == 0.0 {
            return None;
        }
        let current_distance =
            (max_horizontal_distance(&self.last, &self.start) + self.max_move).min(full_distance);
        let progress = current_distance / full_distance;
        let (positions, moved) = shift_legs(
            &self.start,
            &self.last,
            &self.target,
            self.step_height,
            self.grounded_leg_descent,
            self.tripod,
            progress,
            self.aggressive_leg_lift,
        );
        if moved {
            self.last = positions;
            Some(self.last)
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
    grounded_leg_descent: f32,
    tripod: Tripod,
    start_time: Instant,
    /// In this mode the legs will immediately lift to full height
    /// this prevents dragging of feet but also results in a very choppy movement
    aggressive_leg_lift: bool,
}

impl TimedStepIterator {
    pub(crate) fn step(
        start: LegPositions,
        target: LegPositions,
        period: Duration,
        step_height: f32,
        grounded_leg_descent: f32,
        tripod: Tripod,
        aggressive_leg_lift: bool,
    ) -> Self {
        Self {
            start,
            last: start,
            target,
            period,
            step_height,
            grounded_leg_descent,
            tripod,
            start_time: Instant::now(),
            aggressive_leg_lift,
        }
    }
}

impl Iterator for TimedStepIterator {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        let full_distance = max_horizontal_distance(&self.start, &self.target);
        if full_distance <= 0.0 {
            return None;
        }
        let step_portion = self.start_time.elapsed().as_secs_f32() / self.period.as_secs_f32();
        let progress = step_portion / self.period.as_secs_f32();
        trace!("Step progress is {}", progress);

        let (positions, moved) = shift_legs(
            &self.start,
            &self.last,
            &self.target,
            self.step_height,
            self.grounded_leg_descent,
            self.tripod,
            progress,
            self.aggressive_leg_lift,
        );
        if moved {
            self.last = positions;
            Some(self.last)
        } else {
            None
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn shift_legs(
    start: &LegPositions,
    last: &LegPositions,
    target: &LegPositions,
    step_height: f32,
    grounded_leg_descent: f32,
    tripod: Tripod,
    progress: f32,
    aggressive_leg_lift: bool,
) -> (LegPositions, bool) {
    match tripod {
        Tripod::LRL => {
            // lifted
            let (left_front, lf_moved) = step_lifted_leg(
                start.left_front(),
                last.left_front(),
                target.left_front(),
                step_height,
                progress,
                aggressive_leg_lift,
            );
            let (right_middle, rm_moved) = step_lifted_leg(
                start.right_middle(),
                last.right_middle(),
                target.right_middle(),
                step_height,
                progress,
                aggressive_leg_lift,
            );
            let (left_rear, lr_moved) = step_lifted_leg(
                start.left_rear(),
                last.left_rear(),
                target.left_rear(),
                step_height,
                progress,
                aggressive_leg_lift,
            );
            // grounded
            let (right_front, rf_moved) = step_lifted_leg(
                start.right_front(),
                last.right_front(),
                target.right_front(),
                grounded_leg_descent,
                progress,
                aggressive_leg_lift,
            );
            let (left_middle, lm_moved) = step_lifted_leg(
                start.left_middle(),
                last.left_middle(),
                target.left_middle(),
                grounded_leg_descent,
                progress,
                aggressive_leg_lift,
            );
            let (right_rear, rr_moved) = step_lifted_leg(
                start.right_rear(),
                last.right_rear(),
                target.right_rear(),
                grounded_leg_descent,
                progress,
                aggressive_leg_lift,
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
            let (left_front, lf_moved) = step_lifted_leg(
                start.left_front(),
                last.left_front(),
                target.left_front(),
                grounded_leg_descent,
                progress,
                aggressive_leg_lift,
            );
            let (right_middle, rm_moved) = step_lifted_leg(
                start.right_middle(),
                last.right_middle(),
                target.right_middle(),
                grounded_leg_descent,
                progress,
                aggressive_leg_lift,
            );
            let (left_rear, lr_moved) = step_lifted_leg(
                start.left_rear(),
                last.left_rear(),
                target.left_rear(),
                grounded_leg_descent,
                progress,
                aggressive_leg_lift,
            );
            // lifted
            let (right_front, rf_moved) = step_lifted_leg(
                start.right_front(),
                last.right_front(),
                target.right_front(),
                step_height,
                progress,
                aggressive_leg_lift,
            );
            let (left_middle, lm_moved) = step_lifted_leg(
                start.left_middle(),
                last.left_middle(),
                target.left_middle(),
                step_height,
                progress,
                aggressive_leg_lift,
            );
            let (right_rear, rr_moved) = step_lifted_leg(
                start.right_rear(),
                last.right_rear(),
                target.right_rear(),
                step_height,
                progress,
                aggressive_leg_lift,
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
    }
}

pub(crate) fn step_lifted_leg(
    start: &Point3<f32>,
    last_written: &Point3<f32>,
    target: &Point3<f32>,
    step_height: f32,
    progress: f32,
    aggressive_leg_lift: bool,
) -> (Point3<f32>, bool) {
    if last_written == target {
        return (*target, false);
    }
    if progress >= 1.0 {
        return (*target, true);
    }
    let full_ground_translation = target.xy() - start.xy();
    let current_translation = start.xy() + full_ground_translation * progress;
    // raise leg immediately then lower using sine curve
    let height = if aggressive_leg_lift && progress < 0.5 {
        step_height + start.z
    } else {
        (progress * f32::consts::PI).sin() * step_height + start.z
    };
    let new_position = Point3::new(current_translation.x, current_translation.y, height);
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
    // let inverse_rotation =
    //     Rotation3::from_axis_angle(&Vector3::z_axis(), -command.rotation() / 2.0);
    match lifted_tripod {
        Tripod::LRL => {
            // lifted
            let left_front = rotation * (relaxed.left_front() + linear_motion);
            let right_middle = rotation * (relaxed.right_middle() + linear_motion);
            let left_rear = rotation * (relaxed.left_rear() + linear_motion);
            // grounded

            // TODO(David): This is still dragging its feet across the floor
            // fix it
            let start_center_point = (start.left_middle().xy().coords
                + start.right_front().xy().coords
                + start.right_rear().xy().coords)
                / 3.0;
            let relaxed_center_point = (relaxed.left_middle().xy().coords
                + relaxed.right_front().xy().coords
                + relaxed.right_rear().xy().coords)
                / 3.0;
            let translation = (relaxed_center_point - start_center_point).to_homogeneous();

            let average_rotation = start
                .selected_legs(LegFlags::RLR_TRIPOD)
                .into_iter()
                .map(|position| position.xy().coords - start_center_point)
                .zip(
                    relaxed
                        .selected_legs(LegFlags::RLR_TRIPOD)
                        .into_iter()
                        .map(|pos| pos.xy().coords),
                )
                .map(|(start, relaxed)| Rotation2::rotation_between(&start, &relaxed).angle())
                .sum::<f32>()
                / 3.0;

            let correcting_rotation = Rotation3::from_axis_angle(
                &Vector3::z_axis(),
                average_rotation + (-command.rotation() / 2.0),
            );

            let left_middle =
                correcting_rotation * (start.left_middle() + translation - linear_motion);
            let right_front =
                correcting_rotation * (start.right_front() + translation - linear_motion);
            let right_rear =
                correcting_rotation * (start.right_rear() + translation - linear_motion);
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

            let start_center_point = (start.left_front().xy().coords
                + start.left_rear().xy().coords
                + start.right_middle().xy().coords)
                / 3.0;
            let relaxed_center_point = (relaxed.left_front().xy().coords
                + relaxed.left_rear().xy().coords
                + relaxed.right_middle().xy().coords)
                / 3.0;
            let translation = (relaxed_center_point - start_center_point).to_homogeneous();

            let average_rotation = start
                .selected_legs(LegFlags::LRL_TRIPOD)
                .into_iter()
                .map(|position| position.xy().coords - start_center_point)
                .zip(
                    relaxed
                        .selected_legs(LegFlags::LRL_TRIPOD)
                        .into_iter()
                        .map(|pos| pos.xy().coords),
                )
                .map(|(start, relaxed)| Rotation2::rotation_between(&start, &relaxed).angle())
                .sum::<f32>()
                / 3.0;

            let correcting_rotation = Rotation3::from_axis_angle(
                &Vector3::z_axis(),
                average_rotation + (-command.rotation() / 2.0),
            );

            let left_front =
                correcting_rotation * (start.left_front() + translation - linear_motion);
            let left_rear = correcting_rotation * (start.left_rear() + translation - linear_motion);
            let right_middle =
                correcting_rotation * (start.right_middle() + translation - linear_motion);
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
    a.as_legs()
        .into_iter()
        .zip(b.as_legs())
        .map(|(a, b)| distance(&a.xy(), &b.xy()))
        .max_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap_or_default()
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

        let mut final_state = start;
        for step in StepIterator::step(start, target, 0.001, 0.02, Tripod::RLR, false) {
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

        let mut final_state = start;
        for step in StepIterator::step(start, target, 0.001, 0.02, Tripod::LRL, false) {
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
        let mut last_written = *relaxed_stance();
        for _ in 0..4 {
            tripod.invert();
            let step = step_with_relaxed_transformation(
                &last_written,
                relaxed_stance(),
                &tripod,
                MoveCommand::new(Vector2::new(0.00, 0.0), 10_f32.to_radians()),
            );
            for new_pose in
                StepIterator::step(last_written, step, MAX_MOVE, STEP_HEIGHT, tripod, false)
            {
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
        let initial_pose = *relaxed_stance();
        let mut last_written = initial_pose;
        for _ in 0..4 {
            tripod.invert();
            let step = step_with_relaxed_transformation(
                &last_written,
                &initial_pose,
                &tripod,
                MoveCommand::new(Vector2::new(0.04, 0.04), 10_f32.to_radians()),
            );
            for new_pose in
                StepIterator::step(last_written, step, MAX_MOVE, STEP_HEIGHT, tripod, false)
            {
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
        let initial_pose = *relaxed_stance();
        let mut last_written = initial_pose;
        for _ in 0..4 {
            tripod.invert();
            let step = step_with_relaxed_transformation(
                &last_written,
                &initial_pose,
                &tripod,
                MoveCommand::new(Vector2::new(0.04, 0.04), 10_f32.to_radians()),
            );
            for new_pose in
                StepIterator::step(last_written, step, MAX_MOVE, STEP_HEIGHT, tripod, false)
            {
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
    fn step_iterator_handles_zero_steps() {
        let initial_pose = *relaxed_stance();
        let target = initial_pose;
        let mut step_iterator =
            StepIterator::step(initial_pose, target, 1.0, 0.003, Tripod::LRL, false);
        assert_eq!(step_iterator.next(), None);
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

    #[test]
    fn grounded_and_zero_lifte_step_are_same() {
        let start = Point3::new(0.0, 0.0, 0.0);
        let target = Point3::new(1.0, 0.0, 0.0);
        #[allow(clippy::clone_on_copy)]
        let mut last_written = start.clone();
        for progress in 0..=100 {
            let (lifted_position, lifted_moved) = step_lifted_leg(
                &start,
                &last_written,
                &target,
                0.0,
                progress as f32 * 0.01,
                false,
            );
            let lerped = Point3::from(start.coords.lerp(&target.coords, progress as f32 * 0.01));
            assert_relative_eq!(lifted_position, lerped);
            assert!(lifted_moved);
            last_written = lifted_position;
        }
        // check that if we overstep progress result is clamped
        let (lifted_position, lifted_moved) =
            step_lifted_leg(&start, &last_written, &target, 0.0, 1.0 + 0.01, false);
        assert_relative_eq!(lifted_position, target);
        assert!(!lifted_moved);
        // check that last result is correct
        assert_relative_eq!(last_written, target);
    }
}
