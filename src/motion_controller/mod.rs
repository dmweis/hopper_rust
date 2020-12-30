use crate::hexapod::LegFlags;
use crate::ik_controller::{
    leg_positions::{LegPositions, MoveTowards},
    IkControlable,
};
use anyhow::Result;
use nalgebra::{distance, Point3};
use std::f32;
use std::time::Duration;
use tokio::time;
use tokio::{spawn, task::JoinHandle};

pub mod stance;
#[cfg(feature = "visualizer")]
pub mod visualizer;

pub struct MotionController {
    ik_controller: Box<dyn IkControlable>,
    last_tripod: Tripod,
}

impl MotionController {
    pub async fn run(ik_controller: Box<dyn IkControlable>) -> Result<()> {
        let controller = MotionController {
            ik_controller,
            last_tripod: Tripod::LRL,
        };
        controller.control_loop().await?;
        Ok(())
    }

    pub fn start_as_task(ik_controller: Box<dyn IkControlable>) -> JoinHandle<Result<()>> {
        let controller = MotionController {
            ik_controller,
            last_tripod: Tripod::LRL,
        };
        spawn(controller.control_loop())
    }

    async fn control_loop(mut self) -> Result<()> {
        let stances = vec![stance::relaxed_wide_stance(), stance::relaxed_stance()];
        let mut interval = time::interval(Duration::from_millis(1000 / 50));
        let mut last_written_pose = stance::grounded_stance().clone();
        // stand up
        for pose in &[stance::grounded_stance(), stance::relaxed_wide_stance()] {
            for new_pose in last_written_pose.to_move_towards_iter(pose, 0.001) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        for stance in std::iter::repeat(stances).flatten() {
            self.last_tripod.invert();
            let step = last_written_pose.merge_with(stance, self.last_tripod.as_flag());
            for new_pose in StepIterator::step(
                last_written_pose.clone(),
                step.clone(),
                0.0008,
                0.03,
                self.last_tripod.clone(),
            ) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
            self.last_tripod.invert();
            let step = last_written_pose.merge_with(stance, self.last_tripod.as_flag());
            for new_pose in StepIterator::step(
                last_written_pose.clone(),
                step.clone(),
                0.0008,
                0.03,
                self.last_tripod.clone(),
            ) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
enum Tripod {
    LRL,
    RLR,
}

impl Tripod {
    fn invert(&mut self) {
        (*self) = match self {
            Tripod::LRL => Tripod::RLR,
            Tripod::RLR => Tripod::LRL,
        };
    }

    fn as_flag(&self) -> LegFlags {
        match self {
            Tripod::LRL => LegFlags::LRL_TRIPOD,
            Tripod::RLR => LegFlags::RLR_TRIPOD,
        }
    }
}

struct StepIterator {
    start: LegPositions,
    current: LegPositions,
    target: LegPositions,
    max_move: f32,
    step_height: f32,
    tripod: Tripod,
}

impl StepIterator {
    fn step(
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

fn step_lifted_leg(
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
    let progress = distance(&last_written.xy(), &target.xy()) / distance(&start.xy(), &target.xy());
    let height = (progress * f32::consts::PI).sin() * step_height + start.z;
    let ground_position = start.xy() + current_translation;
    let new_position = Point3::new(ground_position.x, ground_position.y, height);
    (new_position, true)
}

fn step_grounded_leg(
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
