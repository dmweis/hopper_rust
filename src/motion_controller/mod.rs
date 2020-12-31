pub mod stance;
#[cfg(feature = "visualizer")]
pub mod visualizer;
mod walking;

use crate::ik_controller::{leg_positions::MoveTowards, IkControllable};
use anyhow::Result;
use nalgebra::Vector2;
use std::time::Duration;
use tokio::time;
use tokio::{spawn, task::JoinHandle};
use walking::*;

pub struct MotionController {
    ik_controller: Box<dyn IkControllable>,
    last_tripod: Tripod,
}

impl MotionController {
    pub async fn run(ik_controller: Box<dyn IkControllable>) -> Result<()> {
        let controller = MotionController {
            ik_controller,
            last_tripod: Tripod::LRL,
        };
        controller.control_loop().await?;
        Ok(())
    }

    pub fn start_as_task(ik_controller: Box<dyn IkControllable>) -> JoinHandle<Result<()>> {
        let controller = MotionController {
            ik_controller,
            last_tripod: Tripod::LRL,
        };
        spawn(controller.control_loop())
    }

    async fn control_loop(mut self) -> Result<()> {
        const MAX_MOVE: f32 = 0.001;
        const STEP_HEIGHT: f32 = 0.03;
        let mut interval = time::interval(Duration::from_millis(1000 / 50));
        let mut last_written_pose = stance::grounded_stance().clone();
        // stand up
        for pose in &[stance::grounded_stance(), stance::relaxed_wide_stance()] {
            for new_pose in last_written_pose.to_move_towards_iter(pose, MAX_MOVE) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        for stance in &[stance::relaxed_wide_stance(), stance::relaxed_stance()] {
            self.last_tripod.invert();
            let step = last_written_pose.merge_with(stance, self.last_tripod.as_flag());
            for new_pose in StepIterator::step(
                last_written_pose.clone(),
                step.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
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
                MAX_MOVE,
                STEP_HEIGHT,
                self.last_tripod.clone(),
            ) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        loop {
            self.last_tripod.invert();
            let step = step_with_relaxed_transformation(
                &last_written_pose,
                stance::relaxed_stance(),
                &self.last_tripod,
                MoveCommand::new(Vector2::new(0.06, 0.0), 10_f32.to_radians()),
            );
            for new_pose in StepIterator::step(
                last_written_pose.clone(),
                step.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                self.last_tripod.clone(),
            ) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
        }
    }
}
