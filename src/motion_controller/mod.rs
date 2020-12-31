pub mod stance;
#[cfg(feature = "visualizer")]
pub mod visualizer;
pub mod walking;

use crate::ik_controller::{leg_positions::MoveTowards, IkControllable};
use anyhow::Result;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time;
use tokio::{spawn, task::JoinHandle};
use walking::*;

pub struct MotionController {
    command: Arc<Mutex<MoveCommand>>,
    _handle: JoinHandle<Result<()>>,
}

impl MotionController {
    pub fn new(ik_controller: Box<dyn IkControllable>) -> Self {
        let command = Arc::new(Mutex::new(MoveCommand::default()));
        let handle = spawn(MotionController::control_loop(
            ik_controller,
            Arc::clone(&command),
        ));
        MotionController {
            command,
            _handle: handle,
        }
    }

    pub fn set_command(&mut self, command: MoveCommand) {
        let mut guard = self.command.lock().unwrap();
        *guard = command;
    }

    pub fn get_command(&self) -> MoveCommand {
        *self.command.lock().unwrap()
    }

    async fn control_loop(
        mut ik_controller: Box<dyn IkControllable>,
        command: Arc<Mutex<MoveCommand>>,
    ) -> Result<()> {
        const MAX_MOVE: f32 = 0.001;
        const STEP_HEIGHT: f32 = 0.03;
        let mut last_tripod = Tripod::LRL;
        let mut interval = time::interval(Duration::from_millis(1000 / 50));
        let mut last_written_pose = stance::grounded_stance().clone();
        // stand up
        for pose in &[stance::grounded_stance(), stance::relaxed_wide_stance()] {
            for new_pose in last_written_pose.to_move_towards_iter(pose, MAX_MOVE) {
                ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        // step to narrow
        for pose in &[stance::relaxed_wide_stance(), stance::relaxed_stance()] {
            last_tripod.invert();
            let target = last_written_pose.merge_with(pose, last_tripod.as_flag());
            for new_pose in StepIterator::step(
                last_written_pose.clone(),
                target.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                last_tripod.clone(),
            ) {
                ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
            last_tripod.invert();
            let target = last_written_pose.merge_with(pose, last_tripod.as_flag());
            for new_pose in StepIterator::step(
                last_written_pose.clone(),
                target.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                last_tripod.clone(),
            ) {
                ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        loop {
            last_tripod.invert();
            let target = step_with_relaxed_transformation(
                &last_written_pose,
                stance::relaxed_stance(),
                &last_tripod,
                *command.lock().unwrap(),
            );
            for new_pose in StepIterator::step(
                last_written_pose.clone(),
                target.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                last_tripod.clone(),
            ) {
                ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
        }
    }
}
