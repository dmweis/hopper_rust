pub mod stance;
#[cfg(feature = "visualizer")]
pub mod visualizer;
pub mod walking;

use crate::ik_controller::leg_positions::LegPositions;
use crate::ik_controller::{leg_positions::MoveTowards, IkControllable};
use anyhow::Result;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time;
use tokio::{spawn, task::JoinHandle};
use walking::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyState {
    Standing,
    Grounded,
}

pub struct MotionController {
    command: Arc<Mutex<MoveCommand>>,
    desired_state: Arc<Mutex<BodyState>>,
    _handle: JoinHandle<Result<()>>,
}

impl MotionController {
    pub async fn new(ik_controller: Box<dyn IkControllable>) -> Result<Self> {
        let command = Arc::new(Mutex::new(MoveCommand::default()));
        let desired_state = Arc::new(Mutex::new(BodyState::Grounded));
        let motion_controller_loop =
            MotionControllerLoop::new(ik_controller, command.clone(), desired_state.clone())
                .await?;
        let handle = spawn(motion_controller_loop.run());
        Ok(MotionController {
            command,
            desired_state,
            _handle: handle,
        })
    }

    pub fn set_command(&mut self, command: MoveCommand) {
        let mut guard = self.command.lock().unwrap();
        *guard = command;
    }

    pub fn get_command(&self) -> MoveCommand {
        *self.command.lock().unwrap()
    }

    pub fn set_body_state(&mut self, state: BodyState) {
        let mut guard = self.desired_state.lock().unwrap();
        *guard = state;
    }
}

const TICK_DURATION: Duration = Duration::from_millis(1000 / 50);
const MAX_MOVE: f32 = 0.001;
const STEP_HEIGHT: f32 = 0.03;

struct MotionControllerLoop {
    ik_controller: Box<dyn IkControllable>,
    move_command: Arc<Mutex<MoveCommand>>,
    move_duration: Duration,
    state: BodyState,
    desired_state: Arc<Mutex<BodyState>>,
    last_tripod: Tripod,
    last_written_pose: LegPositions,
}

impl MotionControllerLoop {
    async fn new(
        mut ik_controller: Box<dyn IkControllable>,
        move_command: Arc<Mutex<MoveCommand>>,
        desired_state: Arc<Mutex<BodyState>>,
    ) -> Result<Self> {
        let last_written_pose = ik_controller.read_leg_positions().await?;
        Ok(Self {
            ik_controller,
            move_command,
            move_duration: Duration::from_secs_f32(0.4),
            state: BodyState::Grounded,
            desired_state,
            last_tripod: Tripod::LRL,
            last_written_pose,
        })
    }

    async fn run(mut self) -> Result<()> {
        self.control_loop().await?;
        Ok(())
    }

    async fn stand_up(&mut self) -> Result<()> {
        let mut interval = time::interval(TICK_DURATION);

        // lift up
        for pose in &[stance::grounded_stance(), stance::relaxed_wide_stance()] {
            for new_pose in self.last_written_pose.to_move_towards_iter(pose, MAX_MOVE) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                self.last_written_pose = new_pose;
                interval.tick().await;
            }
        }

        // step wide
        for pose in &[stance::relaxed_wide_stance(), stance::relaxed_stance()] {
            self.last_tripod.invert();
            let target = self
                .last_written_pose
                .merge_with(pose, self.last_tripod.as_flag());
            for new_pose in StepIterator::step(
                self.last_written_pose.clone(),
                target.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                self.last_tripod.clone(),
            ) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                self.last_written_pose = new_pose;
                interval.tick().await;
            }
            self.last_tripod.invert();
            let target = self
                .last_written_pose
                .merge_with(pose, self.last_tripod.as_flag());
            for new_pose in StepIterator::step(
                self.last_written_pose.clone(),
                target.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                self.last_tripod.clone(),
            ) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                self.last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        self.state = BodyState::Standing;
        Ok(())
    }

    async fn sit_down(&mut self) -> Result<()> {
        self.transition_step(&[stance::relaxed_stance(), stance::relaxed_wide_stance()])
            .await?;
        self.transition_direct(&[stance::relaxed_wide_stance(), stance::grounded_stance()])
            .await?;
        self.state = BodyState::Grounded;
        Ok(())
    }

    async fn transition_direct(&mut self, states: &[&LegPositions]) -> Result<()> {
        let mut interval = time::interval(TICK_DURATION);
        for pose in states {
            for new_pose in self.last_written_pose.to_move_towards_iter(pose, MAX_MOVE) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                self.last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        Ok(())
    }

    async fn transition_step(&mut self, states: &[&LegPositions]) -> Result<()> {
        let mut interval = time::interval(TICK_DURATION);
        for pose in states {
            self.last_tripod.invert();
            let target = self
                .last_written_pose
                .merge_with(pose, self.last_tripod.as_flag());
            for new_pose in StepIterator::step(
                self.last_written_pose.clone(),
                target.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                self.last_tripod.clone(),
            ) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                self.last_written_pose = new_pose;
                interval.tick().await;
            }
            self.last_tripod.invert();
            let target = self
                .last_written_pose
                .merge_with(pose, self.last_tripod.as_flag());
            for new_pose in StepIterator::step(
                self.last_written_pose.clone(),
                target.clone(),
                MAX_MOVE,
                STEP_HEIGHT,
                self.last_tripod.clone(),
            ) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                self.last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        Ok(())
    }

    async fn control_loop(&mut self) -> Result<()> {
        let mut interval = time::interval(TICK_DURATION);

        loop {
            let desired_state = *self.desired_state.lock().unwrap();
            if self.state != desired_state {
                match desired_state {
                    BodyState::Standing => self.stand_up().await?,
                    BodyState::Grounded => self.sit_down().await?,
                }
            }

            // only walk if standing
            if self.state == BodyState::Standing {
                self.last_tripod.invert();
                let target = step_with_relaxed_transformation(
                    &self.last_written_pose,
                    stance::relaxed_stance(),
                    &self.last_tripod,
                    *self.move_command.lock().unwrap(),
                );
                for new_pose in TimedStepIterator::step(
                    self.last_written_pose.clone(),
                    target.clone(),
                    self.move_duration,
                    STEP_HEIGHT,
                    self.last_tripod.clone(),
                ) {
                    self.ik_controller.move_to_positions(&new_pose).await?;
                    self.last_written_pose = new_pose;
                    interval.tick().await;
                }
            }
        }
    }
}
