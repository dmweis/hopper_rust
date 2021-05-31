pub mod stance;
#[cfg(feature = "visualizer")]
pub mod visualizer;
pub mod walking;

use crate::ik_controller::leg_positions::LegPositions;
use crate::ik_controller::{leg_positions::MoveTowards, IkControllable};
use anyhow::Result;
use std::time::Duration;
use tokio::time;
use tokio::{spawn, task::JoinHandle};
use walking::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyState {
    Standing,
    Grounded,
}

impl Default for BodyState {
    fn default() -> Self {
        BodyState::Grounded
    }
}

pub struct MotionController {
    command_sender: last_message_channel::Sender<MotionControllerCommand>,
    command: MotionControllerCommand,
    _handle: JoinHandle<Result<()>>,
}

impl MotionController {
    pub async fn new(ik_controller: Box<dyn IkControllable>) -> Result<Self> {
        let (command_sender, receiver) = last_message_channel::latest_message_channel();
        let command = MotionControllerCommand::default();

        let motion_controller_loop = MotionControllerLoop::new(ik_controller, receiver).await?;
        let handle = spawn(motion_controller_loop.run());
        Ok(MotionController {
            command_sender,
            command,
            _handle: handle,
        })
    }

    pub fn set_command(&mut self, command: MoveCommand) {
        self.command.move_command = command;
        self.command_sender.send(self.command.clone()).unwrap();
    }

    pub fn get_command(&self) -> MoveCommand {
        self.command.move_command
    }

    pub fn set_body_state(&mut self, state: BodyState) {
        self.command.body_state = state;
        self.command_sender.send(self.command.clone()).unwrap();
    }
}

#[derive(Debug, Clone, Default)]
struct MotionControllerCommand {
    move_command: MoveCommand,
    body_state: BodyState,
}

const TICK_DURATION: Duration = Duration::from_millis(1000 / 50);
const MAX_MOVE: f32 = 0.001;
const STEP_HEIGHT: f32 = 0.03;

struct MotionControllerLoop {
    ik_controller: Box<dyn IkControllable>,
    command_receiver: last_message_channel::Receiver<MotionControllerCommand>,
    command: MotionControllerCommand,
    move_duration: Duration,
    state: BodyState,
    last_tripod: Tripod,
    last_written_pose: LegPositions,
}

impl MotionControllerLoop {
    async fn new(
        mut ik_controller: Box<dyn IkControllable>,
        command_receiver: last_message_channel::Receiver<MotionControllerCommand>,
    ) -> Result<Self> {
        let last_written_pose = ik_controller.read_leg_positions().await?;
        Ok(Self {
            ik_controller,
            command_receiver,
            command: MotionControllerCommand::default(),
            move_duration: Duration::from_secs_f32(0.4),
            state: BodyState::Grounded,
            last_tripod: Tripod::LRL,
            last_written_pose,
        })
    }

    async fn run(mut self) -> Result<()> {
        self.control_loop().await?;
        Ok(())
    }

    async fn stand_up(&mut self) -> Result<()> {
        self.transition_direct(&[stance::grounded_stance(), stance::relaxed_wide_stance()])
            .await?;
        self.transition_step(&[stance::relaxed_wide_stance(), stance::relaxed_stance()])
            .await?;
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

    /// Move over sequence of LegPositions using direct motions
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

    /// Move over sequence of LegPositions using step transitions
    ///
    /// Uses the last tripod to alternate steps
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
            if let Some(command) = self.command_receiver.try_recv().unwrap() {
                self.command = command;
            }

            if self.state != self.command.body_state {
                match self.command.body_state {
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
                    self.command.move_command,
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
