pub mod stance;
#[cfg(feature = "visualizer")]
pub mod visualizer;
pub mod walking;

use crate::ik_controller::{
    leg_positions::{LegPositions, MoveTowards},
    IkControllable,
};
use crate::utilities::MpscChannelHelper;
use anyhow::Result;
use lazy_static::lazy_static;
use log::*;
use nalgebra::{UnitQuaternion, Vector3};
use prometheus::{labels, opts, register_gauge, Gauge};
use std::time::Duration;
use std::{sync::mpsc, time::Instant};
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
    blocking_command_sender: mpsc::Sender<BlockingCommand>,
    command: MotionControllerCommand,
    _handle: JoinHandle<Result<()>>,
}

impl MotionController {
    pub async fn new(ik_controller: Box<dyn IkControllable>) -> Result<Self> {
        let (command_sender, receiver) = last_message_channel::latest_message_channel();
        let command = MotionControllerCommand::default();

        let (blocking_command_sender, blocking_command_receiver) = mpsc::channel();

        let motion_controller_loop =
            MotionControllerLoop::new(ik_controller, receiver, blocking_command_receiver).await?;
        let handle = spawn(motion_controller_loop.run());
        Ok(MotionController {
            command_sender,
            blocking_command_sender,
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

    pub fn set_transformation(&mut self, translation: Vector3<f32>, rotation: UnitQuaternion<f32>) {
        self.command.linear_translation = translation;
        self.command.body_rotation = rotation;
        self.command_sender.send(self.command.clone()).unwrap()
    }

    pub fn set_transformation_euler(&mut self, translation: Vector3<f32>, rotation: Vector3<f32>) {
        self.set_transformation(
            translation,
            UnitQuaternion::from_euler_angles(rotation.x, rotation.y, rotation.z),
        )
    }

    pub fn set_body_state(&mut self, state: BodyState) {
        self.command.body_state = state;
        self.command_sender.send(self.command.clone()).unwrap();
    }
}

impl Drop for MotionController {
    fn drop(&mut self) {
        self.blocking_command_sender
            .send(BlockingCommand::Terminate)
            .expect("Failed to send termination command");
    }
}

#[derive(Debug, Clone, Default)]
struct MotionControllerCommand {
    move_command: MoveCommand,
    body_state: BodyState,
    linear_translation: Vector3<f32>,
    body_rotation: UnitQuaternion<f32>,
}

#[derive(Debug, Clone, Copy)]
enum BlockingCommand {
    Terminate,
}

lazy_static! {
    static ref VOLTAGE_GAUGE: Gauge = register_gauge!(opts!(
        "hopper_legs_voltage",
        "Mean rolling voltage of dynamixel motors",
        labels! {"hi" => "bye",}
    ))
    .unwrap();
}

const TICK_DURATION: Duration = Duration::from_millis(1000 / 50);
const MAX_MOVE: f32 = 0.001;
const MAX_TRANSLATION_STEP: f32 = 0.005;
const MAX_ROTATION_STEP: f32 = std::f32::consts::PI / 180.0;
const STEP_HEIGHT: f32 = 0.03;
const VOLTAGE_READ_PERIOD: Duration = Duration::from_millis(200);

struct MotionControllerLoop {
    ik_controller: Box<dyn IkControllable>,
    command_receiver: last_message_channel::Receiver<MotionControllerCommand>,
    blocking_command_receiver: mpsc::Receiver<BlockingCommand>,
    command: MotionControllerCommand,
    move_duration: Duration,
    state: BodyState,
    last_tripod: Tripod,
    last_written_pose: LegPositions,
    current_rotation: UnitQuaternion<f32>,
    current_translation: Vector3<f32>,
    base_relaxed: LegPositions,
    lrl_reset: bool,
    rlr_reset: bool,
    last_voltage_read: Instant,
}

impl MotionControllerLoop {
    async fn new(
        mut ik_controller: Box<dyn IkControllable>,
        command_receiver: last_message_channel::Receiver<MotionControllerCommand>,
        blocking_command_receiver: mpsc::Receiver<BlockingCommand>,
    ) -> Result<Self> {
        let last_written_pose = ik_controller.read_leg_positions().await?;
        Ok(Self {
            ik_controller,
            command_receiver,
            blocking_command_receiver,
            command: MotionControllerCommand::default(),
            move_duration: Duration::from_secs_f32(0.4),
            state: BodyState::Grounded,
            last_tripod: Tripod::LRL,
            last_written_pose,
            current_rotation: UnitQuaternion::identity(),
            current_translation: Vector3::zeros(),
            base_relaxed: stance::relaxed_stance().clone(),
            lrl_reset: false,
            rlr_reset: false,
            last_voltage_read: Instant::now(),
        })
    }

    async fn run(mut self) -> Result<()> {
        self.control_loop().await?;
        Ok(())
    }

    async fn stand_up(&mut self) -> Result<()> {
        self.last_written_pose = self.ik_controller.read_leg_positions().await?;
        self.transition_direct(&[
            &self.last_written_pose.clone(),
            stance::grounded_stance(),
            stance::relaxed_wide_stance(),
        ])
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
        self.ik_controller.disable_motors().await?;
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
                self.last_tripod,
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
                self.last_tripod,
            ) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                self.last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        Ok(())
    }

    /// shifts transformation towards desired pose
    fn shift_transformation(&mut self) {
        let (new_translation, _moved) = self
            .current_translation
            .move_towards(&self.command.linear_translation, &MAX_TRANSLATION_STEP);
        self.current_translation = new_translation;

        let (new_rotation, _moved) = self
            .current_rotation
            .rotate_towards(&self.command.body_rotation, MAX_ROTATION_STEP);
        self.current_rotation = new_rotation;
    }

    fn transformed_relaxed(&self) -> LegPositions {
        self.base_relaxed
            .transform(self.current_translation, self.current_rotation)
    }

    fn should_reset_legs(&self) -> bool {
        !(self.lrl_reset && self.rlr_reset)
    }

    async fn control_loop(&mut self) -> Result<()> {
        let mut interval = time::interval(TICK_DURATION);

        loop {
            if let Some(command) = self.command_receiver.try_recv().unwrap() {
                self.command = command;
            }
            if let Some(blocking_command) =
                self.blocking_command_receiver.try_recv_optional().unwrap()
            {
                match blocking_command {
                    BlockingCommand::Terminate => {
                        trace!("Exiting control loop");
                        break;
                    }
                }
            }

            if self.state != self.command.body_state {
                match self.command.body_state {
                    BodyState::Standing => self.stand_up().await?,
                    BodyState::Grounded => self.sit_down().await?,
                }
            }

            if self.last_voltage_read.elapsed() > VOLTAGE_READ_PERIOD {
                self.last_voltage_read = Instant::now();
                if let Ok(voltage) = self.ik_controller.read_mean_voltage().await {
                    VOLTAGE_GAUGE.set(voltage as f64);
                } else {
                    error!("Failed to read voltage");
                }
            }

            // only walk if standing
            if self.state == BodyState::Standing {
                // shift transformation
                self.shift_transformation();
                if self.command.move_command.should_move() || self.should_reset_legs() {
                    self.last_tripod.invert();
                    let target = step_with_relaxed_transformation(
                        &self.last_written_pose,
                        &self.base_relaxed,
                        &self.last_tripod,
                        self.command.move_command,
                    );
                    for new_pose in TimedStepIterator::step(
                        self.last_written_pose.clone(),
                        target.clone(),
                        self.move_duration,
                        STEP_HEIGHT,
                        self.last_tripod,
                    ) {
                        self.shift_transformation();
                        // transform pose to current tilt
                        let transformed_pose =
                            new_pose.transform(self.current_translation, self.current_rotation);
                        self.ik_controller
                            .move_to_positions(&transformed_pose)
                            .await?;
                        self.last_written_pose = transformed_pose;
                        interval.tick().await;
                    }
                    match self.last_tripod {
                        Tripod::LRL => self.lrl_reset = true,
                        Tripod::RLR => self.rlr_reset = true,
                    }
                    if self.command.move_command.should_move() {
                        // invalidate legs if moving
                        self.lrl_reset = false;
                        self.rlr_reset = false;
                    }
                } else {
                    // we can do transformations here
                    let transformed_pose = self.transformed_relaxed();
                    if self.last_written_pose != transformed_pose {
                        self.ik_controller
                            .move_to_positions(&transformed_pose)
                            .await?;
                        self.last_written_pose = transformed_pose;
                    }
                }
            }
            interval.tick().await;
        }
        Ok(())
    }
}

pub trait RotateTowards {
    type Item;

    fn rotate_towards(&self, target: &Self, max_rotation: f32) -> (Self::Item, bool);
}

impl RotateTowards for UnitQuaternion<f32> {
    type Item = UnitQuaternion<f32>;

    fn rotate_towards(
        &self,
        target: &UnitQuaternion<f32>,
        max_rotation: f32,
    ) -> (UnitQuaternion<f32>, bool) {
        if self == target {
            return (*target, false);
        }
        let angle = self.angle_to(&target);
        if angle <= max_rotation {
            return (*target, true);
        }
        (self.nlerp(target, max_rotation / angle), true)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn rotate_towards_single_step() {
        let a = UnitQuaternion::<f32>::identity();
        let b = UnitQuaternion::from_euler_angles(std::f32::consts::PI, 0.0, 0.0);
        let (res, rotated) = a.rotate_towards(&b, std::f32::consts::PI);
        assert!(rotated);
        assert_relative_eq!(b, res);
    }

    #[test]
    fn rotate_towards_no_overshoot() {
        let a = UnitQuaternion::<f32>::identity();
        let b = UnitQuaternion::from_euler_angles(std::f32::consts::PI, 0.0, 0.0);
        let (res, rotated) = a.rotate_towards(&b, std::f32::consts::PI * 2.0);
        assert!(rotated);
        assert_relative_eq!(b, res);
    }

    #[test]
    fn rotate_towards_already_rotated() {
        let a = UnitQuaternion::<f32>::identity();
        let (res, rotated) = a.rotate_towards(&a, std::f32::consts::PI);
        assert!(!rotated);
        assert_relative_eq!(a, res);
    }

    #[test]
    fn rotate_towards_half_hardcoded() {
        let start = UnitQuaternion::<f32>::identity();
        let target =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::PI * 0.5);
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::PI * 0.25);
        let (res, rotated) = start.rotate_towards(&target, std::f32::consts::PI * 0.25);
        assert!(rotated);
        assert_relative_eq!(expected, res);
    }

    #[test]
    fn rotate_towards_half_with_nlerp() {
        let start = UnitQuaternion::<f32>::identity();
        let target =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::PI * 0.5);
        let expected = start.nlerp(&target, 0.5);
        let (res, rotated) = start.rotate_towards(&target, std::f32::consts::PI * 0.25);
        assert!(rotated);
        assert_relative_eq!(expected, res);
    }
}
