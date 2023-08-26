mod choreographer;
pub mod folding;
pub mod stance;
#[cfg(feature = "visualizer")]
pub mod visualizer;
pub mod walking;

use crate::body_controller::motor_controller::{HexapodCompliance, HexapodMotorSpeed};
use crate::error::{HopperError, HopperResult};
use crate::ik_controller::{
    leg_positions::{LegPositions, MoveTowards},
    IkControllable,
};
use crate::ioc_container::IocContainer;
use crate::speech::SpeechService;
use crate::utilities::{MpscChannelHelper, RateTracker};
pub use choreographer::DanceMove;
use config::Source;
use hopper_face::FaceController;
use nalgebra::{UnitQuaternion, Vector3};
use std::time::Duration;
use std::{sync::mpsc, time::Instant};
use tokio::sync::Mutex;
use tokio::time;
use tokio::{spawn, task::JoinHandle};
use tracing::*;
use walking::*;

use self::folding::FoldingManager;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BodyState {
    Standing,
    #[default]
    Grounded,
    Folded,
}

pub struct MotionController {
    command_sender: last_message_channel::Sender<MotionControllerCommand>,
    blocking_command_sender: mpsc::Sender<BlockingCommand>,
    command: MotionControllerCommand,
    _handle: JoinHandle<HopperResult<()>>,
}

impl MotionController {
    pub async fn new(
        ik_controller: Box<dyn IkControllable>,
        control_loop_rate_tracker: RateTracker,
    ) -> HopperResult<Self> {
        let (command_sender, receiver) = last_message_channel::latest_message_channel();
        let command = MotionControllerCommand::default();

        let (blocking_command_sender, blocking_command_receiver) = mpsc::channel();

        let motion_controller_loop = MotionControllerLoop::new(
            ik_controller,
            receiver,
            blocking_command_receiver,
            control_loop_rate_tracker,
        )
        .await?;

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

    pub fn set_body_compliance_slpe(&mut self, compliance: HexapodCompliance) {
        self.blocking_command_sender
            .send(BlockingCommand::SetCompliance(compliance))
            .unwrap();
    }

    pub fn set_body_motor_speed(&mut self, speed: HexapodMotorSpeed) {
        self.blocking_command_sender
            .send(BlockingCommand::SetMotorSpeed(speed))
            .unwrap();
    }

    pub fn start_sequence(&mut self, dance_move: DanceMove) {
        info!("Starting dance sequence");
        self.blocking_command_sender
            .send(BlockingCommand::Choreography(dance_move))
            .unwrap();
    }

    pub fn disable_motors(&mut self) {
        self.blocking_command_sender
            .send(BlockingCommand::DisableMotors)
            .unwrap();
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
    DisableMotors,
    Choreography(DanceMove),
    SetCompliance(HexapodCompliance),
    SetMotorSpeed(HexapodMotorSpeed),
}

const TICK_DURATION: Duration = Duration::from_millis(1000 / 50);
const MAX_MOVE: f32 = 0.001;
const MAX_TRANSLATION_STEP: f32 = 0.004;
// TODO(David): Decide on this setting or make it configurable
// const MAX_TRANSLATION_STEP: f32 = 0.005;
const MAX_ROTATION_STEP: f32 = std::f32::consts::PI / 180.0;
const STEP_HEIGHT: f32 = 0.03;
const GROUNDED_STEP_HEIGHT: f32 = -0.0;
const VOLTAGE_READ_PERIOD: Duration = Duration::from_millis(200);
const MOVE_DURATION: Duration = Duration::from_millis(400);
// TODO(David): this results in smoother motion
// Step time should probably be a function of time?
// const MOVE_DURATION: Duration = Duration::from_millis(700);

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
    dance_moves: Vec<LegPositions>,
    control_loop_rate_tracker: RateTracker,
}

impl MotionControllerLoop {
    async fn new(
        mut ik_controller: Box<dyn IkControllable>,
        command_receiver: last_message_channel::Receiver<MotionControllerCommand>,
        blocking_command_receiver: mpsc::Receiver<BlockingCommand>,
        control_loop_rate_tracker: RateTracker,
    ) -> HopperResult<Self> {
        let last_written_pose = ik_controller.read_leg_positions().await?;
        Ok(Self {
            ik_controller,
            command_receiver,
            blocking_command_receiver,
            command: MotionControllerCommand::default(),
            move_duration: MOVE_DURATION,
            state: BodyState::Grounded,
            last_tripod: Tripod::LRL,
            last_written_pose,
            current_rotation: UnitQuaternion::identity(),
            current_translation: Vector3::zeros(),
            base_relaxed: *stance::relaxed_stance(),
            lrl_reset: false,
            rlr_reset: false,
            last_voltage_read: Instant::now(),
            dance_moves: vec![],
            control_loop_rate_tracker,
        })
    }

    /// Attempt to estimate current body state
    async fn estimate_current_body_state(&mut self) -> HopperResult<BodyState> {
        self.last_written_pose = self.ik_controller.read_leg_positions().await?;

        let z_heights: Vec<_> = self
            .last_written_pose
            .all_legs()
            .iter()
            .map(|position| position.z)
            .collect();

        // estimate based on height of legs
        let standing_z_position = stance::STANDING_LEG_HEIGHT;
        const TOLERANCE_DISTANCE: f32 = 0.015;

        // check if all legs are in standing height
        let standing = z_heights
            .iter()
            .all(|&z| (z - standing_z_position).abs() < TOLERANCE_DISTANCE);

        if standing {
            self.state = BodyState::Standing;
        } else {
            // if grounded check if we are likely folded
            let folded = FoldingManager::new(&mut self.ik_controller)
                .await?
                .check_if_folded();
            if folded {
                self.state = BodyState::Folded;
            } else {
                self.state = BodyState::Grounded;
            }
        }
        Ok(self.state)
    }

    async fn read_current_pose(&mut self) -> HopperResult<()> {
        self.last_written_pose = self.ik_controller.read_leg_positions().await?;
        Ok(())
    }

    async fn run(mut self) -> HopperResult<()> {
        // try to estimate current body state
        let estimate = self.estimate_current_body_state().await?;
        info!("Estimated body state to be {:?}", estimate);
        loop {
            match self.control_loop().await {
                Err(error) => {
                    error!("Control loop error {}", error);
                    match error {
                        HopperError::GenericIkError => {
                            warn!("Error is generic IK error. Restarting");
                            if let Ok(speech_service) =
                                IocContainer::global_instance().service::<Mutex<SpeechService>>()
                            {
                                if let Err(err) = speech_service
                                    .lock()
                                    .await
                                    .play_sound("hopper_sounds/ik_failure.wav")
                                    .await
                                {
                                    error!("Failed to play sound {}", err);
                                }
                            } else {
                                error!("Failed to get speech service");
                            }
                            // Attempt recovery
                            self.command = MotionControllerCommand::default();
                            self.state = BodyState::Grounded;
                            self.ik_controller.disable_motors().await?;
                            tokio::time::sleep(Duration::from_millis(500)).await;
                            self.dance_moves.clear();
                            self.read_current_pose().await?;
                        }
                        error if error.is_recoverable_driver_error() => {
                            info!("Error is recoverable. Restarting controller");
                            self.attempt_motor_recovery().await?;
                        }
                        error => {
                            error!("Error is not recoverable. Stopping controller");
                            return Err(error);
                        }
                    }
                }
                Ok(()) => {
                    info!("Control loop exited cleanly. Stopping controller");
                    return Ok(());
                }
            }
        }
    }

    async fn attempt_motor_recovery(&mut self) -> HopperResult<()> {
        const MAX_ATTEMPTS: usize = 30;
        for attempts in 0..MAX_ATTEMPTS {
            warn!(attempts, "attempting  to recover motors");
            if let Ok(()) = self.attempt_motor_recovery_internal().await {
                return Ok(());
            }
            tokio::time::sleep(Duration::from_millis(200)).await;
        }
        error!("Failed to recover motors after {} attempts", MAX_ATTEMPTS);
        Ok(())
    }

    async fn attempt_motor_recovery_internal(&mut self) -> HopperResult<()> {
        self.flush_and_clear_motors().await?;
        self.scan_motors().await?;
        self.read_current_pose().await?;
        Ok(())
    }

    async fn stand_up(&mut self) -> HopperResult<()> {
        self.read_current_pose().await?;
        self.transition_direct(
            &[&self.last_written_pose.clone(), stance::grounded_stance()],
            0.005,
        )
        .await?;
        self.transition_direct(
            &[stance::grounded_stance(), stance::relaxed_wide_stance()],
            0.003,
        )
        .await?;
        self.transition_step(&[stance::relaxed_wide_stance(), stance::relaxed_stance()])
            .await?;
        Ok(())
    }

    async fn sit_down(&mut self) -> HopperResult<()> {
        self.transition_step(&[stance::relaxed_stance(), stance::relaxed_wide_stance()])
            .await?;
        self.transition_direct(
            &[stance::relaxed_wide_stance(), stance::grounded_stance()],
            MAX_MOVE,
        )
        .await?;
        self.ik_controller.disable_motors().await?;
        Ok(())
    }

    /// Move over sequence of LegPositions using direct motions
    async fn transition_direct(
        &mut self,
        states: &[&LegPositions],
        stride: f32,
    ) -> HopperResult<()> {
        let mut interval = time::interval(TICK_DURATION);
        for pose in states {
            for new_pose in self.last_written_pose.to_move_towards_iter(pose, stride) {
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
    async fn transition_step(&mut self, states: &[&LegPositions]) -> HopperResult<()> {
        let mut interval = time::interval(TICK_DURATION);
        for pose in states {
            self.last_tripod.invert();
            let target = self
                .last_written_pose
                .merge_with(pose, self.last_tripod.as_flag());
            for new_pose in StepIterator::step(
                self.last_written_pose,
                target,
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
                self.last_written_pose,
                target,
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

    async fn scan_motors(&mut self) -> HopperResult<()> {
        self.ik_controller.scan_motors().await?;
        Ok(())
    }

    async fn flush_and_clear_motors(&mut self) -> HopperResult<()> {
        self.ik_controller.flush_and_clear_motors().await?;
        Ok(())
    }

    async fn handle_body_state_transition(&mut self) -> HopperResult<()> {
        if self.state != self.command.body_state {
            info!(
                "Transitioning body state from {:?} to {:?}",
                self.state, self.command.body_state
            );
        }
        match (self.state, self.command.body_state) {
            (BodyState::Folded, BodyState::Grounded) => {
                IocContainer::global_instance()
                    .service::<FaceController>()?
                    .larson_scanner(hopper_face::driver::PURPLE)?;
                FoldingManager::new(&mut self.ik_controller)
                    .await?
                    .unfold_on_ground()
                    .await?;
                self.state = BodyState::Grounded;
            }
            (BodyState::Folded, BodyState::Standing) => {
                IocContainer::global_instance()
                    .service::<FaceController>()?
                    .larson_scanner(hopper_face::driver::PURPLE)?;
                FoldingManager::new(&mut self.ik_controller)
                    .await?
                    .unfold_on_ground()
                    .await?;
                self.stand_up().await?;
                self.state = BodyState::Standing;
            }
            (BodyState::Grounded, BodyState::Standing) => {
                self.stand_up().await?;
                self.state = BodyState::Standing;
            }
            (BodyState::Standing, BodyState::Grounded) => {
                self.sit_down().await?;
                self.state = BodyState::Grounded;
            }
            (BodyState::Standing, BodyState::Folded) => {
                self.sit_down().await?;
                FoldingManager::new(&mut self.ik_controller)
                    .await?
                    .fold_on_ground()
                    .await?;
                IocContainer::global_instance()
                    .service::<FaceController>()?
                    .off()?;
                self.state = BodyState::Folded;
            }
            (BodyState::Grounded, BodyState::Folded) => {
                FoldingManager::new(&mut self.ik_controller)
                    .await?
                    .fold_on_ground()
                    .await?;
                IocContainer::global_instance()
                    .service::<FaceController>()?
                    .off()?;
                self.state = BodyState::Folded;
            }
            (BodyState::Grounded, BodyState::Grounded) => {
                // do nothing
            }
            (BodyState::Standing, BodyState::Standing) => {
                // do nothing
            }
            (BodyState::Folded, BodyState::Folded) => {
                // do nothing
            }
        }
        Ok(())
    }

    async fn control_loop(&mut self) -> HopperResult<()> {
        let mut interval = time::interval(TICK_DURATION);

        loop {
            match self.command_receiver.try_recv() {
                Ok(Some(command)) => self.command = command,
                Err(_) => {
                    warn!("Motion command sender gone. Exiting control loop");
                    break;
                }
                _ => (),
            }

            let blocking_command = self.blocking_command_receiver.try_recv_optional().unwrap();
            if let Some(blocking_command) = blocking_command {
                match blocking_command {
                    BlockingCommand::Terminate => {
                        info!("Terminate command received. Exiting control loop");
                        break;
                    }
                    BlockingCommand::Choreography(dance_move) => {
                        let moves: Vec<_> = dance_move.to_iterator(self.base_relaxed).collect();
                        if self.dance_moves.is_empty() {
                            self.dance_moves = moves;
                        } else {
                            warn!("Already executing a dance move");
                        }
                    }
                    BlockingCommand::DisableMotors => {
                        self.ik_controller.disable_motors().await?;
                        continue;
                    }
                    BlockingCommand::SetCompliance(compliance) => {
                        self.ik_controller
                            .set_body_compliance_slope(compliance)
                            .await?;
                        continue;
                    }
                    BlockingCommand::SetMotorSpeed(speed) => {
                        self.ik_controller.set_body_motor_speed(speed).await?;
                        continue;
                    }
                }
            }

            self.handle_body_state_transition().await?;

            if self.last_voltage_read.elapsed() > VOLTAGE_READ_PERIOD {
                self.last_voltage_read = Instant::now();
                // match self.ik_controller.read_mean_voltage().await {
                //     Ok(voltage) => {
                //         // TODO(David): Figure out a better way to propagate this
                //         debug!("Voltage is {}", voltage)
                //     }
                //     Err(error) => error!("Failed to read voltage: {}", error),
                // }
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
                        self.last_written_pose,
                        target,
                        self.move_duration,
                        STEP_HEIGHT,
                        GROUNDED_STEP_HEIGHT,
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
                    if let Some(dance_move) = self.dance_moves.pop() {
                        self.ik_controller.move_to_positions(&dance_move).await?;
                        self.last_written_pose = dance_move;
                    }
                }
            }
            interval.tick().await;
            self.control_loop_rate_tracker.tick();
            if let Some(report) = self.control_loop_rate_tracker.report().await? {
                debug!(?report, "motor move rate");
            }
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
        let angle = self.angle_to(target);
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
