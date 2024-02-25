mod choreographer;
pub mod folding;
pub mod stance;
#[cfg(feature = "visualizer")]
pub mod visualizer;
pub mod walking;

use crate::body_controller::motor_controller::{HexapodCompliance, HexapodMotorSpeed};
use crate::error::{HopperError, HopperResult};
use crate::face::FaceController;
use crate::hexapod::LegFlags;
use crate::high_five::{HighFiveCommand, HighFiveDetector};
use crate::ik_controller::{
    leg_positions::{LegPositions, MoveTowards},
    IkControllable,
};
use crate::ioc_container::IocContainer;
use crate::speech::SpeechService;
use crate::utilities::{MpscChannelHelper, RateTracker};
pub use choreographer::DanceMove;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use std::collections::VecDeque;
use std::time::Duration;
use std::{sync::mpsc, time::Instant};
use tokio::sync::mpsc::Receiver;
use tokio::time;
use tokio::{spawn, task::JoinHandle};
use tracing::*;
use walking::*;

use self::choreographer::Choreographer;
use self::folding::FoldingManager;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyState {
    Standing,
    Grounded,
    Folded,
}

pub struct MotionController {
    command_sender: last_message_channel::Sender<MotionControllerCommand>,
    blocking_command_sender: mpsc::Sender<BlockingCommand>,
    command: MotionControllerCommand,
    _handle: JoinHandle<anyhow::Result<()>>,
}

impl MotionController {
    pub async fn new(
        ik_controller: Box<dyn IkControllable>,
        control_loop_rate_tracker: RateTracker,
        high_five_receiver: Receiver<HighFiveCommand>,
    ) -> HopperResult<Self> {
        let (command_sender, receiver) = last_message_channel::latest_message_channel();
        let command = MotionControllerCommand::default();

        let (blocking_command_sender, blocking_command_receiver) = mpsc::channel();

        let motion_controller_loop = MotionControllerLoop::new(
            ik_controller,
            receiver,
            blocking_command_receiver,
            control_loop_rate_tracker,
            high_five_receiver,
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
        self.command.body_state = Some(state);
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

    pub fn set_single_leg_command(&mut self, single_leg_command: SingleLegCommand) {
        self.command.single_leg_mode_command = Some(single_leg_command);
        self.command_sender.send(self.command.clone()).unwrap();
    }

    pub fn clear_single_leg_command(&mut self) {
        self.command.single_leg_mode_command = None;
        self.command_sender.send(self.command.clone()).unwrap();
    }

    pub fn create_dance_service(&self) -> DanceService {
        DanceService {
            blocking_command_sender: self.blocking_command_sender.clone(),
        }
    }
}

impl Drop for MotionController {
    fn drop(&mut self) {
        self.blocking_command_sender
            .send(BlockingCommand::Terminate)
            .expect("Failed to send termination command");
    }
}

pub struct DanceService {
    blocking_command_sender: mpsc::Sender<BlockingCommand>,
}

impl DanceService {
    pub fn start_sequence(&self, dance_move: DanceMove) {
        self.blocking_command_sender
            .send(BlockingCommand::Choreography(dance_move))
            .unwrap();
    }
}

#[derive(Debug, Clone, Default)]
struct MotionControllerCommand {
    move_command: MoveCommand,
    body_state: Option<BodyState>,
    linear_translation: Vector3<f32>,
    body_rotation: UnitQuaternion<f32>,
    single_leg_mode_command: Option<SingleLegCommand>,
}

#[derive(Debug, Clone, Copy)]
pub struct SingleLegCommand {
    leg: LegFlags,
    translation: Vector3<f32>,
}

impl SingleLegCommand {
    pub fn new(leg: LegFlags, translation: Vector3<f32>) -> Self {
        Self { leg, translation }
    }
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
const NON_WALK_STEP_HEIGHT: f32 = 0.03;
const GROUNDED_STEP_HEIGHT: f32 = -0.0;
const VOLTAGE_READ_PERIOD: Duration = Duration::from_millis(1000);
const HARDWARE_ERROR_SOUND_TIMEOUT: Duration = Duration::from_secs(30);
// const MOVE_DURATION: Duration = Duration::from_millis(400);
// TODO(David): this results in smoother motion
// Step time should probably be a function of time?
// const MOVE_DURATION: Duration = Duration::from_millis(700);

struct MotionControllerLoop {
    ik_controller: Box<dyn IkControllable>,
    command_receiver: last_message_channel::Receiver<MotionControllerCommand>,
    blocking_command_receiver: mpsc::Receiver<BlockingCommand>,
    command: MotionControllerCommand,
    state: BodyState,
    last_tripod: Tripod,
    last_written_pose: LegPositions,
    current_rotation: UnitQuaternion<f32>,
    current_translation: Vector3<f32>,
    base_relaxed: LegPositions,
    lrl_reset: bool,
    rlr_reset: bool,
    last_voltage_read: Instant,
    dance_moves: VecDeque<DanceMove>,
    control_loop_rate_tracker: RateTracker,
    was_single_leg_mode: bool,
    high_five_receiver: Receiver<HighFiveCommand>,
    last_hardware_error_sound_player: Instant,
}

impl MotionControllerLoop {
    async fn new(
        mut ik_controller: Box<dyn IkControllable>,
        command_receiver: last_message_channel::Receiver<MotionControllerCommand>,
        blocking_command_receiver: mpsc::Receiver<BlockingCommand>,
        control_loop_rate_tracker: RateTracker,
        high_five_receiver: Receiver<HighFiveCommand>,
    ) -> HopperResult<Self> {
        let last_written_pose = ik_controller.read_leg_positions().await?;
        Ok(Self {
            ik_controller,
            command_receiver,
            blocking_command_receiver,
            command: MotionControllerCommand::default(),
            state: BodyState::Grounded,
            last_tripod: Tripod::LRL,
            last_written_pose,
            current_rotation: UnitQuaternion::identity(),
            current_translation: Vector3::zeros(),
            base_relaxed: *stance::relaxed_stance(),
            lrl_reset: false,
            rlr_reset: false,
            last_voltage_read: Instant::now(),
            dance_moves: VecDeque::new(),
            control_loop_rate_tracker,
            was_single_leg_mode: false,
            high_five_receiver,
            last_hardware_error_sound_player: Instant::now(),
        })
    }

    /// Attempt to estimate current body state
    async fn estimate_current_body_state(&mut self) -> HopperResult<BodyState> {
        self.last_written_pose = self.ik_controller.read_leg_positions().await?;

        let z_heights: Vec<_> = self
            .last_written_pose
            .as_legs()
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
            Ok(BodyState::Standing)
        } else {
            // if grounded check if we are likely folded
            let folded = FoldingManager::new(&mut self.ik_controller)
                .await?
                .check_if_folded();
            if folded {
                Ok(BodyState::Folded)
            } else {
                Ok(BodyState::Grounded)
            }
        }
    }

    async fn initialize_body_state(&mut self) -> HopperResult<()> {
        let estimate = self.estimate_current_body_state().await?;
        info!("Estimated body state to be {:?}", estimate);
        self.state = estimate;
        self.command.body_state = Some(estimate);
        Ok(())
    }

    async fn read_current_pose(&mut self) -> HopperResult<()> {
        self.last_written_pose = self.ik_controller.read_leg_positions().await?;
        Ok(())
    }

    async fn run(mut self) -> anyhow::Result<()> {
        tracing::info!("Running motion controller");
        loop {
            match self.control_loop().await {
                Err(error) => {
                    error!("Control loop error {}", error);
                    match error {
                        HopperError::GenericIkError => {
                            warn!("Error is generic IK error. Restarting");

                            IocContainer::global_instance()
                                .service::<SpeechService>()?
                                .play_sound("hopper_sounds/ik_failure.wav")
                                .await?;

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
                            self.play_hardware_error_sound().await?;
                            self.attempt_motor_recovery().await?;
                        }
                        error => {
                            error!("Error is not recoverable. Stopping controller");
                            IocContainer::global_instance()
                                .service::<SpeechService>()?
                                .say_eleven_with_default_voice(
                                    "Motion controller loop encountered an unrecoverable error",
                                )
                                .await?;
                            return Err(error.into());
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
        warn!("Attempting motor recovery");
        self.clear_serial_io_buffers().await?;
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
        interval.set_missed_tick_behavior(time::MissedTickBehavior::Skip);
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
        interval.set_missed_tick_behavior(time::MissedTickBehavior::Skip);
        for pose in states {
            self.last_tripod.invert();
            let target = self
                .last_written_pose
                .merge_with(pose, self.last_tripod.as_flag());
            for new_pose in StepIterator::step(
                self.last_written_pose,
                target,
                MAX_MOVE,
                NON_WALK_STEP_HEIGHT,
                self.last_tripod,
                false,
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
                NON_WALK_STEP_HEIGHT,
                self.last_tripod,
                false,
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

    async fn clear_serial_io_buffers(&mut self) -> HopperResult<()> {
        self.ik_controller.clear_serial_io_buffers().await?;
        Ok(())
    }

    async fn handle_body_state_transition(&mut self) -> HopperResult<()> {
        let desired_state = if let Some(desired_state) = self.command.body_state {
            desired_state
        } else {
            return Ok(());
        };
        if self.state != desired_state {
            info!(
                "Transitioning body state from {:?} to {:?}",
                self.state, desired_state
            );
        }
        match (self.state, desired_state) {
            (BodyState::Folded, BodyState::Grounded) => {
                IocContainer::global_instance()
                    .service::<FaceController>()?
                    .larson_scanner(crate::face::driver::PURPLE)?;
                FoldingManager::new(&mut self.ik_controller)
                    .await?
                    .unfold_on_ground()
                    .await?;
                self.state = BodyState::Grounded;
            }
            (BodyState::Folded, BodyState::Standing) => {
                IocContainer::global_instance()
                    .service::<FaceController>()?
                    .larson_scanner(crate::face::driver::PURPLE)?;
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
        tracing::info!("Starting control loop");
        // try to estimate current body state
        self.initialize_body_state().await?;

        let mut interval = time::interval(TICK_DURATION);
        interval.set_missed_tick_behavior(time::MissedTickBehavior::Skip);
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
                        self.dance_moves.push_back(dance_move);
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

            // measure voltage only if not walking
            if !self.command.move_command.should_move()
                && self.last_voltage_read.elapsed() > VOLTAGE_READ_PERIOD
            {
                self.last_voltage_read = Instant::now();
                match self.ik_controller.read_mean_voltage().await {
                    Ok(voltage) => {
                        // TODO(David): Figure out a better way to propagate this
                        tracing::debug!("Voltage is {}", voltage)
                    }
                    Err(error) => {
                        error!(?error, "Failed to read voltage: {:?}", error);
                        self.play_hardware_error_sound().await?;
                        warn!("Flushing motors");
                        self.ik_controller.clear_serial_io_buffers().await?;
                    }
                }
            }

            // only walk if standing
            if self.state == BodyState::Standing {
                // only do high fives if standing
                let high_five_command = self.high_five_receiver.try_recv();
                if let Ok(high_five_command) = high_five_command {
                    HighFiveDetector::execute_high_five(
                        &mut self.ik_controller,
                        self.base_relaxed.to_owned(),
                        high_five_command,
                    )
                    .await?;
                }

                if let Some(single_leg_command) = self.command.single_leg_mode_command {
                    // single leg mode
                    let new_position = single_leg_command_handler(
                        &mut self.ik_controller,
                        self.base_relaxed,
                        single_leg_command,
                    )
                    .await?;
                    self.last_written_pose = new_position;
                    interval.tick().await;
                    self.was_single_leg_mode = true;
                    continue;
                } else if self.was_single_leg_mode {
                    // recover from single leg mode
                    self.was_single_leg_mode = false;
                    let relaxed = self.transformed_relaxed();
                    self.transition_direct(&[&self.last_written_pose.clone(), &relaxed], 0.005)
                        .await?;
                    continue;
                }

                if self.command.move_command.should_move() || self.should_reset_legs() {
                    self.last_tripod.invert();
                    let target = step_with_relaxed_transformation(
                        &self.last_written_pose,
                        &self.transformed_relaxed(),
                        &self.last_tripod,
                        self.command.move_command,
                    );
                    for new_pose in TimedStepIterator::step(
                        self.last_written_pose,
                        target,
                        self.command.move_command.step_time(),
                        self.command.move_command.step_height(),
                        GROUNDED_STEP_HEIGHT,
                        self.last_tripod,
                        self.command.move_command.aggressive_leg_lift(),
                    ) {
                        self.shift_transformation();
                        self.ik_controller.move_to_positions(&new_pose).await?;
                        self.last_written_pose = new_pose;
                        self.control_loop_rate_tracker.tick();
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
                    // shift transformation
                    self.shift_transformation();
                    // we can do transformations here
                    let transformed_pose = self.transformed_relaxed();
                    if self.last_written_pose != transformed_pose {
                        self.ik_controller
                            .move_to_positions(&transformed_pose)
                            .await?;
                        self.last_written_pose = transformed_pose;
                    }
                    if let Some(dance_move) = self.dance_moves.pop_front() {
                        let transformed_relaxed = self.transformed_relaxed();
                        Choreographer::new(&mut self.ik_controller, transformed_relaxed)?
                            .execute_move(dance_move)
                            .await?;
                    }
                    // sleep if not walking
                    self.control_loop_rate_tracker.tick();
                    interval.tick().await;
                }
            } else {
                // sleep if not standing
                self.control_loop_rate_tracker.tick();
                interval.tick().await;
            }
            if let Some(report) = self.control_loop_rate_tracker.report().await? {
                debug!(?report, "motor move rate");
            }
        }
        Ok(())
    }

    async fn play_hardware_error_sound(&mut self) -> HopperResult<()> {
        if self.last_hardware_error_sound_player.elapsed() > HARDWARE_ERROR_SOUND_TIMEOUT {
            self.last_hardware_error_sound_player = Instant::now();
            IocContainer::global_instance()
                .service::<SpeechService>()?
                .play_sound(
                    "premium_beat_sounds/sounds/PremiumBeat_0046_sci_fi_beep_electric_4.wav",
                )
                .await?;
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

async fn single_leg_command_handler(
    ik_controller: &mut Box<dyn IkControllable>,
    relaxed_positions: LegPositions,
    single_leg_command: SingleLegCommand,
) -> HopperResult<LegPositions> {
    let selected_legs = relaxed_positions.selected_legs(single_leg_command.leg);

    let target_positions: Vec<_> = selected_legs
        .iter()
        .map(|leg_position| {
            Point3::new(
                leg_position.x * 1.3,
                leg_position.y * 1.3,
                leg_position.z + 0.05,
            ) + single_leg_command.translation
        })
        .collect();

    let selected_legs = relaxed_positions.selected_legs(single_leg_command.leg);
    // slightly rotate away from selected leg
    let (x, y) = if selected_legs.len() == 1 {
        selected_legs
            .first()
            .map(|point| point.coords.normalize())
            .map(|vector| (vector.x, vector.y))
            .unwrap_or_default()
    } else {
        (0.0, 0.0)
    };

    let rotation =
        UnitQuaternion::from_euler_angles(-3_f32.to_radians() * y, 3_f32.to_radians() * x, 0.0);

    let mut desired_position =
        relaxed_positions.transform(Vector3::new(0_f32, 0_f32, -0.03_f32), rotation);

    desired_position.updated_from_selected_legs(&target_positions, single_leg_command.leg)?;

    ik_controller.move_to_positions(&desired_position).await?;

    Ok(desired_position)
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
