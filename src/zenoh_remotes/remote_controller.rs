use crate::body_controller::motor_controller::{HexapodCompliance, HexapodMotorSpeed};
use crate::error::HopperResult;
use crate::hexapod::LegFlags;
use crate::high_five::HighFiveServiceController;
use crate::ioc_container::IocContainer;
use crate::lidar::LidarServiceController;
use crate::motion_controller::walking::{
    DEFAULT_STEP_DISTANCE, DEFAULT_STEP_HEIGHT, DEFAULT_STEP_TIME,
};
use crate::motion_controller::{self, SingleLegCommand};
use crate::speech::SpeechService;
use crate::zenoh_remotes::topic_consts::{
    BODY_MOTOR_SPEED_SUBSCRIBER, COMPLIANCE_SLOPE_SUBSCRIBER, HOPPER_WALKING_CONFIG_PUBLISHER,
    REMOTE_CONTROL_SUBSCRIBER, STANCE_SUBSCRIBER, WALKING_CONFIG_SUBSCRIBER,
};
use crate::{error::HopperError, motion_controller::walking::MoveCommand};
use chrono::{DateTime, Utc};
use gilrs::{GilrsBuilder, PowerInfo};
use nalgebra::{UnitQuaternion, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::time::Duration;
use std::{collections::BTreeMap, sync::Arc};
use tokio::sync::mpsc::{channel, Receiver, Sender};
use tracing::*;
use zenoh::prelude::r#async::*;

pub enum ScheduledCommand {
    MoveCommand(MoveCommand),
    WaitCommand(Duration),
}

pub struct MoveService {
    sender: tokio::sync::mpsc::Sender<MoveCommand>,
    scheduled_sender: tokio::sync::mpsc::Sender<ScheduledCommand>,
}

impl MoveService {
    pub fn new() -> (Self, tokio::sync::mpsc::Receiver<MoveCommand>) {
        let (sender, receiver) = tokio::sync::mpsc::channel(10);
        let (scheduled_sender, mut scheduled_receiver) = tokio::sync::mpsc::channel(10);

        tokio::spawn({
            let command_sender = sender.clone();
            async move {
                while let Some(command) = scheduled_receiver.recv().await {
                    match command {
                        ScheduledCommand::MoveCommand(move_command) => {
                            info!("Executing scheduled move command {:?}", move_command);
                            command_sender.send(move_command).await.unwrap();
                        }
                        ScheduledCommand::WaitCommand(time) => {
                            info!("Executing scheduled sleep {:?}", time);
                            tokio::time::sleep(time).await
                        }
                    }
                }
            }
        });

        (
            Self {
                sender,
                scheduled_sender,
            },
            receiver,
        )
    }

    pub async fn send_move(&self, command: MoveCommand) -> anyhow::Result<()> {
        self.sender.send(command).await?;
        Ok(())
    }

    pub async fn schedule_move(&self, command: ScheduledCommand) -> anyhow::Result<()> {
        self.scheduled_sender.send(command).await?;
        Ok(())
    }
}

pub async fn simple_zenoh_controller(
    motion_controller: &mut motion_controller::MotionController,
    zenoh_session: Arc<zenoh::Session>,
    mut move_command_receiver: tokio::sync::mpsc::Receiver<MoveCommand>,
) -> anyhow::Result<()> {
    info!("Starting simple zenoh controller");
    let stance_subscriber = zenoh_session
        .declare_subscriber(STANCE_SUBSCRIBER)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let gamepad_subscriber = zenoh_session
        .declare_subscriber(REMOTE_CONTROL_SUBSCRIBER)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let walking_config_subscriber = zenoh_session
        .declare_subscriber(WALKING_CONFIG_SUBSCRIBER)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let compliance_slope_subscriber = zenoh_session
        .declare_subscriber(COMPLIANCE_SLOPE_SUBSCRIBER)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let body_motor_speed_subscriber = zenoh_session
        .declare_subscriber(BODY_MOTOR_SPEED_SUBSCRIBER)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let mut controller_reader = start_controller_reader();

    let mut last_gamepad_message: Option<InputMessage> = None;

    let mut gamepad_controller = GamepadController::default();
    gamepad_controller
        .publish_walking_config(&zenoh_session)
        .await?;

    loop {
        tokio::select! {
            sample = stance_subscriber.recv_async() => {
                trace!("got new message");
                let sample = sample?;
                handle_stance_command(sample, motion_controller).await?;
            }
            sample = compliance_slope_subscriber.recv_async() => {
                let sample = sample?;
                handle_compliance_slope_command(sample, motion_controller).await?;
            }
            sample = body_motor_speed_subscriber.recv_async() => {
                let sample = sample?;
                handle_body_motor_speed_command(sample, motion_controller).await?;
            }
            sample = walking_config_subscriber.recv_async() => {
                let sample = sample?;
                gamepad_controller.handle_walking_config_update(sample).await?;
                gamepad_controller.publish_walking_config(&zenoh_session).await?;
            }
            gamepad_message = gamepad_subscriber.recv_async() => {
                trace!("got new gamepad message");
                let gamepad_message = gamepad_message?;
                gamepad_controller.handle_gamepad_command_zenoh(gamepad_message, motion_controller, &mut last_gamepad_message).await?;
            }
            controller_message = controller_reader.recv() => {
                trace!("got new controller message");
                if let Some(controller_message) = controller_message {
                    gamepad_controller.handle_gamepad_command(controller_message, motion_controller, &mut last_gamepad_message).await?;
                }
            }
            move_command = move_command_receiver.recv() => {
                if let Some(move_command) = move_command {
                    motion_controller.set_command(move_command);
                }
            }
            _ = tokio::signal::ctrl_c() => {
                info!("Got ctrl-c");
                break;
            }
        }
    }
    info!("Exiting control loop");
    Ok(())
}

async fn handle_stance_command(
    message: zenoh::sample::Sample,
    controller: &mut motion_controller::MotionController,
) -> anyhow::Result<()> {
    let command: String = message.value.try_into()?;
    if &command.to_lowercase() == "stand" {
        controller.set_body_state(motion_controller::BodyState::Standing);
    } else if &command.to_lowercase() == "ground" {
        controller.set_body_state(motion_controller::BodyState::Grounded);
    } else if &command.to_lowercase() == "folded" {
        controller.set_body_state(motion_controller::BodyState::Folded);
    } else if &command.to_lowercase() == "happy" {
        controller.start_sequence(motion_controller::DanceMove::HappyDance);
    } else if &command.to_lowercase() == "wave" {
        controller.start_sequence(motion_controller::DanceMove::WaveHi);
    } else if &command.to_lowercase() == "random" {
        controller.start_sequence(motion_controller::DanceMove::Random);
    } else if &command.to_lowercase() == "roar" {
        controller.start_sequence(motion_controller::DanceMove::Roar);
    } else if &command.to_lowercase() == "sad_emote" {
        controller.start_sequence(motion_controller::DanceMove::SadEmote);
    } else if &command.to_lowercase() == "combat_cry" {
        controller.start_sequence(motion_controller::DanceMove::CombatCry);
    } else {
        error!("Unknown command {}", command);
    }
    Ok(())
}

async fn handle_compliance_slope_command(
    message: zenoh::sample::Sample,
    controller: &mut motion_controller::MotionController,
) -> anyhow::Result<()> {
    let command: String = message.value.try_into()?;
    let command: HexapodCompliance = serde_json::from_str(&command)?;
    controller.set_body_compliance_slpe(command);
    Ok(())
}

async fn handle_body_motor_speed_command(
    message: zenoh::sample::Sample,
    controller: &mut motion_controller::MotionController,
) -> anyhow::Result<()> {
    let command: String = message.value.try_into()?;
    let command: HexapodMotorSpeed = serde_json::from_str(&command)?;
    controller.set_body_motor_speed(command);
    Ok(())
}

#[derive(Debug)]
struct GamepadController {
    walking_config: WalkingConfig,
    single_leg_foot: LegFlags,
    feet_iterator: std::iter::Cycle<std::vec::IntoIter<LegFlags>>,
    height_offset: f32,
    last_gamepad_event_time: DateTime<Utc>,
}

impl Default for GamepadController {
    fn default() -> Self {
        let feet_iterator = vec![
            LegFlags::LEFT_FRONT,
            LegFlags::RIGHT_FRONT,
            LegFlags::LEFT_MIDDLE,
            LegFlags::RIGHT_MIDDLE,
            LegFlags::LEFT_REAR,
            LegFlags::RIGHT_REAR,
            LegFlags::MIDDLE,
            LegFlags::LRL_TRIPOD,
            LegFlags::RLR_TRIPOD,
        ]
        .into_iter()
        .cycle();

        Self {
            walking_config: WalkingConfig::default(),
            single_leg_foot: LegFlags::LEFT_FRONT,
            feet_iterator,
            height_offset: 0.0,
            last_gamepad_event_time: Utc::now(),
        }
    }
}

impl GamepadController {
    async fn handle_walking_config_update(
        &mut self,
        message: zenoh::sample::Sample,
    ) -> anyhow::Result<()> {
        let message: String = message.value.try_into()?;
        let message: WalkingConfigMsg = serde_yaml::from_str(&message)?;
        self.walking_config.update_from_msg(message);
        info!("Updated walking config: {:?}", self.walking_config);
        Ok(())
    }

    async fn publish_walking_config(&self, zenoh_session: &zenoh::Session) -> HopperResult<()> {
        let message = serde_yaml::to_string(&self.walking_config).unwrap();
        zenoh_session
            .put(HOPPER_WALKING_CONFIG_PUBLISHER, message)
            .res()
            .await
            .map_err(HopperError::ZenohError)?;
        Ok(())
    }

    async fn handle_gamepad_command_zenoh(
        &mut self,
        gamepad_message: zenoh::sample::Sample,
        controller: &mut motion_controller::MotionController,
        last_gamepad_message: &mut Option<InputMessage>,
    ) -> anyhow::Result<()> {
        let gamepad_message: String = gamepad_message.value.try_into()?;
        let gamepad_message: InputMessage = serde_json::from_str(&gamepad_message)?;
        self.handle_gamepad_command(gamepad_message, controller, last_gamepad_message)
            .await
    }

    async fn handle_gamepad_command(
        &mut self,
        input_message: InputMessage,
        controller: &mut motion_controller::MotionController,
        last_input_message: &mut Option<InputMessage>,
    ) -> anyhow::Result<()> {
        let found = input_message
            .gamepads
            .iter()
            .max_by_key(|(_, gamepad)| gamepad.last_event_time);

        let (index, gamepad_message) = if let Some((index, gamepad_message)) = found {
            (index, gamepad_message)
        } else {
            return Ok(());
        };

        // skip outdated messages
        if gamepad_message.last_event_time <= self.last_gamepad_event_time {
            return Ok(());
        };
        self.last_gamepad_event_time = gamepad_message.last_event_time;

        let last_gamepad_message = last_input_message
            .as_ref()
            .and_then(|input_message| input_message.gamepads.get(index));

        let a_pressed = was_button_pressed_since_last_time(
            Button::South,
            gamepad_message,
            last_gamepad_message,
        );
        let b_pressed =
            was_button_pressed_since_last_time(Button::East, gamepad_message, last_gamepad_message);
        let y_pressed = was_button_pressed_since_last_time(
            Button::North,
            gamepad_message,
            last_gamepad_message,
        );
        let x_pressed =
            was_button_pressed_since_last_time(Button::West, gamepad_message, last_gamepad_message);
        let select_pressed = was_button_pressed_since_last_time(
            Button::Select,
            gamepad_message,
            last_gamepad_message,
        );
        let start_pressed = was_button_pressed_since_last_time(
            Button::Start,
            gamepad_message,
            last_gamepad_message,
        );
        let lt_pressed = was_button_pressed_since_last_time(
            Button::LeftTrigger2,
            gamepad_message,
            last_gamepad_message,
        );
        let left_paddle_pressed = was_button_pressed_since_last_time(
            Button::LeftPaddle,
            gamepad_message,
            last_gamepad_message,
        );
        let right_paddle_pressed = was_button_pressed_since_last_time(
            Button::RightPaddle,
            gamepad_message,
            last_gamepad_message,
        );

        if lt_pressed {
            self.single_leg_foot = self.feet_iterator.next().unwrap();
            info!("Switching to single leg foot {:?}", self.single_leg_foot);
        }

        if was_button_pressed_since_last_time(Button::DPadUp, gamepad_message, last_gamepad_message)
        {
            self.height_offset += 0.01;
            info!("Increasing height offset to {}", self.height_offset);
        }
        if was_button_pressed_since_last_time(
            Button::DPadDown,
            gamepad_message,
            last_gamepad_message,
        ) {
            self.height_offset -= 0.01;
            info!("Decreased height offset to {}", self.height_offset);
        }
        if was_button_pressed_since_last_time(
            Button::DPadRight,
            gamepad_message,
            last_gamepad_message,
        ) {
            self.height_offset = 0.0;
            info!("Resetting height offset");
        }

        if was_button_pressed_since_last_time(
            Button::DPadLeft,
            gamepad_message,
            last_gamepad_message,
        ) {
            let high_five_controller =
                IocContainer::global_instance().service::<HighFiveServiceController>()?;
            let is_active = high_five_controller.is_active();
            let desired_state = !is_active;
            info!("Toggling high five controller to {}", desired_state);
            high_five_controller.set_active(desired_state);
            IocContainer::global_instance()
                .service::<LidarServiceController>()?
                .set_active(desired_state);
        }

        // clamp
        self.height_offset = self.height_offset.max(-0.03).min(0.05);

        let lb_down = is_button_down(Button::LeftTrigger, gamepad_message);
        let rb_down = is_button_down(Button::RightTrigger, gamepad_message);
        let lt_down = is_button_down(Button::LeftTrigger2, gamepad_message);
        let rt_down = is_button_down(Button::RightTrigger2, gamepad_message);

        if a_pressed {
            info!("Setting stance to standing");
            controller.set_body_state(motion_controller::BodyState::Standing);
        }
        if y_pressed {
            info!("Starting happy dance");
            controller.start_sequence(motion_controller::DanceMove::HappyDance);
        }
        if x_pressed {
            info!("Starting wave");
            controller.start_sequence(motion_controller::DanceMove::WaveHiWithSound);
        }
        if select_pressed {
            info!("Folding");
            controller.set_body_state(motion_controller::BodyState::Folded);
        }
        if start_pressed {
            info!("Grounding");
            controller.set_body_state(motion_controller::BodyState::Grounded);
        }
        if b_pressed {
            info!("Starting random dance");
            controller.start_sequence(motion_controller::DanceMove::Random);
        }
        if left_paddle_pressed {
            self.walking_config.max_step_distance_m = 0.03;
            self.walking_config.step_time = Duration::from_millis(400);
            tokio::spawn(async move {
                _ = IocContainer::global_instance()
                    .service::<SpeechService>()
                    .expect("Failed to get speech service")
                    .say_eleven_with_default_voice("Fast walking mode!")
                    .await;
            });
        }
        if right_paddle_pressed {
            self.walking_config.max_step_distance_m = DEFAULT_STEP_DISTANCE;
            self.walking_config.step_time = DEFAULT_STEP_TIME;
            tokio::spawn(async move {
                _ = IocContainer::global_instance()
                    .service::<SpeechService>()
                    .expect("Failed to get speech service")
                    .say_eleven_with_default_voice("Regular walking mode!")
                    .await;
            });
        }

        if lb_down {
            // translation mode
            let x = -get_axis(Axis::LeftStickY, gamepad_message) * 0.05;
            let y = get_axis(Axis::LeftStickX, gamepad_message) * 0.05;
            let pitch = -(get_axis(Axis::RightStickY, gamepad_message) * 10.0).to_radians();
            let yaw = (get_axis(Axis::RightStickX, gamepad_message) * 10.0).to_radians();

            let translation = Vector3::new(x, y, 0.0);
            let rotation = UnitQuaternion::from_euler_angles(0.0, pitch, yaw);
            controller.set_transformation(translation, rotation);
        } else if rb_down {
            // translation mode 2
            let x = -get_axis(Axis::LeftStickY, gamepad_message) * 0.05;
            let z = -get_axis(Axis::RightStickY, gamepad_message) * 0.05;
            let roll = (get_axis(Axis::LeftStickX, gamepad_message) * 10.0).to_radians();
            let yaw = (get_axis(Axis::RightStickX, gamepad_message) * 10.0).to_radians();

            let translation = Vector3::new(x, 0.0, z);
            let rotation = UnitQuaternion::from_euler_angles(roll, 0.0, yaw);
            controller.set_transformation(translation, rotation);
        } else if rt_down {
            // walking mode
            let x = get_axis(Axis::LeftStickY, gamepad_message)
                * self.walking_config.max_step_distance_m;
            let y = -get_axis(Axis::LeftStickX, gamepad_message)
                * self.walking_config.max_step_distance_m;
            let yaw = -get_axis(Axis::RightStickX, gamepad_message)
                * self.walking_config.max_yaw_rate_deg.to_radians();

            let move_command = MoveCommand::with_optional_fields(
                Vector2::new(x, y),
                yaw,
                self.walking_config.step_time,
                self.walking_config.step_height_m,
                self.walking_config.aggressive_leg_lift,
            );

            controller.set_transformation(
                Vector3::new(0.0, 0.0, -self.height_offset),
                Default::default(),
            );
            controller.set_command(move_command);
        } else if lt_down {
            let x = get_axis(Axis::LeftStickY, gamepad_message) * 0.07;
            let y = -get_axis(Axis::LeftStickX, gamepad_message) * 0.07;
            let z = get_axis(Axis::RightStickY, gamepad_message) * 0.07;

            controller.set_single_leg_command(SingleLegCommand::new(
                self.single_leg_foot,
                Vector3::new(x, y, z),
            ));
        } else {
            controller.clear_single_leg_command();
            controller.set_transformation(
                Vector3::new(0.0, 0.0, -self.height_offset),
                Default::default(),
            );
            controller.set_command(MoveCommand::with_optional_fields(
                Vector2::zeros(),
                0.0,
                self.walking_config.step_time,
                self.walking_config.step_height_m,
                self.walking_config.aggressive_leg_lift,
            ));
        }

        *last_input_message = Some(input_message);
        Ok(())
    }
}

fn was_button_pressed_since_last_time(
    button: Button,
    gamepad_message: &GamepadMessage,
    last_gamepad_message: Option<&GamepadMessage>,
) -> bool {
    let last_gamepad_message = match last_gamepad_message {
        Some(last_gamepad_message) => last_gamepad_message,
        None => return false,
    };
    let button_down_event_counter = gamepad_message
        .button_down_event_counter
        .get(&button)
        .cloned()
        .unwrap_or_default();
    let last_button_down_event_counter = last_gamepad_message
        .button_down_event_counter
        .get(&button)
        .cloned()
        .unwrap_or_default();
    button_down_event_counter > last_button_down_event_counter
}

fn get_axis(axis: Axis, gamepad_message: &GamepadMessage) -> f32 {
    gamepad_message
        .axis_state
        .get(&axis)
        .cloned()
        .unwrap_or_default()
}

fn is_button_down(button: Button, gamepad_message: &GamepadMessage) -> bool {
    gamepad_message
        .button_pressed
        .get(&button)
        .cloned()
        .unwrap_or_default()
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
#[allow(unused)]
pub struct InputMessage {
    gamepads: BTreeMap<usize, GamepadMessage>,
    time: DateTime<Utc>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
#[allow(unused)]
pub struct GamepadMessage {
    name: String,
    button_down_event_counter: BTreeMap<Button, usize>,
    button_up_event_counter: BTreeMap<Button, usize>,
    button_pressed: BTreeMap<Button, bool>,
    axis_state: BTreeMap<Axis, f32>,
    connected: bool,
    last_event_time: DateTime<Utc>,
}

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq, Hash, PartialOrd, Ord, Clone, Copy)]
pub enum Button {
    South,
    East,
    North,
    West,
    C,
    Z,
    LeftTrigger,
    LeftTrigger2,
    RightTrigger,
    RightTrigger2,
    Select,
    Start,
    Mode,
    LeftThumb,
    RightThumb,
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    Unknown,
    LeftPaddle,
    RightPaddle,
}

impl Button {
    pub fn all_gilrs_buttons() -> &'static [gilrs::ev::Button] {
        &[
            gilrs::ev::Button::South,
            gilrs::ev::Button::East,
            gilrs::ev::Button::North,
            gilrs::ev::Button::West,
            gilrs::ev::Button::C,
            gilrs::ev::Button::Z,
            gilrs::ev::Button::LeftTrigger,
            gilrs::ev::Button::LeftTrigger2,
            gilrs::ev::Button::RightTrigger,
            gilrs::ev::Button::RightTrigger2,
            gilrs::ev::Button::Select,
            gilrs::ev::Button::Start,
            gilrs::ev::Button::Mode,
            gilrs::ev::Button::LeftThumb,
            gilrs::ev::Button::RightThumb,
            gilrs::ev::Button::DPadUp,
            gilrs::ev::Button::DPadDown,
            gilrs::ev::Button::DPadLeft,
            gilrs::ev::Button::DPadRight,
        ]
    }
}

impl From<gilrs::ev::Button> for Button {
    fn from(value: gilrs::ev::Button) -> Self {
        match value {
            gilrs::ev::Button::South => Button::South,
            gilrs::ev::Button::East => Button::East,
            gilrs::ev::Button::North => Button::North,
            gilrs::ev::Button::West => Button::West,
            gilrs::ev::Button::C => Button::C,
            gilrs::ev::Button::Z => Button::Z,
            gilrs::ev::Button::LeftTrigger => Button::LeftTrigger,
            gilrs::ev::Button::LeftTrigger2 => Button::LeftTrigger2,
            gilrs::ev::Button::RightTrigger => Button::RightTrigger,
            gilrs::ev::Button::RightTrigger2 => Button::RightTrigger2,
            gilrs::ev::Button::Select => Button::Select,
            gilrs::ev::Button::Start => Button::Start,
            gilrs::ev::Button::Mode => Button::Mode,
            gilrs::ev::Button::LeftThumb => Button::LeftThumb,
            gilrs::ev::Button::RightThumb => Button::RightThumb,
            gilrs::ev::Button::DPadUp => Button::DPadUp,
            gilrs::ev::Button::DPadDown => Button::DPadDown,
            gilrs::ev::Button::DPadLeft => Button::DPadLeft,
            gilrs::ev::Button::DPadRight => Button::DPadRight,
            gilrs::ev::Button::Unknown => Button::Unknown,
        }
    }
}

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq, Hash, PartialOrd, Ord, Clone, Copy)]
pub enum Axis {
    LeftStickX,
    LeftStickY,
    LeftZ,
    RightStickX,
    RightStickY,
    RightZ,
    DPadX,
    DPadY,
    Unknown,
}

impl From<gilrs::ev::Axis> for Axis {
    fn from(value: gilrs::ev::Axis) -> Self {
        match value {
            gilrs::ev::Axis::LeftStickX => Axis::LeftStickX,
            gilrs::ev::Axis::LeftStickY => Axis::LeftStickY,
            gilrs::ev::Axis::LeftZ => Axis::LeftZ,
            gilrs::ev::Axis::RightStickX => Axis::RightStickX,
            gilrs::ev::Axis::RightStickY => Axis::RightStickY,
            gilrs::ev::Axis::RightZ => Axis::RightZ,
            gilrs::ev::Axis::DPadX => Axis::DPadX,
            gilrs::ev::Axis::DPadY => Axis::DPadY,
            gilrs::ev::Axis::Unknown => Axis::Unknown,
        }
    }
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct WalkingConfig {
    max_step_distance_m: f32,
    step_time: Duration,
    step_height_m: f32,
    max_yaw_rate_deg: f32,
    aggressive_leg_lift: bool,
}

impl Default for WalkingConfig {
    fn default() -> Self {
        WalkingConfig {
            max_step_distance_m: DEFAULT_STEP_DISTANCE,
            step_time: DEFAULT_STEP_TIME,
            step_height_m: DEFAULT_STEP_HEIGHT,
            max_yaw_rate_deg: 15.0,
            aggressive_leg_lift: false,
        }
    }
}

impl WalkingConfig {
    pub fn update_from_msg(&mut self, msg: WalkingConfigMsg) {
        if let Some(max_step_distance_m) = msg.max_step_distance_m {
            self.max_step_distance_m = max_step_distance_m;
        }
        if let Some(step_time_ms) = msg.step_time_ms {
            self.step_time = Duration::from_millis(step_time_ms as u64);
        }
        if let Some(step_height_m) = msg.step_height_m {
            self.step_height_m = step_height_m;
        }
        if let Some(max_yaw_rate_deg) = msg.max_yaw_rate_deg {
            self.max_yaw_rate_deg = max_yaw_rate_deg;
        }
        if let Some(aggressive_leg_lift) = msg.aggressive_leg_lift {
            self.aggressive_leg_lift = aggressive_leg_lift;
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct WalkingConfigMsg {
    #[serde(default)]
    max_step_distance_m: Option<f32>,
    #[serde(default)]
    step_time_ms: Option<u32>,
    #[serde(default)]
    step_height_m: Option<f32>,
    #[serde(default)]
    max_yaw_rate_deg: Option<f32>,
    #[serde(default)]
    aggressive_leg_lift: Option<bool>,
}

fn start_controller_reader() -> Receiver<InputMessage> {
    let (tx, rx) = channel(10);
    std::thread::spawn(move || {
        let mut sender = tx;
        loop {
            if let Err(err) = controller_reader(&mut sender) {
                error!("Error in controller reader: {}", err);
            }
        }
    });
    rx
}

fn controller_reader(sender: &mut Sender<InputMessage>) -> HopperResult<()> {
    info!("Starting gamepad reader");

    let mut gilrs = GilrsBuilder::new()
        .with_default_filters(true)
        .build()
        .map_err(|err| HopperError::GilrsError(err.to_string()))?;

    info!("{} gamepad(s) found", gilrs.gamepads().count());
    for (_id, gamepad) in gilrs.gamepads() {
        info!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    let mut message_data = InputMessage {
        gamepads: BTreeMap::new(),
        time: std::time::SystemTime::now().into(),
    };

    loop {
        while let Some(gilrs_event) = gilrs.next_event() {
            let gamepad_id: usize = gilrs_event.id.into();
            let gamepad_data = message_data.gamepads.entry(gamepad_id).or_default();

            gamepad_data.last_event_time = std::time::SystemTime::now().into();
            match gilrs_event.event {
                gilrs::EventType::ButtonPressed(button, code) => {
                    *gamepad_data
                        .button_down_event_counter
                        .entry(button.into())
                        .or_default() += 1;

                    // hacky hack to support paddles on the steam controller
                    match code.into_u32() {
                        65873_u32 => {
                            *gamepad_data
                                .button_down_event_counter
                                .entry(Button::RightPaddle)
                                .or_default() += 1;
                        }
                        65872_u32 => {
                            *gamepad_data
                                .button_down_event_counter
                                .entry(Button::LeftPaddle)
                                .or_default() += 1;
                        }
                        // ignore others
                        _ => (),
                    }
                }
                gilrs::EventType::ButtonReleased(button, _) => {
                    *gamepad_data
                        .button_up_event_counter
                        .entry(button.into())
                        .or_default() += 1;
                }
                gilrs::EventType::AxisChanged(axis, value, _) => {
                    gamepad_data.axis_state.insert(axis.into(), value);
                }
                gilrs::EventType::Connected => {
                    gamepad_data.connected = true;
                    let power_info = gilrs
                        .connected_gamepad(gilrs_event.id)
                        .map(|gamepad| gamepad.power_info())
                        .unwrap_or(PowerInfo::Unknown);
                    info!(
                        "Gamepad {} - {} : {:?} connected",
                        gamepad_id, gamepad_data.name, power_info
                    )
                }
                gilrs::EventType::Disconnected => {
                    gamepad_data.connected = false;
                    warn!(
                        "Gamepad {} - {} disconnected",
                        gamepad_id, gamepad_data.name
                    )
                }
                _ => {}
            }
        }

        if let Some((gamepad_id, gamepad)) = gilrs.gamepads().next() {
            let gamepad_id: usize = gamepad_id.into();
            let gamepad_data = message_data.gamepads.entry(gamepad_id).or_default();

            gamepad_data.connected = gamepad.is_connected();
            gamepad_data.name = gamepad.name().to_string();

            // steam controller hack
            // Might need to do axis remapping for steam controller here
            // https://github.com/dmweis/Hopper_ROS/blob/229d373305078640cd88984f12a9368f8ddf2347/hopper_steamcontroller/src/controller_node.py#L37
            for (axis_code, axis_data) in gamepad.state().axes() {
                if axis_code.into_u32() == 196625 {
                    gamepad_data
                        .axis_state
                        .insert(Axis::DPadX, axis_data.value());
                }
                if axis_code.into_u32() == 196624 {
                    gamepad_data
                        .axis_state
                        .insert(Axis::DPadY, axis_data.value());
                }
            }

            if gamepad.is_connected() {
                for button in Button::all_gilrs_buttons() {
                    gamepad_data
                        .button_pressed
                        .insert(Button::from(*button), gamepad.is_pressed(*button));
                }

                // should we also get stick values here or use events?
                // let x = gamepad.value(gilrs::Axis::LeftStickY);
                // let x = if x.abs() > 0.2 { x } else { 0.0 };
            }
        }

        message_data.time = std::time::SystemTime::now().into();

        if !message_data.gamepads.is_empty() {
            match sender.try_send(message_data.clone()) {
                Ok(()) => (),
                Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                    warn!("Controller reader queue is full");
                }
                Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
                    error!("Controller reader queue is closed");
                    panic!("Controller reader queue is closed");
                }
            }
        }

        std::thread::sleep(Duration::from_millis(50));
    }
}
