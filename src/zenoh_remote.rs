use crate::body_controller::motor_controller::{HexapodCompliance, HexapodMotorSpeed};
use crate::error::HopperResult;
use crate::hexapod::LegFlags;
use crate::motion_controller::walking::{DEFAULT_STEP_HEIGHT, DEFAULT_STEP_TIME};
use crate::motion_controller::{self, SingleLegCommand};
use crate::{error::HopperError, motion_controller::walking::MoveCommand};
use chrono::{DateTime, Utc};
use nalgebra::{UnitQuaternion, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::time::Duration;
use std::{collections::BTreeMap, sync::Arc};
use tracing::*;
use zenoh::prelude::r#async::*;

pub async fn simple_zenoh_controller(
    controller: &mut motion_controller::MotionController,
    zenoh_session: Arc<zenoh::Session>,
) -> anyhow::Result<()> {
    info!("Starting simple zenoh controller");
    let stance_subscriber = zenoh_session
        .declare_subscriber("hopper/command/simple/stance")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let gamepad_subscriber = zenoh_session
        .declare_subscriber("remote-control/gamepad")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let walking_config_subscriber = zenoh_session
        .declare_subscriber("hopper/command/simple/walking_config")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let compliance_slope_subscriber = zenoh_session
        .declare_subscriber("hopper/command/config/compliance_slope")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let body_motor_speed_subscriber = zenoh_session
        .declare_subscriber("hopper/command/config/motor_speed")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

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
                handle_stance_command(sample, controller).await?;
            }
            sample = compliance_slope_subscriber.recv_async() => {
                let sample = sample?;
                handle_compliance_slope_command(sample, controller).await?;
            }
            sample = body_motor_speed_subscriber.recv_async() => {
                let sample = sample?;
                handle_body_motor_speed_command(sample, controller).await?;
            }
            sample = walking_config_subscriber.recv_async() => {
                let sample = sample?;
                gamepad_controller.handle_walking_config_update(sample).await?;
                gamepad_controller.publish_walking_config(&zenoh_session).await?;
            }
            gamepad_message = gamepad_subscriber.recv_async() => {
                trace!("got new gamepad message");
                let gamepad_message = gamepad_message?;
                gamepad_controller.handle_gamepad_command(gamepad_message, controller, &mut last_gamepad_message).await?;
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
}

impl Default for GamepadController {
    fn default() -> Self {
        Self {
            walking_config: WalkingConfig::default(),
            single_leg_foot: LegFlags::LEFT_FRONT,
            feet_iterator: vec![
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
            .cycle(),
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
            .put("hopper/status/simple/walking_config", message)
            .res()
            .await
            .map_err(HopperError::ZenohError)?;
        Ok(())
    }

    async fn handle_gamepad_command(
        &mut self,
        gamepad_message: zenoh::sample::Sample,
        controller: &mut motion_controller::MotionController,
        last_gamepad_message: &mut Option<InputMessage>,
    ) -> anyhow::Result<()> {
        let gamepad_message: String = gamepad_message.value.try_into()?;
        let gamepad_message: InputMessage = serde_json::from_str(&gamepad_message)?;

        if gamepad_message.gamepads.len() != 1 {
            warn!("Expected 1 gamepad, got {}", gamepad_message.gamepads.len());
            return Ok(());
        }

        let a_pressed = was_button_pressed_since_last_time(
            Button::South,
            &gamepad_message,
            last_gamepad_message,
        );
        let _b_pressed = was_button_pressed_since_last_time(
            Button::East,
            &gamepad_message,
            last_gamepad_message,
        );
        let y_pressed = was_button_pressed_since_last_time(
            Button::North,
            &gamepad_message,
            last_gamepad_message,
        );
        let x_pressed = was_button_pressed_since_last_time(
            Button::West,
            &gamepad_message,
            last_gamepad_message,
        );
        let select_pressed = was_button_pressed_since_last_time(
            Button::Select,
            &gamepad_message,
            last_gamepad_message,
        );
        let start_pressed = was_button_pressed_since_last_time(
            Button::Start,
            &gamepad_message,
            last_gamepad_message,
        );
        let lt_pressed_since_last = was_button_pressed_since_last_time(
            Button::LeftTrigger2,
            &gamepad_message,
            last_gamepad_message,
        );

        if lt_pressed_since_last {
            self.single_leg_foot = self.feet_iterator.next().unwrap();
            info!("Switching to single leg foot {:?}", self.single_leg_foot);
        }

        let lb_pressed = is_button_down(Button::LeftTrigger, &gamepad_message);
        let rb_pressed = is_button_down(Button::RightTrigger, &gamepad_message);
        let lt_pressed = is_button_down(Button::LeftTrigger2, &gamepad_message);
        let rt_pressed = is_button_down(Button::RightTrigger2, &gamepad_message);

        if a_pressed {
            info!("Setting stance to standing");
            controller.set_body_state(motion_controller::BodyState::Standing);
        } else if y_pressed {
            info!("Starting happy dance");
            controller.start_sequence(motion_controller::DanceMove::HappyDance);
        } else if x_pressed {
            info!("Starting wave");
            controller.start_sequence(motion_controller::DanceMove::WaveHi);
        } else if select_pressed {
            info!("Folding");
            controller.set_body_state(motion_controller::BodyState::Folded);
        } else if start_pressed {
            info!("Grounding");
            controller.set_body_state(motion_controller::BodyState::Grounded);
        }

        if lb_pressed {
            // translation mode
            let x = -get_axis(Axis::LeftStickY, &gamepad_message) * 0.05;
            let y = get_axis(Axis::LeftStickX, &gamepad_message) * 0.05;
            let pitch = -(get_axis(Axis::RightStickY, &gamepad_message) * 10.0).to_radians();
            let yaw = (get_axis(Axis::RightStickX, &gamepad_message) * 10.0).to_radians();

            let translation = Vector3::new(x, y, 0.0);
            let rotation = UnitQuaternion::from_euler_angles(0.0, pitch, yaw);
            controller.set_transformation(translation, rotation);
        } else if rb_pressed {
            // translation mode 2
            let x = -get_axis(Axis::LeftStickY, &gamepad_message) * 0.05;
            let z = -get_axis(Axis::RightStickY, &gamepad_message) * 0.05;
            let roll = (get_axis(Axis::LeftStickX, &gamepad_message) * 10.0).to_radians();
            let yaw = (get_axis(Axis::RightStickX, &gamepad_message) * 10.0).to_radians();

            let translation = Vector3::new(x, 0.0, z);
            let rotation = UnitQuaternion::from_euler_angles(roll, 0.0, yaw);
            controller.set_transformation(translation, rotation);
        } else if rt_pressed {
            // walking mode
            let x = get_axis(Axis::LeftStickY, &gamepad_message)
                * self.walking_config.max_step_distance_m;
            let y = -get_axis(Axis::LeftStickX, &gamepad_message)
                * self.walking_config.max_step_distance_m;
            let yaw = -get_axis(Axis::RightStickX, &gamepad_message)
                * self.walking_config.max_yaw_rate_deg.to_radians();

            let move_command = MoveCommand::with_optional_fields(
                Vector2::new(x, y),
                yaw,
                self.walking_config.step_time,
                self.walking_config.step_height_m,
                self.walking_config.aggressive_leg_lift,
            );
            controller.set_command(move_command);
        } else if lt_pressed {
            let x = get_axis(Axis::LeftStickY, &gamepad_message) * 0.07;
            let y = -get_axis(Axis::LeftStickX, &gamepad_message) * 0.07;
            let z = get_axis(Axis::RightStickY, &gamepad_message) * 0.07;

            controller.set_single_leg_command(SingleLegCommand::new(
                self.single_leg_foot,
                Vector3::new(x, y, z),
            ));
        } else {
            controller.clear_single_leg_command();
            controller.set_transformation(Default::default(), Default::default());
            controller.set_command(MoveCommand::with_optional_fields(
                Vector2::zeros(),
                0.0,
                self.walking_config.step_time,
                self.walking_config.step_height_m,
                self.walking_config.aggressive_leg_lift,
            ));
        }

        *last_gamepad_message = Some(gamepad_message);
        Ok(())
    }
}

fn was_button_pressed_since_last_time(
    button: Button,
    gamepad_message: &InputMessage,
    last_gamepad_message: &Option<InputMessage>,
) -> bool {
    let last_gamepad_message = match last_gamepad_message {
        Some(last_gamepad_message) => last_gamepad_message,
        None => return false,
    };
    let button_down_event_counter = gamepad_message
        .gamepads
        .values()
        .next()
        .map(|gamepad| {
            gamepad
                .button_down_event_counter
                .get(&button)
                .cloned()
                .unwrap_or_default()
        })
        .unwrap_or_default();
    let last_button_down_event_counter = last_gamepad_message
        .gamepads
        .values()
        .next()
        .map(|gamepad| {
            gamepad
                .button_down_event_counter
                .get(&button)
                .cloned()
                .unwrap_or_default()
        })
        .unwrap_or_default();
    button_down_event_counter > last_button_down_event_counter
}

fn get_axis(axis: Axis, gamepad_message: &InputMessage) -> f32 {
    gamepad_message
        .gamepads
        .values()
        .next()
        .map(|gamepad| gamepad.axis_state.get(&axis).cloned().unwrap_or_default())
        .unwrap_or_default()
}

fn is_button_down(button: Button, gamepad_message: &InputMessage) -> bool {
    gamepad_message
        .gamepads
        .values()
        .next()
        .map(|gamepad| {
            gamepad
                .button_pressed
                .get(&button)
                .cloned()
                .unwrap_or_default()
        })
        .unwrap_or_default()
}

#[derive(Debug, Deserialize, Default, Clone)]
#[allow(unused)]
pub struct InputMessage {
    gamepads: BTreeMap<usize, GamepadMessage>,
    time: DateTime<Utc>,
}

#[derive(Debug, Deserialize, Clone)]
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

impl GamepadMessage {}

#[derive(Debug, Deserialize, PartialEq, Eq, Hash, PartialOrd, Ord, Clone, Copy)]
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
}

#[derive(Debug, Deserialize, PartialEq, Eq, Hash, PartialOrd, Ord, Clone, Copy)]
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
            max_step_distance_m: 0.025,
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
