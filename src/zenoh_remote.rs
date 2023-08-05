use crate::motion_controller;
use crate::{error::HopperError, motion_controller::walking::MoveCommand};
use chrono::{DateTime, Utc};
use nalgebra::{UnitQuaternion, Vector2, Vector3};
use serde::Deserialize;
use std::{collections::BTreeMap, sync::Arc};
use tracing::*;
use zenoh::prelude::r#async::*;

pub async fn simple_zenoh_controller(
    controller: &mut motion_controller::MotionController,
    zenoh_session: Arc<zenoh::Session>,
) -> anyhow::Result<()> {
    info!("Starting simple zenoh controller");
    let subscriber = zenoh_session
        .declare_subscriber("hopper/command/simple/stance")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let gamepad_subscriber = zenoh_session
        .declare_subscriber("remote-control/gamepad")
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let mut last_gamepad_message: Option<InputMessage> = None;

    loop {
        tokio::select! {
            sample = subscriber.recv_async() => {
                trace!("got new message");
                let sample = sample?;
                let command: String = sample.value.try_into()?;
                if &command.to_lowercase() == "stand" {
                    controller.set_body_state(motion_controller::BodyState::Standing);
                } else if &command.to_lowercase() == "ground" {
                    controller.set_body_state(motion_controller::BodyState::Grounded);
                } else if &command.to_lowercase() == "happy" {
                    controller.start_sequence(motion_controller::DanceMove::HappyDance);
                } else if &command.to_lowercase() == "wave" {
                    controller.start_sequence(motion_controller::DanceMove::WaveHi);
                } else {
                    error!("Unknown command {}", command);
                }
            }
            gamepad_message = gamepad_subscriber.recv_async() => {
                trace!("got new gamepad message");
                let gamepad_message = gamepad_message?;
                let gamepad_message: String = gamepad_message.value.try_into()?;
                let gamepad_message: InputMessage = serde_json::from_str(&gamepad_message)?;

                if gamepad_message.gamepads.len() != 1 {
                    warn!("Expected 1 gamepad, got {}", gamepad_message.gamepads.len());
                    continue;
                }

                let a_pressed = was_button_pressed_since_last_time(Button::South, &gamepad_message, &last_gamepad_message);
                let b_pressed = was_button_pressed_since_last_time(Button::East, &gamepad_message, &last_gamepad_message);
                let y_pressed = was_button_pressed_since_last_time(Button::North, &gamepad_message, &last_gamepad_message);
                let x_pressed = was_button_pressed_since_last_time(Button::West, &gamepad_message, &last_gamepad_message);

                let lb_pressed = is_pressed(Button::LeftTrigger, &gamepad_message);
                let rb_pressed = is_pressed(Button::RightTrigger, &gamepad_message);
                let lt_pressed = is_pressed(Button::LeftTrigger2, &gamepad_message);
                let rt_pressed = is_pressed(Button::RightTrigger2, &gamepad_message);

                if a_pressed {
                    info!("Setting stance to standing");
                    controller.set_body_state(motion_controller::BodyState::Standing);
                } else if b_pressed {
                    info!("Setting stance to grounded");
                    controller.set_body_state(motion_controller::BodyState::Grounded);
                } else if y_pressed {
                    info!("Starting happy dance");
                    controller.start_sequence(motion_controller::DanceMove::HappyDance);
                } else if x_pressed {
                    info!("Starting wave");
                    controller.start_sequence(motion_controller::DanceMove::WaveHi);
                }

                if lb_pressed {
                    // translation mode
                    let x = -get_axis(Axis::LeftStickY, &gamepad_message) * 0.05;
                    let y = get_axis(Axis::LeftStickX, &gamepad_message) * 0.05;
                    let pitch = -(get_axis(Axis::RightStickY, &gamepad_message) * 10.0).to_radians();
                    let yaw = (get_axis(Axis::RightStickX, &gamepad_message) * 10.0).to_radians();

                    let translation = Vector3::new(
                        x,
                        y,
                        0.0,
                    );
                    let rotation = UnitQuaternion::from_euler_angles(0.0, pitch, yaw);
                    controller.set_transformation(translation, rotation);
                }

                if rb_pressed {
                    // translation mode 2
                    let x = -get_axis(Axis::LeftStickY, &gamepad_message) * 0.05;
                    let z = -get_axis(Axis::RightStickY, &gamepad_message) * 0.05;
                    let roll = (get_axis(Axis::LeftStickX, &gamepad_message) * 10.0).to_radians();
                    let yaw = (get_axis(Axis::RightStickX, &gamepad_message) * 10.0).to_radians();

                    let translation = Vector3::new(
                        x,
                        0.0,
                        z,
                    );
                    let rotation = UnitQuaternion::from_euler_angles(roll, 0.0, yaw);
                    controller.set_transformation(translation, rotation);
                }

                if rt_pressed {
                    // walking mode
                    let x = get_axis(Axis::LeftStickY, &gamepad_message) * 0.06;
                    let y = get_axis(Axis::LeftStickX, &gamepad_message) * 0.06;
                    let yaw = get_axis(Axis::RightStickX, &gamepad_message) * 15_f32.to_radians();

                    let move_command =
                    MoveCommand::new(Vector2::new(x, y), yaw);
                    controller.set_command(move_command);
                }

                last_gamepad_message = Some(gamepad_message);
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

fn is_pressed(button: Button, gamepad_message: &InputMessage) -> bool {
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
