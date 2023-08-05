use crate::error::HopperError;
use crate::motion_controller;
use chrono::{DateTime, Utc};
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

    let mut last_gamepad_message = InputMessage::default();

    loop {
        tokio::select! {
            sample = subscriber.recv_async() => {
                info!("got new message");
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
                info!("got new gamepad message");
                let gamepad_message = gamepad_message?;
                let gamepad_message: String = gamepad_message.value.try_into()?;
                let gamepad_message: InputMessage = serde_json::from_str(&gamepad_message)?;

                if gamepad_message.gamepads.len() != 1 {
                    error!("Expected 1 gamepad, got {}", gamepad_message.gamepads.len());
                    continue;
                }

                let a_pressed = gamepad_message.gamepads.values().next().map(|gamepad| gamepad.button_down_event_counter.get(&Button::South).cloned().unwrap_or_default()).unwrap_or_default() > last_gamepad_message.gamepads.values().next().map(|gamepad| gamepad.button_down_event_counter.get(&Button::South).cloned().unwrap_or_default()).unwrap_or_default();
                let b_pressed = gamepad_message.gamepads.values().next().map(|gamepad| gamepad.button_down_event_counter.get(&Button::East).cloned().unwrap_or_default()).unwrap_or_default() > last_gamepad_message.gamepads.values().next().map(|gamepad| gamepad.button_down_event_counter.get(&Button::East).cloned().unwrap_or_default()).unwrap_or_default();

                info!("A pressed: {}, B pressed: {}", a_pressed, b_pressed);
                if a_pressed {
                    controller.set_body_state(motion_controller::BodyState::Standing);
                } else if b_pressed {
                    controller.set_body_state(motion_controller::BodyState::Grounded);
                }

                last_gamepad_message = gamepad_message.clone();

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
