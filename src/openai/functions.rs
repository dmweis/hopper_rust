use async_trait::async_trait;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::sync::{Arc, Mutex};
use zenoh::prelude::r#async::*;

use crate::{
    error::HopperError,
    high_five::HighFiveServiceController,
    ioc_container::IocContainer,
    lidar::LidarServiceController,
    motion_controller::{DanceMove, DanceService},
    zenoh_consts::STANCE_SUBSCRIBER,
};

use super::conversation_handler::AsyncCallback;

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct HopperBodyPoseFuncArgs {
    pub body_pose: HopperBodyPose,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum HopperBodyPose {
    Folded,
    Standing,
    Sitting,
}

pub struct HopperBodyPoseFuncCallback {
    pub zenoh_session: Arc<zenoh::Session>,
}

#[async_trait]
impl AsyncCallback for HopperBodyPoseFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let hopper_body_pose_func: HopperBodyPoseFuncArgs = serde_json::from_str(args)?;

        let message = match hopper_body_pose_func.body_pose {
            HopperBodyPose::Folded => "folded",
            HopperBodyPose::Standing => "stand",
            HopperBodyPose::Sitting => "ground",
        };

        // stop high fives in case we are sitting down or folding
        match hopper_body_pose_func.body_pose {
            HopperBodyPose::Folded | HopperBodyPose::Sitting => {
                IocContainer::global_instance()
                    .service::<HighFiveServiceController>()?
                    .set_active(false);

                IocContainer::global_instance()
                    .service::<LidarServiceController>()?
                    .set_active(false);
            }
            _ => (),
        }

        self.zenoh_session
            .put(STANCE_SUBSCRIBER, message)
            .res()
            .await
            .map_err(HopperError::ZenohError)?;

        let result = json!({
            "success": true
        });
        Ok(result)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct HopperDanceFuncArgs {
    /// dance move to perform
    pub dance_move: DanceMove,
}

pub struct HopperDanceFuncCallback;

#[async_trait]
impl AsyncCallback for HopperDanceFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let hopper_dance_move: HopperDanceFuncArgs = serde_json::from_str(args)?;

        IocContainer::global_instance()
            .service::<DanceService>()?
            .start_sequence(hopper_dance_move.dance_move);

        let result = json!({
            "success": true
        });
        Ok(result)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct HopperHighFiveFuncArgs {
    /// respond to high fives
    pub enable_high_fives: bool,
}

pub struct HopperHighFiveFuncCallback;

#[async_trait]
impl AsyncCallback for HopperHighFiveFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let high_five_args: HopperHighFiveFuncArgs = serde_json::from_str(args)?;

        IocContainer::global_instance()
            .service::<HighFiveServiceController>()?
            .set_active(high_five_args.enable_high_fives);

        IocContainer::global_instance()
            .service::<LidarServiceController>()?
            .set_active(high_five_args.enable_high_fives);

        let result = json!({
            "high_fives_enabled": high_five_args.enable_high_fives
        });
        Ok(result)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct FaceDisplayFuncArgs {
    /// Animation that is currently displayed on face display
    pub animation: FaceAnimation,
    /// Optional color for animations
    pub color: Option<FaceColor>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum FaceColor {
    Red,
    Green,
    Blue,
    Yellow,
    Purple,
    Cyan,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum FaceAnimation {
    /// Similar to KITT from the show Knight Rider
    LarsonScanner,
    /// Looks like a wave form of a human voice
    SpeakingAnimation,
    /// Pulsating light
    BreathingAnimation,
    /// Solid color
    SolidColor,
    /// This animation doesn't need color
    Off,
    CountDownAnimation,
}

pub struct FaceDisplayFuncCallback;

#[async_trait]
impl AsyncCallback for FaceDisplayFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let face_display_args: FaceDisplayFuncArgs = serde_json::from_str(args)?;

        let color = match face_display_args.color {
            Some(color) => match color {
                FaceColor::Red => crate::face::driver::RED,
                FaceColor::Green => crate::face::driver::GREEN,
                FaceColor::Blue => crate::face::driver::BLUE,
                FaceColor::Yellow => crate::face::driver::YELLOW,
                FaceColor::Purple => crate::face::driver::PURPLE,
                FaceColor::Cyan => crate::face::driver::CYAN,
            },
            None => crate::face::driver::PURPLE,
        };

        let face_controller =
            IocContainer::global_instance().service::<crate::face::FaceController>()?;

        match face_display_args.animation {
            FaceAnimation::LarsonScanner => face_controller.larson_scanner(color)?,
            FaceAnimation::SpeakingAnimation => face_controller.speaking(color)?,
            FaceAnimation::BreathingAnimation => face_controller.breathing(color)?,
            FaceAnimation::SolidColor => face_controller.solid_color(color)?,
            FaceAnimation::Off => face_controller.off()?,
            FaceAnimation::CountDownAnimation => face_controller.count_down_basic()?,
        }

        let result = json!({});
        Ok(result)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct SwitchVoiceFuncArgs {
    /// TTS voice provider
    pub voice_provider: VoiceProvider,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, JsonSchema, Default)]
#[serde(rename_all = "snake_case")]
pub enum VoiceProvider {
    /// default voice provider Microsoft Azure
    #[default]
    Default,
    /// expensive voice provider form Eleven Labs
    /// Should be used carefully
    Expensive,
}

pub struct SwitchVoiceFuncCallback {
    pub voice_provider: Arc<Mutex<VoiceProvider>>,
}

#[async_trait]
impl AsyncCallback for SwitchVoiceFuncCallback {
    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let switch_voice: SwitchVoiceFuncArgs = serde_json::from_str(args)?;

        let mut voice_provider = self.voice_provider.lock().unwrap();

        *voice_provider = switch_voice.voice_provider;

        let result = json!({});
        Ok(result)
    }
}
