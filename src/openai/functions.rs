use async_trait::async_trait;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::sync::{Arc, Mutex};

use crate::{
    high_five::HighFiveServiceController,
    ioc_container::IocContainer,
    lidar::LidarServiceController,
    motion_controller::{BodyState, DanceMove, MotionControllerService},
};

use super::conversation_handler::{json_schema_for_func_args, ChatGptFunction};

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct HopperBodyPoseFuncArgs {
    pub body_pose: BodyState,
}

pub struct HopperBodyPoseFuncCallback {
    pub zenoh_session: Arc<zenoh::Session>,
}

#[async_trait]
impl ChatGptFunction for HopperBodyPoseFuncCallback {
    fn name(&self) -> String {
        "set_body_pose".to_string()
    }

    fn description(&self) -> String {
        "set the body pose of the hopper hexapod robot".to_string()
    }

    fn parameters_schema(&self) -> serde_json::Value {
        json_schema_for_func_args::<HopperBodyPoseFuncArgs>()
    }

    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let hopper_body_pose_func: HopperBodyPoseFuncArgs = serde_json::from_str(args)?;

        // stop high fives in case we are sitting down or folding
        match hopper_body_pose_func.body_pose {
            BodyState::Folded | BodyState::Grounded => {
                IocContainer::global_instance()
                    .service::<HighFiveServiceController>()?
                    .set_active(false);

                IocContainer::global_instance()
                    .service::<LidarServiceController>()?
                    .set_active(false);
            }
            _ => (),
        }

        IocContainer::global_instance()
            .service::<MotionControllerService>()?
            .set_body_state(hopper_body_pose_func.body_pose);

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
impl ChatGptFunction for HopperDanceFuncCallback {
    fn name(&self) -> String {
        "execute_hopper_dance".to_string()
    }

    fn description(&self) -> String {
        "perform a dance move with your body. Can be useful to express emotion or react to what user is saying".to_string()
    }

    fn parameters_schema(&self) -> serde_json::Value {
        json_schema_for_func_args::<HopperDanceFuncArgs>()
    }

    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let hopper_dance_move: HopperDanceFuncArgs = serde_json::from_str(args)?;

        IocContainer::global_instance()
            .service::<MotionControllerService>()?
            .start_dance_sequence(hopper_dance_move.dance_move);

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
impl ChatGptFunction for HopperHighFiveFuncCallback {
    fn name(&self) -> String {
        "enable_high_fives".to_string()
    }

    fn description(&self) -> String {
        "Enable automated high fives with the user. This mode will also enable the lidar sensor to detect the user.".to_string()
    }

    fn parameters_schema(&self) -> serde_json::Value {
        json_schema_for_func_args::<HopperHighFiveFuncArgs>()
    }

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
impl ChatGptFunction for FaceDisplayFuncCallback {
    fn name(&self) -> String {
        "set_face_animation".to_string()
    }

    fn description(&self) -> String {
        "Control the face panel on the Hopper robot. You can set the color and animation."
            .to_string()
    }

    fn parameters_schema(&self) -> serde_json::Value {
        json_schema_for_func_args::<FaceDisplayFuncArgs>()
    }

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
impl ChatGptFunction for SwitchVoiceFuncCallback {
    fn name(&self) -> String {
        "switch_voice_provider".to_string()
    }

    fn description(&self) -> String {
        "Switch voice provider".to_string()
    }

    fn parameters_schema(&self) -> serde_json::Value {
        json_schema_for_func_args::<SwitchVoiceFuncArgs>()
    }

    async fn call(&self, args: &str) -> anyhow::Result<serde_json::Value> {
        let switch_voice: SwitchVoiceFuncArgs = serde_json::from_str(args)?;

        let mut voice_provider = self.voice_provider.lock().unwrap();

        *voice_provider = switch_voice.voice_provider;

        let result = json!({});
        Ok(result)
    }
}
