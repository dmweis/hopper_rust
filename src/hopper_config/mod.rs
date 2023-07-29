use crate::hexapod::HexapodTypes;
use anyhow::Result;
use nalgebra::Point3;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct LegConfig {
    pub coxa_id: u8,
    pub femur_id: u8,
    pub tibia_id: u8,
    pub angle_offset: f32,
    pub position: Point3<f32>,
    pub femur_correction: f32,
    pub tibia_correction: f32,
}

pub type BodyConfig = HexapodTypes<LegConfig>;

impl BodyConfig {
    pub fn get_ids(&self) -> [u8; 18] {
        [
            self.left_front().coxa_id,
            self.left_front().femur_id,
            self.left_front().tibia_id,
            self.left_middle().coxa_id,
            self.left_middle().femur_id,
            self.left_middle().tibia_id,
            self.left_rear().coxa_id,
            self.left_rear().femur_id,
            self.left_rear().tibia_id,
            self.right_front().coxa_id,
            self.right_front().femur_id,
            self.right_front().tibia_id,
            self.right_middle().coxa_id,
            self.right_middle().femur_id,
            self.right_middle().tibia_id,
            self.right_rear().coxa_id,
            self.right_rear().femur_id,
            self.right_rear().tibia_id,
        ]
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct HopperConfig {
    pub coxa_length: f32,
    pub femur_length: f32,
    pub tibia_length: f32,
    pub femur_offset: f32,
    pub tibia_offset: f32,
    pub legs: BodyConfig,
}

impl HopperConfig {
    pub fn load(path: &Path) -> Result<HopperConfig> {
        let text = fs::read_to_string(path)?;
        let deserialized_config: HopperConfig = toml::from_str(&text)?;
        Ok(deserialized_config)
    }

    pub fn save_as_toml(&self, path: &Path) -> Result<()> {
        fs::write(path, toml::to_string_pretty(&self)?)?;
        Ok(())
    }
}

impl Default for HopperConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/hopper.toml")).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_hopper_config_loads() {
        let _ = HopperConfig::default();
    }
}
