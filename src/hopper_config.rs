use serde::{Deserialize, Serialize};

use std::error::Error;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct LegConfig {
    pub coxa_id: u8,
    pub femur_id: u8,
    pub tibia_id: u8,
    pub angle_offset: f32,
    pub position: Vector3,
    pub femur_correction: f32,
    pub tibia_correction: f32,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct BodyConfig {
    pub left_front: LegConfig,
    pub left_middle: LegConfig,
    pub left_rear: LegConfig,
    pub right_front: LegConfig,
    pub right_middle: LegConfig,
    pub right_rear: LegConfig,
}

impl BodyConfig {
    pub fn get_ids(&self) -> [u8; 18] {
        [
            self.left_front.coxa_id,
            self.left_front.femur_id,
            self.left_front.tibia_id,
            self.left_middle.coxa_id,
            self.left_middle.femur_id,
            self.left_middle.tibia_id,
            self.left_rear.coxa_id,
            self.left_rear.femur_id,
            self.left_rear.tibia_id,
            self.right_front.coxa_id,
            self.right_front.femur_id,
            self.right_front.tibia_id,
            self.right_middle.coxa_id,
            self.right_middle.femur_id,
            self.right_middle.tibia_id,
            self.right_rear.coxa_id,
            self.right_rear.femur_id,
            self.right_rear.tibia_id,
        ]
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct HopperConfig {
    pub coxa_length: f32,
    pub femur_length: f32,
    pub tibia_length: f32,
    pub legs: BodyConfig,
}

impl HopperConfig {
    pub fn load(path: &Path) -> Result<HopperConfig, Box<dyn Error>> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let deserialized_config: HopperConfig = serde_yaml::from_reader(reader)?;
        Ok(deserialized_config)
    }
}
