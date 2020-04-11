use serde::{Serialize, Deserialize};

use std::error::Error;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct Vector3 {
    x: f32,
    y: f32,
    z: f32
}

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct LegConfig {
    coxa_id: u8,
    femur_id: u8,
    tibia_id: u8,
    angle_offset: f32,
    position: Vector3,
    femur_correction: f32,
    tibia_correction: f32,
}

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyConfig {
    left_front: LegConfig,
    left_middle: LegConfig,
    left_rear: LegConfig,
    right_front: LegConfig,
    right_middle: LegConfig,
    right_rear: LegConfig,
}

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct HopperConfig {
    coxa_length: f32,
    femur_length: f32,
    tibia_length: f32,
    legs: BodyConfig,
}

impl HopperConfig {
    pub fn load(path: &Path) -> Result<HopperConfig, Box<dyn Error>> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let deserialized_config: HopperConfig =
            serde_yaml::from_reader(reader)?;
        Ok(deserialized_config)
    }
}