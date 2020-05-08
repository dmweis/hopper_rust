use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use nalgebra::Point3;


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
    pub femur_offset: f32,
    pub tibia_offset: f32,
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


impl Default for HopperConfig {
    fn default() -> Self {
        serde_yaml::from_slice(include_bytes!("../config/hopper.yaml")).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_hopper_config_loads() {
        let _ = HopperConfig::default();
    }

    #[test]
    fn serialize_point() {
        let point = Point3::new(1.0, 1.0, 1.0);
        let yaml = serde_yaml::to_string(&point).unwrap();
        assert_eq!(yaml, "---\n- 1.0\n- 1.0\n- 1.0");
    }

    #[test]
    fn deserialize_point() {
        let yaml = "--- [1, 1, 1]";
        let point: Point3<f32> = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(point, Point3::new(1.0, 1.0, 1.0));
    }
}
