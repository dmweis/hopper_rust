use super::speech_service::Playable;
use crate::error::{HopperError, HopperResult};
use std::fs::{self, File};
use std::path::Path;

pub struct AudioRepository {
    dir_path: String,
}

impl AudioRepository {
    pub fn new(dir_path: String) -> HopperResult<Self> {
        let path = Path::new(&dir_path);
        fs::create_dir_all(path)?;
        if !path.exists() {
            return Err(HopperError::AudioCacheDirError);
        }
        Ok(Self { dir_path })
    }

    pub fn load(&self, sound_name: &str) -> Option<Box<dyn Playable>> {
        let path = Path::new(&self.dir_path);
        let file_path = path.join(sound_name);
        if let Ok(file) = File::open(file_path) {
            Some(Box::new(file))
        } else {
            None
        }
    }
}
