use super::speech_service::Playable;
use crate::error::{HopperError, HopperResult};
use rand::seq::SliceRandom;
use std::fs::{self, File};
use std::path::{Path, PathBuf};

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

    pub fn random_file_from_dir(&self, subdirectory: &str) -> Option<Box<File>> {
        // This is a pretty complicated way to do this
        // but oh well... it's fast enough for now

        let path = Path::new(&self.dir_path);
        let full_path = path.join(subdirectory);

        if let Ok(paths) = std::fs::read_dir(full_path) {
            let files: Vec<_> = paths
                .filter_map(|path| path.ok())
                .filter(|entry| {
                    entry
                        .file_type()
                        .map(|file_type| file_type.is_file())
                        .unwrap_or(false)
                })
                .map(|file_entry| file_entry.path())
                .collect();
            let mut rng = rand::thread_rng();

            files
                .choose(&mut rng)
                .and_then(|path| File::open(path).ok())
                .map(Box::new)
        } else {
            None
        }
    }

    pub fn random_file_recursive(&self) -> Option<(Box<File>, PathBuf)> {
        let walker = walkdir::WalkDir::new(Path::new(&self.dir_path)).into_iter();

        let files = walker
            // ignore errors
            .filter_map(|entry| entry.ok())
            // take only files
            .filter(|entry| entry.file_type().is_file())
            // turn to paths
            .map(|file_entry| file_entry.path().to_path_buf())
            // ignore astromech because those files are boring
            .filter(|path| !path.to_string_lossy().contains("astromech"))
            .collect::<Vec<_>>();

        let mut rng = rand::thread_rng();

        files.choose(&mut rng).and_then(|path| {
            if let Ok(file) = File::open(path) {
                Some((Box::new(file), path.to_owned()))
            } else {
                None
            }
        })
    }
}
