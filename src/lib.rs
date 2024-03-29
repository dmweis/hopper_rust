#![doc = include_str!("../README.md")]

pub mod audio_transcribe;
pub mod body_controller;
pub mod camera;
pub mod configuration;
pub mod error;
pub mod face;
mod hexapod;
pub mod high_five;
pub mod hopper_body_config;
pub mod ik_controller;
pub mod ioc_container;
pub mod lidar;
pub mod logging;
pub mod monitoring;
pub mod motion_controller;
pub mod openai;
pub mod speech;
pub mod udp_remote;
pub mod utilities;
pub mod zenoh_remotes;

use once_cell::sync::Lazy;
use prost_reflect::DescriptorPool;

static FILE_DESCRIPTOR_SET: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/file_descriptor_set.bin"));

static DESCRIPTOR_POOL: Lazy<DescriptorPool> = Lazy::new(|| {
    DescriptorPool::decode(FILE_DESCRIPTOR_SET).expect("Failed to load file descriptor set")
});

/// protobuf
pub mod foxglove {
    #![allow(non_snake_case)]
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}

pub mod hopper {
    #![allow(non_snake_case)]
    include!(concat!(env!("OUT_DIR"), "/hopper.rs"));
}
