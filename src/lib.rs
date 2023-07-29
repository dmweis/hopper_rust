#![doc = include_str!("../README.md")]

pub mod app_config;
pub mod body_controller;
pub mod camera;
pub mod error;
mod hexapod;
pub mod hopper_config;
pub mod ik_controller;
pub mod lidar;
pub mod motion_controller;
pub mod speech;
pub mod udp_adaptor;
pub mod utilities;

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
