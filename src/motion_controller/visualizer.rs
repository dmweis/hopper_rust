use crate::{
    body_controller::{BodyController, BodyMotorPositions},
    ik_controller::leg_positions::*,
};
use anyhow::Result;
use async_trait::async_trait;
use kiss3d::{self, scene::SceneNode};
use nalgebra::{Isometry3, Point2, Point3, Translation3, UnitQuaternion, Vector3};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread::{spawn, JoinHandle},
    time::Instant,
};

use kiss3d::light::Light;
use kiss3d::window::Window;

use crate::ik_controller::IkControlable;

use super::stance;

pub struct HopperVisualizer {
    keep_running: Arc<AtomicBool>,
    thread_handle: Option<JoinHandle<()>>,
    leg_positions: Arc<Mutex<LegPositions>>,
}

impl Default for HopperVisualizer {
    fn default() -> Self {
        let initial_position = stance::relaxed_stance().clone();
        let leg_positions = Arc::new(Mutex::new(initial_position));
        let keep_running = Arc::new(AtomicBool::new(true));

        let leg_positions_clone = leg_positions.clone();
        let keep_running_clone = keep_running.clone();
        let handle = spawn(move || visualizer_loop(keep_running_clone, leg_positions_clone));
        Self {
            keep_running,
            thread_handle: Some(handle),
            leg_positions,
        }
    }
}

#[async_trait]
impl BodyController for HopperVisualizer {
    async fn move_motors_to(&mut self, _positions: BodyMotorPositions) -> Result<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn set_compliance(&mut self, _compliance: u8) -> Result<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn set_speed(&mut self, _speed: u16) -> Result<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn set_torque(&mut self, _torque: bool) -> Result<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn read_motor_positions(&mut self) -> Result<BodyMotorPositions> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn read_mean_voltage(&mut self) -> Result<f32> {
        unimplemented!("shouldn't be called on a mock");
    }
}

#[async_trait]
impl IkControlable for HopperVisualizer {
    async fn move_to_positions(&mut self, positions: LegPositions) -> Result<()> {
        let mut guard = self.leg_positions.lock().unwrap();
        *guard = positions;
        Ok(())
    }

    async fn read_leg_positions(&mut self) -> Result<LegPositions> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn disable_motors(&mut self) -> Result<()> {
        unimplemented!("shouldn't be called on a mock");
    }
}

impl Drop for HopperVisualizer {
    fn drop(&mut self) {
        self.keep_running.store(false, Ordering::Release);
        if let Some(thread_handle) = self.thread_handle.take() {
            thread_handle
                .join()
                .expect("Failed drop for HopperVisualizer");
        }
    }
}

struct LegVisualizer {
    _root: SceneNode,
    _body: SceneNode,
    left_front: SceneNode,
    left_middle: SceneNode,
    left_rear: SceneNode,
    right_front: SceneNode,
    right_middle: SceneNode,
    right_rear: SceneNode,
}

impl LegVisualizer {
    fn new(window: &mut Window) -> Self {
        const FOOT_SPHERE_SIZE: f32 = 0.01;
        let mut root = window.add_group();
        root.set_local_translation(Vector3::new(0., 0., 0.1).yzx().into());
        let mut body = root.add_cube(0.05, 0.05, 0.15);
        body.set_color(1.0, 0.0, 0.0);
        let mut left_front = root.add_sphere(FOOT_SPHERE_SIZE);
        left_front.set_color(1.0, 0.0, 0.0);
        let mut left_middle = root.add_sphere(FOOT_SPHERE_SIZE);
        left_middle.set_color(1.0, 0.0, 0.0);
        let mut left_rear = root.add_sphere(FOOT_SPHERE_SIZE);
        left_rear.set_color(1.0, 0.0, 0.0);
        let mut right_front = root.add_sphere(FOOT_SPHERE_SIZE);
        right_front.set_color(1.0, 0.0, 0.0);
        let mut right_middle = root.add_sphere(FOOT_SPHERE_SIZE);
        right_middle.set_color(1.0, 0.0, 0.0);
        let mut right_rear = root.add_sphere(FOOT_SPHERE_SIZE);
        right_rear.set_color(1.0, 0.0, 0.0);
        Self {
            _root: root,
            _body: body,
            left_front,
            left_middle,
            left_rear,
            right_front,
            right_middle,
            right_rear,
        }
    }

    fn update_positions(&mut self, leg_positions: &LegPositions) {
        self.left_front
            .set_local_translation(leg_positions.left_front().coords.yzx().into());
        self.left_middle
            .set_local_translation(leg_positions.left_middle().coords.yzx().into());
        self.left_rear
            .set_local_translation(leg_positions.left_rear().coords.yzx().into());
        self.right_front
            .set_local_translation(leg_positions.right_front().coords.yzx().into());
        self.right_middle
            .set_local_translation(leg_positions.right_middle().coords.yzx().into());
        self.right_rear
            .set_local_translation(leg_positions.right_rear().coords.yzx().into());
    }
}

fn visualizer_loop(keep_running: Arc<AtomicBool>, leg_positions: Arc<Mutex<LegPositions>>) {
    let white = Point3::new(1.0, 1.0, 1.0);
    let mut frame_counter = Instant::now();

    let mut window = Window::new("Hopper visualizer");
    window.set_background_color(0.5, 0.5, 0.5);
    add_ground_plane(&mut window);
    let mut leg_visualizer = LegVisualizer::new(&mut window);

    window.set_light(Light::StickToCamera);

    while keep_running.load(Ordering::Acquire) && window.render() {
        let guard = leg_positions.lock().unwrap();
        leg_visualizer.update_positions(&guard);
        window.draw_text(
            &format!("frame time: {}ms", frame_counter.elapsed().as_millis(),),
            &Point2::new(1.0, 1.0),
            50.0,
            &kiss3d::text::Font::default(),
            &white,
        );
        frame_counter = Instant::now();
    }
}

fn add_ground_plane(window: &mut Window) {
    let size = 0.5;
    for i in 0..4 {
        for j in 0..4 {
            let mut cube = window.add_cube(size, size, 0.001);
            if (i + j) % 2 == 0 {
                // cube.set_color(1.0, 0.3, 0.2);
                cube.set_color(0.0, 0.0, 0.0);
            } else {
                // cube.set_color(0.5, 0.04, 0.17);
                cube.set_color(1.0, 1.0, 1.0);
            }
            let distance = (1_f32.powi(2) + 1_f32.powi(2)).sqrt();
            let x_ind = j as f32 - distance;
            let y_ind = i as f32 - distance;
            let trans = Isometry3::from_parts(
                Translation3::new(size * x_ind, 0.0, size * y_ind),
                UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57),
            );
            cube.set_local_transformation(trans);
        }
    }
}
