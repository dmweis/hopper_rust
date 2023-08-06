use crate::{
    body_controller::{
        motor_controller::{HexapodCompliance, HexapodMotorSpeed},
        BodyController, BodyMotorPositions,
    },
    error::HopperResult,
    ik_controller::leg_positions::*,
};
use anyhow::{anyhow, Result};
use async_trait::async_trait;
use kiss3d::{self, scene::SceneNode};
use nalgebra::{Isometry3, Point2, Point3, Translation3, UnitQuaternion, Vector3};
use std::{
    str::FromStr,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread::{spawn, JoinHandle},
    time::Instant,
};

use kiss3d::light::Light;
use kiss3d::window::Window;

use crate::ik_controller::IkControllable;

use super::stance;

pub struct HopperVisualizer {
    keep_running: Arc<AtomicBool>,
    thread_handle: Option<JoinHandle<()>>,
    leg_positions: Arc<Mutex<LegPositions>>,
}

impl HopperVisualizer {
    pub fn new(ground_type: GroundType) -> Self {
        let initial_position = stance::random_grounded_stance();
        let leg_positions = Arc::new(Mutex::new(initial_position));
        let keep_running = Arc::new(AtomicBool::new(true));

        let leg_positions_clone = leg_positions.clone();
        let keep_running_clone = keep_running.clone();
        let handle =
            spawn(move || visualizer_loop(keep_running_clone, leg_positions_clone, &ground_type));
        Self {
            keep_running,
            thread_handle: Some(handle),
            leg_positions,
        }
    }
}

impl Default for HopperVisualizer {
    fn default() -> Self {
        let initial_position = *stance::grounded_stance();
        let leg_positions = Arc::new(Mutex::new(initial_position));
        let keep_running = Arc::new(AtomicBool::new(true));
        let ground_type = GroundType::Circles;

        let leg_positions_clone = leg_positions.clone();
        let keep_running_clone = keep_running.clone();
        let handle =
            spawn(move || visualizer_loop(keep_running_clone, leg_positions_clone, &ground_type));
        Self {
            keep_running,
            thread_handle: Some(handle),
            leg_positions,
        }
    }
}

#[async_trait]
impl BodyController for HopperVisualizer {
    async fn move_motors_to(&mut self, _positions: &BodyMotorPositions) -> HopperResult<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn set_compliance_slope(&mut self, _compliance: u8) -> HopperResult<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn set_body_compliance_slope(
        &mut self,
        _compliance: HexapodCompliance,
    ) -> HopperResult<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn set_motor_speed(&mut self, _speed: u16) -> HopperResult<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn set_body_motor_speed(&mut self, _speed: HexapodMotorSpeed) -> HopperResult<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn set_torque(&mut self, _torque: bool) -> HopperResult<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn read_motor_positions(&mut self) -> HopperResult<BodyMotorPositions> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn read_mean_voltage(&mut self) -> HopperResult<f32> {
        Ok(12.0)
    }

    async fn scan_motors(&mut self) -> HopperResult<()> {
        unimplemented!("shouldn't be called on a mock");
    }

    async fn flush_and_clear_motors(&mut self) -> HopperResult<()> {
        unimplemented!("shouldn't be called on a mock");
    }
}

#[async_trait]
impl IkControllable for HopperVisualizer {
    async fn move_to_positions(&mut self, positions: &LegPositions) -> HopperResult<()> {
        let mut guard = self.leg_positions.lock().unwrap();
        *guard = *positions;
        Ok(())
    }

    async fn read_leg_positions(&mut self) -> HopperResult<LegPositions> {
        Ok(*self.leg_positions.lock().unwrap())
    }

    async fn disable_motors(&mut self) -> HopperResult<()> {
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
        root.set_local_translation(Vector3::new(0., 0., 0.15).yzx().into());
        let mut body = root.add_cube(0.05, 0.05, 0.15);
        body.set_color(1.0, 0.0, 0.0);
        let mut left_front = root.add_sphere(FOOT_SPHERE_SIZE);
        left_front.set_color(0.0, 1.0, 0.0);
        let mut left_middle = root.add_sphere(FOOT_SPHERE_SIZE);
        left_middle.set_color(1.0, 0.5, 0.8);
        let mut left_rear = root.add_sphere(FOOT_SPHERE_SIZE);
        left_rear.set_color(1.0, 0.0, 0.0);
        let mut right_front = root.add_sphere(FOOT_SPHERE_SIZE);
        right_front.set_color(0.0, 1.0, 0.0);
        let mut right_middle = root.add_sphere(FOOT_SPHERE_SIZE);
        right_middle.set_color(1.0, 0.5, 0.8);
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

    fn draw_tripod_lines(&self, leg_positions: &LegPositions, window: &mut Window) {
        let purple = Point3::new(1.0, 0.5, 1.0);
        let greenish = Point3::new(0.0, 0.5, 0.5);
        let translation = Vector3::new(0., 0., 0.15).yzx();
        window.draw_line(
            &(leg_positions.left_front().yzx() + translation),
            &(leg_positions.right_middle().yzx() + translation),
            &purple,
        );
        window.draw_line(
            &(leg_positions.left_rear().yzx() + translation),
            &(leg_positions.right_middle().yzx() + translation),
            &purple,
        );
        window.draw_line(
            &(leg_positions.left_front().yzx() + translation),
            &(leg_positions.left_rear().yzx() + translation),
            &purple,
        );

        window.draw_line(
            &(leg_positions.right_front().yzx() + translation),
            &(leg_positions.left_middle().yzx() + translation),
            &greenish,
        );
        window.draw_line(
            &(leg_positions.right_rear().yzx() + translation),
            &(leg_positions.left_middle().yzx() + translation),
            &greenish,
        );
        window.draw_line(
            &(leg_positions.right_front().yzx() + translation),
            &(leg_positions.right_rear().yzx() + translation),
            &greenish,
        );
    }
}

fn visualizer_loop(
    keep_running: Arc<AtomicBool>,
    leg_positions: Arc<Mutex<LegPositions>>,
    ground_type: &GroundType,
) {
    let white = Point3::new(1.0, 1.0, 1.0);
    let mut frame_counter = Instant::now();

    let mut window = Window::new("Hopper visualizer");
    window.set_background_color(0.5, 0.5, 0.5);
    draw_floor(&mut window, ground_type);
    let mut leg_visualizer = LegVisualizer::new(&mut window);

    window.set_light(Light::StickToCamera);

    while keep_running.load(Ordering::Acquire) && window.render() {
        let guard = leg_positions.lock().unwrap();
        leg_visualizer.update_positions(&guard);
        leg_visualizer.draw_tripod_lines(&guard, &mut window);
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GroundType {
    ChessBoard,
    Circles,
    Squares,
}

impl FromStr for GroundType {
    type Err = anyhow::Error;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        match text.trim().to_lowercase().as_ref() {
            "chessboard" => Ok(GroundType::ChessBoard),
            "circles" => Ok(GroundType::Circles),
            "squares" => Ok(GroundType::Squares),
            _ => Err(anyhow!(
                "Failed to parse {} to GroundType. Options are: chessboard, circles, or squares",
                text
            )),
        }
    }
}

fn draw_floor(window: &mut Window, ground_type: &GroundType) {
    match ground_type {
        GroundType::ChessBoard => add_ground_plane(window),
        GroundType::Circles => add_circles(window),
        GroundType::Squares => add_squares(window),
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

fn add_circles(window: &mut Window) {
    const CIRCLES: usize = 20;
    for i in 0..CIRCLES {
        let mut cylinder = window.add_cylinder(i as f32 * 0.04, 0.001);
        if i % 2 == 0 {
            cylinder.set_color(0.0, 0.0, 0.0);
        } else {
            cylinder.set_color(1.0, 1.0, 1.0);
        }
        cylinder.set_local_translation(Translation3::new(0.0, -0.00001 * i as f32, 0.0));
    }
}

fn add_squares(window: &mut Window) {
    const SQUARES: usize = 20;
    for i in 0..SQUARES {
        let mut rectangle = window.add_cube(i as f32 * 0.04, 0.001, i as f32 * 0.04);
        if i % 2 == 0 {
            rectangle.set_color(0.0, 0.0, 0.0);
        } else {
            rectangle.set_color(1.0, 1.0, 1.0);
        }
        rectangle.set_local_translation(Translation3::new(0.0, -0.00001 * i as f32, 0.0));
    }
}
