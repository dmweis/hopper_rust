use crate::ik_controller::{
    leg_positions::{LegPositions, MoveTowards},
    IkControlable,
};
use anyhow::Result;
use nalgebra::{distance, Point3};
use std::f32;
use std::time::Duration;
use tokio::time;
use tokio::{spawn, task::JoinHandle};

pub mod stance;
#[cfg(feature = "visualizer")]
pub mod visualizer;

pub struct MotionController {
    ik_controller: Box<dyn IkControlable>,
}

impl MotionController {
    pub async fn run(ik_controller: Box<dyn IkControlable>) -> Result<()> {
        let controller = MotionController { ik_controller };
        controller.control_loop().await?;
        Ok(())
    }

    pub fn start_as_task(ik_controller: Box<dyn IkControlable>) -> JoinHandle<Result<()>> {
        let controller = MotionController { ik_controller };
        spawn(controller.control_loop())
    }

    async fn control_loop(mut self) -> Result<()> {
        let stances = vec![
            stance::grounded_stance(),
            stance::relaxed_wide_stance(),
            stance::relaxed_stance(),
            stance::relaxed_wide_stance(),
        ];
        let mut interval = time::interval(Duration::from_millis(1000 / 50));
        let mut last_written_pose = stance::grounded_stance().clone();
        for stance in std::iter::repeat(stances).flatten() {
            for new_pose in last_written_pose.to_move_towards_iter(stance, 0.001) {
                self.ik_controller.move_to_positions(&new_pose).await?;
                last_written_pose = new_pose;
                interval.tick().await;
            }
        }
        Ok(())
    }
}

#[allow(dead_code)]
enum Tripod {
    LRL,
    RLR,
}

#[allow(dead_code)]
struct StepIterator {
    start: LegPositions,
    current: LegPositions,
    target: LegPositions,
    max_move: f32,
    step_height: f32,
    tripod: Tripod,
}

impl Iterator for StepIterator {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        None
    }
}

#[allow(dead_code)]
fn step_lifted_leg(
    start: &Point3<f32>,
    last_written: &Point3<f32>,
    target: &Point3<f32>,
    max_move: f32,
    step_height: f32,
) -> (Point3<f32>, bool) {
    if last_written == target {
        return (*target, false);
    }
    if distance(&last_written.xy(), &target.xy()) <= max_move {
        return (*target, true);
    }
    let full_ground_translation = target.xy() - start.xy();
    let step_translation = last_written.xy() + full_ground_translation.normalize() * max_move;
    let progress = full_ground_translation.magnitude() / full_ground_translation.magnitude();
    let height = (progress * f32::consts::PI).sin() * step_height + start.z;
    let ground_position = start.xy() + step_translation.coords;
    let new_position = Point3::new(ground_position.x, ground_position.y, height);
    (new_position, true)
}

#[allow(dead_code)]
fn step_grounded_leg(
    _start: &Point3<f32>,
    last_written: &Point3<f32>,
    target: &Point3<f32>,
    max_move: f32,
) -> (Point3<f32>, bool) {
    if last_written == target {
        return (*target, false);
    }
    if distance(&last_written.xy(), &target.xy()) <= max_move {
        return (*target, true);
    }
    unimplemented!("Not yet");
}
