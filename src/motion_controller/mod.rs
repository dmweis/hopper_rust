use crate::ik_controller::{leg_positions::MoveTowards, IkControlable};
use anyhow::Result;
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
