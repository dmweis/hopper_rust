use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

use nalgebra::{Point3, Vector3};
use rplidar_driver::ScanPoint;
use tokio::sync::mpsc::{channel, Receiver, Sender};
use tracing::info;

use crate::{
    error::HopperResult,
    hexapod::LegFlags,
    ik_controller::{
        leg_positions::{LegPositions, MoveTowards},
        IkControllable,
    },
};

#[derive(Clone, Debug)]
pub struct HighFiveCommand {
    pub leg: LegFlags,
    pub target: Vector3<f32>,
}

#[derive(Clone, Debug, Default)]
pub struct HighFiveServiceController {
    active: Arc<AtomicBool>,
}

impl HighFiveServiceController {
    pub fn set_active(&self, active: bool) {
        self.active.store(active, Ordering::Relaxed);
    }

    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }
}

#[derive(Clone, Debug)]
pub struct HighFiveDetector {
    last_high_five: Instant,
    sender: Sender<HighFiveCommand>,
    status: HighFiveServiceController,
}

const MIN_DISTANCE: f32 = 0.1;
const MAX_DISTANCE: f32 = 0.3;
const HIGH_FIVE_TIMEOUT: Duration = Duration::from_secs(5);

impl HighFiveDetector {
    pub fn new() -> (Self, Receiver<HighFiveCommand>, HighFiveServiceController) {
        let (sender, receiver) = channel(10);
        let status = HighFiveServiceController::default();
        (
            HighFiveDetector {
                last_high_five: Instant::now(),
                sender,
                status: status.clone(),
            },
            receiver,
            status,
        )
    }

    /// Process a scan and submit a high five if one is detected
    ///
    /// This method expects scans to be sorted before being passed in
    pub async fn process_scan(&mut self, scan: &[ScanPoint]) {
        if self.last_high_five.elapsed() < HIGH_FIVE_TIMEOUT {
            // don't high five if we just did
            return;
        }
        self.status.is_active();

        // 0/360 is straight ahead
        // angle is counter clockwise if z points up
        // so 90.0 degrees is on the right
        // thus it's not following right hand rule assuming that z is upwards
        // remember to invert the angle

        // Safe ranges for front two legs
        const RIGHT_ANGLE_RANGE: (f32, f32) = (20.0, 75.0);
        const LEFT_ANGLE_RANGE: (f32, f32) = (285.0, 340.0);

        let left_points = scan
            .iter()
            .filter(|p| {
                let angle = p.angle().to_degrees();
                angle >= LEFT_ANGLE_RANGE.0 && angle <= LEFT_ANGLE_RANGE.1
            })
            .collect::<Vec<_>>();

        let right_points = scan
            .iter()
            .filter(|p| {
                let angle = p.angle().to_degrees();
                angle >= RIGHT_ANGLE_RANGE.0 && angle <= RIGHT_ANGLE_RANGE.1
            })
            .collect::<Vec<_>>();

        let largest_left = Self::find_largest_group(left_points.as_slice());
        let largest_right = Self::find_largest_group(right_points.as_slice());

        match (largest_left, largest_right) {
            (Some(left), Some(right)) if left.distance() > right.distance() => {
                // right is closer
                self.submit_hight_five(right, LegFlags::RIGHT_FRONT).await;
            }
            (Some(left), Some(_right)) => {
                // left is closer
                self.submit_hight_five(left, LegFlags::LEFT_FRONT).await;
            }
            (Some(left), None) => {
                self.submit_hight_five(left, LegFlags::LEFT_FRONT).await;
            }
            (None, Some(right)) => {
                self.submit_hight_five(right, LegFlags::RIGHT_FRONT).await;
            }
            (None, None) => (),
        }
    }

    /// Find middle point of largest group of points within bounds
    ///
    /// This means if there are two objects in the scene we will find the bigger one
    /// and return its center point
    fn find_largest_group(points: &[&ScanPoint]) -> Option<ScanPoint> {
        // sort into groups
        let mut groups = vec![];
        let mut current_group = vec![];
        for point in points {
            if point.is_valid()
                && point.distance() <= MAX_DISTANCE
                && point.distance() >= MIN_DISTANCE
            {
                current_group.push(point);
            } else if !current_group.is_empty() {
                groups.push(current_group);
                current_group = vec![];
            }
        }
        // grab last group
        if !current_group.is_empty() {
            groups.push(current_group);
        }

        if groups.is_empty() {
            // no points detected
            return None;
        }

        let largest_group = groups
            .into_iter()
            // find largest group
            .max_by(|a, b| a.len().cmp(&b.len()))
            // safe because we check for empty
            .unwrap();

        // can do median because they are already sorted
        let median_point = largest_group.get(largest_group.len() / 2).unwrap();

        // center angle of biggest group
        Some((***median_point).clone())
    }

    async fn submit_hight_five(&mut self, point: ScanPoint, leg: LegFlags) {
        // project to point
        let x: f32 = point.distance() * (-point.angle()).cos();
        let y = point.distance() * (-point.angle()).sin();

        // translate from lidar coordinate space
        let lidar_to_base_translation = Vector3::new(0.035, 0.0, 0.112);
        let point_transform = Vector3::new(x, y, 0.0) + lidar_to_base_translation;

        let command = HighFiveCommand {
            leg,
            target: point_transform,
        };

        info!("Sending high five at {:?}", point_transform);
        self.last_high_five = Instant::now();
        self.sender.send(command).await.unwrap();
    }

    pub async fn execute_high_five(
        ik_controller: &mut Box<dyn IkControllable>,
        starting_pose: LegPositions,
        hight_five_command: HighFiveCommand,
    ) -> HopperResult<()> {
        let mut foot_lifted_pose = starting_pose.to_owned();

        // lift select foot of the ground so that we can poke
        foot_lifted_pose.updated_from_selected_legs(
            starting_pose
                .selected_legs(hight_five_command.leg)
                .into_iter()
                .map(|point| Point3::new(point.x * 1.2, point.y * 1.2, 0.0))
                .collect::<Vec<_>>()
                .as_slice(),
            hight_five_command.leg,
        )?;

        // position in contact with object
        let mut high_five_pose = starting_pose.to_owned();
        high_five_pose.updated_from_selected_legs(
            &[Point3::from(hight_five_command.target)],
            hight_five_command.leg,
        )?;

        const TICK_DURATION: Duration = Duration::from_millis(1000 / 50);
        const BODY_LIFT_SPEED: f32 = 0.003;
        let mut interval = tokio::time::interval(TICK_DURATION);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        // lift foot
        for step in starting_pose.to_move_towards_iter(&foot_lifted_pose, BODY_LIFT_SPEED) {
            ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }
        // boop
        for step in foot_lifted_pose.to_move_towards_iter(&high_five_pose, BODY_LIFT_SPEED) {
            ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }
        // wait for a little while touching
        tokio::time::sleep(Duration::from_millis(1000)).await;
        // pull foot back
        for step in high_five_pose.to_move_towards_iter(&foot_lifted_pose, BODY_LIFT_SPEED) {
            ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }
        // return to home position
        for step in foot_lifted_pose.to_move_towards_iter(&starting_pose, BODY_LIFT_SPEED) {
            ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        Ok(())
    }
}
