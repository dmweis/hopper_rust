use crate::ik_controller::leg_positions::{LegPositions, MoveTowards};
use nalgebra::{UnitQuaternion, Vector3};
use rand::Rng;

#[derive(Debug, Clone, Copy)]
pub enum DanceMove {
    HappyDance,
}

impl DanceMove {
    pub fn to_iterator(
        &self,
        starting_pose: LegPositions,
    ) -> Box<dyn Iterator<Item = LegPositions>> {
        match self {
            Self::HappyDance => Box::new(HappyDanceIter::new(starting_pose)),
        }
    }
}

struct HappyDanceIter {
    poses: Vec<LegPositions>,
}

impl HappyDanceIter {
    pub fn new(starting_pose: LegPositions) -> Self {
        let separated_legs = starting_pose.transform(
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(0.0, 0.08, 0.0),
        );
        let leaning_left = separated_legs.transform(
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(-0.02, 0.0, 0.0),
        );
        let leaning_right = separated_legs.transform(
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(0.02, 0.0, 0.0),
        );
        let mut poses = vec![];
        let mut rng = rand::thread_rng();

        let count = rng.gen_range(3..6);
        for step in starting_pose.to_move_towards_iter(&leaning_left, 0.001) {
            poses.push(step);
        }
        for _ in 0..count {
            for step in leaning_left.to_move_towards_iter(&leaning_right, 0.001) {
                poses.push(step);
            }
            for step in leaning_right.to_move_towards_iter(&leaning_left, 0.001) {
                poses.push(step);
            }
        }
        for step in leaning_left.to_move_towards_iter(&starting_pose, 0.001) {
            poses.push(step);
        }
        Self { poses }
    }
}

impl Iterator for HappyDanceIter {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        self.poses.pop()
    }
}
