use crate::{
    hexapod::LegFlags,
    ik_controller::leg_positions::{LegPositions, MoveTowards},
};
use nalgebra::{UnitQuaternion, Vector3};
use rand::Rng;

#[derive(Debug, Clone, Copy)]
pub enum DanceMove {
    HappyDance,
    SadEmote,
    WaveHi,
}

impl DanceMove {
    pub fn to_iterator(
        &self,
        starting_pose: LegPositions,
    ) -> Box<dyn Iterator<Item = LegPositions>> {
        match self {
            Self::HappyDance => Box::new(HappyDanceIter::new(starting_pose)),
            Self::SadEmote => Box::new(SadEmoteIter::new(starting_pose)),
            Self::WaveHi => Box::new(WaveHiIter::new(starting_pose)),
        }
    }
}

struct HappyDanceIter {
    poses: Vec<LegPositions>,
}

impl HappyDanceIter {
    pub fn new(starting_pose: LegPositions) -> Self {
        const SPEED: f32 = 0.002;
        let separated_legs = starting_pose.transform(
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(0.0, 0.09, 0.0),
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
        for step in starting_pose.to_move_towards_iter(&leaning_left, SPEED) {
            poses.push(step);
        }
        for _ in 0..count {
            for step in leaning_left.to_move_towards_iter(&leaning_right, SPEED) {
                poses.push(step);
            }
            for step in leaning_right.to_move_towards_iter(&leaning_left, SPEED) {
                poses.push(step);
            }
        }
        for step in leaning_left.to_move_towards_iter(&starting_pose, SPEED) {
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

struct SadEmoteIter {
    poses: Vec<LegPositions>,
}

impl SadEmoteIter {
    pub fn new(starting_pose: LegPositions) -> Self {
        const SPEED: f32 = 0.001;

        let relaxed_pose =
            starting_pose.transform(Vector3::new(0.0, 0.0, 0.02), UnitQuaternion::identity());

        let mut poses = vec![];

        for step in starting_pose.to_move_towards_iter(&relaxed_pose, SPEED) {
            poses.push(step);
        }

        let a = relaxed_pose.transform(
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(0.0, -0.08, 0.0),
        );
        for step in relaxed_pose.to_move_towards_iter(&a, SPEED) {
            poses.push(step);
        }

        let last = poses.last().unwrap().clone();
        for _ in 0..40 {
            poses.push(last.clone())
        }

        for step in a.to_move_towards_iter(&starting_pose, SPEED) {
            poses.push(step);
        }

        Self { poses }
    }
}

impl Iterator for SadEmoteIter {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        self.poses.pop()
    }
}

struct WaveHiIter {
    poses: Vec<LegPositions>,
}

impl WaveHiIter {
    pub fn new(starting_pose: LegPositions) -> Self {
        const SPEED: f32 = 0.003;

        let mut lifted_paw = starting_pose
            .transform(
                Vector3::new(0.0, 0.0, -0.02),
                UnitQuaternion::from_euler_angles(-0.05, 0.05, 0.0),
            )
            .transform_selected_legs(
                Vector3::new(0.1, 0.02, 0.0),
                UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
                LegFlags::LEFT_FRONT,
            );
        let left_front = *lifted_paw.left_front();

        lifted_paw =
            lifted_paw.updated_left_front(Vector3::new(left_front.x, left_front.y, 0.04).into());

        let mut poses = vec![];

        let paw_a = lifted_paw.transform_selected_legs(
            Vector3::new(0.0, 0.0, 0.02),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            LegFlags::LEFT_FRONT,
        );

        let paw_b = lifted_paw.transform_selected_legs(
            Vector3::new(0.0, 0.0, -0.02),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            LegFlags::LEFT_FRONT,
        );

        // lift paw
        for step in starting_pose.to_move_towards_iter(&lifted_paw, SPEED) {
            poses.push(step);
        }

        const WAVE_SPEED: f32 = 0.003;

        let mut rng = rand::thread_rng();
        let count = rng.gen_range(3..6);
        for _ in 0..count {
            for step in paw_b.to_move_towards_iter(&paw_a, WAVE_SPEED) {
                poses.push(step);
            }
            for step in paw_a.to_move_towards_iter(&paw_b, WAVE_SPEED) {
                poses.push(step);
            }
        }

        // lower paw
        for step in lifted_paw.to_move_towards_iter(&starting_pose, SPEED) {
            poses.push(step);
        }

        Self { poses }
    }
}

impl Iterator for WaveHiIter {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        self.poses.pop()
    }
}
