use crate::{
    hexapod::LegFlags,
    ik_controller::leg_positions::{LegPositions, MoveTowards},
};
use nalgebra::{UnitQuaternion, Vector3};
use rand::{seq::SliceRandom, Rng};

#[derive(Debug, Clone, Copy)]
pub enum DanceMove {
    Random,
    HappyDance,
    SadEmote,
    WaveHi,
    Roar,
    CombatCry,
    BoredLookingAround,
    BoredStretch,
}

impl DanceMove {
    pub fn to_iterator(
        &self,
        starting_pose: LegPositions,
    ) -> Box<dyn Iterator<Item = LegPositions>> {
        match self {
            Self::Random => {
                let moves = [
                    Self::HappyDance,
                    Self::SadEmote,
                    Self::WaveHi,
                    Self::Roar,
                    Self::CombatCry,
                ];
                let choice = moves.choose(&mut rand::thread_rng()).unwrap();
                choice.to_iterator(starting_pose)
            }
            Self::HappyDance => Box::new(HappyDanceIter::new(starting_pose)),
            Self::SadEmote => Box::new(SadEmoteIter::new(starting_pose)),
            Self::WaveHi => Box::new(WaveHiIter::new(starting_pose)),
            Self::Roar => Box::new(RoarIter::new(starting_pose)),
            Self::CombatCry => Box::new(CombatCryIter::new(starting_pose)),
            Self::BoredLookingAround => Box::new(vec![].into_iter()),
            Self::BoredStretch => Box::new(vec![].into_iter()),
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

        poses.extend(starting_pose.to_move_towards_iter(&relaxed_pose, SPEED));

        let a = relaxed_pose.transform(
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(0.0, -0.08, 0.0),
        );
        poses.extend(relaxed_pose.to_move_towards_iter(&a, SPEED));

        // wait
        let mut rng = rand::thread_rng();
        let count = rng.gen_range(5..8) * 10;
        let last = *poses.last().unwrap();
        for _ in 0..count {
            poses.push(last)
        }

        poses.extend(a.to_move_towards_iter(&starting_pose, SPEED));

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
        const BODY_LIFT_SPEED: f32 = 0.003;

        let mut poses = vec![];

        let lifted_body = starting_pose.transform(
            Vector3::new(0.0, 0.0, -0.02),
            UnitQuaternion::from_euler_angles(-0.07, 0.07, 0.0),
        );

        for step in starting_pose.to_move_towards_iter(&lifted_body, BODY_LIFT_SPEED) {
            poses.push(step);
        }

        const PAW_LIFT_SPEED: f32 = 0.007;

        let mut lifted_paw = lifted_body.transform_selected_legs(
            Vector3::new(0.1, 0.02, 0.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            LegFlags::LEFT_FRONT,
        );
        let left_front = *lifted_paw.left_front();

        lifted_paw =
            lifted_paw.updated_left_front(Vector3::new(left_front.x, left_front.y, 0.04).into());

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
        for step in lifted_body.to_move_towards_iter(&lifted_paw, PAW_LIFT_SPEED) {
            poses.push(step);
        }

        const WAVE_SPEED: f32 = 0.005;

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
        for step in lifted_paw.to_move_towards_iter(&starting_pose, BODY_LIFT_SPEED) {
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

struct RoarIter {
    poses: Vec<LegPositions>,
}

impl RoarIter {
    fn new(starting_pose: LegPositions) -> Self {
        let lifted_middle = starting_pose.transform_selected_legs(
            Vector3::new(0.06, 0.0, 0.04),
            UnitQuaternion::identity(),
            LegFlags::MIDDLE,
        );

        let grounded_middle_front = starting_pose
            .transform_selected_legs(
                Vector3::new(0.06, 0.0, 0.0),
                UnitQuaternion::identity(),
                LegFlags::MIDDLE,
            )
            .transform_selected_legs(
                Vector3::new(0.0, -0.02, 0.0),
                UnitQuaternion::identity(),
                LegFlags::LEFT_MIDDLE,
            )
            .transform_selected_legs(
                Vector3::new(0.0, 0.02, 0.0),
                UnitQuaternion::identity(),
                LegFlags::RIGHT_MIDDLE,
            );

        let lifted_front = grounded_middle_front
            .transform_selected_legs(
                Vector3::new(0.10, 0.0, 0.14),
                UnitQuaternion::identity(),
                LegFlags::FRONT,
            )
            .transform(
                Vector3::zeros(),
                UnitQuaternion::from_euler_angles(0.0, 0.14, 0.0),
            )
            .transform_selected_legs(
                Vector3::new(0.0, -0.06, 0.0),
                UnitQuaternion::identity(),
                LegFlags::LEFT_FRONT,
            )
            .transform_selected_legs(
                Vector3::new(0.0, 0.06, 0.0),
                UnitQuaternion::identity(),
                LegFlags::RIGHT_FRONT,
            );
        let lifted_front_retracted = lifted_front
            .transform_selected_legs(
                Vector3::new(-0.06, 0.0, 0.0),
                UnitQuaternion::identity(),
                LegFlags::FRONT,
            )
            .transform_selected_legs(
                Vector3::new(0.0, 0.06, 0.0),
                UnitQuaternion::identity(),
                LegFlags::LEFT_FRONT,
            )
            .transform_selected_legs(
                Vector3::new(0.0, -0.06, 0.0),
                UnitQuaternion::identity(),
                LegFlags::RIGHT_FRONT,
            );
        let lifted_left = lifted_front
            .transform_selected_legs(
                Vector3::new(0.0, 0.0, -0.02),
                UnitQuaternion::identity(),
                LegFlags::LEFT_FRONT,
            )
            .transform_selected_legs(
                Vector3::new(0.0, 0.0, 0.02),
                UnitQuaternion::identity(),
                LegFlags::RIGHT_FRONT,
            );
        let lifted_right = lifted_front
            .transform_selected_legs(
                Vector3::new(0.0, 0.0, 0.02),
                UnitQuaternion::identity(),
                LegFlags::LEFT_FRONT,
            )
            .transform_selected_legs(
                Vector3::new(0.0, 0.0, -0.02),
                UnitQuaternion::identity(),
                LegFlags::RIGHT_FRONT,
            );

        const SLOW_SPEED: f32 = 0.004;
        const FAST_SPEED: f32 = 0.007;

        let mut poses = vec![];
        poses.extend(starting_pose.to_move_towards_iter(&lifted_middle, SLOW_SPEED));
        poses.extend(lifted_middle.to_move_towards_iter(&grounded_middle_front, SLOW_SPEED));
        poses.extend(grounded_middle_front.to_move_towards_iter(&lifted_front, FAST_SPEED));

        let mut rng = rand::thread_rng();
        let count = rng.gen_range(4..7);

        poses.extend(lifted_front.to_move_towards_iter(&lifted_left, FAST_SPEED));
        for _ in 0..count {
            poses.extend(lifted_left.to_move_towards_iter(&lifted_right, FAST_SPEED));
            poses.extend(lifted_right.to_move_towards_iter(&lifted_left, FAST_SPEED));
        }

        poses.extend(lifted_left.to_move_towards_iter(&lifted_front, SLOW_SPEED));
        poses.extend(lifted_front.to_move_towards_iter(&lifted_front_retracted, FAST_SPEED));
        poses.extend(
            lifted_front_retracted.to_move_towards_iter(&grounded_middle_front, SLOW_SPEED),
        );
        poses.extend(grounded_middle_front.to_move_towards_iter(&lifted_middle, SLOW_SPEED));
        poses.extend(lifted_middle.to_move_towards_iter(&starting_pose, SLOW_SPEED));

        Self { poses }
    }
}

impl Iterator for RoarIter {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        self.poses.pop()
    }
}

struct CombatCryIter {
    poses: Vec<LegPositions>,
}

impl CombatCryIter {
    fn new(starting_pose: LegPositions) -> Self {
        let lifted = starting_pose
            .transform(
                Vector3::new(0.0, 0.0, 0.02),
                UnitQuaternion::from_euler_angles(0.05, 0.05, 0.0),
            )
            .transform_selected_legs(
                Vector3::new(0.1, -0.02, 0.06),
                UnitQuaternion::identity(),
                LegFlags::RIGHT_FRONT,
            );

        let paw_lifted = lifted.transform_selected_legs(
            Vector3::new(0.0, 0.00, 0.02),
            UnitQuaternion::identity(),
            LegFlags::RIGHT_FRONT,
        );

        const SPEED: f32 = 0.004;

        let mut poses = vec![];
        poses.extend(starting_pose.to_move_towards_iter(&lifted, SPEED));

        let mut rng = rand::thread_rng();
        let count = rng.gen_range(4..7);
        for _ in 0..count {
            poses.extend(lifted.to_move_towards_iter(&paw_lifted, SPEED));
            poses.extend(paw_lifted.to_move_towards_iter(&lifted, SPEED));
        }

        poses.extend(paw_lifted.to_move_towards_iter(&lifted, SPEED));
        poses.extend(lifted.to_move_towards_iter(&starting_pose, SPEED));

        Self { poses }
    }
}

impl Iterator for CombatCryIter {
    type Item = LegPositions;

    fn next(&mut self) -> Option<Self::Item> {
        self.poses.pop()
    }
}
