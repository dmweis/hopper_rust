use nalgebra::{Point3, UnitQuaternion, Vector3};
use rand::{seq::SliceRandom, Rng};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::time::Duration;
use tracing::info;

use crate::{
    error::HopperResult,
    hexapod::LegFlags,
    ik_controller::{
        leg_positions::{LegPositions, MoveTowards},
        IkControllable,
    },
    ioc_container::IocContainer,
    speech::SpeechService,
};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum DanceMove {
    /// Perform a random dance move
    Random,
    /// Do a little happy shake to express that you are happy and content
    HappyDance,
    SadEmote,
    /// Wave hello with your paw
    WaveHi,
    /// Wave hello with your paw while playing a sound bite
    /// Ignore for schema because we don't want to expose this to the user
    #[schemars(skip)]
    WaveHiWithSound,
    /// Lift front legs and roar threateningly
    Roar,
    CombatCry,
    /// lift one leg as a demonstration
    LiftLeg {
        leg: LiftedLeg,
        /// time in seconds
        /// Default 4 seconds
        time: u32,
    },
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum LiftedLeg {
    LeftFront,
    LeftMiddle,
    LeftRear,
    RightFront,
    RightMiddle,
    RightRear,
}

impl LiftedLeg {
    pub fn as_leg_flag(&self) -> LegFlags {
        match self {
            LiftedLeg::LeftFront => LegFlags::LEFT_FRONT,
            LiftedLeg::LeftMiddle => LegFlags::LEFT_MIDDLE,
            LiftedLeg::LeftRear => LegFlags::LEFT_REAR,
            LiftedLeg::RightFront => LegFlags::RIGHT_FRONT,
            LiftedLeg::RightMiddle => LegFlags::RIGHT_MIDDLE,
            LiftedLeg::RightRear => LegFlags::RIGHT_REAR,
        }
    }
}

pub struct Choreographer<'a> {
    ik_controller: &'a mut Box<dyn IkControllable>,
    starting_pose: LegPositions,
}

const TICK_DURATION: Duration = Duration::from_millis(1000 / 50);

impl<'a> Choreographer<'a> {
    pub fn new(
        ik_controller: &'a mut Box<dyn IkControllable>,

        starting_pose: LegPositions,
    ) -> HopperResult<Self> {
        Ok(Self {
            ik_controller,
            starting_pose,
        })
    }

    pub async fn execute_move(&mut self, dance: DanceMove) -> HopperResult<()> {
        let dance = if let DanceMove::Random = dance {
            let moves = [
                DanceMove::HappyDance,
                DanceMove::WaveHi,
                DanceMove::Roar,
                DanceMove::CombatCry,
                DanceMove::LiftLeg {
                    leg: LiftedLeg::LeftFront,
                    time: 4,
                },
                DanceMove::LiftLeg {
                    leg: LiftedLeg::RightFront,
                    time: 4,
                },
            ];
            let dance = *moves.choose(&mut rand::thread_rng()).unwrap();
            info!("Executing random dance move: {:?}", dance);
            dance
        } else {
            dance
        };
        match dance {
            DanceMove::Random => unreachable!(),
            DanceMove::WaveHi => self.wave_hi(false).await?,
            DanceMove::WaveHiWithSound => self.wave_hi(true).await?,
            DanceMove::SadEmote => self.sad_emote().await?,
            DanceMove::HappyDance => self.happy_dance().await?,
            DanceMove::Roar => self.roar().await?,
            DanceMove::CombatCry => self.combat_cry().await?,
            DanceMove::LiftLeg { leg, time } => self.lift_leg(leg, time).await?,
        }
        Ok(())
    }

    async fn wave_hi(&mut self, play_audio: bool) -> HopperResult<()> {
        let mut interval = tokio::time::interval(TICK_DURATION);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        const BODY_LIFT_SPEED: f32 = 0.003;

        let lifted_body = self.starting_pose.transform(
            Vector3::new(0.0, 0.0, -0.02),
            UnitQuaternion::from_euler_angles(-0.07, 0.07, 0.0),
        );

        for step in self
            .starting_pose
            .to_move_towards_iter(&lifted_body, BODY_LIFT_SPEED)
        {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
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
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        const WAVE_SPEED: f32 = 0.005;

        let count = {
            let mut rng = rand::thread_rng();
            rng.gen_range(3..6)
        };

        for _ in 0..2 {
            for step in paw_b.to_move_towards_iter(&paw_a, WAVE_SPEED) {
                self.ik_controller.move_to_positions(&step).await?;
                interval.tick().await;
            }
            for step in paw_a.to_move_towards_iter(&paw_b, WAVE_SPEED) {
                self.ik_controller.move_to_positions(&step).await?;
                interval.tick().await;
            }
        }

        if play_audio {
            IocContainer::global_instance()
                .service::<SpeechService>()?
                .play_sound("Turret_turret_active_1.wav")
                .await?;
        }

        for _ in 0..count {
            for step in paw_b.to_move_towards_iter(&paw_a, WAVE_SPEED) {
                self.ik_controller.move_to_positions(&step).await?;
                interval.tick().await;
            }
            for step in paw_a.to_move_towards_iter(&paw_b, WAVE_SPEED) {
                self.ik_controller.move_to_positions(&step).await?;
                interval.tick().await;
            }
        }

        // lower paw
        for step in lifted_paw.to_move_towards_iter(&self.starting_pose, BODY_LIFT_SPEED) {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        Ok(())
    }

    async fn sad_emote(&mut self) -> HopperResult<()> {
        let mut interval = tokio::time::interval(TICK_DURATION);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        const SPEED: f32 = 0.001;

        let relaxed_pose = self
            .starting_pose
            .transform(Vector3::new(0.0, 0.0, 0.02), UnitQuaternion::identity());

        for step in self
            .starting_pose
            .to_move_towards_iter(&relaxed_pose, SPEED)
        {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        let a = relaxed_pose.transform(
            Vector3::zeros(),
            UnitQuaternion::from_euler_angles(0.0, -0.08, 0.0),
        );

        for step in relaxed_pose.to_move_towards_iter(&a, SPEED) {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        // wait
        let count = {
            let mut rng = rand::thread_rng();
            rng.gen_range(5..8) * 10
        };

        tokio::time::sleep(Duration::from_millis(
            TICK_DURATION.as_millis() as u64 * count,
        ))
        .await;

        interval.reset();

        for step in a.to_move_towards_iter(&self.starting_pose, SPEED) {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        Ok(())
    }

    async fn happy_dance(&mut self) -> HopperResult<()> {
        let mut interval = tokio::time::interval(TICK_DURATION);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        const SPEED: f32 = 0.002;
        let separated_legs = self.starting_pose.transform(
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
        let count = {
            let mut rng = rand::thread_rng();
            rng.gen_range(3..6)
        };
        for step in self
            .starting_pose
            .to_move_towards_iter(&leaning_left, SPEED)
        {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }
        for _ in 0..count {
            for step in leaning_left.to_move_towards_iter(&leaning_right, SPEED) {
                self.ik_controller.move_to_positions(&step).await?;
                interval.tick().await;
            }
            for step in leaning_right.to_move_towards_iter(&leaning_left, SPEED) {
                self.ik_controller.move_to_positions(&step).await?;
                interval.tick().await;
            }
        }
        for step in leaning_left.to_move_towards_iter(&self.starting_pose, SPEED) {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }
        Ok(())
    }

    async fn roar(&mut self) -> HopperResult<()> {
        let mut interval = tokio::time::interval(TICK_DURATION);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        let lifted_middle = self.starting_pose.transform_selected_legs(
            Vector3::new(0.06, 0.0, 0.04),
            UnitQuaternion::identity(),
            LegFlags::MIDDLE,
        );

        let grounded_middle_front = self
            .starting_pose
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

        for step in self
            .starting_pose
            .to_move_towards_iter(&lifted_middle, SLOW_SPEED)
        {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        for step in lifted_middle.to_move_towards_iter(&grounded_middle_front, SLOW_SPEED) {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        for step in grounded_middle_front.to_move_towards_iter(&lifted_front, FAST_SPEED) {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        let count = {
            let mut rng = rand::thread_rng();
            rng.gen_range(4..7)
        };

        for step in lifted_front.to_move_towards_iter(&lifted_left, FAST_SPEED) {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        for _ in 0..count {
            for step in lifted_left.to_move_towards_iter(&lifted_right, FAST_SPEED) {
                self.ik_controller.move_to_positions(&step).await?;
                interval.tick().await;
            }
            for step in lifted_right.to_move_towards_iter(&lifted_left, FAST_SPEED) {
                self.ik_controller.move_to_positions(&step).await?;
                interval.tick().await;
            }
        }

        let mut poses = vec![];
        poses.extend(lifted_left.to_move_towards_iter(&lifted_front, SLOW_SPEED));
        poses.extend(lifted_front.to_move_towards_iter(&lifted_front_retracted, FAST_SPEED));
        poses.extend(
            lifted_front_retracted.to_move_towards_iter(&grounded_middle_front, SLOW_SPEED),
        );
        poses.extend(grounded_middle_front.to_move_towards_iter(&lifted_middle, SLOW_SPEED));
        poses.extend(lifted_middle.to_move_towards_iter(&self.starting_pose, SLOW_SPEED));

        for step in poses {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        Ok(())
    }

    async fn combat_cry(&mut self) -> HopperResult<()> {
        let mut interval = tokio::time::interval(TICK_DURATION);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        let lifted = self
            .starting_pose
            .transform(
                Vector3::new(0.0, 0.0, -0.02),
                UnitQuaternion::from_euler_angles(0.1, 0.1, 0.0),
            )
            .transform_selected_legs(
                Vector3::new(0.14, -0.02, 0.14),
                UnitQuaternion::identity(),
                LegFlags::RIGHT_FRONT,
            );

        let paw_lifted = lifted.transform_selected_legs(
            Vector3::new(0.0, 0.0, 0.04),
            UnitQuaternion::identity(),
            LegFlags::RIGHT_FRONT,
        );

        const SPEED: f32 = 0.004;

        let mut poses = vec![];
        poses.extend(self.starting_pose.to_move_towards_iter(&lifted, SPEED));

        let count = {
            let mut rng = rand::thread_rng();
            rng.gen_range(3..5)
        };
        for _ in 0..count {
            poses.extend(lifted.to_move_towards_iter(&paw_lifted, SPEED));
            poses.extend(paw_lifted.to_move_towards_iter(&lifted, SPEED));
        }

        poses.extend(paw_lifted.to_move_towards_iter(&lifted, SPEED));
        poses.extend(lifted.to_move_towards_iter(&self.starting_pose, SPEED));

        for step in poses {
            self.ik_controller.move_to_positions(&step).await?;
            interval.tick().await;
        }

        Ok(())
    }

    async fn lift_leg(&mut self, lifted_leg: LiftedLeg, time: u32) -> HopperResult<()> {
        let mut interval = tokio::time::interval(TICK_DURATION);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        let selected_legs = self.starting_pose.selected_legs(lifted_leg.as_leg_flag());

        let target_positions: Vec<_> = selected_legs
            .iter()
            .map(|leg_position| {
                Point3::new(
                    leg_position.x * 1.3,
                    leg_position.y * 1.3,
                    leg_position.z + 0.05,
                )
            })
            .collect();

        // slightly rotate away from selected leg
        let (x, y) = if selected_legs.len() == 1 {
            selected_legs
                .first()
                .map(|point| point.coords.normalize())
                .map(|vector| (vector.x, vector.y))
                .unwrap_or_default()
        } else {
            (0.0, 0.0)
        };

        let rotation =
            UnitQuaternion::from_euler_angles(-3_f32.to_radians() * y, 3_f32.to_radians() * x, 0.0);

        let mut desired_position = self
            .starting_pose
            .transform(Vector3::new(0_f32, 0_f32, -0.03_f32), rotation);

        desired_position.updated_from_selected_legs(&target_positions, lifted_leg.as_leg_flag())?;

        const SPEED: f32 = 0.004;

        for new_pose in self
            .starting_pose
            .to_move_towards_iter(&desired_position, SPEED)
        {
            self.ik_controller.move_to_positions(&new_pose).await?;
            interval.tick().await;
        }

        // more than 20 seconds would be annoyingly long
        let time = time.min(20);

        tokio::time::sleep(Duration::from_secs(time as u64)).await;
        // just to clear skipped?
        // I think this is not needed
        interval.tick().await;

        for new_pose in desired_position.to_move_towards_iter(&self.starting_pose, SPEED) {
            self.ik_controller.move_to_positions(&new_pose).await?;
            interval.tick().await;
        }

        Ok(())
    }
}
