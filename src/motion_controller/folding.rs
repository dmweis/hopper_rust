use std::time::Duration;

use crate::{
    body_controller::{
        motor_positions::{
            LegMotorPositions, OptionalBodyMotorPositions, OptionalLegMotorPositions,
        },
        BodyMotorPositions,
    },
    error::HopperResult,
    ik_controller::IkControllable,
};

trait Lerp {
    fn lerp(&self, other: &Self, t: f32) -> Self;
}

impl Lerp for f32 {
    fn lerp(&self, other: &Self, t: f32) -> Self {
        self + (other - self) * t
    }
}

trait MoveTowards {
    fn move_towards(&self, other: &Self, max_step: f32) -> Option<f32>;
}

impl MoveTowards for f32 {
    fn move_towards(&self, other: &Self, max_step: f32) -> Option<f32> {
        if self == other {
            return None;
        }
        let diff = other - self;
        if diff.abs() < max_step {
            Some(*other)
        } else {
            Some(self + diff.signum() * max_step)
        }
    }
}

#[allow(dead_code)]
fn lerp_leg_positions(
    a: &LegMotorPositions,
    b: &OptionalLegMotorPositions,
    t: f32,
) -> LegMotorPositions {
    let coxa = a.coxa().lerp(&b.coxa().unwrap_or_else(|| a.coxa()), t);
    let femur = a.femur().lerp(&b.femur().unwrap_or_else(|| a.femur()), t);
    let tibia = a.tibia().lerp(&b.tibia().unwrap_or_else(|| a.tibia()), t);
    LegMotorPositions::new(coxa, femur, tibia)
}

fn move_leg_motors_towards(
    a: &LegMotorPositions,
    b: &OptionalLegMotorPositions,
    max_step: f32,
) -> OptionalLegMotorPositions {
    let coxa = a
        .coxa()
        .move_towards(&b.coxa().unwrap_or_else(|| a.coxa()), max_step);
    let femur = a
        .femur()
        .move_towards(&b.femur().unwrap_or_else(|| a.femur()), max_step);
    let tibia = a
        .tibia()
        .move_towards(&b.tibia().unwrap_or_else(|| a.tibia()), max_step);
    OptionalLegMotorPositions::new(coxa, femur, tibia)
}

fn move_body_motors_towards(
    a: &BodyMotorPositions,
    b: &OptionalBodyMotorPositions,
    max_step: f32,
) -> OptionalBodyMotorPositions {
    let lf = move_leg_motors_towards(a.left_front(), b.left_front(), max_step);
    let lm = move_leg_motors_towards(a.left_middle(), b.left_middle(), max_step);
    let lr = move_leg_motors_towards(a.left_rear(), b.left_rear(), max_step);
    let rf = move_leg_motors_towards(a.right_front(), b.right_front(), max_step);
    let rm = move_leg_motors_towards(a.right_middle(), b.right_middle(), max_step);
    let rr = move_leg_motors_towards(a.right_rear(), b.right_rear(), max_step);
    OptionalBodyMotorPositions::new(lf, lm, lr, rf, rm, rr)
}

pub struct FoldingManager<'a> {
    ik_controller: &'a mut Box<dyn IkControllable>,
    last_written_pose: BodyMotorPositions,
}

impl<'a> FoldingManager<'a> {
    pub async fn new(
        ik_controller: &'a mut Box<dyn IkControllable>,
    ) -> HopperResult<FoldingManager> {
        let last_written_pose = ik_controller.read_motor_positions().await?;
        Ok(Self {
            ik_controller,
            last_written_pose,
        })
    }

    async fn move_towards(
        &mut self,
        target_body_motor_positions: &OptionalBodyMotorPositions,
        max_step: f32,
    ) -> HopperResult<()> {
        loop {
            let optional_body_motor_positions = move_body_motors_towards(
                &self.last_written_pose,
                target_body_motor_positions,
                max_step,
            );
            if optional_body_motor_positions.is_none() {
                break;
            } else {
                self.last_written_pose =
                    optional_body_motor_positions.merge_with_none_optional(&self.last_written_pose);
                self.ik_controller
                    .move_optional_motors_to(&optional_body_motor_positions)
                    .await?;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        Ok(())
    }

    async fn fold_femur_tibia(&mut self) -> HopperResult<()> {
        let rad_60 = Some(60.0_f32.to_radians());
        let rad_240 = Some(240.0_f32.to_radians());
        let max_step = 1.3_f32.to_radians();

        // target
        let lf = OptionalLegMotorPositions::new(None, rad_60, rad_240);
        let lm = OptionalLegMotorPositions::new(None, rad_60, rad_240);
        let lr = OptionalLegMotorPositions::new(None, rad_60, rad_240);
        let rf = OptionalLegMotorPositions::new(None, rad_240, rad_60);
        let rm = OptionalLegMotorPositions::new(None, rad_240, rad_60);
        let rr = OptionalLegMotorPositions::new(None, rad_240, rad_60);
        let target_body_motor_positions = OptionalBodyMotorPositions::new(lf, lm, lr, rf, rm, rr);
        self.move_towards(&target_body_motor_positions, max_step)
            .await?;
        Ok(())
    }

    pub async fn fold(&mut self) -> HopperResult<()> {
        self.fold_femur_tibia().await?;
        let max_step = 1.3_f32.to_radians();
        // TODO: Check if folded
        // straighten out side legs
        let straightened_sides = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&straightened_sides, max_step).await?;

        // fold edge legs
        let folded_front_and_rear = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::new(Some(240.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(60.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::new(Some(60.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(240.0_f32.to_radians()), None, None),
        );
        self.move_towards(&folded_front_and_rear, max_step).await?;
        // fold sides back
        let folded_sides = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(240.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(60.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&folded_sides, max_step).await?;

        // done
        tokio::time::sleep(Duration::from_millis(200)).await;
        self.ik_controller.disable_motors().await?;
        Ok(())
    }

    pub async fn unfold(&mut self) -> HopperResult<()> {
        self.fold_femur_tibia().await?;
        let max_step = 1.3_f32.to_radians();
        // straighten out side legs
        let straightened_sides = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&straightened_sides, max_step).await?;

        // straighten edge legs
        let straightened_out_front_and_readr = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
        );
        self.move_towards(&straightened_out_front_and_readr, max_step)
            .await?;

        // lift tibias
        let lifted_tibias = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::new(None, None, Some(210.0_f32.to_radians())),
            OptionalLegMotorPositions::new(None, None, Some(210.0_f32.to_radians())),
            OptionalLegMotorPositions::new(None, None, Some(210.0_f32.to_radians())),
            OptionalLegMotorPositions::new(None, None, Some(90.0_f32.to_radians())),
            OptionalLegMotorPositions::new(None, None, Some(90.0_f32.to_radians())),
            OptionalLegMotorPositions::new(None, None, Some(90.0_f32.to_radians())),
        );
        self.move_towards(&lifted_tibias, max_step).await?;

        // done
        tokio::time::sleep(Duration::from_millis(200)).await;
        self.ik_controller.disable_motors().await?;
        Ok(())
    }

    pub async fn unfold_on_ground(&mut self) -> HopperResult<()> {
        self.fold_femur_tibia().await?;
        let max_step = 1.3_f32.to_radians();

        let middle_legs_lifted_tibias = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(None, None, Some(200.0_f32.to_radians())),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(None, None, Some(100.0_f32.to_radians())),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&middle_legs_lifted_tibias, max_step)
            .await?;

        let middle_legs_folded_out = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&middle_legs_folded_out, max_step).await?;

        let right_side_lifted = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(
                None,
                Some(170.0_f32.to_radians()),
                Some(100.0_f32.to_radians()),
            ),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&right_side_lifted, max_step).await?;

        let unfolded_right_legs = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
        );
        self.move_towards(&unfolded_right_legs, max_step).await?;

        let lifted_right_legs = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(None, None, Some(90.0_f32.to_radians())),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(None, None, Some(90.0_f32.to_radians())),
        );
        self.move_towards(&lifted_right_legs, max_step).await?;

        // switch sides
        let left_side_lifted = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(
                None,
                Some(130.0_f32.to_radians()),
                Some(200.0_f32.to_radians()),
            ),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(
                None,
                Some(240.0_f32.to_radians()),
                Some(90.0_f32.to_radians()),
            ),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&left_side_lifted, max_step).await?;

        let unfolded_left_legs = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(Some(150.0_f32.to_radians()), None, None),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&unfolded_left_legs, max_step).await?;

        let lifted_left_legs = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::new(None, None, Some(210.0_f32.to_radians())),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(None, None, Some(210.0_f32.to_radians())),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&lifted_left_legs, max_step).await?;

        let lift_left_middle = OptionalBodyMotorPositions::new(
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::new(
                None,
                Some(60.0_f32.to_radians()),
                Some(210.0_f32.to_radians()),
            ),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
            OptionalLegMotorPositions::default(),
        );
        self.move_towards(&lift_left_middle, max_step).await?;

        // done
        tokio::time::sleep(Duration::from_millis(200)).await;
        self.ik_controller.disable_motors().await?;
        Ok(())
    }
}
