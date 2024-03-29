use crate::{error::HopperResult, ik_controller::leg_positions::LegPositions};
use lazy_static::lazy_static;
use nalgebra::Point3;
use std::fs;

pub const STANDING_LEG_HEIGHT: f32 = -0.10;
const LEG_DISTANCE_LONGITUDAL: f32 = 0.13;
const MIDDLE_LEG_LONGITUDAL_OFFSET: f32 = 0.07;
const LEG_DISTANCE_LATERAL: f32 = 0.18;
const OFFSET_DISTANCE: f32 = 0.017;
pub const GROUND_LEG_HEIGHT: f32 = -0.03;

lazy_static! {
    static ref RELAXED: LegPositions = LegPositions::new(
        Point3::new(
            LEG_DISTANCE_LATERAL,
            LEG_DISTANCE_LONGITUDAL,
            STANDING_LEG_HEIGHT
        ),
        Point3::new(
            0.0,
            LEG_DISTANCE_LONGITUDAL + MIDDLE_LEG_LONGITUDAL_OFFSET,
            STANDING_LEG_HEIGHT,
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL,
            LEG_DISTANCE_LONGITUDAL,
            STANDING_LEG_HEIGHT
        ),
        Point3::new(
            LEG_DISTANCE_LATERAL,
            -LEG_DISTANCE_LONGITUDAL,
            STANDING_LEG_HEIGHT
        ),
        Point3::new(
            0.0,
            -LEG_DISTANCE_LONGITUDAL - MIDDLE_LEG_LONGITUDAL_OFFSET,
            STANDING_LEG_HEIGHT,
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL,
            -LEG_DISTANCE_LONGITUDAL,
            STANDING_LEG_HEIGHT
        ),
    );
    static ref GROUNDED: LegPositions = LegPositions::new(
        Point3::new(
            LEG_DISTANCE_LATERAL + OFFSET_DISTANCE,
            LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE,
            GROUND_LEG_HEIGHT,
        ),
        Point3::new(
            0.0,
            LEG_DISTANCE_LONGITUDAL + MIDDLE_LEG_LONGITUDAL_OFFSET + OFFSET_DISTANCE,
            GROUND_LEG_HEIGHT,
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL - OFFSET_DISTANCE,
            LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE,
            GROUND_LEG_HEIGHT,
        ),
        Point3::new(
            LEG_DISTANCE_LATERAL + OFFSET_DISTANCE,
            -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE,
            GROUND_LEG_HEIGHT,
        ),
        Point3::new(
            0.0,
            -LEG_DISTANCE_LONGITUDAL - MIDDLE_LEG_LONGITUDAL_OFFSET - OFFSET_DISTANCE,
            GROUND_LEG_HEIGHT,
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL - OFFSET_DISTANCE,
            -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE,
            GROUND_LEG_HEIGHT,
        ),
    );
    static ref RELAXED_WIDE: LegPositions = LegPositions::new(
        Point3::new(
            LEG_DISTANCE_LATERAL + OFFSET_DISTANCE,
            LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE,
            STANDING_LEG_HEIGHT,
        ),
        Point3::new(
            0.0,
            LEG_DISTANCE_LONGITUDAL + MIDDLE_LEG_LONGITUDAL_OFFSET + OFFSET_DISTANCE,
            STANDING_LEG_HEIGHT,
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL - OFFSET_DISTANCE,
            LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE,
            STANDING_LEG_HEIGHT,
        ),
        Point3::new(
            LEG_DISTANCE_LATERAL + OFFSET_DISTANCE,
            -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE,
            STANDING_LEG_HEIGHT,
        ),
        Point3::new(
            0.0,
            -LEG_DISTANCE_LONGITUDAL - MIDDLE_LEG_LONGITUDAL_OFFSET - OFFSET_DISTANCE,
            STANDING_LEG_HEIGHT,
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL - OFFSET_DISTANCE,
            -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE,
            STANDING_LEG_HEIGHT,
        ),
    );
}

pub fn relaxed_stance() -> &'static LegPositions {
    &RELAXED
}

pub fn grounded_stance() -> &'static LegPositions {
    &GROUNDED
}

pub fn relaxed_wide_stance() -> &'static LegPositions {
    &RELAXED_WIDE
}

fn random_float(range: f32) -> f32 {
    (rand::random::<f32>() - 0.5) * 2.0 * range
}

pub fn random_grounded_stance() -> LegPositions {
    LegPositions::new(
        Point3::new(
            LEG_DISTANCE_LATERAL + OFFSET_DISTANCE + random_float(0.03),
            LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE + random_float(0.03),
            GROUND_LEG_HEIGHT + random_float(0.03),
        ),
        Point3::new(
            0.0 + random_float(0.03),
            LEG_DISTANCE_LONGITUDAL
                + MIDDLE_LEG_LONGITUDAL_OFFSET
                + OFFSET_DISTANCE
                + random_float(0.03),
            GROUND_LEG_HEIGHT + random_float(0.03),
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL + OFFSET_DISTANCE - random_float(0.03),
            LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE + random_float(0.03),
            GROUND_LEG_HEIGHT + random_float(0.03),
        ),
        Point3::new(
            LEG_DISTANCE_LATERAL + OFFSET_DISTANCE + random_float(0.03),
            -LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE - random_float(0.03),
            GROUND_LEG_HEIGHT + random_float(0.03),
        ),
        Point3::new(
            0.0 + random_float(0.03),
            -LEG_DISTANCE_LONGITUDAL
                - MIDDLE_LEG_LONGITUDAL_OFFSET
                - OFFSET_DISTANCE
                - random_float(0.03),
            GROUND_LEG_HEIGHT + random_float(0.03),
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL - OFFSET_DISTANCE + random_float(0.03),
            -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE + random_float(0.03),
            GROUND_LEG_HEIGHT + random_float(0.03),
        ),
    )
}

pub fn save_basic() -> HopperResult<()> {
    fs::write(
        "config/stance/relaxed.toml",
        toml::to_string_pretty(&*RELAXED)?,
    )?;
    fs::write(
        "config/stance/grounded.toml",
        toml::to_string_pretty(&*GROUNDED)?,
    )?;
    fs::write(
        "config/stance/relaxed_wide.toml",
        toml::to_string_pretty(&*RELAXED_WIDE)?,
    )?;
    Ok(())
}
