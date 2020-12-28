use crate::ik_controller::leg_positions::LegPositions;
use anyhow::Result;
use lazy_static::lazy_static;
use nalgebra::Point3;
use std::fs;

const LEG_HEIGHT: f32 = -0.09;
const LEG_DISTANCE_LONGITUDAL: f32 = 0.15;
const MIDDLE_LEG_LONGITUDAL_OFFSET: f32 = 0.07;
const LEG_DISTANCE_LATERAL: f32 = 0.18;
const OFFSET_DISTANCE: f32 = 0.015;
const GROUND_LEG_HEIGHT: f32 = -0.03;

lazy_static! {
    pub(crate) static ref RELAXED: LegPositions = LegPositions::new(
        Point3::new(LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
        Point3::new(
            0.0,
            LEG_DISTANCE_LONGITUDAL + MIDDLE_LEG_LONGITUDAL_OFFSET,
            LEG_HEIGHT,
        ),
        Point3::new(-LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
        Point3::new(LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
        Point3::new(
            0.0,
            -LEG_DISTANCE_LONGITUDAL - MIDDLE_LEG_LONGITUDAL_OFFSET,
            LEG_HEIGHT,
        ),
        Point3::new(-LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    );
    pub(crate) static ref GROUNDED: LegPositions = LegPositions::new(
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
    pub(crate) static ref RELAXED_WIDE: LegPositions = LegPositions::new(
        Point3::new(
            LEG_DISTANCE_LATERAL + OFFSET_DISTANCE,
            LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE,
            LEG_HEIGHT,
        ),
        Point3::new(
            0.0,
            LEG_DISTANCE_LONGITUDAL + MIDDLE_LEG_LONGITUDAL_OFFSET + OFFSET_DISTANCE,
            LEG_HEIGHT,
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL - OFFSET_DISTANCE,
            LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE,
            LEG_HEIGHT,
        ),
        Point3::new(
            LEG_DISTANCE_LATERAL + OFFSET_DISTANCE,
            -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE,
            LEG_HEIGHT,
        ),
        Point3::new(
            0.0,
            -LEG_DISTANCE_LONGITUDAL - MIDDLE_LEG_LONGITUDAL_OFFSET - OFFSET_DISTANCE,
            LEG_HEIGHT,
        ),
        Point3::new(
            -LEG_DISTANCE_LATERAL - OFFSET_DISTANCE,
            -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE,
            LEG_HEIGHT,
        ),
    );
}

pub fn save_basic() -> Result<()> {
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
