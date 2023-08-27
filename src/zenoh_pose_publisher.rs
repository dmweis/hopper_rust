use std::sync::{Arc, Mutex};

use crate::error::HopperResult;
use crate::ik_controller::leg_positions::*;
use prost::Message;
use tracing::*;
use zenoh::prelude::r#async::*;

pub struct ZenohPosePublisher {
    latest: Arc<Mutex<Option<LegPositions>>>,
}

impl ZenohPosePublisher {
    pub fn new(pose_publisher: zenoh::publication::Publisher<'static>) -> Self {
        let latest = Arc::new(Mutex::new(None));

        tokio::spawn({
            let latest = latest.clone();
            let pose_publisher = pose_publisher;
            async move {
                let mut interval = tokio::time::interval(std::time::Duration::from_millis(500));
                interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
                loop {
                    let latest = latest.lock().unwrap().take();
                    if let Some(latest) = latest {
                        if let Err(e) = Self::publish_pose(&pose_publisher, latest).await {
                            error!("Error publishing pose: {}", e);
                        }
                    }
                    interval.tick().await;
                }
            }
        });

        Self { latest }
    }

    pub fn set_pose(&self, pose: LegPositions) {
        self.latest.lock().unwrap().replace(pose);
    }

    async fn publish_pose(
        pose_publisher: &zenoh::publication::Publisher<'static>,
        positions: LegPositions,
    ) -> HopperResult<()> {
        pose_publisher
            .put(positions.to_foxglove_frame_transport().encode_to_vec())
            .res()
            .await?;
        Ok(())
    }
}
