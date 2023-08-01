use std::sync::Arc;

use prost::Message;
use prost_reflect::ReflectMessage;
use prost_types::Timestamp;
use serde::Serialize;
use tokio::process::Command;
use zenoh::prelude::r#async::*;
use zenoh::publication::Publisher;
use zenoh::Session;

use crate::error::HopperError;

pub async fn start_monitoring_loop(zenoh_session: Arc<Session>) -> anyhow::Result<()> {
    let publisher = zenoh_session
        .declare_publisher("hopper/metrics/diagnostic".to_owned())
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let json_publisher = zenoh_session
        .declare_publisher("hopper/metrics/diagnostic/json".to_owned())
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    tokio::spawn(async move {
        loop {
            tokio::time::sleep(std::time::Duration::from_secs(2)).await;
            if let Err(err) = measure(&publisher, &json_publisher).await {
                tracing::error!("Failed to measure metrics: {:?}", err);
            }
        }
    });
    Ok(())
}

async fn measure(
    proto_publisher: &Publisher<'_>,
    json_publisher: &Publisher<'_>,
) -> anyhow::Result<()> {
    let output = Command::new("vcgencmd")
        .arg("measure_temp")
        .kill_on_drop(true)
        .output()
        .await?;
    if !output.status.success() {
        tracing::error!("Failed reading temperature: {:?}", output);
        return Ok(());
    }
    let text = String::from_utf8(output.stdout)?;
    let temp = text
        .split('=')
        .nth(1)
        .ok_or(HopperError::FailedParsingCommandOutput(text.to_owned()))?
        .to_owned()
        .trim()
        .strip_suffix("'C")
        .ok_or(HopperError::FailedParsingCommandOutput(text.to_owned()))?
        .to_owned();
    tracing::info!("CPU temperature: {}", temp);
    let temperature_float = temp.parse::<f32>().unwrap_or_default();

    let diagnostic_data = crate::hopper::DiagnosticMessage {
        timestamp: Some(proto_timestamp_now()),
        string_values: vec![crate::hopper::DiagnosticStringValue {
            key: String::from("cpu_temperature"),
            value: temp,
        }],
        float_values: vec![crate::hopper::DiagnosticFloatValue {
            key: String::from("cpu_temperature"),
            value: temperature_float,
        }],
        ..Default::default()
    };

    proto_publisher
        .put(diagnostic_data.encode_to_vec())
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let mut serializer = serde_json::Serializer::new(vec![]);
    diagnostic_data
        .transcode_to_dynamic()
        .serialize(&mut serializer)?;

    json_publisher
        .put(serializer.into_inner())
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    Ok(())
}

fn proto_timestamp_now() -> Timestamp {
    let now = std::time::SystemTime::now();
    let duration = now.duration_since(std::time::UNIX_EPOCH).unwrap();
    Timestamp {
        seconds: duration.as_secs() as i64,
        nanos: duration.subsec_nanos() as i32,
    }
}
