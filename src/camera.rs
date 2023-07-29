use tracing::info;
use v4l::prelude::*;

pub fn scan_camera() -> anyhow::Result<()> {
    let dev = Device::new(0).expect("Failed to open device");

    let caps = dev.query_caps()?;
    info!("Device capabilities:\n{}", caps);

    let controls = dev.query_controls()?;
    info!("Device controls:");
    let mut max_name_len = 0;
    for ctrl in &controls {
        if ctrl.name.len() > max_name_len {
            max_name_len = ctrl.name.len();
        }
    }
    for ctrl in controls {
        info!(
            "{:indent$} : [{}, {}]",
            ctrl.name,
            ctrl.minimum,
            ctrl.maximum,
            indent = max_name_len
        );
    }

    Ok(())
}
