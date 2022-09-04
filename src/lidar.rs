use crate::error::HopperResult;
use rplidar_driver::{RplidarDevice, RplidarDriver, ScanOptions};

pub struct LidarDriver {
    lidar: Box<dyn RplidarDriver>,
}

impl LidarDriver {
    pub fn new(port: &str) -> HopperResult<Self> {
        let mut lidar = RplidarDevice::open_port(port)?;

        let scan_options = ScanOptions::with_mode(2);
        let _ = lidar.start_scan_with_options(&scan_options)?;

        Ok(Self { lidar })
    }

    pub fn stop(&mut self) -> HopperResult<()> {
        self.lidar.stop_motor()?;
        self.lidar.stop()?;
        Ok(())
    }
}
