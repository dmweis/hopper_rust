use anyhow::Result;
use clap::Parser;
use hopper_rust::{
    body_controller, body_controller::BodyController, camera::start_camera,
    configuration::get_configuration, error::HopperError, hopper_body_config, ik_controller,
    lidar::start_lidar_driver, motion_controller, speech::SpeechService, udp_remote, utilities,
};
use std::{
    path::{Path, PathBuf},
    time::Duration,
};
use tracing::*;
use zenoh::config::Config as ZenohConfig;
use zenoh::prelude::r#async::*;

/// Hopper body controller
#[derive(Parser)]
#[command(author, version)]
struct Args {
    /// Sets path to body config file (.yaml)
    /// If unset uses default value.
    #[arg(long)]
    body_config: Option<String>,
    /// application configuration
    #[arg(long)]
    config: Option<PathBuf>,
    /// Serial port name of the dynamixel port
    #[arg(short, long, default_value = "/dev/dynamixel")]
    dynamixel_port: String,
    /// hopper face serial port
    #[arg(long, default_value = "/dev/hopper_face")]
    face_port: String,
    /// hopper lidar serial port
    #[arg(long, default_value = "/dev/rplidar")]
    lidar: String,
    /// Sets the level of verbosity
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,

    /// Endpoints to connect to.
    #[clap(short = 'e', long)]
    connect: Vec<zenoh_config::EndPoint>,

    /// Endpoints to listen on.
    #[clap(long)]
    listen: Vec<zenoh_config::EndPoint>,

    /// A configuration file.
    #[clap(short, long)]
    zenoh_config: Option<PathBuf>,
}

impl Args {
    fn get_zenoh_config(&self) -> Result<ZenohConfig> {
        let mut config = if let Some(conf_file) = &self.zenoh_config {
            ZenohConfig::from_file(conf_file).unwrap()
        } else {
            ZenohConfig::default()
        };
        if !self.connect.is_empty() {
            config.connect.endpoints = self.connect.clone();
        }
        if !self.listen.is_empty() {
            config.listen.endpoints = self.listen.clone();
        }
        Ok(config)
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    utilities::setup_tracing(args.verbose);
    info!("Started main controller");

    let app_config = get_configuration(&args.config)?;

    let zenoh_config = args.get_zenoh_config()?;
    let zenoh_session = zenoh::open(zenoh_config)
        .res()
        .await
        .map_err(HopperError::ZenohError)?
        .into_arc();

    let face_controller = hopper_face::FaceController::open(&args.face_port)?;
    face_controller.larson_scanner(hopper_face::driver::PURPLE)?;

    start_lidar_driver(zenoh_session.clone(), &args.lidar, false).await?;

    let mut speech_service = SpeechService::new(
        app_config.tts_service_config.azure_api_key,
        app_config.tts_service_config.cache_dir_path,
        app_config.tts_service_config.audio_repository_path,
    )
    .unwrap();

    speech_service
        .play_sound("hopper_sounds/windows_startup.wav")
        .await
        .unwrap();

    let hopper_body_config = args
        .body_config
        .map(|path| hopper_body_config::HopperConfig::load(Path::new(&path)))
        .unwrap_or_else(|| Ok(hopper_body_config::HopperConfig::default()))?;

    let body_controller = body_controller::AsyncBodyController::new(
        &args.dynamixel_port,
        hopper_body_config.legs.clone(),
    )
    .unwrap();

    let mut ik_controller =
        ik_controller::IkController::new(Box::new(body_controller), hopper_body_config);

    // TODO (David): Move to some settings system
    // also maybe tune...
    ik_controller.set_compliance_slope(32).await?;
    ik_controller.set_speed(1023).await?;

    let mut motion_controller = motion_controller::MotionController::new(ik_controller).await?;

    motion_controller.set_body_state(motion_controller::BodyState::Grounded);

    start_camera(zenoh_session.clone()).await?;

    udp_remote::udp_controller_handler(&mut motion_controller)
        .await
        .unwrap();

    motion_controller.set_body_state(motion_controller::BodyState::Grounded);
    tokio::time::sleep(Duration::from_secs_f32(2.0)).await;
    drop(face_controller);
    drop(motion_controller);
    Ok(())
}
