use anyhow::Result;
use clap::Parser;
use log::*;
use std::{
    path::{Path, PathBuf},
    time::Duration,
};

use hopper_rust::{
    app_config::get_configuration, body_controller, body_controller::BodyController, hopper_config,
    ik_controller, lidar::LidarDriver, motion_controller, speech::SpeechService, udp_adaptor,
    utilities,
};

/// Hopper body controller
#[derive(Parser)]
#[clap(author, version)]
struct Args {
    /// Sets path to body config file (.yaml)
    /// If unset uses default value.
    #[clap(long)]
    body_config: Option<String>,
    /// application configuration
    #[clap(long)]
    config: Option<PathBuf>,
    /// Path for external log file.
    /// If no give will only log to out
    #[clap(long)]
    log_path: Option<String>,
    /// Serial port name of the dynamixel port
    #[clap(short, long, default_value = "/dev/dynamixel")]
    dynamixel_port: String,
    /// hopper face serial port
    #[clap(long, default_value = "/dev/hopper_face")]
    face_port: String,
    /// hopper lidar serial port
    #[clap(long, default_value = "/dev/rplidar")]
    lidar: String,
    /// Sets the level of verbosity
    #[clap(short, parse(from_occurrences))]
    verbose: u8,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    utilities::start_loggers(args.log_path, args.verbose)?;
    info!("Started main controller");

    let app_config = get_configuration(args.config)?;

    let face_controller = hopper_face::FaceController::open(&args.face_port)?;
    face_controller.larson_scanner(hopper_face::driver::PURPLE)?;

    let mut lidar_driver = LidarDriver::new(&args.lidar)?;
    lidar_driver.stop()?;

    let hopper_config = args
        .body_config
        .map(|path| hopper_config::HopperConfig::load(Path::new(&path)))
        .unwrap_or_else(|| Ok(hopper_config::HopperConfig::default()))?;

    let body_controller =
        body_controller::AsyncBodyController::new(&args.dynamixel_port, hopper_config.legs.clone())
            .unwrap();

    let mut ik_controller =
        ik_controller::IkController::new(Box::new(body_controller), hopper_config);

    // TODO (David): Move to some settings system
    // also maybe tune...
    ik_controller.set_compliance_slope(32).await?;
    ik_controller.set_speed(1023).await?;

    let mut motion_controller = motion_controller::MotionController::new(ik_controller).await?;

    let mut speech_service = SpeechService::new(
        app_config.tts_service_config.azure_api_key,
        app_config.tts_service_config.cache_dir_path,
        app_config.tts_service_config.audio_repository_path,
    )
    .unwrap();

    speech_service.say_azure("Good morning!").await.unwrap();
    speech_service
        .play_sound("Turret_turret_deploy_3.wav")
        .await
        .unwrap();

    udp_adaptor::udp_controller_handler(&mut motion_controller)
        .await
        .unwrap();

    motion_controller.set_body_state(motion_controller::BodyState::Grounded);
    tokio::time::sleep(Duration::from_secs_f32(2.0)).await;
    drop(face_controller);
    drop(motion_controller);
    Ok(())
}
