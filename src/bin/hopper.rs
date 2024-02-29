use anyhow::{Context, Result};
use clap::Parser;
use hopper_rust::{
    body_controller,
    body_controller::BodyController,
    camera::start_camera,
    configuration::get_configuration,
    error::HopperError,
    high_five::HighFiveDetector,
    hopper_body_config, ik_controller,
    ioc_container::IocContainer,
    lidar::start_lidar_driver,
    logging,
    monitoring::start_monitoring_loop,
    motion_controller,
    openai::start_openai_controller,
    speech::SpeechService,
    utilities::RateTracker,
    zenoh_remotes::{
        face_controller::start_face_controller,
        remote_controller::simple_zenoh_controller,
        speech_controller::start_speech_controller,
        topic_consts::{HOPPER_CONTROL_LOOP_RATE, HOPPER_MOTOR_RATE, HOPPER_POSE_FRAMES},
    },
};
use std::{
    path::{Path, PathBuf},
    time::Duration,
};
use tracing::*;
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

    /// Sets the level of verbosity
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Args = Args::parse();
    logging::setup_tracing(args.verbose);
    info!("Started main controller");

    let app_config = get_configuration(&args.config)?;

    let zenoh_config = app_config.zenoh.get_zenoh_config()?;
    let zenoh_session = zenoh::open(zenoh_config)
        .res()
        .await
        .map_err(HopperError::ZenohError)?
        .into_arc();

    let face_controller = hopper_rust::face::FaceController::open(&app_config.base.face_port)?;
    face_controller.larson_scanner(hopper_rust::face::driver::PURPLE)?;

    let ioc_container = IocContainer::global_instance();
    ioc_container.register(zenoh_session.clone());

    ioc_container.register(face_controller);
    start_face_controller(
        ioc_container.service::<hopper_rust::face::FaceController>()?,
        zenoh_session.clone(),
    )
    .await?;

    let (high_five_detector, high_five_receiver, high_five_service_controller) =
        HighFiveDetector::new();

    ioc_container.register(high_five_service_controller);

    let lidar_service_controller =
        start_lidar_driver(zenoh_session.clone(), &app_config.lidar, high_five_detector).await?;

    ioc_container.register(lidar_service_controller);

    start_monitoring_loop(zenoh_session.clone()).await?;

    let speech_service = SpeechService::new(
        app_config.tts_service_config.azure_api_key,
        app_config.tts_service_config.eleven_labs_api_key,
        app_config.tts_service_config.cache_dir_path,
        app_config.tts_service_config.audio_repository_path,
    )
    .await?;

    ioc_container.register(speech_service);

    start_speech_controller(
        ioc_container.service::<SpeechService>()?,
        zenoh_session.clone(),
    )
    .await?;

    let hopper_body_config = args
        .body_config
        .map(|path| hopper_body_config::HopperConfig::load(Path::new(&path)))
        .unwrap_or_else(|| Ok(hopper_body_config::HopperConfig::default()))?;

    let motor_rate_publisher = zenoh_session
        .declare_publisher(HOPPER_MOTOR_RATE)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let body_controller = body_controller::AsyncBodyController::new(
        &app_config.base.dynamixel_port,
        hopper_body_config.legs.clone(),
        motor_rate_publisher,
    )?;

    let pose_publisher = zenoh_session
        .declare_publisher(HOPPER_POSE_FRAMES)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let mut ik_controller = ik_controller::IkController::new(
        Box::new(body_controller),
        hopper_body_config,
        pose_publisher,
    );

    // TODO (David): Move to some settings system
    // also maybe tune...
    ik_controller.set_compliance_slope(64).await?;
    ik_controller.set_motor_speed(1023).await?;

    let motion_controller_rate_publisher = zenoh_session
        .declare_publisher(HOPPER_CONTROL_LOOP_RATE)
        .res()
        .await
        .map_err(HopperError::ZenohError)?;

    let motion_controller_rate_reporter =
        RateTracker::new(Duration::from_secs(1), motion_controller_rate_publisher);

    let mut motion_controller = motion_controller::MotionController::new(
        ik_controller,
        motion_controller_rate_reporter,
        high_five_receiver,
    )
    .await?;

    let dance_service = motion_controller.create_dance_service();
    ioc_container.register(dance_service);

    start_camera(zenoh_session.clone(), &app_config.camera).await?;

    let open_ai_service = start_openai_controller(
        &app_config.openai.api_key,
        &app_config.openai.wakeword_topic_prefix,
        zenoh_session.clone(),
    )
    .await?;

    ioc_container.register(open_ai_service);

    // audio is sent to us transcribed now
    // start_audio_transcribe_service(&app_config.openai.api_key, zenoh_session.clone()).await?;

    // hopper_rust::udp_remote::udp_controller_handler(&mut motion_controller)
    //     .await
    //     .unwrap();

    // announce that we are ready
    {
        let speech_service = ioc_container.service::<SpeechService>()?;

        // speech_service
        //     .play_sound("hopper_sounds/windows_startup.wav")
        //     .await
        //     .unwrap();
        speech_service
            .say_eleven_with_default_voice("Hopper ready")
            .await?;
        // speech_service.say_home_speak("Hopper ready").await.unwrap();
    }

    simple_zenoh_controller(&mut motion_controller, zenoh_session.clone())
        .await
        .context("Controller reader failed")?;
    info!("Controller stopped");

    motion_controller.set_body_state(motion_controller::BodyState::Grounded);
    tokio::time::sleep(Duration::from_secs_f32(2.0)).await;
    motion_controller.disable_motors();
    drop(motion_controller);
    Ok(())
}
