// face
pub const FACE_COLOR_SUBSCRIBER: &str = "hopper/command/face/color";
pub const FACE_ANIMATION_SUBSCRIBER: &str = "hopper/command/face/animation";
pub const FACE_RANDOM_SUBSCRIBER: &str = "hopper/command/face/random";

// remote
pub const STANCE_SUBSCRIBER: &str = "hopper/command/simple/stance";
pub const REMOTE_CONTROL_SUBSCRIBER: &str = "remote-control/gamepad";
pub const WALKING_CONFIG_SUBSCRIBER: &str = "hopper/command/simple/walking_config";
pub const COMPLIANCE_SLOPE_SUBSCRIBER: &str = "hopper/command/config/compliance_slope";
pub const BODY_MOTOR_SPEED_SUBSCRIBER: &str = "hopper/command/config/motor_speed";

pub const HOPPER_WALKING_CONFIG_PUBLISHER: &str = "hopper/status/simple/walking_config";

// speech
pub const SPEECH_SAY_SUBSCRIBER: &str = "hopper/command/speech/say";
pub const SPEECH_SAY_ASTROMECH_SUBSCRIBER: &str = "hopper/command/speech/astromech";
pub const SPEECH_SAY_ASTROMECH_RANDOM_SUBSCRIBER: &str = "hopper/command/speech/astromech/random";
pub const SPEECH_PLAY_SOUND_SUBSCRIBER: &str = "hopper/command/speech/play_sound";
pub const SPEECH_PLAY_SOUND_RANDOM_SUBSCRIBER: &str = "hopper/command/speech/play_sound/random";

// telemetry
pub const DIAGNOSTIC_METRICS: &str = "hopper/metrics/diagnostic";
pub const DIAGNOSTIC_METRICS_JSON: &str = "hopper/metrics/diagnostic/json";
pub const HOPPER_MOTOR_RATE: &str = "hopper/metrics/motor/rate";
pub const HOPPER_POSE_FRAMES: &str = "hopper/pose/frames";
pub const HOPPER_CONTROL_LOOP_RATE: &str = "hopper/metrics/control_loop/rate";

// openai
pub const HOPPER_OPENAI_COMMAND_SUBSCRIBER: &str = "hopper/openai/simple/text/command";
pub const HOPPER_OPENAI_VOICE_COMMAND_SUBSCRIBER: &str = "audio_to_mqtt/windows/simple";
pub const OPENAI_DIAGNOSTICS_HISTORY: &str = "hopper/openai/diagnostics/history";
pub const OPENAI_DIAGNOSTICS_TRANSCRIPT: &str = "hopper/openai/diagnostics/transcript";
