use anyhow::Result;
use clap::Parser;
use hopper_rust::speech::SpeechService;
use std::io::Read;

/// Hopper body controller
#[derive(Parser)]
#[command(author, version)]
struct Args {
    #[arg()]
    text: Option<String>,
    #[arg(long)]
    audio: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();

    let mut speech_service =
        SpeechService::new(String::from(""), String::from(""), None, Some(args.audio))
            .await
            .unwrap();

    if let Some(text) = args.text {
        speech_service.say_astromech(&text).await.unwrap();
        println!("Press enter to exit");
        _ = std::io::stdin().read(&mut [0]).unwrap();
    } else {
        println!("No input provided.\nPlaying random noises");
        loop {
            speech_service.random_astromech_noise(20).await.unwrap();
            std::thread::sleep(std::time::Duration::from_secs(6));
        }
    }

    Ok(())
}
