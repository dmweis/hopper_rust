use anyhow::Result;
use clap::Parser;
use hopper_rust::speech::ElevenLabsTtsClient;
use tokio::{fs::File, io::AsyncWriteExt};

/// Hopper body controller
#[derive(Parser)]
#[command(author, version)]
struct Args {
    #[arg()]
    text: String,
    #[arg(long)]
    api_key: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let client = ElevenLabsTtsClient::new(args.api_key.clone());
    let voices = client.voices().await?;
    let voice_id = voices.name_to_id_table().get("Natasha").unwrap().clone();
    let data = client.tts(&args.text, &voice_id).await.unwrap();

    let mut file = File::create("tmp/audio.mp3").await?;
    file.write_all(&data).await?;
    println!("Wrote file");

    let files = client.stream_tts(&args.text, &voice_id).await?;
    for (i, audio_file_contents) in files.into_iter().enumerate() {
        let path = format!("tmp/audio_chunk_{i}.mp3");
        let mut file = File::create(&path).await?;
        file.write_all(&audio_file_contents).await?;
        println!("Wrote {path}");
    }

    Ok(())
}
