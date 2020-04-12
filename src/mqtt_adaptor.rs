use rumqtt::{MqttClient, MqttOptions, QoS};
use std::thread;
use std::sync::mpsc::{ channel, Sender };
use log::*;

enum Command {
    SendMessage(String),
    Exit
}

pub struct MqttAdaptor {
    sender: Sender<Command>,
    join_handle: Option<thread::JoinHandle<()>>,
}

impl MqttAdaptor {
    pub fn new(host: &str) -> MqttAdaptor {
        let host = host.to_owned();
        let (tx, rx): (Sender<Command>, _) = channel();
        let mqtt_thread = thread::spawn(move || {

            let mqtt_options = MqttOptions::new("hopper-telemetry", host, 1883);
            let (mut mqtt_client, _) = match MqttClient::start(mqtt_options) {
                Ok(client) => client,
                Err(error) => {
                    error!("Failed to connect to MQTT host {}", error);
                    return;
                }
            };

            for message in rx.iter() {
                match message {
                    Command::SendMessage(message) => {
                        if let Err(error) = mqtt_client.publish("hopper/telemetry", QoS::AtMostOnce, false, message) {
                            error!("Failed to send MQTT message {}", error);
                        }
                    },
                    Command::Exit => break,
                }
            }
        });
        MqttAdaptor {
            sender: tx,
            join_handle: Some(mqtt_thread),
        }
    }

    pub fn send(&self, message: &str) {
        if let Err(error) = self.sender.send(Command::SendMessage(message.to_owned())) {
            error!("Failed to load MQTT message to channel {}", error);
        }
    }
}

impl Drop for MqttAdaptor {
    fn drop(&mut self) {
        if let Err(error) = self.sender.send(Command::Exit) {
            error!("Failed to load Exit message to channel {}", error);
        }
        match self.join_handle.take() {
            Some(handle) => {
                    if let Err(error) = handle.join() {
                        error!("Failed joining MQTT thread with {:?}", error);
                    }
                },
            None => error!("Missing join handle for MQTT thread"),
        }
    }
}
