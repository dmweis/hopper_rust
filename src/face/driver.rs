use cobs_rs::stuff;
use serialport::TTYPort;
use std::{convert::TryInto, io::Write};
use thiserror::Error;

#[derive(Error, Debug)]
#[non_exhaustive]
pub enum LedControllerError {
    #[error("connection error")]
    WriteError(#[from] std::io::Error),
    #[error("connection error")]
    ConnectionError(#[from] serialport::Error),
    #[error("communication thread poisoned")]
    CommThreadError,
}

pub(crate) type Result<T> = std::result::Result<T, LedControllerError>;

const DEFAULT_BAUD_RATE: u32 = 115200;

pub const PIXEL_COUNT: usize = 40;

pub const BIGGER_RING_PIXEL_COUNT: usize = 24;
pub const SMALLER_RING_PIXEL_COUNT: usize = 16;

pub const BIGGER_TOP_INDEX: usize = 7;
pub const BIGGER_BOTTOM_INDEX: usize = 19;
pub const SMALLER_TOP_INDEX: usize = 35;
pub const SMALLER_BOTTOM_INDEX: usize = 27;

pub const OFF: RGB = RGB::new(0, 0, 0);
pub const RED: RGB = RGB::new(10, 0, 0);
pub const BLUE: RGB = RGB::new(0, 0, 10);
pub const GREEN: RGB = RGB::new(0, 10, 0);
pub const YELLOW: RGB = RGB::new(10, 10, 0);
pub const PURPLE: RGB = RGB::new(10, 0, 10);
pub const CYAN: RGB = RGB::new(0, 10, 10);
pub const BRIGHT_RED: RGB = RGB::new(180, 0, 0);
pub const BRIGHT_BLUE: RGB = RGB::new(0, 0, 180);
pub const BRIGHT_GREEN: RGB = RGB::new(0, 180, 0);
pub const BRIGHT_YELLOW: RGB = RGB::new(100, 100, 0);
pub const BRIGHT_PURPLE: RGB = RGB::new(100, 0, 100);
pub const BRIGHT_CYAN: RGB = RGB::new(0, 100, 100);

pub const ALL_COLORS: [RGB; 12] = [
    RED,
    BLUE,
    GREEN,
    YELLOW,
    PURPLE,
    CYAN,
    BRIGHT_RED,
    BRIGHT_BLUE,
    BRIGHT_GREEN,
    BRIGHT_YELLOW,
    BRIGHT_PURPLE,
    BRIGHT_CYAN,
];

pub const NORMAL_COLORS: [RGB; 6] = [RED, BLUE, GREEN, YELLOW, PURPLE, CYAN];

pub const BRIGHT_COLORS: [RGB; 6] = [
    BRIGHT_RED,
    BRIGHT_BLUE,
    BRIGHT_GREEN,
    BRIGHT_YELLOW,
    BRIGHT_PURPLE,
    BRIGHT_CYAN,
];

fn corrected_pixel_index(index: i32) -> usize {
    let index = index.rem_euclid(PIXEL_COUNT as i32) as usize;
    if index >= BIGGER_RING_PIXEL_COUNT {
        let index_on_smaller = index - BIGGER_RING_PIXEL_COUNT;
        SMALLER_RING_PIXEL_COUNT - 1 - index_on_smaller + BIGGER_RING_PIXEL_COUNT
    } else {
        index
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct RGB {
    red: u8,
    green: u8,
    blue: u8,
}

impl RGB {
    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        RGB { red, green, blue }
    }

    pub fn fade_out(&self, percentage: f32) -> Self {
        let percentage = percentage.clamp(0.0, 1.0);
        RGB {
            red: (self.red as f32 * percentage) as u8,
            green: (self.green as f32 * percentage) as u8,
            blue: (self.blue as f32 * percentage) as u8,
        }
    }

    fn from_data(color: [u8; 3]) -> Self {
        RGB {
            red: color[0],
            green: color[1],
            blue: color[2],
        }
    }

    fn as_data(&self) -> [u8; 3] {
        [self.red, self.green, self.blue]
    }
}

#[derive(Clone, Debug)]
pub struct ColorPacket {
    payload: [u8; PIXEL_COUNT * 3],
}

impl Default for ColorPacket {
    fn default() -> Self {
        ColorPacket::off()
    }
}

impl ColorPacket {
    pub fn with_color(color: RGB) -> Self {
        let payload = color
            .as_data()
            .iter()
            .cycle()
            .take(PIXEL_COUNT * 3)
            .cloned()
            .collect::<Vec<_>>()
            .as_slice()
            .try_into()
            .unwrap();

        ColorPacket { payload }
    }

    pub const fn off() -> Self {
        ColorPacket {
            payload: [0; PIXEL_COUNT * 3],
        }
    }

    pub fn get_pixel(&self, index: i32) -> RGB {
        let index = corrected_pixel_index(index);
        let offset = index * 3;
        let slice = self.payload[offset..offset + 3].try_into().unwrap();
        RGB::from_data(slice)
    }

    pub fn set_pixel(&mut self, index: i32, color: RGB) {
        let index = corrected_pixel_index(index);
        let offset = index * 3;
        self.payload[offset] = color.red;
        self.payload[offset + 1] = color.green;
        self.payload[offset + 2] = color.blue;
    }

    fn to_data(&self) -> [u8; PIXEL_COUNT * 3 + 2] {
        let mut buffer = [0; PIXEL_COUNT * 3 + 2];
        buffer.clone_from(&stuff(self.payload, 0));
        buffer
    }
}

pub struct LedDriver {
    port: TTYPort,
}

impl LedDriver {
    pub fn open(port_name: &str) -> Result<Self> {
        let port = serialport::new(port_name, DEFAULT_BAUD_RATE).open_native()?;
        let mut controller = LedDriver { port };
        controller.turn_off()?;
        Ok(controller)
    }

    pub fn send(&mut self, message: &ColorPacket) -> Result<()> {
        self.port.write_all(&message.to_data())?;
        Ok(())
    }

    pub fn turn_off(&mut self) -> Result<()> {
        self.send(&ColorPacket::off())
    }
}

impl Drop for LedDriver {
    fn drop(&mut self) {
        let _ = self.turn_off();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn creating_color_packet_does_not_panic() {
        let color = RGB::new(255, 0, 0);
        let _packet = ColorPacket::with_color(color);
    }

    #[test]
    fn get_pixel_does_not_panic() {
        let color = RGB::new(255, 0, 0);
        let packet = ColorPacket::with_color(color);
        let pixel = packet.get_pixel(3);
        assert_eq!(pixel, color);
    }

    #[test]
    fn get_pixel_works_for_out_of_bounds() {
        let color = RGB::new(255, 0, 0);
        let packet = ColorPacket::with_color(color);
        let pixel = packet.get_pixel(PIXEL_COUNT as i32 + 3);
        assert_eq!(pixel, color);
    }

    #[test]
    fn get_pixel_wraps_negative() {
        let red = RGB::new(255, 0, 0);
        let green = RGB::new(0, 255, 0);
        let mut packet = ColorPacket::with_color(red);
        packet.set_pixel(0, green);
        assert_eq!(red, packet.get_pixel(39));
        assert_eq!(green, packet.get_pixel(40));
        assert_eq!(red, packet.get_pixel(41));
        assert_eq!(red, packet.get_pixel(1));
    }

    #[test]
    fn set_pixel_works() {
        let red = RGB::new(255, 0, 0);
        let green = RGB::new(0, 255, 0);
        let mut packet = ColorPacket::with_color(red);
        packet.set_pixel(10, green);
        assert_eq!(red, packet.get_pixel(9));
        assert_eq!(green, packet.get_pixel(10));
        assert_eq!(red, packet.get_pixel(11));
    }

    #[test]
    fn set_pixel_works_out_of_bounds() {
        let red = RGB::new(255, 0, 0);
        let green = RGB::new(0, 255, 0);
        let mut packet = ColorPacket::with_color(red);
        packet.set_pixel(PIXEL_COUNT as i32 + 10, green);
        assert_eq!(red, packet.get_pixel(9));
        assert_eq!(green, packet.get_pixel(10));
        assert_eq!(red, packet.get_pixel(11));
    }

    #[test]
    fn set_pixel_wraps_negative() {
        let red = RGB::new(255, 0, 0);
        let green = RGB::new(0, 255, 0);
        let mut packet = ColorPacket::with_color(red);
        packet.set_pixel(-1, green);
        assert_eq!(red, packet.get_pixel(38));
        assert_eq!(green, packet.get_pixel(39));
        assert_eq!(red, packet.get_pixel(40));
        assert_eq!(red, packet.get_pixel(0));
    }

    #[test]
    fn encoded_contains_no_random_zeros() {
        let color = RGB::new(255, 0, 0);
        let packet = ColorPacket::with_color(color);
        let encoded = packet.to_data();
        assert!(encoded.iter().any(|element| *element == 0));
    }

    #[test]
    fn encoded_terminates_in_zero() {
        let color = RGB::new(255, 0, 0);
        let packet = ColorPacket::with_color(color);
        let encoded = packet.to_data();
        assert_eq!(encoded[PIXEL_COUNT * 3 + 1], 0);
    }
}
