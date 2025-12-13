#![no_std]

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct Measurement {
    pub id: u32,
    pub temperature: f32,
    pub pressure: f32,
    pub humidity: f32,
}

pub type MeasurementBuffer = [u8; core::mem::size_of::<Measurement>()];
