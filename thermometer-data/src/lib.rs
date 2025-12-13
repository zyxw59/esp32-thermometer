#![no_std]

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct Measurement {
    pub temperature: f32,
    pub pressure: f32,
    pub humidity: f32,
}

impl<E> From<bme280::Measurements<E>> for Measurement {
    fn from(m: bme280::Measurements<E>) -> Self {
        Self {
            temperature: m.temperature,
            pressure: m.pressure,
            humidity: m.humidity,
        }
    }
}
