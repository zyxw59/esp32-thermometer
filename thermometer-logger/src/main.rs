use std::{env, net::TcpStream};

use thermometer_data::{Measurement, MeasurementBuffer};

type Error = Box<dyn std::error::Error + Send + Sync>;

fn main() -> Result<(), Error> {
    let host = env::var("HOST").unwrap_or("0.0.0.0".into());
    let port: u16 = if let Ok(val) = env::var("PORT") {
        val.parse()?
    } else {
        7878
    };
    let listener = std::net::TcpListener::bind((host, port))?;
    eprintln!("listening on {}", listener.local_addr()?);
    println!("time,timestamp,location_id,temperature,pressure,humidity");
    loop {
        let (stream, src) = listener.accept()?;
        eprintln!("connection from {src}");
        std::thread::spawn(move || handle_stream(stream));
    }
}

fn handle_stream(stream: TcpStream) {
    let mut buffer: MeasurementBuffer = [0; _];
    let now = chrono::Local::now();
    match postcard::from_io::<Measurement, TcpStream>((stream, &mut buffer)) {
        Ok((measurement, _)) => println!(
            "{},{},{},{},{},{}",
            now.format("%Y-%m-%d %H:%M:%S %:z"),
            now.format("%s"),
            measurement.id,
            measurement.temperature,
            measurement.pressure,
            measurement.humidity,
        ),
        Err(err) => eprintln!("failed to deserialize data: {err}"),
    }
}
