use std::net::{Ipv4Addr, SocketAddr, TcpStream};

use thermometer_data::Measurement;

type Error = Box<dyn std::error::Error + Send + Sync>;

fn main() -> Result<(), Error> {
    let addr = SocketAddr::from((Ipv4Addr::UNSPECIFIED, 7878));
    let listener = std::net::TcpListener::bind(addr)?;
    loop {
        let (stream, src) = listener.accept()?;
        std::thread::spawn(move || handle_stream(stream, src));
    }
}

fn handle_stream(stream: TcpStream, src: SocketAddr) -> Result<(), Error> {
    let mut buffer = [0; 12];
    let measurement = postcard::from_io::<Measurement, TcpStream>((stream, &mut buffer))?;
    println!("{src:?}: {measurement:?}");
    Ok(())
}
