#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_net::{IpAddress, Stack, tcp::TcpSocket};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Watch},
};
use embassy_time::{Delay, Duration, Timer, WithTimeout};
use embedded_io_async::Write;
use esp_hal::{
    clock::CpuClock,
    gpio::AnyPin,
    i2c::master::{I2c, Instance as I2cInstance},
    timer::timg::TimerGroup,
};
use esp_radio::wifi::{self, WifiController, WifiDevice, WifiStaState};
use thermometer_data::Measurement;
use {esp_backtrace as _, esp_println as _};

macro_rules! make_static {
    ($ty:ty, $val:expr $(,)?) => {{
        static CELL: static_cell::StaticCell<$ty> = static_cell::StaticCell::new();
        CELL.uninit().write($val)
    }};
}

macro_rules! from_str_radix {
    ($var:literal, $value:expr, $ty:ty) => {
        match <$ty>::from_str_radix($value, 10) {
            Ok(value) => value,
            Err(_) => panic!(concat!("failed to parse ", $var, " as ", stringify!($ty),)),
        }
    };
}

macro_rules! parse_env {
    ($var:literal as $ty:ty) => {
        from_str_radix!($var, env!($var), $ty)
    };
}

macro_rules! parse_env_or_default {
    ($var:literal as $ty:ty, $default:expr) => {
        if let Some(value) = option_env!($var) {
            from_str_radix!($var, value, $ty)
        } else {
            $default
        }
    };
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SERVER_HOST: &str = env!("SERVER_HOST");
const SERVER_PORT: u16 = parse_env_or_default!("SERVER_PORT" as u16, 7878);
const LOCATION_ID: u32 = parse_env!("LOCATION_ID" as u32);
const INTERVAL: Duration = Duration::from_secs(parse_env_or_default!("INTERVAL" as u64, 60));
const RETRY_INTERVAL: Duration =
    Duration::from_secs(parse_env_or_default!("RETRY_INTERVAL" as u64, 5));

macro_rules! gpio_pin {
    ($var:literal, $default:expr) => {
        match parse_env_or_default!($var as u8, $default) {
            x @ 0..=5 | x @ 12..=19 | x @ 21..=23 | x @ 25..=27 | x @ 32..=33 => x,
            _ => panic!(concat!("invalid ", $var)),
        }
    };
}

const SCL_PIN: u8 = gpio_pin!("SCL_PIN", 32);
const SDA_PIN: u8 = match gpio_pin!("SDA_PIN", 33) {
    p if p == SCL_PIN => panic!("SCL_PIN and SDA_PIN cannot be the same"),
    p => p,
};

macro_rules! timeout_loop {
    ($name:literal, $body:expr $(,)?) => {
        loop {
            if $body
                .with_timeout(INTERVAL)
                .await
                .map_err(|_| warn!("[{}] timeout in loop", $name))
                .flatten()
                .is_ok()
            {
                Timer::after(INTERVAL).await;
            } else {
                Timer::after(RETRY_INTERVAL).await;
            }
        }
    };
}

static SERVER_ADDR: Watch<CriticalSectionRawMutex, IpAddress, 1> = Watch::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("embassy initialized");

    let radio_init = &*make_static!(
        esp_radio::Controller,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller"),
    );
    let (wifi_controller, interfaces) = wifi::new(radio_init, peripherals.WIFI, Default::default())
        .expect("Failed to initialize Wi-Fi controller");
    let (stack, runner) = embassy_net::new(
        interfaces.sta,
        embassy_net::Config::dhcpv4(Default::default()),
        make_static!(
            embassy_net::StackResources::<3>,
            embassy_net::StackResources::<3>::new(),
        ),
        0,
    );

    spawner
        .spawn(wifi_connection(wifi_controller, stack))
        .unwrap();
    spawner.spawn(net_runner(runner)).unwrap();
    spawner.spawn(heap_stats()).unwrap();

    info!("initializing BME280...");
    let mut bme = initialize_bme(
        peripherals.I2C0,
        // SAFETY: these are the only pins we use, and we have already checked that they are not
        // the same
        unsafe { AnyPin::steal(SCL_PIN) },
        unsafe { AnyPin::steal(SDA_PIN) },
    )
    .await
    .unwrap();
    info!("BME280 initialized");

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut server_ip_rx = SERVER_ADDR.receiver().unwrap();

    let mut serialization_buffer: thermometer_data::MeasurementBuffer = [0; _];

    timeout_loop!("measurement", {
        let socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        measurement_loop(
            socket,
            &mut server_ip_rx,
            &mut bme,
            &mut serialization_buffer,
        )
    })
}

async fn measurement_loop(
    mut socket: TcpSocket<'_>,
    server_ip_rx: &mut Receiver<'_, CriticalSectionRawMutex, IpAddress, 1>,
    bme: &mut bme280::i2c::AsyncBME280<I2c<'static, esp_hal::Async>>,
    serialization_buffer: &mut thermometer_data::MeasurementBuffer,
) -> Result<(), ()> {
    let server_ip = if let Some(server_ip) = server_ip_rx.try_get() {
        server_ip
    } else {
        info!("[measurement] waiting for server ip");
        server_ip_rx.get().await
    };
    info!("[measurement] server ip: {}", server_ip);
    socket
        .connect((server_ip, SERVER_PORT))
        .await
        .map_err(|err| error!("[measurement] failed to connect to server: {:?}", err))?;
    info!("[measurement] connected to server");

    let measurement = bme
        .measure(&mut Delay)
        .await
        .map_err(|err| error!("[measurement] failed to read measurement: {:?}", err))?;
    let measurement = Measurement {
        id: LOCATION_ID,
        temperature: measurement.temperature,
        pressure: measurement.pressure,
        humidity: measurement.humidity,
    };
    info!(
        "[measurement] temperature: {=f32} °C, pressure: {=f32} Pa, humidity: {=f32}%",
        measurement.temperature, measurement.pressure, measurement.humidity,
    );

    let data = postcard::to_slice(&measurement, serialization_buffer)
        .map_err(|err| error!("[measurement] failed to serialize data: {:?}", err))?;
    socket
        .write_all(data)
        .await
        .map_err(|err| error!("[measurement] failed to write measurements: {:?}", err))?;
    info!("[measurement] data written");
    socket
        .flush()
        .await
        .map_err(|err| error!("[measurement] failed to flush data: {:?}", err))?;
    drop(socket);
    info!("[measurement] socket closed");
    Ok(())
}

#[embassy_executor::task]
async fn wifi_connection(mut controller: WifiController<'static>, stack: Stack<'static>) -> ! {
    info!("[wifi] starting wifi connection task...");
    timeout_loop!("wifi", wifi_connection_loop(&mut controller, stack))
}

async fn wifi_connection_loop(
    controller: &mut WifiController<'static>,
    stack: Stack<'static>,
) -> Result<(), ()> {
    start_wifi(controller).await?;
    connect_wifi(controller).await?;
    wait_for_dhcp(stack).await;
    if let Some(server_ip) = SERVER_ADDR.try_get() {
        info!("[wifi] server_ip already set to {:?}", server_ip);
    } else {
        let server_ip = get_server_ip(stack).await?;
        SERVER_ADDR.sender().send(server_ip);
        info!("[wifi] server_ip: {:?}", server_ip);
    }
    Ok(())
}

async fn start_wifi(controller: &mut WifiController<'static>) -> Result<(), ()> {
    let config = wifi::ModeConfig::Client(
        wifi::ClientConfig::default()
            .with_ssid(SSID.into())
            .with_password(PASSWORD.into()),
    );
    while !controller.is_started().is_ok_and(|b| b) {
        // configure and start wifi
        controller
            .set_config(&config)
            .map_err(|err| error!("[wifi] unable to set wifi config: {:?}", err))?;
        info!("[wifi] starting wifi...");
        controller
            .start_async()
            .await
            .map_err(|err| error!("[wifi] unable to start wifi: {:?}", err))?;
        info!("[wifi] started");
    }
    Ok(())
}

async fn connect_wifi(controller: &mut WifiController<'static>) -> Result<(), ()> {
    while wifi::sta_state() != WifiStaState::Connected {
        let state = wifi::sta_state();
        if state != WifiStaState::Disconnected {
            error!("[wifi] unexpected state: {}", state);
            return Err(());
        }
        warn!("[wifi] wifi disconnected");
        SERVER_ADDR.sender().clear();

        info!("[wifi] wifi connecting...");
        controller
            .connect_async()
            .await
            .map_err(|err| warn!("[wifi] failed to connect to wifi: {:?}", err))?;
        info!("[wifi] wifi connected");
    }
    Ok(())
}

async fn wait_for_dhcp(stack: Stack<'_>) {
    loop {
        if let Some(config) = stack.config_v4() {
            info!("[wifi] got IP: {:?}", config.address);
            break;
        }
        stack.wait_link_up().await;
        info!("[wifi] waiting to get IP address...");
        stack.wait_config_up().await;
    }
}

async fn get_server_ip(stack: Stack<'_>) -> Result<IpAddress, ()> {
    match stack
        .dns_query(SERVER_HOST, embassy_net::dns::DnsQueryType::A)
        .await
        .map(|ips| ips.first().copied())
    {
        Ok(Some(ip)) => Ok(ip),
        Ok(None) => {
            error!(
                "[wifi] DNS did not return any IP addresses for {:?}",
                SERVER_HOST
            );
            Err(())
        }
        Err(err) => {
            error!("[wifi] DNS error: {:?}", err);
            Err(())
        }
    }
}

#[embassy_executor::task]
async fn net_runner(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

async fn initialize_bme(
    i2c: impl I2cInstance + 'static,
    scl: AnyPin<'static>,
    sda: AnyPin<'static>,
) -> Result<bme280::i2c::AsyncBME280<I2c<'static, esp_hal::Async>>, ()> {
    let i2c = I2c::new(i2c, Default::default())
        .map_err(|err| error!("unable to configure I2C: {:?}", err))?
        .with_scl(scl)
        .with_sda(sda)
        .into_async();
    let mut bme = bme280::i2c::AsyncBME280::new_primary(i2c);
    bme.init(&mut Delay).await.map_err(|err| {
        error!("unable to configure BME280: {:?}", err);
        if let bme280::Error::Bus(err) = err {
            error!("caused by: {:?}", err);
        }
    })?;
    Ok(bme)
}

#[embassy_executor::task]
async fn heap_stats() -> ! {
    let mut stats;
    let mut last = 0;
    loop {
        stats = esp_alloc::HEAP.stats();
        if stats.current_usage != last {
            info!("[heap] {}", stats);
            last = stats.current_usage;
        }
        Timer::after(INTERVAL).await;
    }
}
