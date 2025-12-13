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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Timer};
use embedded_io_async::Write;
use esp_hal::{
    clock::CpuClock,
    gpio::OutputPin,
    i2c::master::{I2c, Instance as I2cInstance},
    timer::timg::TimerGroup,
};
use esp_radio::wifi::{self, WifiController, WifiDevice, WifiEvent, WifiStaState};
use thermometer_data::Measurement;
use {esp_backtrace as _, esp_println as _};

macro_rules! make_static {
    ($ty:ty, $val:expr $(,)?) => {{
        static CELL: static_cell::StaticCell<$ty> = static_cell::StaticCell::new();
        CELL.uninit().write($val)
    }};
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const SERVER_HOST: &str = env!("SERVER_HOST");
const SERVER_PORT: u16 = match u16::from_str_radix(env!("SERVER_PORT"), 10) {
    Ok(port) => port,
    Err(_) => panic!(concat!(
        "failed to parse SERVER_PORT='",
        env!("SERVER_PORT"),
        "' as u16",
    )),
};

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

    let mut bme = initialize_bme(peripherals.I2C0, peripherals.GPIO32, peripherals.GPIO33).await;

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut server_ip_rx = SERVER_ADDR.receiver().unwrap();

    let mut data_buffer = [0; 12];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        let server_ip = server_ip_rx.get().await;
        if let Err(err) = socket.connect((server_ip, SERVER_PORT)).await {
            error!("failed to connect to server: {:?}", err);
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        let measurement = Measurement::from(bme.measure(&mut Delay).await.unwrap());

        let data = postcard::to_slice(&measurement, &mut data_buffer).unwrap();
        if let Err(err) = socket.write_all(data).await {
            error!("failed to write measurements: {:?}", err);
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        info!(
            "temperature: {=f32} °C, pressure: {=f32} Pa, humidity: {=f32}%",
            measurement.temperature, measurement.pressure, measurement.humidity,
        );
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn wifi_connection(mut controller: WifiController<'static>, stack: Stack<'static>) {
    info!("starting wifi connection task...");
    loop {
        if wifi::sta_state() == WifiStaState::Connected {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            SERVER_ADDR.sender().clear();
            warn!("wifi disconnected");
            Timer::after(Duration::from_secs(5)).await;
        }
        if !matches!(controller.is_started(), Ok(true)) {
            // configure and start wifi
            let config = wifi::ModeConfig::Client(
                wifi::ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into()),
            );
            controller.set_config(&config).unwrap();
            info!("starting wifi...");
            controller.start_async().await.unwrap();
            info!("wifi started");
        }
        info!("wifi connecting...");
        if let Err(e) = controller.connect_async().await {
            warn!("failed to connect to wifi: {:?}", e);
            Timer::after(Duration::from_secs(5)).await;
        }
        info!("wifi connected");
        wait_for_dhcp(stack).await;
        wait_for_dns(stack).await;
    }
}

async fn wait_for_dhcp(stack: Stack<'_>) {
    stack.wait_link_up().await;
    info!("waiting to get IP address...");
    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            info!("got IP: {:?}", config.address);
            break;
        }
    }
}

async fn wait_for_dns(stack: Stack<'_>) {
    let server_ip = loop {
        match stack
            .dns_query(SERVER_HOST, embassy_net::dns::DnsQueryType::A)
            .await
        {
            Ok(ips) if !ips.is_empty() => break *ips.first().unwrap(),
            Ok(_) => error!("DNS did not return any IP addresses for {:?}", SERVER_HOST),
            Err(e) => error!("DNS error: {:?}", e),
        }
    };
    SERVER_ADDR.sender().send(server_ip);
    info!("server_ip: {:?}", server_ip);
}

#[embassy_executor::task]
async fn net_runner(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

async fn initialize_bme(
    i2c: impl I2cInstance + 'static,
    scl: impl OutputPin + 'static,
    sda: impl OutputPin + 'static,
) -> bme280::i2c::AsyncBME280<I2c<'static, esp_hal::Async>> {
    let i2c = I2c::new(i2c, Default::default())
        .unwrap()
        .with_scl(scl)
        .with_sda(sda)
        .into_async();
    let mut bme = bme280::i2c::AsyncBME280::new_primary(i2c);
    bme.init(&mut Delay).await.unwrap();
    bme
}
