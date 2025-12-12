#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use esp_hal::{
    clock::CpuClock,
    gpio::OutputPin,
    i2c::master::{I2c, Instance as I2cInstance},
    timer::timg::TimerGroup,
};
use esp_radio::wifi::{self, WifiController, WifiDevice, WifiEvent, WifiStaState};
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

macro_rules! make_static {
    ($ty:ty, $val:expr $(,)?) => {{
        static CELL: static_cell::StaticCell<$ty> = static_cell::StaticCell::new();
        CELL.uninit().write($val)
    }};
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

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

    spawner.spawn(wifi_connection(wifi_controller)).unwrap();
    spawner.spawn(net_runner(runner)).unwrap();

    let mut bme = initialize_bme(peripherals.I2C0, peripherals.GPIO32, peripherals.GPIO33);
    wait_for_dhcp(stack).await;

    loop {
        let measurements = bme.measure(&mut Delay).unwrap();
        info!(
            "temperature: {=f32} °C, pressure: {=f32} Pa, humidity: {=f32}%",
            measurements.temperature, measurements.pressure, measurements.humidity,
        );
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn wifi_connection(mut controller: WifiController<'static>) {
    info!("starting wifi connection task...");
    loop {
        if wifi::sta_state() == WifiStaState::Connected {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
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
        match controller.connect_async().await {
            Ok(()) => info!("wifi connected"),
            Err(e) => {
                warn!("failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_runner(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

fn initialize_bme(
    i2c: impl I2cInstance + 'static,
    scl: impl OutputPin + 'static,
    sda: impl OutputPin + 'static,
) -> bme280::i2c::BME280<I2c<'static, esp_hal::Blocking>> {
    let i2c = I2c::new(i2c, Default::default())
        .unwrap()
        .with_scl(scl)
        .with_sda(sda);
    let mut bme = bme280::i2c::BME280::new_primary(i2c);
    bme.init(&mut Delay).unwrap();
    bme
}

async fn wait_for_dhcp(stack: embassy_net::Stack<'_>) {
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
