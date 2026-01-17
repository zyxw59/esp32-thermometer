# esp32-thermometer

This is a tool for reporting and logging data from one or more ESP32 devices
with BME280 (or BMP280) sensors.
There are two binary crates:
- `thermometer-logger` is a server that accepts connections from the ESP32 and
  prints out the data it receives.
- `esp32-thermometer` is the code that runs on the ESP32 and handles reading
  the data from the BME280 and sending that data to the server.

## Running

### Server

From the `thermometer-logger` directory, do `cargo run`. The server outputs its
received data to standard out, so if you want to save the data, pipe it to a
file.

You can optionally set the host and port the server will listen on by setting
the `HOST` and `PORT` environment variables. It defaults to `0.0.0.0:7878`

### ESP32

From the `esp32-thermometer` directory, do `cargo run` with the ESP32 plugged
in. This requires `espflash` to be installed.

The following environment variables must be set:
- `SSID`: SSID to connect to
- `PASSWORD`: WiFi password
- `SERVER_HOST`: Hostname or IP address of the server
- `LOCATION_ID`: Unique identifier to include in this sensor's data

The following environment variables are recognized and have default values:
- `SERVER_PORT`: Port to use for the server. Defaults to 7878
- `SCL_PIN`: GPIO pin for I²C clock. Defaults to 32
- `SDA_PIN`: GPIO pin for I²C data. Defaults to 33
- `INTERVAL`: Interval (in seconds) between measurements
- `RETRY_INTERVAL`: Interval (in seconds) to wait before retrying after an
  error or timeout.
- `MAX_TIMEOUTS`: Maximum number of timeouts in the measurement loop before
  resetting. Defaults to 15. Set to 0 to disable resets
