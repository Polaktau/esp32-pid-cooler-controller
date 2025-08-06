```markdown
# ESP32 PID Cooler Controller

ESP32-based temperature controller with PID regulation and web interface for precise cooling management.

## Features

- Precise temperature control using Dallas DS18B20 sensor
- PID algorithm with anti-windup protection for stable operation
- Web interface for monitoring and configuration
- Real-time temperature and PWM monitoring
- NTP time synchronization
- Over-the-Air (OTA) updates
- EEPROM settings storage

## Hardware

- ESP32 development board (e.g., ESP32 DevKit)
- Dallas DS18B20 temperature sensor
- N-channel MOSFET or transistor for fan control
- 12V fan (or appropriate voltage for your fan)
- Pull-up resistor for DS18B20 data line (often built into sensor modules)
- Resistors and capacitors as needed
- 12V power supply

### Pin Connections

| ESP32 Pin | Component         |
|-----------|-------------------|
| GPIO 4    | DS18B20 Data Pin  |
| GPIO 5    | Fan Control (PWM) |
| 3.3V      | DS18B20 VCC       |
| GND       | DS18B20 GND, Fan GND |

*Note: Fan VCC should be connected to an external power supply (e.g., 12V), controlled via the MOSFET.*

## Software

### Dependencies

This project requires the following Arduino libraries:
- [OneWire](https://www.arduino.cc/reference/en/libraries/onewire/)
- [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)
- [QuickPID](https://github.com/dlloydev/QuickPID) (Standard version, no modifications needed with the latest fix)
- [NTPClient](https://github.com/arduino-libraries/NTPClient) (Standard version)
- ESP32 Arduino Core

Install these libraries via the Arduino IDE Library Manager.

### Configuration

1.  Open `cooler_controller.ino` in the Arduino IDE.
2.  Modify the Wi-Fi credentials in the `// === НАСТРОЙКИ WI-FI ===` section:
    ```cpp
    const char* _ssid = "WiFi";
    const char* _password = "12345678";
    ```
3.  Adjust pin definitions in the `// Пин-конфигурация` section if needed:
    ```cpp
    #define ONE_WIRE_BUS 4
    #define FAN_PIN 5
    ```
4.  Compile and upload the sketch to your ESP32 board.
### Time Zone Configuration

The device uses NTP to get the current time. By default, it is configured for Kaliningrad time (GMT+2).

To change the time zone:

1.  Open `cooler_controller.ino` in the Arduino IDE.
2.  Find the section `// === НАСТРОЙКИ NTP ===`.
3.  Modify the `GMT_OFFSET_SEC` value:
    *   Calculate the offset in seconds from UTC (GMT). For example:
        *   Moscow Time (GMT+3): `#define GMT_OFFSET_SEC 10800`
        *   London Time (GMT+0): `#define GMT_OFFSET_SEC 0`
        *   New York Time (EST, GMT-5): `#define GMT_OFFSET_SEC -18000`
        *   Tokyo Time (GMT+9): `#define GMT_OFFSET_SEC 32400`
    *   Also adjust `DAYLIGHT_OFFSET_SEC` if your region observes Daylight Saving Time (e.g., set to `3600` for +1 hour during DST, `0` otherwise). For Kaliningrad, it's `0`.
4.  Save the file and re-upload the sketch to your ESP32.

### Accessing the Web Interface

1.  Ensure the device is powered on and successfully connected to your Wi-Fi network.
2.  Find the assigned IP address:
    *   **Method 1 (Serial Monitor):** Open the Serial Monitor in the Arduino IDE (baud rate 115200). Look for a line like `IP адрес: 192.168.1.100` after the device boots.
    *   **Method 2 (Router Dashboard):** Log into your router's administration page. Look for a list of connected devices. Find the device named `cryomind-esp32` or with a similar MAC address and note its IP address.
3.  Open a web browser on any device connected to the same network.
4.  Enter the IP address (e.g., `http://192.168.1.100`) into the browser's address bar and press Enter.
5.  The web interface should load. You can also try accessing it via mDNS if your network supports it: `http://cryomind-esp32.local` (Note: mDNS might not work on all devices or networks).
## Usage

1.  Power on the device. It will connect to the configured Wi-Fi network.
2.  Open the Serial Monitor (or check your router's DHCP table) to find the assigned IP address of the ESP32.
3.  Enter the IP address in a web browser to access the control interface.
4.  Set the desired temperature (Setpoint) via the web interface.
5.  Monitor the current temperature and fan PWM level in real-time.
6.  Adjust PID parameters (Kp, Ki, Kd) through the web interface if necessary for optimal performance.
7.  Use the "Save Settings" button to store parameters in EEPROM.

## Web Interface

The web interface provides:
- Real-time temperature display
- Current fan PWM level
- Uptime counter
- Setpoint adjustment
- PID parameter tuning
- Sensor offset calibration
- System counters (resets, errors)
- NTP time display
- Reboot and factory reset options

## Troubleshooting

- **Cannot connect to Wi-Fi:** Verify SSID and password in the code.
- **Sensor error:** Check wiring, pull-up resistor, and sensor functionality.
- **Fan not responding:** Check MOSFET wiring, power supply, and PWM pin configuration.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Thanks to the creators of the OneWire, DallasTemperature, QuickPID, and NTPClient libraries.
```
