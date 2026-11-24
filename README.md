# Smart Home Automation, Vision & Safety System (ESP32 Hub + ESP32-S3 Stream Camera)

A multi-node ESP32 smart home safety and biometric access-control system built with **PlatformIO**. It combines a **Main ESP32 Hub** (`esp32dev`) with one **ESP32-S3 stream camera**. The camera sends MJPEG frames to a laptop for face AI and sends verdicts to the hub over Wi-Fi and **ESP-NOW**.

---

## System Architecture

1. **Main Hub (`[env:esp32dev]`, [`src/main.cpp`](src/main.cpp))**
   - **Safety Sensors**: Digital Flame IR (`DO` → `GPIO34`), Digital Smoke/Gas MQ-2 (`DO` → `GPIO35`), PIR Motion (`GPIO27`), and HC-SR04 Ultrasonic (`TRIG=GPIO32`, `ECHO=GPIO33`, `<= 25.0 cm`) → sweeping **Siren Buzzer (`GPIO25`)**.
   - **Per-Sensor Alarm Bypass / Ignore**: Dedicated 1-click toggles and NVS settings allow bypassing/muting the siren trigger for individual sensors (e.g. ignore MQ-2 or Flame sensor alarm) while keeping real-time visual telemetry active and other alarms fully armed.
   - **PIR Scheduled Night Guard**: Restricts the loud siren activation on motion to the overnight window between **Midnight (00:00) and 7:00 AM (07:00)**; movement during the day displays in the UI but remains silent.
   - **Smart Lighting Controller (`GPIO21`)**: 3.3V digital output driving a relay module or MOSFET circuit with dashboard controls, automated event logging, and voice feedback.
   - **HW-104 Spoken Voice Speaker (`GPIO4`)**: PAM8403 2×3W Class-D audio amplifier driven via 312.5 kHz LEDC PWM carrier (4× oversampled) with non-blocking FreeRTOS streaming, providing **100% offline natural human spoken sentences** for completed actions (*"Light turned on/off"*, *"Door unlocked/locked"*, *"Car gate opened/closed"*, *"Access granted, welcome home"*, *"Access denied"*, *"Warning: smoke/flame detected"*, and *"Command confirmed"*), with an NVS toggle between natural speech and musical chimes.
   - **INMP441 I2S Digital Microphone (`GPIO17`, `GPIO16`, `GPIO23`)**: Hardware I2S Master RX sampling at 16 kHz, real-time RMS audio level meter, Voice Activity Detection (VAD), and extensible voice command dispatcher.
   - **MFRC522 RFID Card Access**: Hardware SPI reader (`SS=GPIO5`, `RST=GPIO22`, `SCK=GPIO14`, `MOSI=GPIO13`, `MISO=GPIO12`) opening the dual car gate on authorized UID `52411902`.
   - **Car Gate Servos (`GPIO18`, `GPIO19`)**: `0° ↔ 160°` smooth non-blocking PWM ramp, opened (`5 seconds`) when an authorized RFID card is presented.
   - **3rd Face Door Servo (`GPIO26`)**: `0° ↔ 160°` smooth non-blocking PWM ramp, opened exclusively (`3 seconds`) when Camera 1 recognizes an enrolled face.
   - **Intruder Siren Pulse**: Triggers a **`4-second`** siren burst (`GPIO25` + `GPIO4`) whenever Camera 1 reports an unrecognized face or an unauthorized RFID card is scanned.
   - **Wireless Hub & Web Admin Panel**: Hosts Wi-Fi SoftAP **`SmartHome-Net`** (password `smarthome123`, channel `1`, IP `http://192.168.4.1`) + optional Home Wi-Fi STA uplink, ESP-NOW receiver, and a **Full Web Admin Dashboard**:
     - **Live System Clock & Time Sync**: Displays live clock (default boot time: 12:00 PM), 1-click Browser Time Sync, and manual time setter.
     - **Smart Lighting Switch**: Interactive ON/OFF/Toggle dashboard buttons with live state feedback on `GPIO21`.
     - **Voice Assistant & VU Meter**: Real-time microphone VU level bar, speech activity indicator badge, speaker status, and instant voice verification test triggers.
     - **Live Camera Streams**: Embedded face-camera viewport with live FPS/latency and direct/proxy streaming with live FPS/latency and direct/proxy streaming.
     - **Sensors & Telemetry**: Dynamic visual gauges and alert badges for Flame (`AO`), Smoke (`MQ-2`), Motion (`PIR`), and Ultrasonic (`HC-SR04`).
     - **Manual Overrides**: Quick-action controls for Car Gate (open/close), Face Door (unlock/lock), Smart Light (toggle), and Siren Alarm (mute/test).
     - **NVS Settings & Event Log**: Save custom thresholds, PIR schedule hours, timers, and Home Wi-Fi credentials to Flash, and view the live rolling system event log.

2. **Face Camera — ESP32-S3 stream with laptop AI** (`[env:cam_face_stream_s3]`, `src/cam_face_stream/main.cpp`)
   - Captures JPEG frames on the ESP32-S3 and serves a low-latency MJPEG stream on port 81.
   - Face detection and recognition run on the laptop. Recognition verdicts return to the camera and Main Hub over HTTP and ESP-NOW.
   - A recognized face unlocks Door Servo 3. An unknown face starts the intruder siren.
   - PSRAM triple buffering, Wi-Fi power-save disabled, and maximum transmit power keep the camera focused on transmission.

---

## Hardware Requirements

- **1× ESP32 DevKit** (`esp32dev`) — Main Hub
- **1× ESP32-S3 camera board** with OV2640-compatible camera and PSRAM
- **USB-to-TTL (`FTDI` / `CP2102` / `CH340`) Programmer** (3.3 V UART TX/RX, for flashing ESP32-S3 camera board; or an ESP32-CAM-MB daughterboard)
- **Sensors & Actuators on Main Hub**:
  - Digital IR Flame Sensor (`DO` → `GPIO34`)
  - Digital MQ-2 Smoke / Gas Sensor (`DO` → `GPIO35`)
  - PIR Motion Sensor (`OUT` → `GPIO27`)
  - HC-SR04 Ultrasonic Sensor (`TRIG` → `GPIO32`, `ECHO` → `GPIO33` via 5 V → 3.3 V resistor divider)
  - Passive Piezo Buzzer (`SIG` → `GPIO25`)
  - **3× 180° Servo Motors** (e.g., MG996R / DS3218, `500–2500 µs`): **Car Gate Servo 1 (`GPIO18`)**, **Car Gate Servo 2 (`GPIO19`)**, **Face Door Servo 3 (`GPIO26`)**
  - **Dedicated 5 V ≥ 3 A Power Supply** (common `GND` shared with Main ESP32 and servos; ESP32-S3 camera board should be powered cleanly at **5 V** on the `5V` pin rather than a weak `3.3 V` regulator)

---

## Wiring & Pinout

### Main ESP32 Hub (`esp32dev`)

| Component | Module Pin | ESP32 GPIO | Mode | Notes |
| --- | --- | --- | --- | --- |
| **Smart Light (Relay / MOSFET)** | `IN / Gate` | `GPIO21` | Digital Output (`3.3 V`) | Connect to 5V relay module IN or MOSFET gate to switch light loads |
| **HW-104 Spoken Voice Speaker** | `L / R` (Audio In) | `GPIO4` | LEDC 312.5 kHz PWM (8-bit, Timer 3 Ch 6) | PAM8403 2×3W Class-D amplifier; non-blocking FreeRTOS natural spoken sentences & dual siren alarms |
| **INMP441 I2S Microphone** | `SCK` (BCLK)<br>`WS` (LRCLK)<br>`SD` (Data)<br>`L/R` (Channel)<br>`VDD` | `GPIO17`<br>`GPIO16`<br>`GPIO23`<br>`GND`<br>`3V3` | Hardware I2S (`I2S_NUM_0`) | 16 kHz Master RX digital microphone; VDD connects strictly to 3.3V (NOT 5V) |
| **Flame Sensor** | `DO` | `GPIO34` | Digital Input (`INPUT`) | Active-LOW (0V = Flame, 3.3V = Safe); potentiometer calibrated on module; siren trigger can be ignored via dashboard |
| **MQ-2 Smoke Sensor** | `DO` | `GPIO35` | Digital Input (`INPUT`) | Active-LOW (0V = Smoke/Gas, 3.3V = Clean); potentiometer calibrated on module; siren trigger can be ignored via dashboard |
| **PIR Motion Sensor** | `OUT` | `GPIO27` | `INPUT_PULLDOWN` | Active `HIGH` (`3.3 V` logic); siren only activates during night window (**Midnight–7 AM**) |
| **Ultrasonic Sensor** | `TRIG` | `GPIO32` | Digital Output | 10 µs trigger pulse every `60 ms` (`<= 25.0 cm` triggers siren) |
| **Ultrasonic Sensor** | `ECHO` | `GPIO33` | Digital Input | **Use 5 V → 3.3 V voltage divider** |
| **Buzzer** | `SIG` / `+` | `GPIO25` | LEDC Tone PWM (8-bit, Timer 2 Ch 4) | `700 Hz ↔ 1800 Hz` alarm siren |
| **Car Gate Servo 1** | `PWM` | `GPIO18` | `ESP32Servo` 50 Hz (16-bit, Timer 0) | Opens `0° → 160°` (`5 s`) on valid RFID UID `52411902` |
| **Car Gate Servo 2** | `PWM` | `GPIO19` | `ESP32Servo` 50 Hz (16-bit, Timer 0) | Opens `0° → 160°` (`5 s`) on valid RFID UID `52411902` |
| **Face Door Servo 3** | `PWM` | `GPIO26` | `ESP32Servo` 50 Hz (16-bit, Timer 0) | Opens `0° → 160°` (`3 s`) on recognized face from Camera 1 |
| **MFRC522 RFID Reader** | `SDA` / `SS`<br>`SCK`<br>`MOSI`<br>`MISO`<br>`RST` | `GPIO5`<br>`GPIO14`<br>`GPIO13`<br>`GPIO12`<br>`GPIO22` | Hardware SPI (`MFRC522`) | Reads 13.56 MHz RFID tags; accepted UID: `52411902` |

### ESP32-S3 camera power and flash

The camera firmware uses the ESP32-S3-EYE camera pinout. If you have a different ESP32-S3 camera board, verify its schematic before wiring or flashing. On the Espressif ESP32-S3-EYE, use the onboard USB port for power and flashing.

The ESP32-S3-EYE has no dedicated camera flash. GPIO2 is microphone data, GPIO48 is the LCD backlight, and GPIO3 is the module power indicator. Flash controls stay disabled until a verified flash-control GPIO is configured. [Espressif board guide](https://docs.espressif.com/projects/esp-board-manager/en/latest/references/boards/esp32_s3_eye.html) · [Espressif pin map](https://github.com/espressif/esp-bsp/blob/master/bsp/esp32_s3_eye/esp32_s3_eye.json)

For an external light, use a driver with a 3.3 V-compatible active-high PWM input. Wire the verified GPIO to the driver input, share ground between the camera and driver, and power the light from its rated supply through the driver. Set `-DCAM_HW_FLASH_LED_PIN=<verified GPIO>` in the camera build flags.

---

## Build & Upload (PlatformIO)

[`platformio.ini`](platformio.ini) defines three firmware targets:

```bash
# Build all firmwares at once
pio run

# 1) Flash Main ESP32 Hub (esp32dev)
pio run -e esp32dev -t upload

# 2) Flash ESP32-S3 face camera for use with the hub
pio run -e cam_face_stream_s3 -t upload

```

Monitor any board at **`115200` baud**:
```bash
pio device monitor -b 115200
```

---

## Web Dashboards & Enrollment

1. Power on the **Main Hub** first (`SmartHome-Net` SoftAP starts on channel `1`).
2. Power on the **ESP32-S3 face camera**. It automatically joins `SmartHome-Net`.
3. Connect your phone or laptop to Wi-Fi **`SmartHome-Net`** (password: **`smarthome123`**):
   - **Main Hub Status**: `http://192.168.4.1`
   - **Face camera stream**: `http://192.168.4.101` — The laptop runs enrollment, face detection, and recognition against the live MJPEG stream.
   - **Main Hub**: `http://192.168.4.1` — RFID authorization controls the car gate.

> **Standalone bench setup**
>
> If the camera cannot join `SmartHome-Net` within about 12 seconds, connect to `SmartHome-FaceCam` using `smarthome123`, then open `http://192.168.11.1`. This network has no internet access. The camera retries the hub after the hotspot has been idle for 30 seconds and closes the hotspot once it rejoins the hub.

### Standalone camera preview and speed test

1. Flash just the camera with `pio run -e cam_face_stream_s3_standalone -t upload`. This build starts the hotspot at boot and keeps it dedicated to the direct laptop link.
2. Join `SmartHome-FaceCam` using `smarthome123`. Open the dashboard at `http://192.168.11.1` or the direct MJPEG preview at `http://192.168.11.1:81/stream`.
3. Close the preview before measuring single-client speed. Run `python3 tools/test_camera_stream.py` on the laptop. It checks 120 consecutive JPEGs and reports time to first frame, received FPS, and JPEG throughput.
4. Compare VGA and QVGA in the dashboard settings. QVGA uses less bandwidth. VGA gives the laptop AI more detail. Repeat the speed test after changing the resolution or JPEG quality.
5. For a laptop AI preview on this hotspot, run `python3 tools/laptop_face_bridge.py --ip 192.168.11.1`. Flash `cam_face_stream_s3` when you are ready to reconnect the camera to the hub.

---

## Project Structure

```text
.
├── platformio.ini                 # Main hub and ESP32-S3 face-stream environments
├── ├── include/
│   ├── smart_home_json.h          # Shared JSON string escaping helper
│   └── smart_home_protocol.h      # Shared Wi-Fi SoftAP & ESP-NOW CamEventPacket definitions
├── src/
│   ├── main.cpp                   # Main Hub firmware (sensors, servos, SoftAP, ESP-NOW, Web UI)
│   ├── cam_common/                # Shared camera hardware, JPEG stream, and network code
│   └── cam_face_stream/
│       └── main.cpp               # ESP32-S3 MJPEG stream and laptop-AI bridge
└── README.md                      # Project documentation
```
