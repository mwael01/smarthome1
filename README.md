# Smart Home Automation, Vision & Safety System (ESP32 Hub + ESP32-S3 Stream Camera)

A PlatformIO smart home system with an **ESP32 Hub**, an **ESP32-S3 camera**, and a laptop. The camera sends JPEG frames to the laptop through the CH340 programmer. The laptop runs face recognition, serves the browser preview, and sends access events to the hub over Wi-Fi.

---

## System Architecture

1. **Main Hub (`[env:esp32dev]`, [`src/main.cpp`](src/main.cpp))**
   - **Safety Sensors**: Digital Flame IR (`DO` → `GPIO34`), analog Smoke/Gas MQ-2 (`AO` → divider → `GPIO35`), PIR Motion (`GPIO27`), and HC-SR04 Ultrasonic (`TRIG=GPIO32`, `ECHO=GPIO33`, `<= 25.0 cm`) → sweeping **Siren Buzzer (`GPIO25`)**.
   - **Per-Sensor Alarm Bypass / Ignore**: Dedicated 1-click toggles and NVS settings allow bypassing/muting the siren trigger for individual sensors (e.g. ignore MQ-2 or Flame sensor alarm) while keeping real-time visual telemetry active and other alarms fully armed.
   - **PIR Scheduled Night Guard**: Restricts the loud siren activation on motion to the overnight window between **Midnight (00:00) and 7:00 AM (07:00)**; movement during the day displays in the UI but remains silent.
   - **Smart Lighting Controller (`GPIO21`)**: 3.3V digital output driving a relay module or MOSFET circuit with dashboard controls, automated event logging, and voice feedback.
   - **HW-104 Spoken Voice Speaker (`GPIO4`)**: PAM8403 2×3W Class-D audio amplifier driven via 312.5 kHz LEDC PWM carrier (4× oversampled) with non-blocking FreeRTOS streaming, providing **100% offline natural human spoken sentences** for completed actions (*"Light turned on/off"*, *"Door unlocked/locked"*, *"Car gate opened/closed"*, *"Access granted, welcome home"*, *"Access denied"*, *"Warning: smoke/flame detected"*, and *"Command confirmed"*), with an NVS toggle between natural speech and musical chimes.
   - **INMP441 I2S Digital Microphone (`GPIO17`, `GPIO16`, `GPIO23`)**: Hardware I2S Master RX sampling at 16 kHz, real-time RMS audio level meter, Voice Activity Detection (VAD), and extensible voice command dispatcher.
   - **MFRC522 RFID Card Access**: Hardware SPI reader (`SS=GPIO5`, `RST=GPIO22`, `SCK=GPIO14`, `MOSI=GPIO13`, `MISO=GPIO12`) opening the dual car gate on authorized UID `52411902`.
   - **Car Gate Servos (`GPIO18`, `GPIO19`)** travel to 160° on an authorized RFID scan and hold for 8 seconds by default once both reach their commanded open angle.
   - **Face Door Servo (`GPIO26`)** travels to 160° on recognition and holds for 3 seconds by default once it reaches its commanded open angle.
   - **Intruder Siren Pulse** starts a 4-second alarm by default when the laptop confirms an unknown face or the hub receives an unauthorized RFID scan.
   - **Wireless Hub & Web Admin Panel** serves `http://192.168.4.1` on **`SmartHome-Net`** (password `smarthome123`) and optionally on your home Wi-Fi.
     - **Live System Clock & Time Sync**: Displays live clock (default boot time: 12:00 PM), 1-click Browser Time Sync, and manual time setter.
     - **Smart Lighting Switch**: Interactive ON/OFF/Toggle dashboard buttons with live state feedback on `GPIO21`.
     - **Voice Assistant & VU Meter**: Real-time microphone VU level bar, speech activity indicator badge, speaker status, and instant voice verification test triggers.
     - **Live Camera Stream** loads the laptop's video directly in the browser.
     - **Sensors & Telemetry** displays flame, MQ-2 analog voltage, PIR motion, and ultrasonic distance.
     - **Manual Overrides**: Quick-action controls for Car Gate (open/close), Face Door (unlock/lock), Smart Light (toggle), and Siren Alarm (mute/test).
     - **NVS Settings & Event Log**: Save custom thresholds, PIR schedule hours, timers, and Home Wi-Fi credentials to Flash, and view the live rolling system event log.

2. **Face Camera** (`[env:cam_face_stream_s3_standalone_uart]`, `src/cam_usb_stream/main.cpp`) captures sensor JPEGs into PSRAM and sends them to the laptop over the CH340 connection.
3. **Laptop Face Bridge** reads UART once, serves the newest JPEG to the dashboard, runs recognition, and sends verdicts to the main hub. The dashboard, RFID, manual controls, and sensors remain available when the laptop or camera goes offline.

---

## Hardware Requirements

- **1× ESP32 DevKit** (`esp32dev`) — Main Hub
- **1× ESP32-S3 camera board** with OV2640-compatible camera and PSRAM
- **CH340 USB-to-UART programmer** for camera flashing and live streaming, with 3.3 V logic on TX/RX.
- **Sensors & Actuators on Main Hub**:
  - Digital IR Flame Sensor (`DO` → `GPIO34`)
  - Analog MQ-2 Smoke / Gas Sensor (`AO` → divider → `GPIO35`)
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
| **MQ-2 Smoke Sensor** | `AO` | `GPIO35` | ADC1, 11 dB attenuation | Divide the module output before connecting it to GPIO35. Set the alarm threshold in mV in the dashboard. |
| **PIR Motion Sensor** | `OUT` | `GPIO27` | `INPUT_PULLDOWN` | Active `HIGH` (`3.3 V` logic); siren only activates during night window (**Midnight–7 AM**) |
| **Ultrasonic Sensor** | `TRIG` | `GPIO32` | Digital Output | 10 µs trigger pulse every `60 ms` (`<= 25.0 cm` triggers siren) |
| **Ultrasonic Sensor** | `ECHO` | `GPIO33` | Digital Input | **Use 5 V → 3.3 V voltage divider** |
| **Buzzer** | `SIG` / `+` | `GPIO25` | LEDC Tone PWM (8-bit, Timer 2 Ch 4) | `700 Hz ↔ 1800 Hz` alarm siren |
| **Car Gate Servo 1** | `PWM` | `GPIO18` | `ESP32Servo` 50 Hz (16-bit, Timer 0) | Opens `0° → 160°` on valid RFID UID `52411902` |
| **Car Gate Servo 2** | `PWM` | `GPIO19` | `ESP32Servo` 50 Hz (16-bit, Timer 0) | Opens `0° → 160°` on valid RFID UID `52411902` |
| **Face Door Servo 3** | `PWM` | `GPIO26` | `ESP32Servo` 50 Hz (16-bit, Timer 0) | Opens `0° → 160°` (`3 s`) on recognized face from Camera 1 |
| **MFRC522 RFID Reader** | `SDA` / `SS`<br>`SCK`<br>`MOSI`<br>`MISO`<br>`RST` | `GPIO5`<br>`GPIO14`<br>`GPIO13`<br>`GPIO12`<br>`GPIO22` | Hardware SPI (`MFRC522`) | Reads 13.56 MHz RFID tags; accepted UID: `52411902` |

For the **face door servo**, move its signal wire to main ESP32 **GPIO26**. Connect servo power to a supply rated for the motors' voltage and stall current. Connect the servo supply ground to the main ESP32 ground.

For a **5 V MQ-2 module**, connect module `VCC` to the rated 5 V supply and share its ground with the hub. Wire `AO` through **12 kΩ** to the GPIO35 junction, and **10 kΩ** from that junction to ground. A 5 V output becomes about 2.27 V at GPIO35. Leave `DO` disconnected. After sensor warm-up, use the dashboard reading to set a site-specific threshold above the clean-air baseline. The initial **1200 mV** setting is measured at GPIO35. The project reports voltage and has no gas concentration calibration. [Winsen MQ-2 datasheet](https://www.winsen-sensor.com/d/files/MQ-2.pdf) · [Espressif ADC documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html)

### ESP32-S3 camera to laptop via CH340

Leave the camera on the CH340 programmer after uploading the UART build. For a separate, wired 3.3 V logic adapter, connect adapter `TX` to camera `U0RXD` (GPIO44), adapter `RX` to `U0TXD` (GPIO43), and ground to ground. Use the camera board's rated power input. The camera has no servo or signal wires to the hub. [Espressif ESP32-S3 UART pin documentation](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html#uart)

The camera targets are configured for the **ESP32-S3-CAM N16R8** with 16 MB of quad flash at 80 MHz and 8 MB of octal PSRAM. The app and settings keep the existing partition layout. Use the CH340 USB-to-UART programmer for the board sold with that adapter. [Espressif memory settings](https://docs.espressif.com/projects/arduino-esp32/en/latest/troubleshooting.html)

The camera firmware uses the ESP32-S3-EYE camera pinout. If you have a different ESP32-S3 camera board, verify its schematic before wiring or flashing. On the Espressif ESP32-S3-EYE, use the onboard USB port for power and flashing.

The serial firmware uses Adafruit NeoPixel for the camera board's one addressable RGB LED. Equal red, green, and blue values produce white. Select the LED data GPIO that your board actually uses. Espressif's original DevKitC-1 uses GPIO48 and v1.1 uses GPIO38. An unmarked camera board may wire it differently. The light turns off when the preview closes or stops renewing it. [Espressif RGB LED pin assignments](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html#hardware-revision-details) · [Adafruit NeoPixel Arduino guide](https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use)

---

## Build & Upload (PlatformIO)

[`platformio.ini`](platformio.ini) defines the firmware targets:

```bash
# Build all firmwares at once
pio run

# 1) Flash Main ESP32 Hub (esp32dev)
pio run -e esp32dev -t upload

# 2) Flash ESP32-S3 camera for UART streaming
pio run -e cam_face_stream_s3_standalone_uart -t upload

```

Monitor the hub at **`115200` baud**. Close the camera serial monitor before running the bridge.
```bash
pio device monitor -b 115200
```

---

## Run the integrated UART system on Windows

In VS Code, open **PlatformIO → Project Tasks**. For the hub, select **esp32dev → General → Upload**. To flash the camera, connect the CH340 programmer and close any preview or serial monitor using its COM port. Then expand the matching profile and select **General → Upload**. If both boards are connected, unplug the other board so PlatformIO can find the intended upload port automatically. [PlatformIO Project Tasks](https://docs.platformio.org/en/latest/integration/ide/vscode.html#project-tasks)

| UART upload profile | LED data pin |
| --- | --- |
| `cam_face_stream_s3_uart_led38` | GPIO38 |
| `cam_face_stream_s3_uart_led48` | GPIO48 |

Both profiles start white illumination at level 32 out of 255 when the updated laptop preview or face bridge starts streaming. The generic `cam_face_stream_s3_standalone_uart` profile uses an explicit `--light GPIO LEVEL` option.

Set up the laptop Python environment in PowerShell. In the preview commands below, replace `COM5` with the CH340 port from Device Manager.

```powershell
py -3.11 -m venv "$env:LOCALAPPDATA\smarthome-face-venv"
$facePython = "$env:LOCALAPPDATA\smarthome-face-venv\Scripts\python.exe"
& $facePython -m pip install -r face_recigntion2\requirements.txt pyserial "setuptools<81"
```

If `dlib` needs to compile on your machine, install the C++ prerequisites listed in the [Windows face recognition setup](face_recigntion2/docs/03-implementation/setup.md).

Connect Windows to **SmartHome-Net** and open **http://192.168.4.1**. Close any USB preview window that owns the camera COM port. Start the laptop bridge with the profile you tested.

```powershell
& $facePython tools\laptop_face_bridge.py --uart-port COM5 --resolution qvga --jpeg-quality 16
```

Press **E** in the preview to enroll a face. Use `--resolution hvga` or `--resolution vga` if the tested cable profile sustains it. For headless operation, add `--no-gui`. If the hub and laptop are on your home network, pass the hub's home-network IP with `--hub`. Open the dashboard from the same network as the laptop. If the laptop preview works but the dashboard reports **PREVIEW UNREACHABLE**, allow inbound TCP **8080** for the bridge from the trusted local network in Windows Firewall. If port 8080 is in use, pass `--preview-port` with an available port.

To adjust brightness, add `--light GPIO LEVEL` with the GPIO from the uploaded profile. Use level 0 to keep the light off. The level range is 0–255.

```powershell
& $facePython tools\laptop_face_bridge.py --uart-port COM5 --resolution qvga --light 48 32
```

### Standalone USB and UART camera tests

For the 30–40 FPS trial at HVGA (480×320), use the camera board's native USB connector if it exposes the ESP32-S3 USB data pins. The ESP32-S3 provides full-speed USB on GPIO20 (D+) and GPIO19 (D−). The separate CH340 programmer connects to UART0 and has its own test build. At its rated 2 Mbaud with 8N1 framing, it carries at most 195 KiB/s, or about 4.9 KiB per frame at 40 FPS before packet overhead. [Espressif USB device documentation](https://docs.espressif.com/projects/esp-usb/en/latest/esp32s3/usb_device.html) · [WCH CH340 datasheet](https://datasheet.lcsc.com/lcsc/2304140030_WCH-Jiangsu-Qin-Heng-CH340C_C84681.pdf)

```bash
python3 -m pip install pyserial opencv-python
pio run -e cam_face_stream_s3_standalone_usb -t upload
python3 tools/usb_camera_preview.py --list-ports
python3 tools/usb_camera_preview.py --port /dev/ttyACM0 --resolution hvga
```

You can upload through the programmer, then move the cable to the camera's verified native USB connector. Select the port that appears after the firmware boots. On Windows use the listed `COM` port. If the CH340 programmer is your only connection, start the UART test at QVGA instead.

```bash
pio run -e cam_face_stream_s3_standalone_uart -t upload
python3 tools/usb_camera_preview.py --port /dev/ttyUSB0 --uart --resolution qvga
```

The standalone preview accepts the same `--light GPIO LEVEL` option. On Windows, replace the COM port and select the verified data pin.

```powershell
& $facePython tools\usb_camera_preview.py --port COM5 --uart --resolution qvga --light 48 32
```

If the adapter or laptop driver is unreliable at 2 Mbaud, build and upload with `PLATFORMIO_BUILD_FLAGS=-DCAMERA_USB_UART_BAUD=921600` and add `--baud 921600` to the preview command.

Measure reception without preview scaling or JPEG decoding. Repeat with `--resolution qvga` and `--resolution vga` to find the largest profile that sustains your target. Higher `--jpeg-quality` numbers use stronger compression. Compare settings in the same lighting and scene, as exposure and JPEG size affect throughput. Remove `--no-gui` for the resizable live preview.

```bash
python3 tools/usb_camera_preview.py --port /dev/ttyACM0 --resolution hvga --no-gui --frames 300
```

The receiver reports verified FPS, byte rate, gaps between frames, and damaged packets. The USB test transfers the sensor's JPEG bytes with a checksum and keeps only the newest frame for display. It uses the Python receiver and runs with Wi-Fi and recognition off.

### Internet access while streaming

The hub hotspot carries local dashboard and recognition traffic. Use Ethernet for internet access while connected to the hotspot, or connect the laptop and hub to your home Wi-Fi and pass the hub's address with `--hub`.

---

## Project Structure

```text
.
├── platformio.ini                 # Main hub and ESP32-S3 face-stream environments
├── ├── include/
│   ├── smart_home_json.h          # Shared JSON string escaping helper
│   └── smart_home_protocol.h      # Hub hotspot and face event definitions
├── src/
│   ├── main.cpp                   # Main hub firmware (sensors, servos, Wi-Fi dashboard)
│   ├── cam_common/                # Serial camera capture and sensor pinout
│   └── cam_usb_stream/
│       └── main.cpp               # ESP32-S3 USB CDC and CH340 UART camera firmware
└── README.md                      # Project documentation
```
