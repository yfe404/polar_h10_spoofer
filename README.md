# ESP32 Polar H10 BLE Spoofer

A BLE (Bluetooth Low Energy) device emulator that mimics a Polar H10 heart rate monitor. Built for ESP32-S3 (Seeed Studio XIAO) and compatible with any ESP32 variant with BLE support.

## Features

- **Heart Rate Service (0x180D)** - Standard BLE heart rate monitoring
- **Device Information Service (0x180A)** - Polar H10 device identification
- **Battery Service (0x180F)** - Battery level reporting
- **Polar PMD Service** - Proprietary ECG streaming protocol
- **Simulated ECG waveform** - Realistic PQRST complex at 130Hz
- **BLE bonding support** - Secure pairing with "Just Works" method

## Hardware Requirements

- **Seeed Studio XIAO ESP32S3** (primary target)
- Or any ESP32/ESP32-S3 development board with BLE support

## Software Requirements

- [PlatformIO](https://platformio.org/) (recommended)
- Or Arduino IDE with ESP32 board support

## Quick Start

### Using PlatformIO (Recommended)

```bash
# Clone the repository
git clone https://github.com/yourusername/polar_h10_spoofer.git
cd polar_h10_spoofer

# Build and upload (Seeed XIAO ESP32S3)
pio run -t upload -e seeed_xiao_esp32s3

# Or for generic ESP32-S3
pio run -t upload -e esp32s3

# Or for standard ESP32
pio run -t upload -e esp32

# Monitor serial output
pio device monitor
```

### Using Arduino IDE

1. Install ESP32 board support via Board Manager
2. Open `src/main.cpp`
3. Select your board (ESP32S3 Dev Module or similar)
4. Upload

## BLE Services

| Service | UUID | Description |
|---------|------|-------------|
| Heart Rate | `0x180D` | Heart rate measurement notifications |
| Device Info | `0x180A` | Manufacturer, model, serial, firmware info |
| Battery | `0x180F` | Battery level (85%) |
| Polar PMD | `FB005C80-02E7-F387-1CAD-8ACD2D8DF0C8` | ECG streaming |

## ECG Streaming

The device emulates Polar's proprietary PMD (Polar Measurement Data) protocol:

- **Sample Rate**: 130 Hz
- **Resolution**: 24-bit signed integers
- **Format**: Realistic PQRST waveform template
- **Packet Size**: ~50 samples per BLE packet

### PMD Commands

| Command | Code | Description |
|---------|------|-------------|
| GET_MEASUREMENT_SETTINGS | `0x01` | Query ECG settings |
| START_MEASUREMENT | `0x02` | Start ECG streaming |
| STOP_MEASUREMENT | `0x03` | Stop ECG streaming |
| GET_SDK_MODE | `0x04` | Query SDK mode status |

## Configuration

Edit the device name and ID in `src/main.cpp`:

```cpp
#define DEVICE_NAME "Polar H10 0A3BA92B"
#define DEVICE_ID "0A3BA92B"
```

## Project Structure

```
polar_h10_spoofer/
├── src/
│   └── main.cpp          # Main firmware source
├── platformio.ini        # PlatformIO configuration
├── README.md
├── LICENSE
└── .gitignore
```

## Serial Output

Connect to serial monitor at 115200 baud to see:

```
Starting Polar H10 BLE Spoofer...
BLE advertising started as: Polar H10 0A3BA92B
Waiting for connection...
Device connected!
PMD CP received: 01 00
GET_MEASUREMENT_SETTINGS for ECG
PMD CP received: 02 00 ...
START ECG MEASUREMENT
```

## Compatibility

Tested with:
- Polar Beat app
- Polar Flow app
- Third-party heart rate apps
- Custom BLE applications

## Disclaimer

**This project is for educational and research purposes only.**

- Do not use for medical purposes
- Do not use to falsify fitness/health data
- Respect Polar's intellectual property
- Check local regulations regarding BLE device emulation

## License

MIT License - See [LICENSE](LICENSE) file

## Contributing

Contributions welcome! Please open an issue or pull request.

## Acknowledgments

- Polar Electro for the H10 protocol documentation
- ESP32 Arduino BLE library developers
