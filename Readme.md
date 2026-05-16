<div align="center">
  <img src="logo.svg" alt="Aegion Dynamics Logo" width="200">
  <h1>Mimic</h1>
  <p><strong>Hardware Bridge and Sensor Emulation Starter Pack</strong></p>
</div>

---

## Overview

Mimic is an open-source hardware bridge designed for prototyping IoT and aerospace applications. It allows you to connect a host computer to physical hardware peripherals via Python, enabling basic sensor emulation and protocol testing.

This project provides a collection of sensor simulation modules (such as MPU6050, BMP280, and GPS) that integrate with the Mimic Framework, allowing developers to interact with hardware without complex or proprietary tools.

## Supported Boards

- **STM32F411CEU6** (BlackPill)
- **ESP8266 / ESP32** (Experimental)

## Features

- **I2C**: Master and Slave mode emulation.
- **SPI**: Master and Slave transactions.
- **UART**: Serial communication and RS485 testing.
- **GPIO**: Digital input and output control.
- **Sensor Emulation**: Pre-built modules for MPU6050, BMP280, GPS, and more.

---

## Hardware Reference (STM32 BlackPill)

| Peripheral | Pins |
| :--- | :--- |
| Host Bridge | PA9 (TX) / PA10 (RX) |
| I2C 1 | PB8 (SCL) / PB9 (SDA) |
| SPI 1 | PA5 (SCK) / PA6 (MISO) / PA7 (MOSI) |
| UART 6 | PA11 (TX) / PA12 (RX) |
| UART 2 | PA2 (TX) / PA3 (RX) |

---

## Installation

Install the library using pip:

```bash
git clone https://github.com/Karthik-Sarvan/Mimic.git
cd Mimic
pip install .
```

---

## Usage

### CLI Emulation
To emulate a device like the MPU6050:
```bash
mimic-sim mpu6050
```

### Python API
Basic script to toggle a pin:
```python
from mimic import MimicBridge

bridge = MimicBridge()
if bridge.connect():
    bridge.execute("PIN_HIGH A5")
```

---

## Contributors

Aegion Dynamics welcomes all contributors. If you want to add new sensors, improve the firmware, or fix bugs, please submit a pull request.

- **Karthik Sarvan** (Project Maintainer)

*To be listed as a contributor, submit your first Pull Request.*

---

## License

MIT License.

---

**Aegion Dynamics**