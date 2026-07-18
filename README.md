# LTC2983 Temperature Module

ESPHome firmware and hardware designs for reading precision temperature sensors through the [Analog Devices LTC2983](https://www.analog.com/en/products/ltc2983.html) — a 20-channel multi-sensor ADC supporting RTDs, thermocouples, diodes, and thermistors.


## Hardware

| Component | Description |
|-----------|-------------|
| **MCU** | ESP32-S3-DevKitC-1 class (16 MB flash, 8 MB octal PSRAM) |
| **Sensor IC** | LTC2983 on Analog Devices [DC2213A](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/dc2213a.html) demo board |
| **Current sensor** | External PT1000 RTD on DC2213A **J3** connector |
| **Carrier PCB** | Custom [DC2209A1](KiCad/DC2209A1.kicad_sch) KiCad design — LTC2983 breakout with screw terminals and SPI header |

### SPI wiring (ESP32-S3 → LTC2983)

| Signal | GPIO |
|--------|------|
| CS     | 2    |
| CLK    | 9    |
| MOSI   | 5    |
| MISO   | 4    |

### DC2213A J3 PT1000 wiring

The firmware configures **CH8** as a 4-wire Kelvin PT1000 using the onboard **CH3** 2 kΩ sense resistor (DC2213A official topology). CH5 is the onboard demo RTD and is **not** used for the external sensor.

| J3 pin | LTC2983 channel | Connect to |
|--------|-----------------|------------|
| 4 (RTDFH) | CH3 | PT1000 leg 1 |
| 3 (RTDSH) | CH7 | PT1000 leg 1 (tied with pin 4) |
| 2 (RTDSL) | CH8 | PT1000 leg 2 |
| 1 (RTDFL) | CH9 | Leave open |

See `schematics/dc2213a-pt1000-wiring.png` for the full diagram.

## Firmware

The LTC2983 is driven by a custom C++ handler included directly in the ESPHome config — no external component package required.

- **`LTC2983_Temp_Module.yaml`** — main ESPHome config (WiFi, API, OTA, sensor polling)
- **`ltc2983_handler.h`** — SPI init, channel assignment, conversion, and temperature parsing
- **`common/wifi.yaml`** — shared WiFi / captive portal / web server settings

A template sensor **PT1000 Temperature** is polled every 5 seconds via `read_ltc2983_single_rtd()`.

### Planned expansion

`LTC2983 sketch and info.c` documents channel configuration for a stacked layout with 5× PT1000 RTDs and 5× K-type thermocouples (target hardware: [DC2296A](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/dc2296a.html) / [DC2210A](schematics/dc2210a-5rtd-5tc-wiring.png)). The current firmware implements a single external RTD only.

## Project layout

```
LTC2983_Temp_Module.yaml   # ESPHome config
ltc2983_handler.h          # LTC2983 SPI driver
common/wifi.yaml           # WiFi package
schematics/                # Wiring diagrams and reference schematics
KiCad/                     # DC2209A1 carrier board (sch, pcb, gerbers)
Demo Circuit Board/        # Analog Devices DC2209A reference files
```

## Getting started

### Prerequisites

- [ESPHome](https://esphome.io/) installed
- ESP32-S3-DevKitC-1 connected via USB
- DC2213A (or compatible LTC2983 board) wired per SPI table above
- PT1000 probe connected to J3

### Secrets

Create `secrets.yaml` in the project root (gitignored):

```yaml
wifi_ssid: "YourSSID"
wifi_password: "YourPassword"
```

### Build and flash

Because the project path contains spaces, copy to `/tmp` before running ESPHome:

```bash
cp -r "./LTC2983_Temp_Module" /tmp/ltc2983-temp-module/
cd /tmp/ltc2983-temp-module
esphome run LTC2983_Temp_Module.yaml --device /dev/cu.usbserial-XXXX
```

Replace `/dev/cu.usbserial-XXXX` with your serial port. Logs stream automatically — do not use `--no-logs`.

### Home Assistant

After flashing, add the device through the ESPHome integration. The **PT1000 Temperature** entity appears under `sensor.pt1000_temperature`.

## References

- [LTC2983 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ltc2983-2984.pdf)
- [DC2213A demo manual](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/dc2213a.html)
- `schematics/dc2213a1-sch.pdf` — DC2213A schematic
- `schematics/dc2210a1-sch.pdf` — DC2210A schematic (5 RTD + 5 TC reference)
- `KiCad/Gerbers/` — fabrication outputs for the DC2209A1 carrier board
