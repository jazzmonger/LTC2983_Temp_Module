# LTC2983 Temperature Module

ESPHome firmware and KiCad hardware designs for reading precision temperature sensors through the [Analog Devices LTC2983](https://www.analog.com/en/products/ltc2983.html) — a 20-channel multi-sensor ADC supporting RTDs, thermocouples, diodes, and thermistors.

This repo is a **standalone** ESPHome device that reads **5× PT1000 RTDs** and **5× K-type thermocouples** in one multi-channel conversion cycle, with per-channel fault sensors. The channel map matches the CST 5-Pot IO board and can be adapted to other sensor combinations on unused LTC2983 channels.

## Hardware

| Component | Description |
|-----------|-------------|
| **MCU** | ESP32-S3-DevKitC-1 class (16 MB flash, 8 MB octal PSRAM) |
| **Sensor IC** | LTC2983 |
| **Target layout** | Stacked 5 RTD + 5 TC topology ([DC2210A](schematics/dc2210a-5rtd-5tc-wiring.png) reference) |
| **Carrier PCB** | Custom [DC2209A1](KiCad/DC2209A1.kicad_sch) KiCad design — LTC2983 breakout with screw terminals and SPI header |

### SPI wiring (ESP32-S3 → LTC2983)

| Signal | GPIO |
|--------|------|
| CS     | 2    |
| CLK    | 9    |
| MOSI   | 5    |
| MISO   | 4    |

### Channel map

| Sensor | LTC2983 CH | Type | Notes |
|--------|------------|------|-------|
| RTD 1 | 4 | PT1000, 2-wire | Sense pair CH4 ↔ CH3 (COM) |
| RTD 2 | 6 | PT1000, 2-wire | CH6 ↔ CH5 (COM) |
| RTD 3 | 8 | PT1000, 2-wire | CH8 ↔ CH7 (COM) |
| RTD 4 | 10 | PT1000, 2-wire | CH10 ↔ CH9 (COM) |
| RTD 5 | 12 | PT1000, 2-wire | CH12 ↔ CH11 (COM) |
| TC 1 | 13 | K-type, single-ended | TC− → COM; CJC via CH20 diode |
| TC 2 | 14 | K-type | |
| TC 3 | 15 | K-type | |
| TC 4 | 16 | K-type | |
| TC 5 | 17 | K-type | |

Shared excitation: **CH2** = 2 kΩ sense resistor (RSENSE hub). Odd channels CH3/5/7/9/11 tie to CH2. Cold-junction compensation uses a **2N3906 diode on CH20** (emitter → CH20, base + collector → COM).

See `schematics/dc2210a-5rtd-5tc-wiring.png` and `schematics/dc2210a1-sch.pdf` for full wiring of demo circuit or KiCad schematics for this implementation.

## Firmware

The LTC2983 is driven by a custom C++ handler included directly in the ESPHome config — no external component package required.

| File | Purpose |
|------|---------|
| `LTC2983_Temp_Module.yaml` | ESPHome config — 10 template sensors, fault text sensors, 500 ms poll |
| `ltc2983_handler.h` | SPI init, channel assignment, multi-channel conversion, fault decode |
| `common/wifi.yaml` | WiFi / captive portal / web server |

Polling uses non-blocking `read_ltc2983_sensors()`: start a multi-channel conversion (~2 s), publish all channels when DONE, then start the next cycle.

### Home Assistant entities

**Temperature sensors**

- `RTD CH4` … `RTD CH12` — RTD 1–5 (°C)
- `TC Firepot 1` … `TC Firepot 5` — thermocouples (°C)

**Fault text sensors** (one per channel — `OK`, `Open Circuit`, `Overrange`, etc.)

- `Fault CH4`, `Fault CH6`, `Fault CH8`, `Fault CH10`, `Fault CH12`
- `Fault TC1` … `Fault TC5`

## Project layout

```
LTC2983_Temp_Module.yaml   # ESPHome config
ltc2983_handler.h          # LTC2983 SPI driver
common/wifi.yaml           # WiFi package
schematics/                # Wiring diagrams and reference schematics
KiCad/                     # DC2209A1 carrier board (sch, pcb, gerbers)
Demo Circuit Board/        # Analog Devices DC2209A reference files
LTC2983 sketch and info.c  # Reference channel-config notes
```

## Getting started

### Prerequisites

- [ESPHome](https://esphome.io/) installed
- ESP32-S3-DevKitC-1 connected via USB
- LTC2983 board wired per SPI table and channel map above

### Secrets

Create `secrets.yaml` in the project root (gitignored):

```yaml
wifi_ssid: "YourSSID"
wifi_password: "YourPassword"
```

### Build and flash

Copy to `/tmp` before running ESPHome (avoids path issues):

```bash
cp -r "./LTC2983_Temp_Module" /tmp/ltc2983-temp-module/
cd /tmp/ltc2983-temp-module
esphome run LTC2983_Temp_Module.yaml --device /dev/cu.usbserial-XXXX
```

Replace `/dev/cu.usbserial-XXXX` with your serial port. Logs stream automatically — do not use `--no-logs`.

### Home Assistant

Add the device through the ESPHome integration after flashing. All 10 temperature sensors and 10 fault sensors appear under the `ltc2983-temp-module` device.

## References

- [LTC2983 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ltc2983-2984.pdf)
- [DC2210A demo board](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/dc2210a.html) — 5 RTD + 5 TC reference
- [DC2296A evaluation kit](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/dc2296a.html)
- `schematics/dc2210a1-sch.pdf` — DC2210A schematic
- `schematics/dc2213a1-sch.pdf` — DC2213A schematic (single RTD demo)
- `KiCad/Gerbers/` — fabrication outputs for the DC2209A1 carrier board
