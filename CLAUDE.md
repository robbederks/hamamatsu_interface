# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Hamamatsu camera interface combining Teensy 4.1 firmware (C++) with a Python USB control application. Interfaces with a Hamamatsu C7942 X-ray flat panel sensor and Faxitron X-ray system.

## Sensor: Hamamatsu C7942

- **Pixel array**: 2400 x 2400 (native), 2400 x 2320 captured (12-bit packed to fit 8MB PSRAM)
- **Pixel size**: 50 um
- **Active area**: 120 x 120 mm
- **Output**: 12-bit RS422 differential at 12.5 MHz
- **Frame rate**: 2 fps (1x1), 4 fps (2x2 binning)
- **Scintillator**: CsI:Tl
- **X-ray energy range**: 20-100 kVp

The pixel buffer uses 12-bit packing (2 pixels = 3 bytes) in EXTMEM (Teensy 4.1 PSRAM). Frame is 2400×2320 (8.35 MB packed) to fit in 8MB PSRAM.

## Build Commands

### Firmware (PlatformIO)

```bash
# Build and flash to Teensy 4.1
./flash.sh

# Monitor serial output
pio device monitor -p /dev/ttyACM0
```

### Python Application

```bash
# Run the control app (requires usb1, numpy, matplotlib)
python app/app.py

# Run frame acquisition test (no matplotlib, outputs diagnostics)
python app/test_frame.py
```

## Architecture

### Communication Flow

```
Python App (USB) <--> Teensy 4.1 <--> Hamamatsu Sensor
                           |
                           +--> Faxitron X-ray (Serial)
```

### Custom USB Protocol

The firmware patches Arduino core USB libraries via `core_patches/apply.py` to implement custom endpoints:
- Endpoint 5 (OUT): Control commands from host
- Endpoint 6 (IN): Control responses to host
- Endpoint 7 (IN): Bulk frame data transfer

### Command Protocol (app.py ↔ main.cpp)

| Command | Description |
|---------|-------------|
| 0x00 | Ping |
| 0x01 | Get state (returns state, row, col) |
| 0x02 | Get frame (initiates bulk read) |
| 0x03 | Start trigger (begin acquisition) |
| 0x10 | Forward serial command to Faxitron |

### Firmware State Machine

- `STATE_IDLE` (0): Waiting for trigger command
- `STATE_WAITING_ON_VSYNC` (1): Waiting for vertical sync
- `STATE_ACQUIRING` (2): Reading pixel data
- `STATE_DONE` (3): Frame ready for transfer

### Key Files

- `firmware/hamamatsu_interface/src/main.cpp`: Firmware entry point with sensor interface and USB handlers
- `app/app.py`: `HamamatsuTeensy` class for USB communication and Faxitron control
- `app/test_frame.py`: Frame acquisition test script with diagnostics (no GUI dependencies)
- `firmware/hamamatsu_interface/core_patches/`: Arduino core USB customizations applied at build time
- `C7921.pdf`: Hamamatsu X-ray Flat Panel Sensor Application Manual
