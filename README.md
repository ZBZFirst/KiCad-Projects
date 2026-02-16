# ESP32 BP1 Controller

---

## Reference Images

### PCB Layout

![Controller PCB](https://raw.githubusercontent.com/ZBZFirst/KiCad-Projects/main/ControllerV1.jpg)

### Attachment

![Attachment](https://raw.githubusercontent.com/ZBZFirst/KiCad-Projects/main/AttachmentV2.jpg)

### Finished Build

![Finished Controller](https://raw.githubusercontent.com/ZBZFirst/KiCad-Projects/main/Finished.jpg)

---

Custom ESP32 BLE HID controller + matching PCB design.

This project combines:

- ESP32 firmware (BLE composite keyboard + mouse)
- KiCad PCB layout for a dedicated 7-button controller
- Multi-mode input system for productivity + automation

Designed as a personal hardware controller that behaves like a wireless keyboard/mouse macro pad.

---

## Features

- BLE composite HID device (keyboard + mouse)
- 7 physical buttons
- Mode switching system
- Auto-click / auto-key spam mode
- Windows quick-launch macros
- Non-blocking firmware architecture
- Visual LED status patterns
- Custom PCB layout (KiCad)

---

## Hardware

### Board

ESP32-based custom PCB with:

- 7 tactile switches
- Onboard LED status indicator
- ESP32 dev module header
- Dedicated GPIO routing per button

KiCad layout included in repository.

File:

```
hardware/Controller.kicad_pcb
```

This board is designed specifically to run the included firmware.

---

## Firmware Behavior

The controller has **3 operating groups**.

Button 6 cycles groups.  
Button 7 forces disconnect (reboot).

---

### Group 1 — Default Productivity

| Button | Action |
|--------|--------|
| B1 | Ctrl + C |
| B2 | Ctrl + V |
| B3 | Ctrl + X |
| B4 | Ctrl + Z |
| B5 | 8-second mouse movement run |

LED: solid ON when connected

---

### Group 2 — Work Launch Macros

| Button | Action |
|--------|--------|
| B1 | Windows search → cmd → ssh CB37 |
| B2 | Launch Edge |
| B3 | Launch Super |
| B4 | Launch Arduino |
| B5 | Launch Bluetooth |

LED: 2 blink pattern

---

### Group 3 — Auto Mode

Toggle behaviors:

| Button | Mode |
|--------|------|
| B1 | Auto left click |
| B2 | Auto right click |
| B3 | Auto middle click |
| B4 | Spam key “E” |
| B5 | Spam key “F” |

Press the same button again to stop.

LED: 3 blink pattern

---

## LED Status

- Off → not connected
- Solid → group 1
- 2 blinks → group 2
- 3 blinks → group 3

Blink sequence repeats every 2 seconds.

---

## Installation

### Requirements

- Arduino IDE
- ESP32 board package
- NimBLE / BLE HID libraries

Open:

```
firmware/PaulsESP32.ino
```

Compile and upload to the board.

---

## Usage

1. Power the controller
2. Pair over Bluetooth
3. Use button 6 to select mode
4. Use buttons 1–5 for actions

Designed primarily for Windows but functions as a generic BLE HID device.

---

## Project Structure

```
firmware/
  PaulsESP32.ino

hardware/
  Controller.kicad_pcb
  schematics/
```

---

## Credits

PCB + firmware design: Paul Casillas  
Artwork: Bree Pierce

Personal project.

---

## License

MIT — free to modify and build.
