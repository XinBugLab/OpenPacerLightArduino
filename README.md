# PacerLightArduino

A flexible and responsive LED controller for Arduino, designed to be controlled remotely via serial commands. This project provides a smooth way to manage LED animations like fade-in, hold, and fade-out, making it ideal for creating visual cues, status indicators, or pacer lights in various applications.

## Features

- **Smooth LED Animations:** Supports configurable fade-in, hold, and fade-out durations for seamless transitions.
- **Remote Control:** Easily controlled via a simple serial command protocol.
- **Addressable Devices:** Each controller can be assigned a unique ID (stored in EEPROM), allowing multiple devices to be controlled independently on the same bus.
- **Broadcast Commands:** Supports broadcast commands to control all devices simultaneously.
- **Encapsulated Logic:** The LED control logic is encapsulated in a clean, object-oriented `LedController` class, making the code easy to read and maintain.
- **Non-blocking:** The code is written in a non-blocking style to ensure responsiveness.

## Hardware Requirements

- An Arduino-compatible microcontroller: Arduino Uno R3.
- A PWM-controllable LED or LED strip.
- A serial communication module to send commands.

### Default Pinout

- **LED PWM Pin:** `D6`
- **SoftwareSerial RX:** `D2`
- **SoftwareSerial TX:** `D4`
- **Wireless Module Mode Pin:** `D7`, used to set the mode of wireless module `JDY-40`.

## Software Requirements

- [Arduino IDE](https://www.arduino.cc/en/software)
- Standard Arduino Libraries:
  - `SoftwareSerial.h`
  - `EEPROM.h`

## Installation

1.  Clone this repository or download the source code.
2.  Connect your hardware according to the pinout described above. You can change the pins in the `sketch_oct19a.ino` file if needed.
3.  Open `sketch_oct19a.ino` in the Arduino IDE.
4.  Select your board and port from the `Tools` menu.
5.  Upload the sketch to your Arduino.

## Usage: Serial Command Protocol

The controller listens for serial commands in the following format. Each command is a single line ending with a newline character (`\n`).

**Format:**
```
<id_mask_hex>,<brightness>,<fade_in_ms>,<hold_ms>,<fade_out_ms>
```

**Example:**
```
1FFF,255,500,1000,500
```

### Command Breakdown

1.  **`id_mask_hex` (Hexadecimal `unsigned long`)**:
    - A 16-bit value where the most significant 3 bits represent the `opcode` and the lower 13 bits represent the `target_id`.
    - **Opcode `0` (Light Control):** Triggers an LED animation. The command is executed if the `target_id` matches the device's ID, is a broadcast address (`0x1FFF`), or is a mask that includes the device's ID.
    - **Opcode `1` (Set Device ID):** Updates the device's unique ID.

2.  **`brightness` (Integer `0-255`)**: The target brightness for the LED.

3.  **`fade_in_ms` (Integer, milliseconds)**: The duration of the fade-in phase.

4.  **`hold_ms` (Integer, milliseconds)**: The duration to hold the LED at the target brightness.

5.  **`fade_out_ms` (Integer, milliseconds)**: The duration of the fade-out phase.

### Command Examples

- **Fade in a light to full brightness and hold it:**
  `1FFF,255,1000,5000,500`
  (Broadcast to all devices, fade in over 1s, hold for 5s, fade out over 0.5s)

- **Instantly turn on a light for device with ID `0x00A`:**
  `A,200,0,3000,0`
  (Target device `0x0A`, set brightness to 200 instantly, hold for 3s, then turn off instantly)

- **Set a new Device ID:**
  To change a device's ID, you need to know its current ID. For a device with ID `0x00A`, to change it to `0x00B`:
  `200A,11,0,0,165`
  - Opcode `1` is `0b001`, so the mask starts with `2` (e.g., `0x2000`).
  - Target ID is `0x00A`. So `id_mask` is `0x200A`.
  - The new ID `0x00B` is split into two bytes: `brightness` = `0x0B` (11) and `fade_in` = `0x00`.
  - A security key `165` (`0xA5`) must be placed in the `fade_out_ms` field.

## Contributing

Contributions are welcome! If you have ideas for improvements or find any bugs, please open an issue or submit a pull request.

## License

This project is licensed under the TWPC License. You can view the license file for more details.