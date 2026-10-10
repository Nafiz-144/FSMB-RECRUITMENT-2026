# Smart Voltage and Over-Current Protector (PIC18F4550)

A microcontroller-based protector that disconnects an appliance through a relay when the mains voltage or load current goes out of range. Settings can be changed from the device (buttons + LCD) or from a computer (serial terminal).

> **Status:** the firmware has only been syntax-checked on a PC. It has not been verified on hardware unless the project report says otherwise. Always test at low voltage first.

## Features

- RMS voltage and current measurement (100 samples per 20 ms mains cycle)
- Faults: over-voltage (OV), under-voltage (UV), over-current (OC), peak surge, short circuit
- Normal faults trip after a user-set delay; peak surge and short circuit trip immediately
- Auto-reconnect after a user-set time, with hysteresis to avoid relay chatter
- Lockout after 3 trips within 5 minutes (needs manual reset)
- Relay is OFF at power-up and only turns ON after about 3 s of healthy voltage
- Settings stored in EEPROM with magic byte and checksum (falls back to defaults if corrupted)
- 16x2 LCD + 3 buttons, and a text command interface over UART (9600 baud)
- Watchdog and brown-out reset enabled

## Hardware

| Part | Notes |
|---|---|
| PIC18F4550 | 20 MHz crystal (HS mode) + 2 x 22 pF, 10k pull-up on MCLR, 100 nF bypass caps |
| ZMPT101B module | isolated mains voltage sensing, output centred at about 2.5 V |
| ACS712 (5A / 20A / 30A) | current sensing, output centred at about 2.5 V |
| 5 V relay (10 A or more) | driven by an NPN transistor (BC547 / 2N2222) with a 1N4007 flyback diode |
| 16x2 HD44780 LCD | 4-bit mode, RW tied to GND |
| 3 push buttons | UP, DOWN, OK (to GND) |
| Buzzer, green LED, red LED | with series resistors for the LEDs |
| USB-UART adapter (CH340 / CP2102) | connects the PC to the UART pins |
| Isolated 5 V supply (e.g. HLK-PM01) | keeps the low-voltage side isolated from mains |
| MOV (275 V) and fuse | on the mains input, for spikes and short circuits |

### Pin map

| Pin | Function |
|---|---|
| RA0 (AN0) | Voltage sensor (ZMPT101B) |
| RA1 (AN1) | Current sensor (ACS712) |
| RB0 | Button UP |
| RB1 | Button DOWN |
| RB2 | Button OK |
| RC0 | LCD RS |
| RC1 | LCD EN |
| RD4 to RD7 | LCD D4 to D7 |
| RD0 | Relay driver (HIGH = relay ON) |
| RD1 | Buzzer |
| RD2 | Green LED (relay ON) |
| RD3 | Red LED (fault) |
| RC6 (TX) | to adapter RX |
| RC7 (RX) | to adapter TX |

Connect the adapter GND to the circuit GND. Do not connect the adapter's 5 V pin if the circuit has its own supply.

## Build and flash

1. Install **MPLAB X IDE** and the **XC8 compiler** (v2.x) from Microchip.
2. Create a new *Standalone Project*, device **PIC18F4550**, compiler **XC8**.
3. Add `firmware/main.c` to *Source Files*. The configuration bits are set inside `main.c` with `#pragma config`, so no other setup is needed.
4. Build the project (hammer icon). If the compiler complains about a configuration-bit or register name, check the names in `pic18f4550.h` of your XC8 version.
5. Program the chip with a **PICkit 3/4** (or any supported programmer), using the ICSP pins MCLR, PGC (RB6), PGD (RB7), VDD and GND.

The 20 MHz crystal is required: timing, ADC sampling and the 9600 baud rate all assume `_XTAL_FREQ = 20000000`.

## Using the device

### On the device

| Action | Result |
|---|---|
| Hold **OK** for about 1 s | Open the settings menu |
| **UP / DOWN** | Change the value (hold to repeat) |
| **OK** (short press) | Go to the next setting; after the last one the settings are validated and saved |
| Hold **OK** again, or wait 10 s | Cancel the menu without saving |
| **OK** (short press) while LOCKOUT is shown | Clear the lockout |

The LCD shows voltage and current on line 1, and the state (RELAY ON, trip reason with a countdown, or LOCKOUT) on line 2.

### From a computer

Open a serial terminal (PuTTY, Arduino Serial Monitor, `screen`, etc.) on the adapter's COM port at **9600 baud, 8N1**, and send a line ending with Enter.

| Command | Action | Example reply |
|---|---|---|
| `STATUS` | Current readings and state | `V=224.5 I=1.20 RELAY=ON FAULT=NONE STATE=NORMAL` |
| `GET` | All settings | `OV=240 UV=180 OC=5.0 DELAY=200 RECON=30 VCAL=100 ICAL=4888 SURGE=380` |
| `SET <name> <value>` | Change a setting (not saved yet) | `OK`, `ERR RANGE` or `ERR ARG` |
| `SAVE` | Write settings to EEPROM | `OK SAVED` |
| `RESET` | Clear a trip or lockout (reconnects after about 3 s if healthy) | `OK` |
| `LOG` | Last fault | `LAST=OV@252.3V` |
| `DEFAULTS` | Restore factory defaults (use `SAVE` to keep them) | `OK (use SAVE to keep)` |
| `HELP` | Command list | |

The device also sends events by itself, for example `EVENT TRIP OV 252.3V`.

### Settings

| Name | Meaning | Range | Default |
|---|---|---|---|
| `OV` | Over-voltage limit (V rms) | 220 to 280 | 240 |
| `UV` | Under-voltage limit (V rms) | 120 to 210 | 180 |
| `OC` | Over-current limit (A rms) | 0.5 to 20.0 | 5.0 |
| `DELAY` | Delay before a normal trip (ms) | 20 to 3000 | 200 |
| `RECON` | Reconnect time after a trip (s) | 5 to 600 | 30 |
| `VCAL` | Voltage calibration (0.01 V per ADC count) | 20 to 300 | 100 |
| `ICAL` | Current calibration (see below) | 1000 to 20000 | 4888 |
| `SURGE` | Peak voltage for instant trip (V) | 300 to 500 | 380 |

`OV` must be at least `UV + 20`. The short-circuit limit is fixed at 3 times the peak equivalent of `OC`.

## Calibration (required)

The default `VCAL` and `ICAL` are only estimates, so the readings will be wrong until you calibrate.

1. **Sensor setup:** turn the ZMPT101B trimmer so the signal at 230 V does not clip (peak well below 5 V, centred at 2.5 V).
2. **Voltage:** compare the displayed voltage with a multimeter. New `VCAL` = old `VCAL` x (multimeter value / displayed value). Send `SET VCAL <new>` and then `SAVE`.
3. **Current:** with a known load, compare in the same way and adjust `ICAL`. Starting values: ACS712-20A is 4888, ACS712-5A is about 2642, ACS712-30A is about 7406.
4. Repeat until the readings agree. Accuracy of about +/-2 to 3 % is typical for these modules.

## Testing safely

Use a low-voltage AC source (for example a 12 V transformer) first. To simulate faults without touching mains, change the limits instead: lower `OV` (and `UV` below `OV - 20`) or lower `OC` below the load current, then watch the relay trip, the buzzer sound, and the relay reconnect after `RECON` seconds.

## Warnings

- Mains voltage can kill. Keep the high-voltage and low-voltage parts isolated, use a closed insulated enclosure, and always use a fuse.
- Choose a relay rated at least twice the load current.
- A relay opens in several milliseconds, so it cannot stop microsecond spikes. That is the job of the MOV. The relay protects against sustained over-voltage and over-current.

## Known limitations

- Not verified on hardware (see the project report for what was actually tested).
- Native USB is not used; communication is through the hardware UART and a USB-UART adapter.
- `SAVE` and fault logging pause sampling for a few milliseconds while the EEPROM is written.
- Surge detection only sees what the sensor can measure (about +/-2.5 V around the centre point).
- If the UART transmit buffer is full, extra characters are dropped so that protection is never stalled.

## Repository layout

Adjust this to the structure required by the assignment.

```
repo/
├── firmware/main.c
├── docs/report.pdf
└── README.md
```
