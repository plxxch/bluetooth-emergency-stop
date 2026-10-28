# Bluetooth Wireless Emergency Stop for a Mobile Robotic Platform

Firmware for a battery-powered, Bluetooth Low Energy (BLE5) wireless
emergency-stop prototype for a mobile robotic platform.

The prototype was developed and validated end-to-end: system architecture,
schematic and PCB design, embedded firmware, prototype assembly, and system
testing. The firmware implements:

- automatic connection and reconnection;
- passcode-based device authorization;
- disconnection indication on the robot-side unit;
- fail-safe emergency-state retention through a relay output;
- long-range BLE5 operation, with approximately **18 m** open-space working
  range and **72 h** continuous operation measured on the prototype.

## System overview

Two CC2640R2F boards form the link:

| Unit | Role |
|---|---|
| Peripheral (`ble5_simple_peripheral_*`) | Emergency button held by the operator. Advertises with BLE5 long-range (coded PHY) advertising. |
| Central (`ble5_simple_central_*`) | Robot-side receiver. Scans the coded PHY, pairs with the button, and drives the emergency-stop relay. |

The central unit drives the relay on **DIO0** (`GUA_LED_NO_4`). The relay/GPIO
state is held by the firmware until a new command is received, providing the
fail-safe emergency-state retention; the LEDs indicate connection and emergency
status.

## Hardware pin map

The pin map is inherited from the prototype board and is unchanged:

| Signal | Pin |
|---|---|
| LED1 | DIO7 |
| LED2 | DIO9 |
| LED3 | DIO11 |
| LED4 / RELAY | DIO0 |
| LED5 | DIO20 |
| Key UP | DIO21 |
| Key DOWN | DIO16 |
| Key LEFT | DIO17 |
| Key RIGHT | DIO18 |
| Key SELECT | DIO30 |

LEDs and the relay are active-low. Schematic and PCB design files are not part
of this code snapshot; add them separately if needed.

## Repository layout

```
.
├── ble5_simple_central_cc2640r2lp_app/           # robot-side application
├── ble5_simple_central_cc2640r2lp_stack_library/ # BLE5 stack library project
├── ble5_simple_peripheral_cc2640r2lp_app/        # emergency-button application
├── ble5_simple_peripheral_cc2640r2lp_stack_library/
├── THIRD_PARTY_NOTICES.md
└── LICENSE
```

## Building

1. Install **Code Composer Studio 8.3 or newer** and the
   **SimpleLink CC2640R2 SDK 3.30.00.20** (the version used by the original
   project).
2. Import all four projects with `File → Import → CCS Projects`.
3. Set the SimpleLink CC2640R2 SDK path in *Project Properties → Products* if
   CCS does not resolve it automatically.
4. Build the stack library projects first, using the `FlashROM_Library`
   configuration:
   - `ble5_simple_central_cc2640r2lp_stack_library`
   - `ble5_simple_peripheral_cc2640r2lp_stack_library`
5. Build the application projects using `FlashROM_StackLibrary`:
   - `ble5_simple_central_cc2640r2lp_app`
   - `ble5_simple_peripheral_cc2640r2lp_app`
6. Flash the peripheral (emergency button) and central (robot side) boards.

The SDK is **not** included in this repository; it is provided by Texas
Instruments. The `TOOLS/` folders contain the project build support files from
the SDK example.

## Pairing and passcode

The peripheral requests a passcode during pairing and bonding is enabled, so
the central reconnects automatically after a link loss. The default passcode is
defined by `B_APP_DEFAULT_PASSCODE` in the SDK/project configuration; **change
it before deploying the prototype**.

The connection state is indicated by the LEDs. On `GAP_LINK_TERMINATED_EVENT`
the central switches LED2 off; when the link is re-established the LED is
restored.

## Long-range operation

Both applications are configured for BLE5 coded PHY:

- the peripheral advertises with `GAPADV_PARAMS_AE_LONG_RANGE_CONN`;
- the central scans and initiates with `SCAN_PRIM_PHY_CODED` /
  `INIT_PHY_CODED`.

This is the configuration used to obtain the approximately 18 m open-space
range reported for the prototype.

## Implementation notes

- `GUA_Key.c/.h` and `GUA_Led.c/.h` are project-local, MIT-licensed
  reimplementations of the small key-scan and LED helpers used by the original
  prototype. The pin map, API and behaviour are unchanged.
- The peripheral includes the ADXL362 driver (`ADXL362.h`) used by the robotic
  platform integration; the driver remains under the Analog Devices license.
- The files `simple_central.c` and `simple_peripheral.c` are derived from the
  TI SimpleLink SDK examples and retain TI's BSD 3-Clause terms; the
  emergency-stop modifications are by Pengyu Li.

## License

The MIT license in [LICENSE](LICENSE) covers the original code and
modifications authored by Pengyu Li. Texas Instruments SDK files remain under
their BSD 3-Clause license, and the ADXL362 driver remains under the Analog
Devices license. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
