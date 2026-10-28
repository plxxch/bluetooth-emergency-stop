# Third-party notices

The MIT license in [LICENSE](LICENSE) applies to the original code and
modifications authored by Pengyu Li. Third-party files remain under their own
licenses; their copyright notices are retained where present in the file
headers.

## Texas Instruments SimpleLink CC2640R2 SDK

This repository is derived from the **SimpleLink CC2640R2 SDK 3.30.00.20**
BLE5 `simple_central` and `simple_peripheral` examples. The following files are
TI SDK code (some with application modifications) and remain under TI's
BSD 3-Clause license:

- `simple_central.c`, `simple_peripheral.c` and their headers/menus
- `util.c/.h`, `board_key.c/.h`, `two_btn_menu.c/.h`
- `PROFILES/*`, `Startup/*`, `Include/gapbondmgr.h`
- `TOOLS/*` and the CCS project metadata (`.cproject`, `.project`,
  `.ccsproject`)

The BSD 3-Clause terms permit redistribution provided the copyright notice,
conditions and disclaimer are retained, and TI's name is not used to endorse
derived products without written permission. The software is provided “AS IS”,
without warranty.

The original TI header is present in `simple_central.c` and the other SDK
files. In the working copy of `simple_peripheral.c` the original header was
absent; an attribution header has been added at the top of that file.

The SDK itself is **not** included in this repository. Install
SimpleLink CC2640R2 SDK 3.30.00.20 through Code Composer Studio. The
prebuilt `TOOLS/src/sysbios/rom_sysbios.aem3` library is also not included; the
SDK build support regenerates it.

## Analog Devices ADXL362

`ble5_simple_peripheral_cc2640r2lp_app/Application/ADXL362.h` is an Analog
Devices driver header. It is distributed under the ADI license reproduced in
the file header, which permits redistribution with the copyright notice and
conditions retained and requires use with, or direct connection to, an Analog
Devices component. The prototype uses the ADXL362, so this condition is met.

## Key and LED helpers

`GUA_Key.c/.h` and `GUA_Led.c/.h` are project-local MIT-licensed
reimplementations of the small key-scan and LED helpers used during prototype
development. The original third-party tutorial library is **not** included;
the pin map, API and behaviour are preserved.

## Trademarks

MSP430, SimpleLink, Code Composer Studio and CC2640R2 are trademarks of Texas
Instruments Incorporated. ADXL362 is a trademark of Analog Devices, Inc.
Bluetooth is a trademark of the Bluetooth SIG. All other trademarks are the
property of their respective owners.
