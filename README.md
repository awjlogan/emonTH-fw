# _emonTH3_ Firmware

This describes the firmware provided for the [_emonTH3_](https://github.com/awjlogan/emonTH3) temperature, humidity, and pulse counting system.

This firmware is intended to be used with the [OpenEnergyMonitor](https://openenergymonitor.org) platform. Hardware systems are available directly from them.

## Getting in contact

### Problems

Issues can be reported:

- As a [GitHub issue](https://github.com/awjlogan/emonTH3-fw/issues)
- On the [OpenEnergyMonitor forums](https://community.openenergymonitor.org/)

Please include as much information as possible, including at least:

- The emonTH3 hardware that you using and the emonTH3 firmware version (run the 'v' command on the serial link)
- All settings (run the `l` command on the serial link)
- A full description, including a reproduction if possible, of the issue

### Contributing

Contributions are welcome! Small PRs can be accepted at any time. Please get in touch before making _large_ changes to see if it's going to fit before spending too much time on things.

> [!TIP]
> A [clang-format](https://clang.llvm.org/docs/ClangFormat.html) autoformat pattern is included in the repository. Run the `install-hooks.sh` script to install the pre-commit hook to the autoformatter. You may need to install `clang-format` using your OS's package manager.

> [!NOTE]
> Please bear in mind that this is an open source project and PRs and enhancements may not be addressed quickly, or at all. This is no comment on the quality of the contribution, and please feel free to fork as you like!

## Functional Description

### Version information

The firmware version numbering follows [semantic versioning](https://semver.org/). That is, for version `X.Y.Z`:

- `X` : major version with no guaranteed backward compatibility with previous major versions
- `Y` : minor version where any added functionality has backward compatibility
- `Z` : improvements and bug fixes

Any firmware with `X == 0` is considered unstable and subject to change without notice.

> [!NOTE]
> Build information, including compiler version and commit, is generated during the build process and included in the binary.

### Hardware serial connection

A UART is provided for configuration and, optionally, data transmission. It has the following configuration:

- 115200 baud
- 8N1

### Run time configuration

When the emonTH3 is powered on or reset, the **STATUS** indicator will blink for 5 seconds. If any character is received over the UART connection in this time, the emonTH3 will enter configuration mode. It is not possible to configure the emonTH3 outside this period.

> [!NOTE]
> All options can be listed by entering `?`.

The following options are available:

> [!WARNING]
> The RFM69 transmitter will be damaged if it is run at maximum power without an antenna.

## Compiling and uploading

### Compiling

Compiling the firmware requires the the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (may be available as a package in your distribution). The Makefile is for a Cortex-23 based microcontroller, specifically the Microchip ATSAML10E15 ([datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/SAM-L10-L11-Family-Data-Sheet-DS60001513.pdf), [errata](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/Errata/SAM-L10-L11-Family-Silicon-Errata-and-Data-Sheet-Clarification-DS80000795.pdf)).

> [!NOTE]
> To find which version, if any, of the toolchain is on your path, enter `arm-none-eabi-gcc --version`. You can set the path to a compiler off your path by setting the `TC_PATH` variable in `Makefile`.

To build the firmware:

  `> make -j`

In `bin/`, the following binary files will be generated:

- `emonTH-vX.Y.Z-(commit[-dirty]).bin`
- `emonTH-vX.Y.Z-(commit[-dirty]).elf`
- `emonTH-vX.Y.Z-(commit[-dirty]).hex`
- `emonTH-vX.Y.Z-(commit[-dirty]).uf2`

The `-dirty` tag (if present) indicates that there are uncommitted changes when the binaries are built.

### Uploading

The emonTH3 is supplied with a serial bootloader installed. To enter the bootloader, press the **BOOT** button while powering on the emonTH3. The LED will blink to indicate it has entered the bootloader.

## Modifications

### Helper scripts

> [!NOTE]
> A Python virtual environment shoulde be setup by running `python3 -m venv venv && source venv/bin/activate && pip3 install -r requirements.txt` in `./scripts/`.

- `a2l.sh`: converts a hex address to a file line. Usage: `a2l.sh <address>`
- `elf-size.sh`: this script decomposes the built `.elf` file into functions with their sizes.

### Compile Time Configuration

There are no compile time configuration options.

### Assertions

Assertions are [implemented](https://interrupt.memfault.com/blog/asserts-in-embedded-systems) by the **EMONTH_ASSERT(_condition_)** macro. The microcontroller will enter a breakpoint when an assertion fails and the PC is stored in the `g_assert_info` variable. The PC is used to find the file and line where the assertion failed using `arm-none-eabi-addr2line`.

## Hardware Description

### Peripherals (SAML10)

The following table lists the peripherals used in the SAML10.

|Peripheral       | Alias           | Description                   | Usage                             |
|-----------------|-----------------|-------------------------------|-----------------------------------|
|ADC              |                 |Analog-to-digital converter    |Acquire battery voltage            |
|DMAC             |                 |DMA Controller                 |UART transmission                  |
|EIC              |                 |External interrupt controller  |External device sense              |
|PORT             |                 |GPIO handling                  |                                   |
|SERCOM2          |SERCOM_UART      |UART                           |Configuration and data UART        |
|SERCOM3          |SERCOM_I2CM      |I2C (internal)                 |I2C for internal peripherals       |
|SERCOM4          |SERCOM_SPI       |SPI                            |Drives RFM module                  |
|TC1              |TIMER_DELAY      |Timer/Counter (16bit)          |Delay counter, 8 us resolution     |
|TC2              |TIMER_PULSE      |Timer/Counter (16bit)          |Low power 1 ms resolution          |

### Designing a new board

The files `/src/board_def.h` and `/src/board_def.c` contain options for configuring the microcontroller for a given board. Pin mappings and peripheral usage will need to be adjusted to your design.

### Porting to different microcontroller

Within the top level loop, there are no direct calls to low level hardware. You must provide functions that handle the hardware specific to the microcontroller you are using.

All peripheral drivers are in header/source pairs named **driver_\<PERIPHERAL\>**. For example, the ADC driver is in **driver_ADC.\***. If you are porting to a new microcontroller, you will need to provide implementations of all the functions exposed in **driver_\<PERIPHERAL\>.h** and any internal functions within **driver_\<PERIPHERAL\>.c**. If your microcontroller does not support a particular function (for example, it doesn't have a DMA), then either no operation or an alternative must be provided.

You will also need to ensure that the vendor's headers are included and visible to the compiler.

## Acknowledgements

### Third party libraries and tools

- [mcu-starter-projects](https://github.com/ataradov/mcu-starter-projects) - good starting point for build chains for microcontrollers.
- [RFM69](https://github.com/LowPowerLab/RFM69) - RFM69 driver from Low Power Labs used as reference.
- [Using Asserts in Embedded Systems](https://interrupt.memfault.com/blog/asserts-in-embedded-systems) - custom assertions from _Interrupt by Memfault_.
- [Wintertools](https://github.com/https://github.com/wntrblm/wintertools) - various build and linker scripts from Winterbloom.

### Others

- Glyn Hudson @ [OpenEnergyMonitor](https://openenergymonitor.org/)
- Trystan Lea @ [OpenEnergyMonitor](https://openenergymonitor.org/)
