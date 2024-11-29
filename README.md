# _emonTH3_ Firmware

This describes the firmware provided for the [_emonTH3_](https://github.com/awjlogan/emonTH3) temperature, humidity, and pulse counting system.

This firmware is intended to be used with the [OpenEnergyMonitor](https://openenergymonitor.org) platform. Hardware systems are available directly from them.

## Getting in contact

### Problems

Issues can be reported:

- As a [GitHub issue](https://github.com/awjlogan/emonTH3-fw/issues)
- On the [OpenEnergyMonitor forums](https://community.openenergymonitor.org/)

Please include as much information as possible (run the `v` command on the serial link), including at least:

- The emonTH3 hardware that you using and the emonTH3-fw version (run the 'v' command on the serial link)
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

A dedicated UART is used for debug, configuration, and data transmission. It has the following UART configuration:

- 115200 baud
- 8N1

### Run time configuration

REVISIT Configuration options

> [!NOTE]
> All options can be listed by entering `?`.

> [!WARNING]
> The RFM69 transmitter will be damaged if it is run at maximum power without an antenna.

## Compiling and uploading

### Compiling

REVISIT Update description for SAML

Compiling the firmware requires the the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (may be available as a package in your distribution). The Makefile is for a Cortex-M0+ based microcontrollers, specifically the Microchip ATSAMD21J17 ([datasheet](https://www.microchip.com/en-us/product/ATSAMD21J17), [errata](https://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-%20Family-Silicon-%20Errata-and-DataSheet-Clarification-DS80000760C.pdf)).

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

REVISIT Instructions to flash through SWD

## Modifications

### Helper scripts

> [!NOTE]
> A Python virtual environment shoulde be setup by running `python3 -m venv venv && source venv/bin/activate && pip3 install -r requirements.txt` in `./scripts/`.

- `a2l.sh`: converts a hex address to a file line. Usage: `a2l.sh <address>`
- `elf-size.sh`: this script decomposes the built `.elf` file into functions with their sizes.

### Floating point support

The Cortex-M23 does not have a hardware floating point unit, so all floating point operations are done in software. The gcc built in floating point functions are quite large and slow, and have been replaced with the [Qfplib](https://www.quinapalus.com/qfplib.html) library. All floating point operations, including type conversions, should use these functions.

### Compile Time Configuration

REVISIT Anything to do at compile time?

### Assertions

Assertions are [implemented](https://interrupt.memfault.com/blog/asserts-in-embedded-systems) by the **EMONTH_ASSERT(_condition_)** macro. The microcontroller will enter a breakpoint when an assertion fails and the PC is stored in the `g_assert_info` variable. The PC is used to find the file and line where the assertion failed using `arm-none-eabi-addr2line`.

## Hardware Description

## Acknowledgements

### Third party libraries and tools

- [mcu-starter-projects](https://github.com/ataradov/mcu-starter-projects) - good starting point for build chains for microcontrollers.
- [Qfplib](https://www.quinapalus.com/qfplib.html) - soft floating point library for Arm Cortex-M0.
- [RFM69](https://github.com/LowPowerLab/RFM69) - RFM69 driver from Low Power Labs used as reference.
- [Using Asserts in Embedded Systems](https://interrupt.memfault.com/blog/asserts-in-embedded-systems) - custom assertions from _Interrupt by Memfault_.
- [Wintertools](https://github.com/https://github.com/wntrblm/wintertools) - various build and linker scripts from Winterbloom.

### Others

- Glyn Hudson @ [OpenEnergyMonitor](https://openenergymonitor.org/)
- Trystan Lea @ [OpenEnergyMonitor](https://openenergymonitor.org/)
