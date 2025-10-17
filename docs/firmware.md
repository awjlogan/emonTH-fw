# Firmware

```{tip}
Most users do not need to compile their own firmware. Pre-built firmware for each release can be downloaded from GitHub.
```

## Available Firmware

### [emonTH3](https://github.com/awjlogan/emonTH-fw)

The emonTH3 only supports the LPL RFM69 wireless format.

## Updating firmware using an emonPi/emonBase

The easiest way of updating the emonTH3 firmware is to connect it to an emonPi or emonBase with a USB to UART cable and then use the firmware upload tool available at `Setup > Admin > Update > Firmware`.

The example images below show the earlier [Wicked Device / OpenEnergyMonitor Programmer](../electricity-monitoring/programmers/wicked-device.md). The programmer is the small board that plugs in to the emonTH3 on the UART header. The [newer programmer](../electricity-monitoring/programmers/ftdi-programmer.md) currently available in the shop needs to be orientated the other way around. **Make sure that GND on the programmer matches up with GND on the emonTH3 board.**

Refresh the update page after connecting the USB cable. You should now see port `ttyUSB0` appear in the ‘Select port` list.

- emonPi to emonTH3 programmer image.
Select port: `ttyUSB0`, Hardware: `emonTH3`.

- emonTH firmware upload image.
Click `Update Firmware` to upload the firmware.

## Upload from the command line

Compiled firmware can be uploaded using a USB to UART programmer.

### Environment setup

The first time you do this, you will need to setup a Python virtual environment.

```{bash}
cd scripts
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Uplaoding firmware

First, if you haven't yet done so, activate your virtual environment.

```{bash}
cd scripts
source venv/bin/activate
```

Then, you can upload the firmware using the following command:

```{bash}
python3 boot.py -v -i /dev/ttyUSB0 -f firmware.bin -o 0x400
```

You will need to change `/dev/ttyUSB0` to match the USB to UART serial device you are using. You will need to change `firmware.bin` to the path of your compiled firmware.

## Development

### How to compile firmware

The firmware for the emonTH v3 is self contained, with no external libraries required and does not require any frameworks like Arduino or Platform.io.

Compiling the firmware requires the the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain). This also may be available as a package in your distribution. The Makefile is for a Cortex-M23 based microcontroller, specifically the Microchip ATSAML10E15 ([datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/SAM-L10-L11-Family-Data-Sheet-DS60001513.pdf), [errata](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/Errata/SAM-L10-L11-Family-Silicon-Errata-and-Data-Sheet-Clarification-DS80000795.pdf)).

Ensure the toolchain is available on the path by running:

```{bash}
arm-none-eabi-gcc --version
```

Clone the `emonTH-fw` repo and compile the firmware:

```{bash}
git clone https://github.com/awjlogan/emonTH-fw
cd emonTH-fw
make -j
```

Images in `.bin`, `.hex`, and `.elf` formats will be in the `bin/` folder. The image names include the version and the git commit hash for traceability.
