# Firmware

## Available Firmware

### [emonTH3](https://github.com/awjlogan/emonTH-fw)

The emonTH3 only supports the LPL RFM69 wireless format.

## Updating firmware using an emonPi/emonBase

The easiest way of updating the emonTH£ firmware is to connect it to an emonPi or emonBase with a USB to UART cable and then use the firmware upload tool available at `Setup > Admin > Update > Firmware`.

The example images below show the earlier [Wicked Device / OpenEnergyMonitor Programmer](../electricity-monitoring/programmers/wicked-device.md). The programmer is the small board that plugs in to the emonTH3 on the UART header. The [newer programmer](../electricity-monitoring/programmers/ftdi-programmer.md) currently available in the shop needs to be orientated the other way around. **Make sure that GND on the programmer matches up with GND on the emonTH3 board.**

Refresh the update page after connecting the USB cable. You should now see port `ttyUSB0` appear in the ‘Select port` list.

*emonPi to emonTH3 programmer image*

Select port: `ttyUSB0`, Hardware: `emonTH3`, Radio format e.g: `RFM69 LowPowerLabs`:

*emonTh firmware upload image*

Click `Update Firmware` to upload the firmware.

## Upload from the command line

Pre compiled firmware can be uploaded using a USB to UART programmer and avrdude: 

`avrdude -uV -c arduino -p ATMEGA328P -P /dev/ttyUSB0  -b 115200 -U flash:w:firmware.hex`

## How to compile the firmware


PlatformIO works great from the command line. See the excellent [PlatformIO Quick Start Guide](https://docs.platformio.org/en/latest/core/installation/index.html#super-quick-mac-linux) for installation instructions.

**Compile and upload emonTH3 firmware**

    git clone https://github.com/openenergymonitor/emonth2
    cd emonth2/firmware
    pio run -t upload

