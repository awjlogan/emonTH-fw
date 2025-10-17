# Configuration

- Battery operation is expected, although 5 V d.c. can be supplied via the screw terminal block if desired
- Transmits readings from internal temperature and humidity sensor
- External DS18B20 temperature sensor can be connected if desired
- Settings can be adjusted at power-up using the UART interface

## Setting up

By default, data is transmitted by radio approximately every 55 s, identified as node 27. Changing the internal DIP switches before power-up can change this to node 28, 29 or 30.

Adding one or more external temperature sensor before power-up will allow them to be recognised and the readings automatically included in the sent data.

## Additional set-up options

At power-up, if you have a serial monitor connected to the UART port set to 115200 baud, you will see a “Welcome” message giving the software version number, the radio, pulse counter and temperature settings, and some diagnostic messages. There is a 5 second count down during which time the LED will flash slowly. Press any key while in the serial monitor to enter the configuration menu.

Available commands:

- **?** show the available commands
- **c\<n\>** enable (n = 1) or disable (n = 0) data over UART
- **d\<n\>** set the data acquistion period in seconds
- **e\<n\>** sets the maximum number of external temperature sensors. 0, 1, or 4
- **f** exit configuration and start monitoring
- **j\<n\>** enable (n = 1) or disable (n = 0) JSON format for serial data
- **l** list settings
- **m\<x\> \<y\>** meter pulse counting:
  - x = 0 for OFF, x = 1 for ON,
  - \<y\> = an integer for the pulse minimum period in ms. (y is not needed, or ignored when x = 0)
- **n\<n\>** sets the base node ID. \[1..60\]
- **r** restore default settings
- **s** save settings to NVM
- **t\<x\> \<y\> \<y\> \<y\> \<y\> \<y\> \<y\> \<y\> \<y\>**
  - set an external temperature sensor's position:
  - x = a single numeral: the position of the sensor in the list (1-based)
  - y = 8 hexadecimal bytes representing the sensor's address
- **v** display board and firmware information
- **w\<n\>** enable (n = 1) or disable (n = 0) wireless data transmission
- **x\<n\>** use 433.00 MHz RF compatibility (n = 1), or 433.92 MHz (n = 0)

Normally you should save, **s**, the settings before you exit, **x**, so that they will be retained and persist across reboots.

Sending data over serial will increase power consumption, even when the emonTH is not connected. The default setting is for wireless transmission only..

Turning the RF power up will reduce the battery life. Conversely turning the power down will increase the battery life.

The internal DIP switches add 0, 1, 2 or 3 to the base node ID.

```{note}
The node ID can only be changed at power on.
```

The meter pulse minimum period inhibits the effect of ‘contact bounce’ if the pulses come from a mechanical switch. The period can be lengthened if necessary, or shortened to zero if the pulses come from an electronic switch.

The external temperature sensor is only turned on if one or more is detected at power up.

## Data Output

The output by radio is, in order:

1. Internal temperature
2. External temperature
3. Humidity
4. Battery voltage
5. Pulse Count

The “key:value” pairs serial output for the EmonESP & EmonHubOEMInterfacer sends the following

1. Internal temperature temp
2. External temperature tempex
3. Humidity humidity
4. Battery voltage batt
5. Pulse Count pulse
