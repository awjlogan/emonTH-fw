# Configuration

- Battery operation is expected, although 5 V d.c. can be supplied via the screw terminal block if desired.
- Transmits readings from internal temperature and humidity sensor, also external DS18B20 temperature sensor can be connected if desired.
- Settings can be adjusted at power-up using the UART connector.

## Setting up

By default, data is transmitted by radio approximately every 55 s, identified as Node 27. Changing the internal DIP switches before power-up can change this to Node 28, 29 or 30.

Adding an external temperature sensor before power-up will allow it to be recognised and the readings automatically included in the sent data.

## Additional set-up options

At power-up, if you have a serial monitor connected to the UART port set to 115200 baud, you will see a “Welcome” message giving the software version number, the radio, pulse counter and temperature settings, and some more diagnostic messages. There is a 5 second count down during which time the LED will flash slowly. Press any key while in the serial monitor to enter the configuration menu.

Entering Settings mode...

Available commands:

- **l** list the settings
- **r** restore sketch defaults
- **s** save settings to EEPROM
- **v** show firmware version
- **x** exit, lock and continue
- **?** show this text again

- **w\<x\>** turn RFM Wireless data off: x = 0 or on: x = 1
- **b\<n\>** set r.f. band n = a single numeral: 4 = 433MHz, 8 = 868MHz, 9 = 915MHz (may require hardware change)
- **p\<nn\>** set the r.f. power. nn - an integer 0 - 31 representing -18 dBm to +13 dBm. Default: 25 (+7 dBm)
- **g\<nnn\>** set Network Group nnn - an integer (OEM default = 210)
- **n\<nn\>** set node ID n= an integer (standard node ids are 1..60)

- **m\<x\> \<yy\>** meter pulse counting:
  - x = 0 for OFF, x = 1 for ON, 
  - \<yy\> = an integer for the pulse minimum period in ms. (y is not needed, or ignored when x = 0)

- **t0 \<y\>** turn external temperature measurement on or off: y = 0 for OFF, y = 1 for ON

- **t\<x\> \<yy\> \<yy\> \<yy\> \<yy\> \<yy\> \<yy\> \<yy\> \<yy\>**
  - change an external temperature sensor's address or position:
  - x = a single numeral: the position of the sensor in the list (1-based)
  - yy = 8 hexadecimal bytes representing the sensor's address
  - e.g. 28 81 43 31 07 00 00 D
  - N.B. Sensors CANNOT be added.

Only the radio, pulse and temperature sensor settings can be changed, the temperature and humidity sensors cannot be calibrated. Normally, you should save ‘ **s’** the settings before you exit ‘ **x’** , so that they will be retained and used forever (until changed again).

If you turn the radio off and serial data on ( **w2** ), only the serial data in a format compatible with the emonHub Serial Interfacer will be sent to the FTDI port. ( **w3** ) will send data both by radio and the serial port, the default is radio only ( **w1** ).

If you turn the radio power up above the default value of 25, ensure you select the frequency band that matches the radio module fitted, else the radio module itself could be destroyed. Turning the power up will significantly reduce the battery life, conversely turning the power down will increase the battery life.

If you change the NodeID, then the internal DIP switches add 0, 1, 2 or 3 to the new NodeID.

The meter pulse minimum period inhibits the effect of ‘contact bounce’ if the pulses come from a mechanical switch. The period can be lengthened if necessary, or shortened to zero if the pulses come from an electronic switch.

The external temperature sensor is only turned on if one or more is detected at the start. A connected sensor can be turned off to save power without needing to disconnect it.

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
