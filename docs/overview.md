# Overview

The emonTH v3 is an open-source, wireless, battery-powered temperature and humidity monitoring node.

Data from the emonTH is transmitted *via* wireless RF (433.92 MHz) to an emonPi / emonBase web-connected base-station with emonCMS for data logging, processing, and graphing.

The emonTH v3 is a upgrade of the emonTH v2, with the following improvements:

- Increased battery life while using smaller batteries
- On-board antenna
- Reduced BoM cost
- Internal expansion socket
- User configuration moved to runtime

As with all our hardware units, the emonTH v3 is fully open-source.

## emonTH v3 Features

- Wireless temperature and humidity monitoring node
- Communicates with emonPi and emonBase *via* RF (433.92 MHz)
- 6 year battery life (2x AAA not included)
- Support for external OneWire sensors, for example [Analog Devices DS18B20](https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf)
- Support for external pulse sensor
- Internal I2C header for other sensors, for example CO2 sensing

## Components

- Microcontroller: Microchip [ATSAML10E15](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/SAM-L10-L11-Family-Data-Sheet-DS60001513.pdf)
- Internal sensor: [TI HDC2010](https://www.ti.com/lit/ds/symlink/hdc2010.pdf) and I2C expansion header
- External sensors: OneWire and pulse sensing
- Power: 2x AAA from onboard holder, [TI TPS61291 DC/DC boost converter](https://www.ti.com/lit/ds/symlink/tps61291.pdf) with bypass
- RF radio: [RFM69W](https://hoperf.com/modules/rf_transceiver/RFM69W.html) @ 433.92 MHz
