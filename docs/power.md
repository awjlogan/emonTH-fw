# Power Consumption

The emonTH v3 aimed to reduce the power consumption to allow the use of smaller AAA, or even coil cell, batteries. This was accomplished using a combination of hardware and software changes:

- Use of a modern energy efficient 32 bit microcontroller with flexible clocking
- Gating clock and power to unused peripherals
- Interrupt and event driven software architecture
- Boost converter with bypass mode

The following trace was taken using a [Nordic Semiconductor Power Profiler Kit II](https://www.nordicsemi.com/Products/Development-hardware/Power-Profiler-Kit-2), with the voltage set to 3.0 V.

![emonth2 power consumption](img/emonth2_power.png)

1. Atmega328 wakes up, starts sensors and takes readings: 17.5ms @ 8.4mA

2. Atmega328 goes back to watchdog sleep but leaves SPI initialised waiting for wait for RFM to initialise: 100ms @ 2mA - \*now reduced ([see code](https://github.com/openenergymonitor/emonth2/blob/master/firmware/src/src.ino#L331))

3. RFM69CW transmission: 4ms @ 44mA

4. Wait for RF transmission to finish: 100ms @ 0.06mA

5. ATmega328 wakes up to Print data to serial UART: 5ms @ 7.2mA

Time taken for sample = 17.5 + 100 + 4 + 100 + 5 = **226.5ms**

Average current per sample = (17.5*8.4) + (100*2) + (4*44) + (100*0.06) + (5*7.2) / 225.5 =  565 / 225.5 = 2.5mA

After sensor sample is complete the ATmega328 goes back to full watchdog sleep for consuming 0.06mA. This base quiescent consumption includes the quiescent power consumption of the LTC3525 DC-DC converter. See original emonTH hardware blog post for [emonTH DC-DC converter design](https://blog.openenergymonitor.org/2013/10/emonth-update-hardware/).

Assuming one reading per min (current default) the emonTH is off (sleeping for): number of ms in one min - on time = 60000ms - 225.5ms = 59774.5

***

To summarise:

- On current  = 2.5mA (average)
- On time     = 226ms
- Off current = 0.06mA
- Off time    = 59774.5 ms
