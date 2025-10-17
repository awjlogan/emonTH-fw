# emonTH v3 Installation

Data from the emonTH is transmitted by wireless RF (433.92 MHz) to an emonPi / emonBase web-connected base-station for logging to emonCMS for data logging, processing and graphing.

With the 2x AAA batteries installed and the emonTH powered up, **the emonTH is designed to appear automatically in the emonCMS input list on the emonPi/base**. For systems with only one emonTH, no further hardware setup is required.

If more than one emonTH is to be used with the same base-station, each emonTH will need a different RF node ID.

![emonTH V2](img/emonth_green.png)

## Standard setup

### 1. Power Up

- Power emonTH from 2x AAA batteries
- Alternatively 5 V DC can be wired into terminal block if required

### 2. Indicator LED

- LED will flash slowly at first power up for 5 seconds
- If the configuration menu is not entered, the LED will then extinguish to indicate successful sensor detection
- If the LED starts to rapidly flash, there is a fatal error. Information will be available through the UART.
- To preserve battery LED does NOT flash regularly during operation

### 3. emonCMS Setup

- RF transmission from the emonTH will be picked up automatically and data will appear in local emonCMS Inputs page.
- If [Remote logging](../emoncms/intro-remote.md) has been setup, data will also be posted to Emoncms.org.
- See guide [Log Locally](../emoncms/intro-rpi.md) for an overview of logging inputs to feeds. For EmonTH inputs select a feed interval 60s or greater:

```{note}
Important: the emonTH reports data at a ~60 s interval. It's important to log the data to emonCMS with a 60 s interval as shown below.
```

![emonth input process](img/emonth-inputprocess.png)

```{note}
If using more than 4x emonTH units [emonhub.conf node decoders will need to be setup](https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md) and the base node ID will need to be changed in the configuration menu.
```

---

## Changing the emonTH node ID

The on-board DIP switch can be used to select up to four node IDs:

| DIP 1 | DIP 2 | RF node ID   |
|-------|-------|--------------|
| OFF   | OFF   | 27 (default) |
| ON    | OFF   | 28           |
| OFF   | ON    | 29           |
| ON    | ON    | 30           |

The base emonTH node ID can be changed using a USB to UART cable and the Arduino IDE.

## Add: External temperature sensor(s)

The emonTH3 firmware supports up to 4 external OneWire temperature sensors.

- Black: GND
- Red: Power (the external sensors are powered down when not in use)
- White: OneWire data

![Adding external DS18B20 sensor](img/emonth_external_ds18b20.jpg)

## Add: External optical pulse sensor

The emonTH3 has a digital input (with interrupt) that can be used for pulse counting. This can be used for wired pulse counting or with the Optical LED pulse sensor.

To use the Optical LED Pulse sensor the easiest way is to remove the RJ45 connector and then strip back the black sheathing to reveal the red (3.3V power), black (GND) and blue (pulse) wires.

Connect the red wire to the 3.3V terminal, the black wire to the GND terminal and the blue wire to the terminal labelled D3 (top) or IRQ1/D3 Pulse Counting (bottom of the board).

- Red: 3.3V
- Black: GND
- Blue: IRQ1/D3

![Adding optical pulse sensor](img/emonth_optical_pulse.jpg)

![Adding optical pulse sensor (bottom)](img/emonth_optical_pulse_bottom.jpg)

There is an input pull-up inside the pulse (IRQ) input that is enabled in the standard sketch. Therefore, you can connect a volt-free contact or an SO output between screw terminal 4 (IRQ input, SO+) and screw terminal 3 (GND, SO-) without the need for an additional resistor. If you must connect your contacts between VCC (screw terminal 2) and screw terminal 4, then you must add a pull-down resistor of resistance low enough to overcome the internal pull-up resistor, or you can use a higher-value resistor and modify the sketch to disable the internal pull-up.

If you are using a reed switch, you may find that you get more than one count per pulse. Adding a 0.1 µF capacitor across the reed switch has been shown to eliminate this problem.

To record the total accumulated pulse count in emonCMS use the `wh_accumulator` input process which detects resets continuing the total pulse count accumulation from the last value before the reset.
