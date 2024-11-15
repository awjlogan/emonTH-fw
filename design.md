# emonTH design notes

## Clocks

- OSCULP32K
  - RTC

- OSC8M
  -

## Flow

1. Initialise
2. STANDBY for 55 s
3. On wake

- Trigger ADC read
- Trigger T/H read (I2C)

4. Read T/H (I2C)
5. Send over SPI
