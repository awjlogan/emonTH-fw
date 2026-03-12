#pragma once

#include <stdint.h>

#include "temperature.h"

typedef struct DS18B20_Slot_ {
  bool     active;
  uint64_t address;
} DS18B20_Slot_t;

typedef struct DS18B20_Res_ {
  TempStatus_t status;
  int16_t      temp;
} DS18B20_Res_t;

/*! @brief Configure the OneWire port and initialise device discovery.
 *  @param [in] pSlot : pointer to device slot array (size TEMP_MAX_ONEWIRE)
 *  @return number of sensors found
 */
size_t ds18b20InitSensors(DS18B20_Slot_t *pSlot);

/*! @brief Power off OneWire interface */
void ds18b20PowerOff(void);

/*! @brief Power on OneWire interface */
void ds18b20PowerOn(void);

/*! @brief Start a temperature conversion on all OneWire devices.
 *  @return status of the start command
 */
TempStatus_t ds18b20StartSample(void);

/*! @brief Read temperature data from a OneWire device.
 *         Returned temperature is fixed-point in 1/16 °C.
 *  @param [in] dev : index of OneWire device
 *  @return status and temperature data
 */
DS18B20_Res_t ds18b20ReadSample(const unsigned int dev);
