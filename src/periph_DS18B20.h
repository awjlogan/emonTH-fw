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

/*! @brief Configure the OneWire port and initialise
 *  @return the number of sensors found
 */
int ds18b20InitSensors(DS18B20_Slot_t *pSlot);

/*! @brief Start a temperature conversion on all OneWire devices
 *  @return Status of the start
 */
TempStatus_t ds18b20StartSample(void);

/*! @brief Read the temperature data from a OneWire device
 *  @param [in] dev : index of OneWire device
 *  @return Status and temperature data
 */
DS18B20_Res_t ds18b20ReadSample(const unsigned int dev);
