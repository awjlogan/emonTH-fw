#pragma once

#include <stdint.h>

#include "temperature.h"

typedef struct DS18B20_Res_ {
  TempStatus_t status;
  int16_t      temp;
} DS18B20_Res_t;

/*! @brief Configure the OneWire port and initialise
 *  @return : the number of sensors found
 */
unsigned int ds18b20InitSensors(void);

/*! @brief Start a temperature conversion on all OneWire devices
 *  @return : Status of the start
 */
TempStatus_t ds18b20StartSample(void);

/*! @brief Read the temperature data from a OneWire device
 *  @param [in] dev : index of OneWire device
 *  @return : Status and temperature data
 */
DS18B20_Res_t ds18b20ReadSample(const unsigned int dev);
