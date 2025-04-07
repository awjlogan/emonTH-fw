#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum TEMP_INTF_ { TEMP_INTF_ONEWIRE, TEMP_INTF_I2C } TEMP_INTF_t;

typedef enum TempStatus_ {
  TEMP_OK,
  TEMP_OVERRUN,
  TEMP_NO_SENSORS,
  TEMP_FAILED,
  TEMP_NO_SAMPLE,
  TEMP_BAD_CRC,
  TEMP_BAD_SENSOR,
  TEMP_OUT_OF_RANGE
} TempStatus_t;

typedef struct TempRead_ {
  TempStatus_t status;
  int16_t      result;
} TempRead_t;

/*! @brief Return the temperature as a float
 *  @param [in] tFixed : fixed point temperature
 *  @return the temperature as a float
 */
float tempAsFloat(const TEMP_INTF_t intf, const int16_t tFixed);

/*! @brief Remove power from temperature sensors */
void tempPowerOff(void);

/*! @brief Apply power to temperature sensors */
void tempPowerOn(void);

/*! @brief Find and initialise sensors
 *  @param [in] intf : interface type
 *  @param [in] pParams : parameters for given interface type
 *  @return number of sensors found
 */
unsigned int tempSensorsInit(const TEMP_INTF_t intf, const void *pParams);

/*! @brief Read temperature samples from all monitors
 *  @param [in] intf : interface type
 *  @param [out] pDst : pointer to array for output
 */
TempStatus_t tempSampleRead(const TEMP_INTF_t intf, int16_t *pDst);

/*! @brief Get the status of the sample ready.
 *  @return true if the sample is ready
 */
bool tempSampleReady(void);

/*! @brief Sets the status of the sample ready */
void tempSampleReadySet(void);

/*! @brief Start a temperature sample
    @param [in] intf : interface type
 *  @param [in] dev : device index
 */
TempStatus_t tempSampleStart(const TEMP_INTF_t intf, const uint32_t dev);
