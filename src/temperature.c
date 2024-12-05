#include <stdbool.h>
#include <stdint.h>

#include "driver_PORT.h"
#include "driver_TIME.h"
#include "emonTH_assert.h"
#include "periph_DS18B20.h"
#include "temperature.h"

static volatile bool tempSampleReadyFlag = false;
static bool          tempSampled         = false;
static int           numSensors          = 0;

/*! @brief Remove power from temperature sensors */
void tempPowerOff(void) { portPinDrv(PIN_ONEWIRE_PWR, PIN_DRV_CLR); }

/*! @brief Apply power to temperature sensors */
void tempPowerOn(void) { portPinDrv(PIN_ONEWIRE_PWR, PIN_DRV_SET); }

unsigned int tempSensorsInit(const TEMP_INTF_t intf, const void *pParams) {
  EMONTH_ASSERT(pParams);

  if (TEMP_INTF_ONEWIRE == intf) {
    numSensors = ds18b20InitSensors((DS18B20_conf_t *)pParams);
  }

  return numSensors;
}

bool tempSampleReady(void) { return tempSampleReadyFlag; }

void tempSampleReadySet(void) { tempSampleReadyFlag = true; }

TempStatus_t tempSampleRead(const TEMP_INTF_t intf, int16_t *pDst) {

  tempSampleReadyFlag = false;

  if (!tempSampled) {
    return TEMP_NO_SAMPLE;
  }
  if (0 == numSensors) {
    return TEMP_NO_SENSORS;
  }

  if (TEMP_INTF_ONEWIRE == intf) {
    bool presence = true;
    int  i        = 0;
    while ((i < numSensors) && presence) {
      int16_t dsbResult = ds18b20ReadSample(i);
      if (INT16_MIN == dsbResult) {
        presence = false;
      } else {
        pDst[i] = dsbResult;
      }
      i++;
    }

    /* No presence pulse detected, scrub and exit */
    if (!presence) {
      for (i = 0; i < TEMP_MAX_ONEWIRE; i++) {
        pDst[i] = INT16_MIN;
      }
      return TEMP_NO_SENSORS;
    }

    /* Fill any unused entries in the buffer */
    for (i = numSensors; i < TEMP_MAX_ONEWIRE; i++) {
      pDst[i] = INT16_MIN;
    }
  }

  return TEMP_OK;
}

TempStatus_t tempSampleStart(const TEMP_INTF_t intf, const uint32_t dev) {

  if (0 == numSensors) {
    return TEMP_NO_SENSORS;
  }

  if (TEMP_INTF_ONEWIRE == intf) {
    tempSampled = true;
    (void)dev;
    if (0 == ds18b20StartSample())
      return TEMP_OK;
  }

  return TEMP_FAILED;
}
