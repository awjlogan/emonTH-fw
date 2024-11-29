#include <stdbool.h>
#include <stdint.h>

#include "driver_TIME.h"
#include "emonTH_assert.h"
#include "periph_DS18B20.h"
#include "temperature.h"

static bool tempSampled = false;
static int  numSensors  = 0;

unsigned int tempSensorsInit(const TEMP_INTF_t intf, const void *pParams) {
  EMONTH_ASSERT(pParams);

  if (TEMP_INTF_ONEWIRE == intf) {
    numSensors = ds18b20InitSensors((DS18B20_conf_t *)pParams);
  }

  return numSensors;
}

TempRead_t tempSampleRead(const TEMP_INTF_t intf, const uint8_t dev) {
  TempRead_t res = {TEMP_FAILED, INT16_MIN};

  if (!tempSampled) {
    res.status = TEMP_NO_SAMPLE;
    return res;
  }
  if (0 == numSensors) {
    res.status = TEMP_NO_SENSORS;
    return res;
  }

  if (TEMP_INTF_ONEWIRE == intf) {
    res.result = ds18b20ReadSample(dev);
    res.status = TEMP_OK;
  }

  return res;
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

  /* Default to failure */
  return TEMP_FAILED;
}
