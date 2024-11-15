#include <stdbool.h>
#include <stdint.h>

#include "driver_TIME.h"
#include "emonTH_assert.h"
#include "periph_DS18B20.h"
#include "temperature.h"

static bool tempSampled      = false;
static int  numSensors       = 0;
static int  millisLastSample = 0;

float tempAsFloat(const TEMP_INTF_t intf, const int16_t tFixed) {
  float ret = -1000.0;
  if (TEMP_INTF_ONEWIRE == intf) {
    ret = ds18b20SampleToCelsius(tFixed);
  }
  return ret;
}

unsigned int tempInitSensors(const TEMP_INTF_t intf, const void *pParams) {
  EMONTH_ASSERT(pParams);

  if (TEMP_INTF_ONEWIRE == intf) {
    numSensors = ds18b20InitSensors((DS18B20_conf_t *)pParams);
  }

  return numSensors;
}

TempRead_t tempReadSample(const TEMP_INTF_t intf, const uint8_t dev) {
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

TempStatus_t tempStartSample(const TEMP_INTF_t intf, const uint32_t dev) {

  if (0 == numSensors) {
    return TEMP_NO_SENSORS;
  }

  if (timerMillisDelta(millisLastSample) < TEMP_CONVERSION_T) {
    return TEMP_OVERRUN;
  }

  if (TEMP_INTF_ONEWIRE == intf) {
    tempSampled = true;
    (void)dev;
    millisLastSample = timerMillis();
    int samp         = ds18b20StartSample();
    if (0 == samp)
      return TEMP_OK;
  }

  /* Default to failure */
  return TEMP_FAILED;
}
