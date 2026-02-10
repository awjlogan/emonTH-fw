#include <stdbool.h>
#include <stdint.h>

#include "driver_PORT.h"
#include "driver_TIME.h"
#include "emonTH.h"
#include "emonTH_assert.h"
#include "periph_DS18B20.h"
#include "temperature.h"
#include "util.h"

static volatile bool tempSampleReadyFlag = false;
static bool          tempSampled         = false;
static size_t        numSensors          = 0;

void printOneWireDetails(DS18B20_Slot_t *pSlot, size_t numOneWire);
void tempPowerOff(void);
void tempPowerOn(void);

void printOneWireDetails(DS18B20_Slot_t *pSlot, size_t numOneWire) {
  uartPuts("  - DS18B20... ");
  if (numOneWire) {
    char s[4] = {0};
    uartPuts("\r\n");
    for (size_t i = 0; i < numOneWire; i++) {
      /*    > 1. xx xx xx xx xx xx xx xx */
      uartPuts("    > ");
      utilUtoa(s, (i + 1u), ITOA_BASE10);
      uartPuts(s);
      uartPuts(". ");
      for (size_t j = 0; j < 8u; j++) {
        uint32_t a = (pSlot[i].address >> (8 * j)) & 0xFF;
        utilUtoa(s, a, ITOA_BASE16);
        uartPuts(s);
        uartPuts((j == 7u) ? "\r\n" : " ");
      }
    }
  } else {
    uartPuts("None\r\n");
  }
}

/*! @brief Remove power from temperature sensors */
void tempPowerOff(void) { portPinDrv(PIN_ONEWIRE_PWR, PIN_DRV_CLR); }

/*! @brief Apply power to temperature sensors */
void tempPowerOn(void) { portPinDrv(PIN_ONEWIRE_PWR, PIN_DRV_SET); }

size_t tempSensorsInit(const TEMP_INTF_t intf, const void *pParams) {
  (void)pParams;

  if (TEMP_INTF_ONEWIRE == intf) {
    size_t         numOneWire          = 0;
    DS18B20_Slot_t oneWireAddresses[4] = {0};

    numOneWire += ds18b20InitSensors(oneWireAddresses);
    numSensors += numOneWire;

    printOneWireDetails(oneWireAddresses, numOneWire);
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
    bool   presence = true;
    size_t i        = 0;
    ds18b20PowerOn();
    while ((i < numSensors) && presence) {
      DS18B20_Res_t dsbResult = ds18b20ReadSample(i);
      if (TEMP_OK == dsbResult.status) {
        pDst[i] = dsbResult.temp;
      } else if (TEMP_OUT_OF_RANGE == dsbResult.status) {
        pDst[i] = 4832; /* 302°C */
      } else {
        pDst[i]  = 4864; /* 304°C */
        presence = false;
      }
      i++;
    }
    ds18b20PowerOff();

    /* No presence pulse detected, scrub and exit */
    if (!presence) {
      for (i = 0; i < TEMP_MAX_ONEWIRE; i++) {
        pDst[i] = INT16_MIN;
      }
      return TEMP_NO_SENSORS;
    }

    /* Fill any unused entries in the buffer */
    for (i = numSensors; i < TEMP_MAX_ONEWIRE; i++) {
      pDst[i] = 4800; /* 300°C */
    }
  }

  return TEMP_OK;
}

TempStatus_t tempSampleStart(const TEMP_INTF_t intf, const size_t dev) {

  if (0 == numSensors) {
    return TEMP_NO_SENSORS;
  }

  if (TEMP_INTF_ONEWIRE == intf) {
    tempSampled = true;
    (void)dev;
    if (TEMP_OK == ds18b20StartSample()) {
      tempSampleReadyFlag = false;
      timerDelaySleepAsync_ms(800, &tempSampleReadySet);
      return TEMP_OK;
    }
  }

  return TEMP_FAILED;
}
