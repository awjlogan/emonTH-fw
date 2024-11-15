#pragma once

#include <stdint.h>

#include "emonTH_samd.h"

typedef enum Calibration_ {
  CAL_ADC_LINEARITY,
  CAL_ADC_BIAS,
  CAL_OSC32K,
  CAL_USB_TRANSN,
  CAL_USB_TRANSP,
  CAL_USB_TRIM,
  CAL_DFLL48M_COARSE
} Calibration_t;

typedef enum SleepMode_t {
  SLEEP_MODE_IDLE0 = PM_SLEEP_IDLE_CPU, /* CPU clock gated */
  SLEEP_MODE_IDLE1 = PM_SLEEP_IDLE_AHB, /* IDLE0 + AHB gated */
  SLEEP_MODE_IDLE2 = PM_SLEEP_IDLE_APB, /* IDLE1 + APB gated */
  SLEEP_MODE_STANDBY                    /* All off except OSCULP32K and RTC */
} SleepMode_t;

/*! @brief Return the calibration value from the NVM Calibration Row, described
 *         in Table 9-4
 *  @param [in] cal : enumeration of the calibration value required
 *  @return : calibration value
 */
uint32_t samdCalibration(const Calibration_t cal);

/*! @brief Disable all unused peripherals */
void samdGateUnused(void);

/*! @brief Sets the sleep mode. */
void samdSleep(SleepMode_t sm);
