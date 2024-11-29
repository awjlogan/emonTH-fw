#pragma once

#include <stdint.h>

#include "board_def.h"
#include "emonTH_saml.h"

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
  SLEEP_MODE_IDLE1   = PM_SLEEPCFG_SLEEPMODE_IDLE,    /* CPU clock gated */
  SLEEP_MODE_IDLE2   = PM_SLEEPCFG_SLEEPMODE_STANDBY, /* IDLE0 + AHB gated */
  SLEEP_MODE_IDLE3   = PM_SLEEPCFG_SLEEPMODE_OFF,     /* IDLE1 + APB gated */
  SLEEP_MODE_STANDBY = 3, /* All off except OSCULP32K and RTC */
  SLEEP_MODE_ACTIVE  = -1 /* No sleep mode selected */
} SleepMode_t;

/*! @brief Return the calibration value from the NVM Calibration Row, described
 *         in Table 9-4
 *  @param [in] cal : enumeration of the calibration value required
 *  @return : calibration value
 */
uint32_t samlCalibration(const Calibration_t cal);

/*! @brief Returns the minimum allowed sleep mode
 *  @return : the sleep mode required
 */
SleepMode_t samlGetActivity(void);

/*! @brief Set the activity level for each peripheral
 *  @param [in] sm : the minimum level required
 *  @param [in] periphIdx : the peripheral index
 */
void samlSetActivity(const SleepMode_t sm, const PeriphIndex_t periphIdx);

/*! @brief Sets the sleep mode. */
void samlSleep(SleepMode_t sm);
