#pragma once

#include <stdint.h>

#include "board_def.h"
#include "emonTH_saml.h"

typedef enum Calibration_ {
  CAL_ADC_BIASREFBUF,
  CAL_ADC_BIASCOMP,
  CAL_DFLLULP_PL0,
  CAL_DFLLULP_PL1,
} Calibration_t;

typedef enum SleepMode_t {
  SLEEP_MODE_ACTIVE  = 0,                             /* No sleep mode  */
  SLEEP_MODE_IDLE    = PM_SLEEPCFG_SLEEPMODE_IDLE,    /* CPU clock gated */
  SLEEP_MODE_STANDBY = PM_SLEEPCFG_SLEEPMODE_STANDBY, /* Only OSCULP32K */
  SLEEP_MODE_OFF     = PM_SLEEPCFG_SLEEPMODE_OFF
} SleepMode_t;

/*! @brief Return the calibration value from the NVM Calibration Row, described
 *         in Table 9-4
 *  @param [in] cal : enumeration of the calibration value required
 *  @return calibration value
 */
uint32_t samlCalibration(const Calibration_t cal);

/*! @brief Returns the minimum allowed sleep mode
 *  @return the sleep mode required
 */
SleepMode_t samlGetActivity(void);

/*! @brief Set the activity level for each peripheral
 *  @param [in] sm : the minimum level required
 *  @param [in] periphIdx : the peripheral index
 */
void samlSetActivity(const SleepMode_t sm, const PeriphIndex_t periphIdx);

/*! @brief Configure sleep controller */
void samlSleepConfigure();

/*! @brief Enter sleep state with data flush */
void samlSleepEnter(void);

/*! @brief Configure shallow sleep state */
void samlSleepIdle(void);

/*! @brief Configure deep sleep state */
void samlSleepStandby(void);
