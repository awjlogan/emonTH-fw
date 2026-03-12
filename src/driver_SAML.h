#pragma once

#include <stdint.h>

typedef enum Calibration_ {
  CAL_ADC_BIASREFBUF,
  CAL_ADC_BIASCOMP,
  CAL_DFLLULP_PL0,
  CAL_DFLLULP_PL1,
} Calibration_t;

/*! @brief Return the calibration value from the NVM Calibration Row, described
 *         in Table 9-4
 *  @param [in] cal : enumeration of the calibration value required
 *  @return calibration value
 */
uint32_t samlCalibration(const Calibration_t cal);

/*! @brief Configure sleep controller (generic). */
void samlSleepConfigure();

/*! @brief Enter sleep state with data flush (blocking). */
void samlSleepEnter(void);

/*! @brief Configure shallow sleep state. */
void samlSleepIdle(void);

/*! @brief Configure deep sleep state. */
void samlSleepStandby(void);
