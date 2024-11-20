#pragma once

#include <stdint.h>

/*! @brief Get the last ADC value */
int16_t adcGetResult(void);

/*! @brief Configure the ADC for the board */
void adcSetup(void);

/*! @brief Start the ADC conversion for battery sensing */
void adcTriggerSample(void);
