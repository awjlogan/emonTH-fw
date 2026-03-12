#pragma once

#include <stdbool.h>
#include <stdint.h>

/*! @brief Get the last ADC value.
 *  @return last ADC result (raw ADC counts)
 */
uint16_t adcGetResult(void);

/*! @brief Get the status of the ADC conversion
 *  @return true if the sample is ready, false otherwise
 */
bool adcSampleReady(void);

/*! @brief Start the ADC conversion for battery sensing.
 *         Clears the ready flag; use adcSampleReady() to poll completion.
 */
void adcSampleTrigger(void);

/*! @brief Configure the ADC for the board. */
void adcSetup(void);
