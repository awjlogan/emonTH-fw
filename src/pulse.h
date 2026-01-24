#pragma once

#include <stdint.h>

/*! @brief Initialise the pulse counter
 *  @param [in] timeMask_ms : time to mask pulse counting (ms)
 *  @param [in] pullCfg : pull configuration
 */
void pulseInit(const uint16_t timeMask_ms, const uint8_t pullCfg);

/*! @brief Get the current pulse count value
 *  @return the current pulse count
 */
uint32_t pulseGetCount(void);
