#pragma once

#include <stdint.h>

/*! @brief Get the pulse count value.
 *  @return pulse count
 */
uint32_t pulseGetCount(void);

/*! @brief Initialise the pulse counter.
 *  @param [in] timeMask_ms : time to mask pulse counting (ms)
 *  @param [in] pullCfg : pull configuration (0: none, 1: down, 2: up)
 */
void pulseInit(const uint16_t timeMask_ms, const uint8_t pullCfg);
