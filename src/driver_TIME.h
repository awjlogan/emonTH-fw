#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver_SAMD.h"

/*! @brief  Blocking delay. Use with caution.
 *  @param [in] delay : period in ms
 *  @return true if successful, false otherwise.
 */
bool timerDelay_ms(uint8_t delay);

/*! @brief  Blocking delay. Use with caution
 *  @param [in] delay : period in us
 *  @return true if successful, false otherwise.
 */
bool timerDelay_us(uint16_t delay);

/*! @brief Blocking delay in sleep mode
 *  @param [in] t_us : delay in microseconds
 *  @param [in] sm: sleep mode
 *  @param [in] disable : disable at the end if true
 *  @return true if successful, false otherwise.
 */
bool timerDelaySleep_us(const uint16_t t_us, const SleepMode_t sm,
                        const bool disable);

/*! @brief Async delay in sleep mode with optional call back
 *  @param [in] t_us : delay in microseconds
 *  @param [in] sm: sleep mode
 *  @param [in] cb : pointer to call back function
 *  @return true if successful, false otherwise.
 */
bool timerDelaySleepAsync_us(const uint16_t t_us, const SleepMode_t sm,
                             void (*cb)());

bool timerFlush(void);

/*! @brief  Sets up the system timer units */
void timerSetup(void);
