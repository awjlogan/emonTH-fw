#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver_SAML.h"

/*! @brief  Blocking delay, only used for short delays where enabling and
 *          configuring the timer has excessive overhead.
 *  @param [in] delay : period in us
 */
void timerDelay_us(uint16_t delay);

/*! @brief Blocking delay in sleep mode
 *  @param [in] t_ms : delay in milliseconds
 *  @return true if successful, false otherwise.
 */
bool timerDelaySleep_ms(const uint16_t t_ms);

/*! @brief Async delay in sleep mode with optional call back
 *  @param [in] t_ms : delay in milliseconds
 *  @param [in] cb : pointer to call back function
 *  @return true if successful, false otherwise.
 */
bool timerDelaySleepAsync_ms(const uint16_t t_ms, void (*cb)());

/*! @brief Blocking delay in sleep mode
 *  @param [in] t_us : delay in microseconds
 *  @return true if successful, false otherwise.
 */
bool timerDelaySleep_us(const uint32_t t_us);

/*! @brief Async delay in sleep mode with optional call back
 *  @param [in] t_us : delay in microseconds
 *  @param [in] cb : pointer to call back function
 *  @return true if successful, false otherwise.
 */
bool timerDelaySleepAsync_us(const uint32_t t_us, void (*cb)());

/*! @brief Disable the timer */
void timerFlush(void);

/*! @brief Sets up the system timer units */
void timerSetup(void);

/*! @brief Set up the timer for pulse timing, if enabled */
void timerPulseSetup(void (*cb)());

/*! @brief Start the timer for pulse masking
 *  @param [in] tMask_ms : time to mask interrupts in milliseconds.
 */
void timerPulseStart(uint16_t tMask_ms);
