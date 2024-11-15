#pragma once

#include <stdint.h>

/*! @brief  Blocking delay. Use with caution. Returns -1 if the timer is
 *          already in use.
 *  @param [in] delay : period in ms
 */
int timerDelay_ms(uint16_t delay);

/*! @brief  Blocking delay. Use with caution. Returns -1 if the timer is
 *          already in use.
 *  @param [in] delay : period in us
 */
int timerDelay_us(uint32_t delay);

/*! @brief Returns the current microsecond count value
 */
uint32_t timerMicros(void);

/*! @brief Returns the time delta between microseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return : time delta in microseconds
 */
uint32_t timerMicrosDelta(const uint32_t prevMicros);

/*! @brief Returns the current millisecond count value
 */
uint32_t timerMillis(void);

/*! @brief Returns the time delta between milliseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return : time delta in milliseconds
 */
uint32_t timerMillisDelta(const uint32_t prevMillis);

/*! @brief  Sets up the system timer units */
void timerSetup(void);
