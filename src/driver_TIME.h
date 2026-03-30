#pragma once

#include <stdbool.h>
#include <stdint.h>

/*! @brief Busy-wait delay for very short intervals.
 *         Does not configure hardware timers; CPU remains active.
 *  @param [in] delay : delay in microseconds (coarse, use only for short waits)
 */
void timerDelay_us(const uint16_t delay);

/*! @brief Blocking delay in sleep mode.
 *         Requires timerSetup() to have been called.
 *  @param [in] t_ms : delay in milliseconds
 *  @return true if the delay completed, false otherwise.
 */
bool timerDelaySleep_ms(const uint16_t t_ms);

/*! @brief Non-blocking delay in sleep mode with optional callback.
 *         Requires timerSetup() to have been called. Callback runs in
 *         TIMER_DELAY ISR context and must be ISR-safe.
 *  @param [in] t_ms : delay in milliseconds
 *  @param [in] cb : callback function pointer (NULL permitted)
 *  @return true if successfully started, false otherwise.
 */
bool timerDelaySleepAsync_ms(const uint16_t t_ms, void (*cb)(void));

/*! @brief Blocking delay in sleep mode.
 *         Very short delays may busy-wait instead of sleeping.
 *         Requires timerSetup() to have been called.
 *  @param [in] t_us : delay in microseconds
 *  @return true if the delay completed, false otherwise.
 */
bool timerDelaySleep_us(const uint32_t t_us);

/*! @brief Non-blocking delay in sleep mode with optional callback.
 *         Requires timerSetup() to have been called. Callback runs in
 *         TIMER_DELAY ISR context and must be ISR-safe.
 *  @param [in] t_us : delay in microseconds
 *  @param [in] cb : callback function pointer (NULL permitted)
 *  @return true if successfully started, false otherwise.
 */
bool timerDelaySleepAsync_us(const uint32_t t_us, void (*cb)(void));

/*! @brief Disable the delay timer and clear any pending callbacks. */
void timerFlush(void);

/*! @brief Start the pulse timer for masking. */
void timerPulseStart(void);

/*! @brief Configure system timer units for delay/pulse timing. */
void timerSetup(void);

/*! @brief Set up the timer for LED pulse at startup.
 *         Callback runs in TIMER_LP ISR context and must be ISR-safe.
 *  @param [in] cb : pointer to callback on overflow
 */
void timerSetupLED(void (*cb)(void));

/*! @brief Set up the timer for pulse timing.
 *         Callback runs in TIMER_PULSE ISR context and must be ISR-safe.
 *  @param [in] timeMask_ms : pulse counting masked for this period (ms)
 *  @param [in] cb : pointer to callback function
 */
void timerSetupPulse(const uint16_t timeMask_ms, void (*cb)(void));
