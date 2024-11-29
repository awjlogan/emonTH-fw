#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct PulseCfg_ {
  int  timeMask;
  bool active;
} PulseCfg_t;

/*! Initialise the pulse counter */
void pulseInit(int timeMask_ms);

/*! @brief Get the current pulse count value
 *  @return the current pulse count
 */
uint32_t pulseGetCount();

/*! @brief Callback function when the external interrupt fires */
void pulseInterruptCB(void);

/*! @brief Callback function when the masking timer expires */
void pulseTimerCB(void);
