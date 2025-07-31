#include "pulse.h"
#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "driver_TIME.h"
#include "emonTH.h"

static volatile bool pulseMasked = false;
static uint32_t      pulseCount  = 0;

void pulseInit(uint8_t timeMask_ms) {
  eicCallbackSet(EIC_CH_PULSE, &pulseInterruptCB);
  timerSetupPulse(timeMask_ms, &pulseTimerCB);
}

uint32_t pulseGetCount() { return pulseCount; }

void pulseTimerCB(void) { pulseMasked = false; }

void pulseInterruptCB(void) {
  if (!pulseMasked) {
    pulseCount++;
    pulseMasked = true;
    timerStartPulse();
  }
}
