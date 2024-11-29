#include "pulse.h"
#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "driver_TIME.h"
#include "emonTH.h"

static volatile bool pulseMasked;
static uint32_t      pulseCount;
static int           mask_ms;

void pulseInit(int timeMask_ms) {
  mask_ms = timeMask_ms;
  eicCallbackSet(EIC_CH_PULSE, &pulseInterruptCB);
  timerPulseSetup(&pulseTimerCB);
}

uint32_t pulseGetCount() { return pulseCount; }

void pulseTimerCB(void) { pulseMasked = false; }

void pulseInterruptCB(void) {
  if (!pulseMasked) {
    pulseCount++;
    pulseMasked = true;
    timerPulseStart(mask_ms);
  }
}
