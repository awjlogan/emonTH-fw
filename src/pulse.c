#include "pulse.h"
#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "driver_TIME.h"
#include "emonTH.h"

static volatile bool pulseMasked = false;
static uint32_t      pulseCount  = 0;

static void pulseEICCB(void);
static void pulseTimerCB(void);

void pulseInit(const uint16_t timeMask_ms, const uint8_t pullCfg) {
  if (0u == pullCfg) {
    portPinCfg(PIN_PULSE, PORT_PINCFG_PULLEN, PIN_CFG_CLR);
  } else {
    portPinCfg(PIN_PULSE, PORT_PINCFG_PULLEN, PIN_CFG_SET);
    portPinDrv(PIN_PULSE, (1u == pullCfg) ? PIN_DRV_CLR : PIN_DRV_SET);
  }

  eicCallbackSet(EIC_CH_PULSE, &pulseEICCB);
  timerSetupPulse(timeMask_ms, &pulseTimerCB);
}

uint32_t pulseGetCount(void) { return pulseCount; }

static void pulseEICCB(void) {
  if (!pulseMasked) {
    pulseCount++;
    pulseMasked = true;
    timerPulseStart();
  }
}

static void pulseTimerCB(void) { pulseMasked = false; }
