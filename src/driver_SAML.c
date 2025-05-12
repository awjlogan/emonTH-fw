#include "driver_SAML.h"
#include "emonTH_saml.h"

static SleepMode_t actives[NUM_PERIPHERALS] = {0};

uint32_t samlCalibration(const Calibration_t cal) {
  uint32_t position = 0;
  uint16_t calRow   = *(const volatile uint16_t *)((0x00806020));

  switch (cal) {
  case CAL_ADC_BIASREFBUF:
    position = 0u;
    break;
  case CAL_ADC_BIASCOMP:
    position = 3u;
    break;
  case CAL_DFLLULP_PL0:
    position = 6u;
    break;
  case CAL_DFLLULP_PL1:
    position = 9u;
    break;
  }

  return (uint32_t)(calRow >> position) & 0x7u;
}

SleepMode_t samlGetActivity(void) {
  SleepMode_t sm = SLEEP_MODE_STANDBY;
  for (int i = 0; i < NUM_PERIPHERALS; i++) {
    if (actives[i] < sm) {
      sm = actives[i];
    }
  }
  return sm;
}

void samlSetActivity(const SleepMode_t sm, const PeriphIndex_t periphIdx) {
  SleepMode_t        sm_set     = SLEEP_MODE_OFF;
  static SleepMode_t sm_current = SLEEP_MODE_OFF;

  if (sm == actives[periphIdx]) {
    return;
  }
  actives[periphIdx] = sm;

  if (sm != sm_current) {
    for (int i = 0; i < NUM_PERIPHERALS; i++) {
      if (actives[i] < sm_set) {
        sm_set = actives[i];
      }
    }

    if ((SLEEP_MODE_STANDBY == sm_set) || (SLEEP_MODE_OFF == sm_set)) {
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    } else {
      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    }
    PM->SLEEPCFG.reg = sm_set;
    sm_current       = sm_set;
  }
}

void samlSleepConfigure(void) {
  PM->STDBYCFG.reg = PM_STDBYCFG_BBIASHS | PM_STDBYCFG_VREGSMOD_AUTO |
                     PM_STDBYCFG_DPGPDSW | PM_STDBYCFG_BBIASTR;
}

void samlSleepEnter(void) {
  __DSB();
  __WFI();
}

void samlSleepIdle(void) {
  PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_IDLE;
  while (PM->SLEEPCFG.reg != PM_SLEEPCFG_SLEEPMODE_IDLE)
    ;

  SCB->SCR &=
      ~SCB_SCR_SLEEPDEEP_Msk; /* Revisit : Harmony does not have this? */
}

void samlSleepStandby(void) {
  PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_STANDBY;
  while (PM->SLEEPCFG.reg != PM_SLEEPCFG_SLEEPMODE_STANDBY)
    ;

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; /* Revisit : Harmony does not have this? */
}
