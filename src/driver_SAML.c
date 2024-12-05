#include "driver_SAML.h"
#include "emonTH_saml.h"

static SleepMode_t actives[NUM_PERIPHERALS] = {0};

uint32_t samlCalibration(const Calibration_t cal) {
  uint32_t mask     = 0;
  uint32_t position = 0;
  uint64_t cal_row  = *(const volatile uint64_t *)(0x00806020);

  switch (cal) {
  case CAL_ADC_LINEARITY:
    mask     = 0xFFu;
    position = 27u;
    break;
  case CAL_ADC_BIAS:
    mask     = 0x7u;
    position = 35u;
    break;
  case CAL_OSC32K:
    mask     = 0x7Fu;
    position = 38u;
    break;
  case CAL_USB_TRANSN:
    mask     = 0x1Fu;
    position = 45u;
    break;
  case CAL_USB_TRANSP:
    mask     = 0x1Fu;
    position = 50u;
    break;
  case CAL_USB_TRIM:
    mask     = 0x7u;
    position = 55u;
    break;
  case CAL_DFLL48M_COARSE:
    mask     = 0x3Fu;
    position = 58u;
    break;
  }

  return (uint32_t)(cal_row >> position) & mask;
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
