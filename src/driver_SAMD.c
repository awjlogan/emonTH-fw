#include "driver_SAMD.h"
#include "emonTH_samd.h"

static SleepMode_t actives[NUM_PERIPHERALS];

uint32_t samdCalibration(const Calibration_t cal) {
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

SleepMode_t samdGetActivity(void) {
  SleepMode_t sm = SLEEP_MODE_STANDBY;
  for (int i = 0; i < NUM_PERIPHERALS; i++) {
    if (actives[i] < sm) {
      sm = actives[i];
    }
  }
  return sm;
}

void samdSetActivity(const SleepMode_t sm, const PeriphIndex_t periphIdx) {
  actives[periphIdx] = sm;
  SleepMode_t sm_set = SLEEP_MODE_STANDBY;
  for (int i = 0; i < NUM_PERIPHERALS; i++) {
    if (actives[i] < sm_set) {
      sm_set = actives[i];
    }
  }

  if (SLEEP_MODE_STANDBY == sm_set) {
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
  } else if (SLEEP_MODE_ACTIVE == sm_set) {
    SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
  } else {
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    PM->SLEEPCFG.reg = sm;
  }
}

void samdSleep(SleepMode_t sm) {
  if (SLEEP_MODE_STANDBY == sm) {
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  } else if ((!(SLEEP_MODE_ACTIVE == sm))) {
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    PM->SLEEPCFG.reg = sm;
  }
}

// IDLE1 = 8
// IDLE2 = 128
// IDLE3 = 2048
// STANDBY = 16384
