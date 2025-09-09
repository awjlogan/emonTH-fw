#include "driver_SAML.h"
#include "emonTH_saml.h"

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

void samlSleepConfigure(void) {
  PM->STDBYCFG.reg = PM_STDBYCFG_BBIASHS | PM_STDBYCFG_VREGSMOD_LP |
                     PM_STDBYCFG_DPGPDSW | PM_STDBYCFG_BBIASTR;

  SUPC->VREG.reg = SUPC_VREG_LPEFF;
}

void samlSleepEnter(void) {
  __DSB();
  __WFI();
}

void samlSleepIdle(void) {
  PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_IDLE;
  while (PM->SLEEPCFG.reg != PM_SLEEPCFG_SLEEPMODE_IDLE)
    ;
}

void samlSleepOff(void) {
  PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_OFF;
  while (PM->SLEEPCFG.reg != PM_SLEEPCFG_SLEEPMODE_OFF)
    ;
}

void samlSleepStandby(void) {
  PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_STANDBY;
  while (PM->SLEEPCFG.reg != PM_SLEEPCFG_SLEEPMODE_STANDBY)
    ;
}
