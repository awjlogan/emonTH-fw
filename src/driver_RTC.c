#include "driver_RTC.h"
#include "emonTH_samd.h"

void rtcIntOvfClr(void) { RTC->MODE1.INTFLAG.reg = RTC_MODE1_INTFLAG_OVF; }

bool rtcIntOvfStat(void) {
  return RTC->MODE1.INTFLAG.reg & RTC_MODE1_INTFLAG_OVF;
}

void rtcSetup(int wakePeriod_s) {
  /* Configure RTC in MODE1 (16 bit counter) at 32 Hz (OSCULP32K / 1024)
   *
   * Divide generator 2 by 128 initially, scale a futher /8 in RTC prescalar.
   * This limits the clock rate higher in the clock tree for maximum benefit.
   * APB already unmasked.
   * Drive from GCLK1 initially to set up, otherwise the resync period is long
   */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_DIVSEL | GCLK_GENCTRL_GENEN;

  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID_RTC | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;

  RTC->MODE1.CTRL.reg |=
      RTC_MODE1_CTRL_PRESCALER_DIV8 | RTC_MODE1_CTRL_MODE_COUNT16;

  RTC->MODE1.INTENSET.reg = RTC_MODE1_INTENSET_OVF;

  RTC->MODE1.PER.reg = wakePeriod_s * 32;
  while (RTC->MODE1.STATUS.reg & RTC_STATUS_SYNCBUSY)
    ;
  RTC->MODE1.CTRL.reg |= RTC_MODE1_CTRL_ENABLE;
  while (RTC->MODE1.STATUS.reg & RTC_STATUS_SYNCBUSY)
    ;

  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID_RTC | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;
}
