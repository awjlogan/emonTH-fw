
#include "driver_RTC.h"
#include "emonTH.h"

void rtcEnable(int period) {
  RTC->MODE1.PER.reg = (period * 4) - 1;
  while (RTC->MODE1.SYNCBUSY.reg & RTC_MODE1_SYNCBUSY_PER)
    ;
  RTC->MODE1.CTRLA.reg |= RTC_MODE1_CTRLA_ENABLE;
  NVIC_EnableIRQ(RTC_IRQn);
}

void rtcSetup(void) {
  /* RTC clock is driven by ULP 32 kHz oscillator @ 1024 Hz (24.8.5 RTC Clock
   * Selection Control). It is in 16 bit count mode (MODE1), clocked at 4 Hz
   * after the prescalar. The count is cleared on match, which causes the system
   * to wake.  */

  RTC->MODE1.CTRLA.reg =
      RTC_MODE1_CTRLA_PRESCALER_DIV256 | RTC_MODE1_CTRLA_MODE_COUNT16;

  RTC->MODE1.INTENSET.reg |= RTC_MODE1_INTENSET_OVF;
}

void irq_handler_rtc(void) {
  RTC->MODE1.INTFLAG.reg |= RTC_MODE1_INTFLAG_OVF;
  emonTHEventSet(EVT_WAKE_TIMER);
}
