#include "driver_RTC.h"
#include "emonTH_samd.h"

void rtcIntOvfClr(void) { RTC->MODE1.INTFLAG.reg = RTC_MODE1_INTFLAG_OVF; }

bool rtcIntOvfStat(void) {
  return RTC->MODE1.INTFLAG.reg & RTC_MODE1_INTFLAG_OVF;
}

void rtcSetup(int wakePeriod_s) {
  /* Configure RTC in MODE1 (16 bit counter) at 32 Hz (OSCULP32K / 1024) */
}
