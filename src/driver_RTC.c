
#include "driver_RTC.h"
#include "emonTH_assert.h"

#define EVT_MAX 4

static int intervalToRatio(const int smpIntTime);

static int       evtDivCnt[EVT_MAX] = {0};
static int       evtIdx             = 0;
static RTC_Evt_t rtcEvt[EVT_MAX]    = {0};

static int rtcPeriod = 0;

static int intervalToRatio(const int smpIntTime) {
  int rem = smpIntTime % rtcPeriod;
  int div = smpIntTime / rtcPeriod;
  if (rem > (rtcPeriod / 2)) {
    div++;
  }
  return div;
}

void rtcEnable(int period) {
  rtcPeriod = period;

  /* Convert sample intervals in seconds to ratio of report times */
  for (int i = 0; i < evtIdx; i++) {
    rtcEvt[i].smpInterval = intervalToRatio(rtcEvt[i].smpInterval);
  }

  RTC->MODE1.PER.reg = (period * 4) - 1;
  while (RTC->MODE1.SYNCBUSY.reg & RTC_MODE1_SYNCBUSY_PER)
    ;
  RTC->MODE1.CTRLA.reg |= RTC_MODE1_CTRLA_ENABLE;
  NVIC_EnableIRQ(RTC_IRQn);
}

void rtcEvtReg(const RTC_Evt_t rtcevt) {
  EMONTH_ASSERT(evtIdx < EVT_MAX);
  rtcEvt[evtIdx].smpInterval = rtcevt.smpInterval;
  rtcEvt[evtIdx].evt         = rtcevt.evt;
  evtIdx++;
}

void rtcSetup(void) {
  /* RTC clock is driven by ULP 32 kHz oscillator @ 1024 Hz (24.8.5 RTC Clock
   * Selection Control). It is in 16 bit count mode (MODE1), clocked at 4 Hz
   * after the prescalar. The count is cleared on match, which causes the system
   * to wake. Software reset on setup. */
  RTC->MODE1.CTRLA.reg = RTC_MODE1_CTRLA_SWRST;
  while (RTC->MODE1.SYNCBUSY.reg & RTC_MODE1_SYNCBUSY_SWRST)
    ;
  RTC->MODE1.CTRLA.reg =
      RTC_MODE1_CTRLA_PRESCALER_DIV256 | RTC_MODE1_CTRLA_MODE_COUNT16;

  RTC->MODE1.INTENSET.reg = RTC_MODE1_INTENSET_OVF;
}

void irq_handler_rtc(void) {
  RTC->MODE1.INTFLAG.reg = RTC_MODE1_INTFLAG_OVF;
  emonTHEventSet(EVT_WAKE_TIMER);

  /* Set any lower cadence events */
  for (int i = 0; i < evtIdx; i++) {
    evtDivCnt[i]++;
    if (evtDivCnt[i] == rtcEvt[i].smpInterval) {
      emonTHEventSet(rtcEvt[i].evt);
      evtDivCnt[i] = 0;
    }
  }
}
