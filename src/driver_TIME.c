#include <stdbool.h>

#include "board_def.h"
#include "driver_TIME.h"
#include "emonTH_assert.h"
#include "emonTH_saml.h"

typedef struct PrescaledTimer_ {
  uint16_t period;
  uint16_t prescalar;
} PrescaledTimer_t;

PrescaledTimer_t calcPrescalar(const uint32_t t_us);
void             tcSync(void);
void             timerDisable(void);
void             timerEnable(void);
static bool      timerSleepCommon(const uint32_t t_us);

static void (*tcCB)(void);
static void (*tcPulseCB)(void);

static volatile bool tcEnabled = false;
static volatile bool tdMatch   = false;
static volatile bool tdLPMatch = false;

void timerDelay_us(uint16_t delay) {
  // clang-format off
  __asm volatile (	"MOV R0,%[loops]\n\t"
      "1: \n\t"
			"SUB R0, #1\n\t"
			"CMP R0, #0\n\t"
			"BNE 1b \n\t" : : [loops] "r" (2*delay) : "memory");
  // clang-format on
}

PrescaledTimer_t calcPrescalar(const uint32_t t_us) {
  PrescaledTimer_t pt = {.period = t_us, .prescalar = TC_CTRLA_PRESCALER_DIV1};

  /* Drop to millisecond resolution, with 3% correction for divider. */
  if (t_us > 1000) {
    pt.prescalar = TC_CTRLA_PRESCALER_DIV1024;
    pt.period    = t_us * 1024 / 1000000;
  } else if (!(t_us & ((1 << 8) - 1))) {
    pt.prescalar = TC_CTRLA_PRESCALER_DIV256_Val;
    pt.period    = t_us >> 8;
  } else if (!(t_us & ((1 << 6) - 1))) {
    pt.prescalar = TC_CTRLA_PRESCALER_DIV64_Val;
    pt.period    = t_us >> 6;
  }

  pt.period -= 1;
  return pt;
}

void tcSync(void) {
  while (TIMER_DELAY->COUNT16.SYNCBUSY.reg)
    ;
}

void timerDisable(void) {
  TIMER_DELAY->COUNT16.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV1;
  TIMER_DELAY->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  tcSync();
  GCLK->PCHCTRL[TIMER_DELAY_GCLK_ID].reg &= ~GCLK_PCHCTRL_CHEN;
  MCLK->APBCMASK.reg &= ~TIMER_DELAY_APBCMASK;
  tcEnabled = false;
}

void timerEnable(void) {
  MCLK->APBCMASK.reg |= TIMER_DELAY_APBCMASK;
  GCLK->PCHCTRL[TIMER_DELAY_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
  TIMER_DELAY->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  tcSync();
  tcEnabled = true;
}

bool timerDelaySleep_ms(const uint16_t t_ms) {
  return timerDelaySleep_us((uint32_t)t_ms * 1000);
}

bool timerDelaySleepAsync_ms(const uint16_t t_ms, void (*cb)()) {
  return timerDelaySleepAsync_us((uint32_t)t_ms * 1000, cb);
}

bool timerDelaySleep_us(const uint32_t t_us) {
  /* For short delays, the entry/exit delay is a significant fraction, so just
   * do blocking delay in this case. */
  if (t_us < 64) {
    timerDelay_us(t_us);
    return true;
  }

  timerSleepCommon(t_us);
  while (!tdMatch) {
    samlSleepEnter();
  }

  TIMER_DELAY->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  return true;
}

bool timerDelaySleepAsync_us(const uint32_t t_us, void (*cb)()) {
  tcCB = cb;
  return timerSleepCommon(t_us);
}

bool timerDelaySleepLP(const uint16_t t_ms) {
  uint32_t cc = ((t_ms * 2048) / 1000) - 1;
  tdLPMatch   = false;

  TIMER_LP->COUNT16.CC[0].reg = cc;
  while (TIMER_LP->COUNT16.SYNCBUSY.reg & TC_SYNCBUSY_CC0)
    ;
  TIMER_LP->COUNT16.COUNT.reg = 0;
  while (TIMER_LP->COUNT16.SYNCBUSY.reg & TC_SYNCBUSY_COUNT)
    ;
  TIMER_LP->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

  while (!tdLPMatch) {
    samlSleepEnter();
  }
  TIMER_LP->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  return true;
}

static bool timerSleepCommon(const uint32_t t_us) {

  uint32_t cc = t_us / 8;
  if (0 != cc) {
    cc--;
  }

  uint32_t ctrla = TIMER_DELAY->COUNT16.CTRLA.reg;
  ctrla &= ~(TC_CTRLA_PRESCALER_Msk);
  if (UINT16_MAX < cc) {
    ctrla |= TC_CTRLA_PRESCALER_DIV256;
    cc = cc / 4;
  } else {
    ctrla |= TC_CTRLA_PRESCALER_DIV64;
  }
  TIMER_DELAY->COUNT16.CTRLA.reg = ctrla;

  tdMatch = false;

  TIMER_DELAY->COUNT16.CC[0].reg = cc;
  TIMER_DELAY->COUNT16.COUNT.reg = 0;
  TIMER_DELAY->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

  return true;
}

void timerFlush(void) {
  /* Flush internal flags and values */
  tcCB      = 0;
  tcEnabled = false;
  TIMER_DELAY->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  tcSync();
}

void timerSetup() {
  /* TIMER_DELAY is used for higher precision times */
  MCLK->APBCMASK.reg |= TIMER_DELAY_APBCMASK;
  GCLK->PCHCTRL[TIMER_DELAY_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[TIMER_DELAY_GCLK_ID].reg & GCLK_PCHCTRL_CHEN))
    ;

  TIMER_DELAY->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_RUNSTDBY |
                                   TC_CTRLA_PRESCSYNC_RESYNC |
                                   TC_CTRLA_PRESCALER_DIV64; /* 8 us tick */

  TIMER_DELAY->COUNT16.WAVE.reg     = TC_WAVE_WAVEGEN_NFRQ;
  TIMER_DELAY->COUNT16.COUNT.reg    = 0;
  TIMER_DELAY->COUNT16.INTENSET.reg = TC_INTENSET_MC0;

  NVIC_EnableIRQ(TIMER_DELAY_IRQn);

  /* TIMER_LP is 0.5 ms resolution time off the ULP32K */
  MCLK->APBCMASK.reg |= TIMER_LP_APBCMASK;
  GCLK->PCHCTRL[TIMER_LP_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[TIMER_LP_GCLK_ID].reg & GCLK_PCHCTRL_CHEN))
    ;

  TIMER_LP->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_RUNSTDBY |
                                TC_CTRLA_ONDEMAND | TC_CTRLA_PRESCSYNC_RESYNC |
                                TC_CTRLA_PRESCALER_DIV16;
  TIMER_LP->COUNT16.WAVE.reg     = TC_WAVE_WAVEGEN_NFRQ;
  TIMER_LP->COUNT16.COUNT.reg    = 0;
  TIMER_LP->COUNT16.INTENSET.reg = TC_INTENSET_MC0;

  NVIC_EnableIRQ(TIMER_LP_IRQn);
}

void timerSetupPulse(const uint8_t per, void (*cb)()) {
  EMONTH_ASSERT(cb);
  tcPulseCB = cb;

  MCLK->APBCMASK.reg |= TIMER_PULSE_APBCMASK;
  GCLK->PCHCTRL[TIMER_PULSE_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[TIMER_PULSE_GCLK_ID].reg & GCLK_PCHCTRL_CHEN))
    ;

  TIMER_PULSE->COUNT8.CTRLA.reg    = TC_CTRLA_MODE_COUNT8 | TC_CTRLA_RUNSTDBY;
  TIMER_PULSE->COUNT8.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;
  TIMER_PULSE->COUNT8.INTENSET.reg = TC_INTENSET_OVF;
  TIMER_PULSE->COUNT8.PER.reg      = per;

  NVIC_EnableIRQ(TIMER_PULSE_IRQn);
}

void timerStartPulse(void) {
  TIMER_PULSE->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TIMER_PULSE->COUNT8.SYNCBUSY.reg)
    ;
  TIMER_PULSE->COUNT8.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
  while (TIMER_PULSE->COUNT8.SYNCBUSY.reg)
    ;
}

void TIMER_DELAY_HANDLER(void) {
  if ((TIMER_DELAY->COUNT16.INTFLAG.reg & TC_INTFLAG_MC0)) {
    TIMER_DELAY->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

    tdMatch = true;
    if (tcCB) {
      tcCB();
      tcCB = 0;
    }
  }
}

void TIMER_PULSE_HANDLER(void) {
  TIMER_PULSE->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  tcPulseCB();
}
