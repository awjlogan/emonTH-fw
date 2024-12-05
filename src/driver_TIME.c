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
static bool      timerSleepCommon(const uint32_t t_us, SleepMode_t sm);

static void (*tcCB)(void);
static void (*tcPulseCB)(void);

static volatile bool tcInUse   = false;
static volatile bool tcEnabled = false;

/* REVISIT check correctness of this @ 8 MHz */
void timerDelay_us(uint16_t delay) {
  // clang-format off
  __asm volatile (	"MOV R0,%[loops]\n\t"
      "1: \n\t"
			"SUB R0, #1\n\t"
			"CMP R0, #0\n\t"
			"BNE 1b \n\t" : : [loops] "r" (8*delay) : "memory");
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
      GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  TIMER_DELAY->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  tcSync();
  tcEnabled = true;
}

bool timerDelaySleep_ms(const uint16_t t_ms, const SleepMode_t sm,
                        const bool disable) {
  return timerDelaySleep_us(t_ms * 1000, sm, disable);
}

bool timerDelaySleepAsync_ms(const uint16_t t_ms, const SleepMode_t sm,
                             void (*cb)()) {
  return timerDelaySleepAsync_us(t_ms * 1000, sm, cb);
}

bool timerDelaySleep_us(const uint32_t t_us, const SleepMode_t sm,
                        const bool disable) {

  /* For short delays, the entry/exit delay is a significant fraction, so just
   * do blocking delay in this case. */
  if (t_us < 64) {
    timerDelay_us(t_us);
    return true;
  }

  /* Calculate prescalar (mod time), and set timer and interrupt */
  if (!timerSleepCommon(t_us, sm)) {
    return false;
  }

  while (!(TIMER_DELAY->COUNT16.INTFLAG.reg & TC_INTFLAG_MC0)) {
    __WFI();
  }

  tcInUse = false;
  if (disable) {
    timerDisable();
  }
  return true;
}

bool timerDelaySleepAsync_us(const uint32_t t_us, const SleepMode_t sm,
                             void (*cb)()) {
  tcCB = cb;
  return timerSleepCommon(t_us, sm);
}

static bool timerSleepCommon(const uint32_t t_us, SleepMode_t sm) {
  /* Calculate prescalar (mod time), set timer and interrupt. */
  if (tcInUse) {
    return false;
  }
  if (!tcEnabled) {
    timerEnable();
  }
  tcInUse = true;

  PrescaledTimer_t pt = calcPrescalar(t_us);

  TIMER_DELAY->COUNT16.CC[0].reg = pt.period - 1;
  tcSync();
  TIMER_DELAY->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER(pt.prescalar) |
                                   TC_CTRLA_MODE_COUNT16 | TC_CTRLA_ENABLE;
  tcSync();

  TIMER_DELAY->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;

  samlSetActivity(sm, PERIPH_IDX_TC);
  return true;
}

void timerFlush(void) {
  /* Disable and clear all interrupts and the whole TC */
  tcCB                              = 0;
  tcInUse                           = false;
  tcEnabled                         = false;
  TIMER_DELAY->COUNT16.INTENCLR.reg = TIMER_DELAY->COUNT16.INTENCLR.reg;
  TIMER_DELAY->COUNT16.INTFLAG.reg  = TIMER_DELAY->COUNT16.INTFLAG.reg;
  TIMER_DELAY->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  tcSync();
  tcInUse = false;
}

void timerSetup() {
  /* TIMER_DELAY is used as the delay and elapsed time counter
   * Enable APB clock, set TIMER_DELAY to generator 1 @ F_PERIPH
   * Enable the interrupt for Compare Match 0, and route to NVIC
   */
  MCLK->APBCMASK.reg |= TIMER_DELAY_APBCMASK;
  GCLK->PCHCTRL[TIMER_DELAY_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;

  TIMER_DELAY->COUNT16.CTRLA.reg =
      TC_CTRLA_MODE_COUNT16 | TC_CTRLA_RUNSTDBY | TC_CTRLA_PRESCSYNC_RESYNC;

  TIMER_DELAY->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  TIMER_DELAY->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;
  tcSync();

  NVIC_EnableIRQ(TIMER_DELAY_IRQn);

  timerDisable();
}

void timerPulseSetup(void (*cb)()) {
  EMONTH_ASSERT(cb);

  tcPulseCB = cb;
  MCLK->APBCMASK.reg |= TIMER_PULSE_APBCMASK;
  GCLK->PCHCTRL[TIMER_PULSE_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  TIMER_PULSE->COUNT16.CTRLA.reg =
      TC_CTRLA_MODE_COUNT16 | TC_CTRLA_RUNSTDBY | TC_CTRLA_PRESCALER_DIV1024;
  TIMER_PULSE->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  TIMER_PULSE->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;
  GCLK->PCHCTRL[TIMER_DELAY_GCLK_ID].reg &= ~GCLK_PCHCTRL_CHEN;
  MCLK->APBCMASK.reg &= ~TIMER_PULSE_APBCMASK;
  NVIC_EnableIRQ(TIMER_PULSE_IRQn);
}

void timerPulseStart(uint16_t tMask_ms) {
  int tMaskScale = (tMask_ms * 1024) / 1000;
  MCLK->APBCMASK.reg |= TIMER_PULSE_APBCMASK;
  GCLK->PCHCTRL[TIMER_PULSE_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;

  TIMER_PULSE->COUNT16.CC[0].reg = tMaskScale;
  while (TIMER_PULSE->COUNT16.SYNCBUSY.reg)
    ;
  TIMER_PULSE->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TIMER_PULSE->COUNT16.SYNCBUSY.reg)
    ;
  TIMER_PULSE->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
  while (TIMER_PULSE->COUNT16.SYNCBUSY.reg)
    ;
  GCLK->PCHCTRL[TIMER_DELAY_GCLK_ID].reg &= ~GCLK_PCHCTRL_CHEN;
  MCLK->APBCMASK.reg &= ~TIMER_PULSE_APBCMASK;
}

void irq_handler_tc1(void) {
  tcInUse = false;
  if (tcCB) {
    tcCB();
    tcCB = 0;
  }
}

void irq_handler_tc2(void) {
  tcPulseCB();
  MCLK->APBCMASK.reg |= TIMER_PULSE_APBCMASK;
  GCLK->PCHCTRL[TIMER_PULSE_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  TIMER_PULSE->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  TIMER_PULSE->COUNT16.INTFLAG.reg = TIMER_PULSE->COUNT16.INTFLAG.reg;
  GCLK->PCHCTRL[TIMER_DELAY_GCLK_ID].reg &= ~GCLK_PCHCTRL_CHEN;
  MCLK->APBCMASK.reg &= ~TIMER_PULSE_APBCMASK;
}
