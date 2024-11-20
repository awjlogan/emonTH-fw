#include <stdbool.h>

#include "board_def.h"
#include "driver_TIME.h"
#include "driver_WDT.h"
#include "emonTH_samd.h"

typedef struct PrescaledTimer_ {
  uint16_t period;
  uint16_t prescalar;
} PrescaledTimer_t;

/*! @brief Common setup for 1 us resolution timer
 *  @param [in] delay : delay in us
 */
static void      commonSetup(uint32_t delay);
PrescaledTimer_t calcPrescalar(const uint16_t t_us);
void             tcSync(void);
void             timerDisable(void);
void             timerEnable(void);
static void (*tcCB)(void);

static volatile bool tcInUse   = false;
static volatile bool tcEnabled = false;

static void commonSetup(uint32_t delay) {
  TIMER_DELAY->COUNT16.COUNT.reg = 0u;
  tcSync();

  TIMER_DELAY->COUNT16.CC[0].reg = delay;
  tcSync();

  TIMER_DELAY->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  tcSync();
}

bool timerDelay_ms(uint8_t delay) { return timerDelay_us(delay * 1000u); }

bool timerDelay_us(uint16_t delay) {
  /* Return -1 if timer is already in use */
  if (tcInUse) {
    return false;
  }

  tcInUse = true;
  timerEnable();
  commonSetup(delay);

  /* Wait for timer to complete, then disable */
  while (0 == (TIMER_DELAY->COUNT16.INTFLAG.reg & TC_INTFLAG_MC0))
    ;
  TIMER_DELAY->COUNT16.INTFLAG.reg |= TC_INTFLAG_MC0;
  TIMER_DELAY->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;

  timerDisable();
  tcInUse = false;
}

PrescaledTimer_t calcPrescalar(const uint16_t t_us) {
  PrescaledTimer_t pt = {.period = t_us, .prescalar = TC_CTRLA_PRESCALER_DIV1};
  if (!(t_us & ((1 << 10) - 1))) {
    pt.prescalar = TC_CTRLA_PRESCALER_DIV1024_Val;
    pt.period    = t_us >> 10;
  } else if (!(t_us & ((1 << 8) - 1))) {
    pt.prescalar = TC_CTRLA_PRESCALER_DIV256_Val;
    pt.period    = t_us >> 8;
  } else if (!(t_us & ((1 << 6) - 1))) {
    pt.prescalar = TC_CTRLA_PRESCALER_DIV64_Val;
    pt.period    = t_us >> 6;
  } else {
    for (int i = 4; i > 0; i++) {
      if (!(t_us & ((1 << i) - 1))) {
        pt.prescalar = i;
        pt.period    = t_us >> i;
        break;
      }
    }
  }
}

void tcSync(void) {
  while (TIMER_DELAY->COUNT16.SYNCBUSY.reg)
    ;
}

void timerDisable(void) {
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

bool timerDelaySleep_us(const uint16_t t_us, const SleepMode_t sm,
                        const bool disable) {
  /* Calculate prescalar (mod time)
   * Set timer and interrupt
   */
  if (tcInUse) {
    return false;
  }
  tcInUse = true;

  if (!tcEnabled) {
    timerEnable();
  }
  NVIC_DisableIRQ(TIMER_DELAY_IRQn);

  PrescaledTimer_t pt = calcPrescalar(t_us);

  TIMER_DELAY->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;
  tcSync();
  TIMER_DELAY->COUNT16.CC[0].reg = pt.period - 1;
  tcSync();
  TIMER_DELAY->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER(pt.prescalar) |
                                   TC_CTRLA_MODE_COUNT16 | TC_CTRLA_ENABLE;
  tcSync();

  TIMER_DELAY->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;

  samdSetActivity(sm, PERIPH_IDX_TC);
  while (!(TIMER_DELAY->COUNT16.INTFLAG.reg & TC_INTFLAG_MC0)) {
    __WFI();
  }

  tcInUse = false;
  if (disable) {
    timerDisable();
  }
  return true;
}

bool timerDelaySleepAsync_us(const uint16_t t_us, const SleepMode_t sm,
                             void (*cb)()) {
  /* Calculate prescalar (mod time)
   * Set timer and interrupt
   */
  if (tcInUse) {
    return false;
  }
  tcInUse = true;

  tcCB = cb;

  if (!tcEnabled) {
    timerEnable();
  }
  NVIC_EnableIRQ(TIMER_DELAY_IRQn);
  PrescaledTimer_t pt = calcPrescalar(t_us);

  timerEnable();

  TIMER_DELAY->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;
  tcSync();
  TIMER_DELAY->COUNT16.CC[0].reg = pt.period - 1;
  tcSync();
  TIMER_DELAY->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER(pt.prescalar) |
                                   TC_CTRLA_MODE_COUNT16 | TC_CTRLA_ENABLE;
  tcSync();

  TIMER_DELAY->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;

  samdSetActivity(sm, PERIPH_IDX_TC);
  return true;
}

void timerSetup(void) {
  /* TIMER_DELAY is used as the delay and elapsed time counter
   * Enable APB clock, set TIMER_DELAY to generator 1 @ F_PERIPH
   * Enable the interrupt for Compare Match 0, and route to NVIC
   */
  MCLK->APBCMASK.reg |= TIMER_DELAY_APBCMASK;
  GCLK->PCHCTRL[TIMER_DELAY_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  TIMER_DELAY->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |
                                   TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_RUNSTDBY |
                                   TC_CTRLA_PRESCSYNC_RESYNC;

  TIMER_DELAY->COUNT16.INTENSET.reg = TC_INTENSET_MC0;

  timerDisable();
}

void irq_handler_tc1(void) {
  tcInUse = false;
  if (tcCB) {
    tcCB();
  }
}
