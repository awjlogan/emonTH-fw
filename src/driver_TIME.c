#include "emonTH_samd.h"

#include "board_def.h"
#include "driver_TIME.h"
#include "driver_WDT.h"

/*! @brief Common setup for 1 us resolution timer
 *  @param [in] delay : delay in us
 */
static void commonSetup(uint32_t delay);

static volatile uint32_t timeMillisCounter  = 0;
static volatile uint32_t timeSecondsCounter = 0;

/* Function pointer for non-blocking timer callback */
void (*tc2_cb)();
static unsigned int TIMER_DELAYInUse = 0;

static void commonSetup(uint32_t delay) {

  /* Unmask match interrrupt, zero counter, set compare value */
  TIMER_DELAY->COUNT32.INTENSET.reg |= TC_INTENSET_MC0;
  TIMER_DELAY->COUNT32.COUNT.reg = 0u;
  while (TIMER_DELAY->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY)
    ;
  TIMER_DELAY->COUNT32.CC[0].reg = delay;
  while (TIMER_DELAY->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY)
    ;
  TIMER_DELAY->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TIMER_DELAY->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY)
    ;
}

int timerDelay_ms(uint16_t delay) { return timerDelay_us(delay * 1000u); }

int timerDelay_us(uint32_t delay) {
  /* Return -1 if timer is already in use */
  if (TIMER_DELAYInUse) {
    return -1;
  }

  TIMER_DELAYInUse = 1;
  commonSetup(delay);

  /* Wait for timer to complete, then disable */
  while (0 == (TIMER_DELAY->COUNT32.INTFLAG.reg & TC_INTFLAG_MC0))
    ;
  TIMER_DELAY->COUNT32.INTENCLR.reg = TC_INTENCLR_MC0;
  TIMER_DELAY->COUNT32.INTFLAG.reg |= TC_INTFLAG_MC0;
  TIMER_DELAY->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;

  TIMER_DELAYInUse = 0;
  return 0;
}

void timerDisable(void) {
  TIMER_DELAY->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  NVIC_DisableIRQ(TIMER_DELAY_IRQn);
}

uint32_t timerMillis(void) { return timeMillisCounter; }

uint32_t timerMillisDelta(const uint32_t prevMillis) {
  uint32_t delta = 0;

  /* Check for wrap around (every 49 days, so rare!) */
  if (prevMillis > timeMillisCounter) {
    delta = (UINT32_MAX - prevMillis) + timeMillisCounter;
  } else {
    delta = timeMillisCounter - prevMillis;
  }
  return delta;
}

void timerSetup(void) {
  /* TIMER_DELAY is used as the delay and elapsed time counter
   * Enable APB clock, set TIMER_DELAY to generator 3 @ F_PERIPH
   * Enable the interrupt for Compare Match, do not route to NVIC
   */
  PM->APBCMASK.reg |= TIMER_DELAY_APBCMASK;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TIMER_DELAY_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;
  TIMER_DELAY->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 |
                                   TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_RUNSTDBY |
                                   TC_CTRLA_PRESCSYNC_RESYNC;
}

/*! @brief On delay timer (TIMER_DELAY) expiration, call the callback function
 */
void IRQ_TIMER_DELAY(void) {
  if (0 != tc2_cb) {
    tc2_cb();
  }
}
