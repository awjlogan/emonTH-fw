#include <stdbool.h>

#include "board_def.h"
#include "driver_SAML.h"
#include "driver_TIME.h"
#include "emonTH_assert.h"
#include "emonTH_saml.h"

static void tcSync(Tc *t);
static bool timerDelaySleepLP(const uint16_t t_ms);
static bool timerSleepCommon(const uint32_t t_us);

static void (*tcCB)(void);
static void (*ovfCB)(void);
static void (*tcLPCB)(void);
static void (*tcPulseCB)(void);

static volatile bool tcEnabled = false;
static volatile bool tdMatch   = false;
static volatile bool tdLPMatch = false;

static volatile int   ledPulseIdx      = 0;
static volatile bool  ledPulseDown     = false;
static const uint16_t ledPulseTop      = 1953;
static const uint16_t ledIntensity[32] = {
    1,    7,    17,   30,   47,   68,   93,   122,  154,  190, 230,
    274,  322,  373,  429,  488,  551,  617,  688,  762,  841, 923,
    1008, 1098, 1192, 1289, 1390, 1495, 1604, 1716, 1832, 1953};

typedef struct tcCfg_ {
  Tc      *instance;
  uint32_t apbmask;
  uint8_t  gclk_id;
  uint32_t phctrl_gclk;
  uint32_t prescalar;
  uint8_t  irqn;
} tcCfg_t;

void timerDelay_us(const uint16_t delay) {
  // clang-format off
  __asm volatile (	"MOV R0,%[loops]\n\t"
      "1: \n\t"
			"SUB R0, #1\n\t"
			"CMP R0, #0\n\t"
			"BNE 1b \n\t" : : [loops] "r" (2*delay) : "memory");
  // clang-format on
}

static void tcSync(Tc *t) {
  while (t->COUNT16.SYNCBUSY.reg)
    ;
}

bool timerDelaySleep_ms(const uint16_t t_ms) {
  if (0 == t_ms) {
    return true;
  }
  if (t_ms < 75u) {
    return timerDelaySleep_us((uint32_t)t_ms * 1000u);
  } else {
    return timerDelaySleepLP(t_ms);
  }
}

bool timerDelaySleepAsync_ms(const uint16_t t_ms, void (*cb)()) {
  return timerDelaySleepAsync_us((uint32_t)t_ms * 1000u, cb);
}

bool timerDelaySleep_us(const uint32_t t_us) {
  /* For short delays, the entry/exit delay is a significant fraction, so just
   * do blocking delay in this case. */
  if (t_us < 64u) {
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

static bool timerDelaySleepLP(const uint16_t t_ms) {
  uint32_t cc = ((t_ms * 1024u) / 1000u) - 1u;
  tdLPMatch   = false;

  TIMER_LP->COUNT16.CC[0].reg = cc;
  tcSync(TIMER_LP);
  TIMER_LP->COUNT16.COUNT.reg = 0;
  tcSync(TIMER_LP);
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
  tcSync(TIMER_DELAY);
  TIMER_DELAY->COUNT16.COUNT.reg = 0;
  tcSync(TIMER_DELAY);
  TIMER_DELAY->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

  return true;
}

void timerFlush(void) {
  /* Flush internal flags and values */
  tcCB      = 0;
  tcEnabled = false;
  TIMER_DELAY->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  tcSync(TIMER_DELAY);
}

void timerPulseStart(void) {
  tcSync(TIMER_PULSE);
  TIMER_PULSE->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
}

void timerSetup() {

  tcCfg_t tcCfg[TC_NUM_INST] = {/* High resolution (8 us) timer */
                                {.instance    = TIMER_DELAY,
                                 .apbmask     = TIMER_DELAY_APBCMASK,
                                 .gclk_id     = TIMER_DELAY_GCLK_ID,
                                 .phctrl_gclk = GCLK_PCHCTRL_GEN_GCLK0,
                                 .prescalar   = TC_CTRLA_PRESCALER_DIV64,
                                 .irqn        = TIMER_DELAY_IRQn},
                                /* Low power ~1 ms resolution timer */
                                {.instance    = TIMER_LP,
                                 .apbmask     = TIMER_LP_APBCMASK,
                                 .gclk_id     = TIMER_LP_GCLK_ID,
                                 .phctrl_gclk = GCLK_PCHCTRL_GEN_GCLK1,
                                 .prescalar   = 0,
                                 .irqn        = TIMER_LP_IRQn},
                                /* Dedicated pulse channel timer */
                                {.instance    = TIMER_PULSE,
                                 .apbmask     = TIMER_PULSE_APBCMASK,
                                 .gclk_id     = TIMER_PULSE_GCLK_ID,
                                 .phctrl_gclk = GCLK_PCHCTRL_GEN_GCLK1,
                                 .prescalar   = 0,
                                 .irqn        = TIMER_PULSE_IRQn}};

  for (int i = 0; i < TC_NUM_INST; i++) {
    tcCfg_t *t = &tcCfg[i];
    MCLK->APBCMASK.reg |= t->apbmask;
    GCLK->PCHCTRL[t->gclk_id].reg = t->phctrl_gclk | GCLK_PCHCTRL_CHEN;
    while (!(GCLK->PCHCTRL[t->gclk_id].reg & GCLK_PCHCTRL_CHEN))
      ;

    t->instance->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (t->instance->COUNT16.SYNCBUSY.reg & TC_SYNCBUSY_SWRST)
      ;

    t->instance->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_RUNSTDBY |
                                     TC_CTRLA_PRESCSYNC_RESYNC | t->prescalar;

    t->instance->COUNT16.WAVE.reg     = TC_WAVE_WAVEGEN_NFRQ;
    t->instance->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
    t->instance->COUNT16.COUNT.reg    = 0;

    NVIC_EnableIRQ(t->irqn);
  }
}

void timerSetupLED(void (*cb)()) {
  EMONTH_ASSERT(cb);
  ovfCB = cb;

  MCLK->APBCMASK.reg |= TIMER_LP_APBCMASK;
  GCLK->PCHCTRL[TIMER_LP_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[TIMER_LP_GCLK_ID].reg & GCLK_PCHCTRL_CHEN))
    ;

  TIMER_LP->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_RUNSTDBY |
                                TC_CTRLA_PRESCSYNC_RESYNC |
                                TC_CTRLA_PRESCALER_DIV64;

  /* Enable NPWM mode with CC update buffering */
  TIMER_LP->COUNT16.WAVE.reg     = TC_WAVE_WAVEGEN_NPWM;
  // TIMER_LP->COUNT16.CTRLBSET.reg = TC_CTRLBSET_LUPD;
  TIMER_LP->COUNT16.PER.reg      = ledPulseTop;
  TIMER_LP->COUNT16.COUNT.reg    = 0;
  TIMER_LP->COUNT16.INTENSET.reg = TC_INTENSET_OVF;
  TIMER_LP->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

  NVIC_EnableIRQ(TIMER_LP_IRQn);
}

void timerSetupPulse(const uint16_t timeMask_ms, void (*cb)()) {
  EMONTH_ASSERT(cb);

  tcPulseCB                      = cb;
  TIMER_PULSE->COUNT16.CC[0].reg = (timeMask_ms * 1024u / 1000u) - 1u;
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

void TIMER_LP_HANDLER(void) {
  if ((TIMER_LP->COUNT16.INTFLAG.reg & TC_INTFLAG_MC0)) {
    TIMER_LP->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    tdLPMatch                     = true;

    if (tcLPCB) {
      tcLPCB();
      tcLPCB = 0;
    }
  }

  /* Pulsed LED at startup + configuration */
  if ((TIMER_LP->COUNT16.INTFLAG.reg & TC_INTFLAG_OVF)) {
    TIMER_LP->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
    if (false == ledPulseDown) {
      ledPulseIdx++;
      if (32 == ledPulseIdx) {
        ledPulseIdx  = 31;
        ledPulseDown = true;
      }
    } else {
      ledPulseIdx--;
      if (-1 == ledPulseIdx) {
        ledPulseIdx  = 0;
        ledPulseDown = false;
      }
    }
    TIMER_LP->COUNT16.CCBUF[0].reg = ledIntensity[ledPulseIdx];
    ovfCB();
  }
}

void TIMER_PULSE_HANDLER(void) {
  TIMER_PULSE->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  tcPulseCB();
  tcSync(TIMER_PULSE);
  TIMER_PULSE->COUNT16.COUNT.reg = 0;
}
