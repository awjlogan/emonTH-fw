#include "emonTH_saml.h"

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "emonTH.h"

static int  chToIndex(const int ch);
static void intHandler(const int ch);
static void intWake(void);

static void (*eicCB[EIC_CH_NUM])(void);
static bool pulseEn;

static int chToIndex(const int ch) {
  int index = 0;
  switch (ch) {
  case EIC_CH_RFM:
    index = 0;
    break;
  case EIC_CH_HDC:
    index = 1;
    break;
  case EIC_CH_OW:
    index = 2;
    break;
  case EIC_CH_PULSE:
    index = 3;
    break;
  case EIC_CH_WAKE:
    index = 4;
    break;
  }
  return index;
}

void eicCallbackSet(const int ch, void (*cb)()) { eicCB[chToIndex(ch)] = cb; }

void eicChannelDisable(const int ch) {
  switch (ch) {
  case EIC_CH_RFM:
    EIC->INTENCLR.reg &= EIC_INTENCLR_EXTINT(0);
    EIC->CONFIG[0].bit.SENSE0 = EIC_CONFIG_SENSE0_NONE_Val;
    break;
  case EIC_CH_HDC:
    EIC->INTENCLR.reg &= EIC_INTENCLR_EXTINT(1);
    EIC->CONFIG[0].bit.SENSE1 = EIC_CONFIG_SENSE1_NONE_Val;
    break;
  case EIC_CH_OW:
    break;
  }
}

void eicChannelEnable(const int ch, const int sense, void (*cb)()) {
  /* Only a single set of EIC inputs on SAML10 */
  switch (ch) {
  case EIC_CH_RFM:
    EIC->INTENSET.reg |= EIC_INTENSET_EXTINT(0);
    EIC->CONFIG[0].bit.SENSE0 = sense;
    eicCB[0]                  = cb;
    break;
  case EIC_CH_HDC:
    EIC->INTENSET.reg |= EIC_INTENSET_EXTINT(1);
    EIC->CONFIG[0].bit.SENSE1 = sense;
    eicCB[1]                  = cb;
    break;
  case EIC_CH_OW:
    break;
  }
}

void eicSetup(void) {
  /* EIC APB clock is unmasked on reset (19.8.7).
   * If pulse counting is enabled, use asynchronous rising edge mode.
   * REVISIT : could use the ULP32K clock here, limited documentation on the
   * asynch edge mode.
   */

  /* Enable asynch rising edge interrupt on the wake up pin */
  EIC->INTENSET.reg = EIC_INTENSET_EXTINT(EIC_CH_WAKE);
  EIC->ASYNCH.reg |= EIC_ASYNCH_ASYNCH(EIC_CH_WAKE);
  EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE0_RISE;
  eicCB[EIC_CH_WAKE] = &intWake;

  NVIC_EnableIRQ(EIC_0_IRQn);
}

void eicSetupClose(void) { MCLK->APBAMASK.reg &= ~MCLK_APBAMASK_EIC; }

void eicSetupPulse(void) {
  EIC->INTENSET.reg = EIC_INTENSET_EXTINT(EIC_CH_PULSE);
  EIC->ASYNCH.reg |= EIC_ASYNCH_ASYNCH(EIC_CH_PULSE);
  EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE7_RISE;
  pulseEn = true;
}

static void intHandler(const int ch) {
  int chIndex = chToIndex(ch);
  if (eicCB[chIndex]) {
    eicCB[chIndex]();
  }
}

static void intWake(void) { emonTHEventSet(EVT_WAKE_TIMER); }

void irq_handler_eic(void) {
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_EIC;
  for (int i = 0; i < 8; i++) {
    if (EIC->INTFLAG.reg & EIC_INTFLAG_EXTINT(i)) {
      EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT(i);
      intHandler(i);
    }
  }
  MCLK->APBAMASK.reg &= ~MCLK_APBAMASK_EIC;
}
