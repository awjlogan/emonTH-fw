#include "emonTH_samd.h"

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"

static void (*eicCB[EIC_CH_NUM])(void);

void eicLevelDisable(const int ch) {
  switch (ch) {
  case EIC_CH_RFM:
    EIC->INTENCLR.reg &= EIC_INTENCLR_EXTINT(0);
    break;
  case EIC_CH_HDC:
    EIC->INTENCLR.reg &= EIC_INTENCLR_EXTINT(1);
    break;
  case EIC_CH_OW:
    break;
  }
}

void eicLevelEnable(const int ch, void (*cb)()) {
  /* Only a single set of EIC inputs on SAML10 */
  switch (ch) {
  case EIC_CH_RFM:
    EIC->INTENSET.reg |= EIC_INTENSET_EXTINT(0);
    eicCB[0] = cb;
    break;
  case EIC_CH_HDC:
    EIC->INTENSET.reg |= EIC_INTENSET_EXTINT(1);
    eicCB[1] = cb;
    break;
  case EIC_CH_OW:
    break;
  }
}

void eicSetup(void) {
  /* EIC APB clock is unmasked on reset (16.8.8).
   * Require EIC GCLK for edge detection. (21.6.2.1)
   * GCLK->CLKCTRL.reg =
   *    GCLK_CLKCTRL_ID(EIC_GCLK_ID) | GCLK_CLKCTRL_GEN(3u) |
   * GCLK_CLKCTRL_CLKEN;
   */
  NVIC_EnableIRQ(EIC_0_IRQn);
}

void irq_handler_eic(void) {
  if (EIC->INTFLAG.reg & EIC_INTFLAG_EXTINT(0)) {
    if (eicCB[0]) {
      eicCB[0];
    }
    EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT(0);
  }
  if (EIC->INTFLAG.reg & EIC_INTFLAG_EXTINT(1)) {
    if (eicCB[1]) {
      eicCB[1]();
    }
    EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT(1);
  }
}
