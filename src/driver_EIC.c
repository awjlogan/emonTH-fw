#include "emonTH_saml.h"

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "emonTH.h"

static void (*eicCB[EIC_CH_NUM])(void);
static bool pulseEn;

void eicCallbackSet(const int ch, void (*cb)()) { eicCB[ch] = cb; }

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
  }
}

void eicSetup(void) {
  /* EIC APB clock is unmasked on reset (19.8.7 APBA Mask).
   * If pulse counting is enabled, use asynchronous rising edge mode.
   * REVISIT : could use the ULP32K clock here, limited documentation on the
   * asynch edge mode.
   */

  EIC->CONFIG[0].reg =
      (EIC_SENSE_HDC | EIC_FILTEN_HDC) | (EIC_SENSE_RFM | EIC_FILTEN_RFM);
  EIC->INTENSET.reg = EIC_INTENSET_HDC;

  portPinMux(PIN_HDC_DRDY, PORT_PMUX_PMUXE(0));
  NVIC_EnableIRQ(EIC_IRQn_HDC);

  // portPinMux(PIN_RFM_IRQ, PORT_PMUX_PMUXE(1));
  // NVIC_EnableIRQ(EIC_IRQn_RFM);

  // Revisit : disable EIC outside of active loop?
  EIC->CTRLA.reg = EIC_CTRLA_CKSEL | EIC_CTRLA_ENABLE;
}

void EIC_IRQ_HANDLER_HDC(void) {
  EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT(2);
  portPinDrv(PIN_LED, PIN_DRV_TGL);
  (*eicCB[EIC_CH_HDC])();
}
