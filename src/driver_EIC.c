#include <stdbool.h>

#include "emonTH_saml.h"

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "emonTH.h"

static void (*cbHDC)(void);
static void (*cbPulse)(void);

void eicDisable(void) {
  EIC->CTRLA.reg &= ~EIC_CTRLA_ENABLE;
  while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE)
    ;
}

void eicEnable(void) {
  EIC->CTRLA.reg |= EIC_CTRLA_ENABLE;
  while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE)
    ;
}

void eicSetup(void) {
  /* EIC APB clock is unmasked on reset (19.8.7 APBA Mask). */
  portPinCfg(PIN_HDC_DRDY, PORT_PINCFG_INEN, PIN_CFG_SET);
  portPinMux(PIN_HDC_DRDY, PORT_PMUX_PMUXE(0));
  // portPinMux(PIN_PULSE, PORT_PMUX_PMUXE(0));

  EIC->CONFIG[0].reg = EIC_CONFIG_SENSE4_RISE | EIC_CONFIG_FILTEN4;
  EIC->INTENSET.reg  = 0x10;
  EIC->ASYNCH.reg    = 0x10;

  /* Both HDC and pulse are on EXTINT[4] */
  NVIC_EnableIRQ(EIC_4_IRQn);

  EIC->CTRLA.bit.CKSEL = 1u;
}

void EIC_IRQ_HANDLER(void) {
  EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT(0x10);
  if (cbHDC) {
    cbHDC();
  }
  if (cbPulse) {
    cbPulse();
  }
}

void eicSetupHDC(void (*cb)(void)) { cbHDC = cb; }

void eicSetupPulse(void (*cb)(void)) { cbPulse = cb; }
