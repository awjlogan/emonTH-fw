#include <stdbool.h>

#include "emonTH_saml.h"

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "emonTH.h"

#define INT_EXTINT(ch) (1u << ch)

static void (*eicCB[8])(void);

void eicCallbackSet(const size_t ch, void (*cb)()) { eicCB[ch] = cb; }

void eicChannelDisable(const size_t ch) {
  EIC->INTENCLR.reg = ~INT_EXTINT(ch);
  EIC->CONFIG[0].reg &= ~(0xFu << (ch * 4u));

  /* Channels 0-3 have individual NVIC lines, starting from 3 */
  if (ch < 4u) {
    NVIC_DisableIRQ(3u + ch);
  } else {
    NVIC_DisableIRQ(7u);
  }
}

void eicChannelEnable(const EIC_Cfg_t eiccfg) {
  /* EIC CONFIG is enable protected, so must be disabled before continuing. */
  bool enabled = EIC->CTRLA.reg & EIC_CTRLA_ENABLE;
  if (enabled) {
    EIC->CTRLA.reg &= ~EIC_CTRLA_ENABLE;
    while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE)
      ;
  }

  uint32_t config = EIC->CONFIG[0].reg;
  config &= ~(0xFu << (eiccfg.ch * 4u));
  config |= eiccfg.sense << (eiccfg.ch * 4u);
  EIC->CONFIG[0].reg = config;
  EIC->INTENSET.reg  = INT_EXTINT(eiccfg.ch);

  /* Channels 0-3 have individual NVIC lines, starting from 3 */
  if (eiccfg.ch < 4u) {
    NVIC_EnableIRQ(3u + eiccfg.ch);
  } else {
    NVIC_EnableIRQ(7u);
  };

  portPinMux(eiccfg.pin, PORT_PMUX_PMUXE(0));
  eicCB[eiccfg.ch] = eiccfg.cb;

  if (enabled) {
    EIC->CTRLA.reg |= EIC_CTRLA_ENABLE;
    while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE)
      ;
  }
}

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
  /* EIC APB clock is unmasked on reset (19.8.7 APBA Mask).
   * If pulse counting is enabled, use asynchronous rising edge mode.
   * REVISIT : could use the ULP32K clock here, limited documentation on the
   * asynch edge mode.
   */

  // Revisit : disable EIC outside of active loop?
  EIC->CTRLA.reg = EIC_CTRLA_CKSEL | EIC_CTRLA_ENABLE;
}

void EIC_IRQ_HANDLER_HDC(void) {
  EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT(INT_EXTINT(EIC_CH_HDC));
  (*eicCB[EIC_CH_HDC])();
}
