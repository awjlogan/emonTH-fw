#include "driver_SAML.h"
#include "emonTH_saml.h"
#include "fuses.h"

void clkSetup(void) {
  /* Set 1WS in PL0 at 8 MHz, 3V3 (Table 46-40) */
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS(1);

  /* Set OSC16M to 8 MHz, running on-demand in standby */
  OSCCTRL->OSC16MCTRL.reg =
      OSCCTRL_OSC16MCTRL_ONDEMAND | OSCCTRL_OSC16MCTRL_RUNSTDBY |
      OSCCTRL_OSC16MCTRL_ENABLE | OSCCTRL_OSC16MCTRL_FSEL_8;

  /* Setup Clock Generator 1 to run at OSC16M / 8 -> 1 MHz for peripherals.
   * GENDIV divides the frequency by 2^(GENDIV + 1)
   */
  GCLK->GENCTRL[1].reg = GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_DIVSEL |
                         GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSC16M;

  /* Mask off peripherals that are never used */
  MCLK->APBCMASK.reg &= ~MCLK_APBCMASK_OPAMP & ~MCLK_APBCMASK_CCL &
                        ~MCLK_APBCMASK_TRNG & ~MCLK_APBCMASK_PTC &
                        ~MCLK_APBCMASK_DAC & ~MCLK_APBCMASK_TC2 &
                        ~MCLK_APBCMASK_EVSYS;
}
