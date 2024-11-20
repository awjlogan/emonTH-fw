#include "driver_SAMD.h"
#include "emonTH_samd.h"
#include "fuses.h"

void clkSetup(void) {
  /* Boost OSC8M to 8 MHz from initial 1 MHz */
  OSCCTRL->OSC16MCTRL.reg =
      OSCCTRL_OSC16MCTRL_ENABLE | OSCCTRL_OSC16MCTRL_FSEL_16;

  /* Setup Clock Generator 1 to run at OSC16M / 16 -> 1 MHz for peripherals.
   * GENDIV divides the frequency by 2^(GENDIV + 1)
   */
  GCLK->GENCTRL[1].reg = GCLK_GENCTRL_DIV(3) | GCLK_GENCTRL_DIVSEL |
                         GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSC16M;
}
