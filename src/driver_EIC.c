#include "emonTH_samd.h"

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_PORT.h"

void eicSetup(void) {
  /* EIC APB clock is unmasked on reset (16.8.8).
   * Require EIC GCLK for edge detection. (21.6.2.1)
   * GCLK->CLKCTRL.reg =
   *    GCLK_CLKCTRL_ID(EIC_GCLK_ID) | GCLK_CLKCTRL_GEN(3u) |
   * GCLK_CLKCTRL_CLKEN;
   */
}
