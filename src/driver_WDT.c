#include "emonTH_samd.h"

#include "driver_WDT.h"

void wdtSetup(WDT_PER_t per) {
  /* OSCULP32 is enabled and connected to Generator 2, undivided
   * Connect Gen 2 -> WDT
   */
  WDT->CONFIG.reg |= per;

  /* Enable and synchronise (18.6.5) */
  WDT->CTRLA.reg |= WDT_CTRLA_ENABLE;
  while (WDT->SYNCBUSY.reg)
    ;
}

void wdtFeed(void) {
  /* Write key (18.6.2.4) and synchronise (18.6.5) */
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->SYNCBUSY.reg)
    ;
}
