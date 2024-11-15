#include "driver_SAMD.h"
#include "emonTH_samd.h"
#include "fuses.h"

void clkSetup(void) {
  /* Boost OSC8M to 8 MHz from initial 1 MHz */
  SYSCTRL->OSC8M.bit.PRESC = 0;
}
