#include <stdint.h>

#include "board_def.h"

/* Each pin are defined in {GROUP, PIN} pairs. Pin numberings are logical,
 * not physical. Each collection of pins is terminated with a {0xFF, 0} pair.
 */

const uint8_t pinsGPIO_Out[] = {PIN_ONEWIRE_PWR, PIN_SPI_RFM_SS, 0xFF};

const uint8_t pinsGPIO_In[] = {PIN_SW_NODE0, PIN_SW_NODE1, PIN_PULSE,
                               PIN_ONEWIRE, 0xFF};

const uint8_t pinsUnused[] = {0, 1, 0xFF};
