#include <stdint.h>

#include "board_def.h"

/* Each pin are defined in {GROUP, PIN} pairs. Pin numberings are logical,
 * not physical. Each collection of pins is terminated with a {0xFF, 0} pair.
 */

const uint8_t pinsGPIO_Out[][2] = {{GRP_PINA, PIN_SPI_RFM_SS}, {0xFF, 0}};

const uint8_t pinsGPIO_In[][2] = {
    {GRP_PULSE, PIN_PULSE1}, {GRP_ONEWIRE, PIN_ONEWIRE}, {0xFF, 0}};

const uint8_t pinsUnused[][2] = {{GRP_PINA, 0}, {GRP_PINA, 1}, {0xFF, 0}};
