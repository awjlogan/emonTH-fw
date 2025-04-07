#include <stdint.h>

#include "board_def.h"

/* Pin numberings are logical, not physical. Each collection of pins is
 * terminated with a {0xFF, 0} pair.
 */

const uint8_t pinsGPIO_Out[] = {
    PIN_ONEWIRE_PWR, PIN_SPI_RFM_SS, PIN_REG_EN, PIN_EXT_EN,
    PIN_GPIO0,       PIN_LED,        PIN_GPIO1,  0xFF};

const uint8_t pinsGPIO_In[] = {PIN_SW_NODE0, PIN_SW_NODE1, PIN_PULSE,
                               PIN_ONEWIRE,  PIN_HDC_DRDY, 0xFF};

const uint8_t pinsUnused[] = {1, 18, 8, 9, 0xFF};
