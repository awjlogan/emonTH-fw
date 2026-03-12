#pragma once

#include <stddef.h>
#include <stdint.h>

#include "emonTH_saml.h"

/* Types */
typedef struct Pin_ {
  uint16_t pin;
} Pin_t;

typedef enum PINDIR_ { PIN_DIR_IN, PIN_DIR_OUT } PINDIR_t;

typedef enum PINCFG_ { PIN_CFG_SET, PIN_CFG_CLR } PINCFG_t;

typedef enum PINDRV_ { PIN_DRV_CLR, PIN_DRV_SET, PIN_DRV_TGL } PINDRV_t;

/*! @note Group corresponds to the A, B, .. mapping. A -> 0, B -> 1, ..
 *        This driver targets PORT group 0 (A); pin numbers are group-local.
 */

/*! @brief Sets the pin configuration
 *  @param [in] pin : pin number within PORT group 0 (A)
 *  @param [in] cfg : configuration option
 *  @param [in] cs  : clear or set configuration
 */
void portPinCfg(size_t pin, unsigned int cfg, PINCFG_t cs);

/*! @brief Sets a pin as input or output
 *  @param [in] pin : pin number within PORT group 0 (A)
 *  @param [in] mode: PIN_DIR_IN for input, PIN_DIR_OUT for output
 */
void portPinDir(size_t pin, PINDIR_t mode);

/*! @brief Sets the pin driver value
 *  @param [in] pin : pin number within PORT group 0 (A)
 *  @param [in] drv : Clear, set, or toggle pin
 */
void portPinDrv(size_t pin, PINDRV_t drv);

/*! @brief Sets the mux for pin alternate function
 *  @param [in] pin : pin number within PORT group 0 (A)
 *  @param [in] mux : Mux mode
 */
void portPinMux(size_t pin, unsigned int mux);

/*! @brief Clear the mux for pin alternate function
 *  @param [in] pin : pin number within PORT group 0 (A)
 */
void portPinMuxClear(size_t pin);

/*! @brief Returns the pin value
 *  @param [in] pin : pin number within PORT group 0 (A)
 *  @return current pin value
 */
unsigned int portPinValue(size_t pin);

/*! @brief   Configure the ports.
 *           Ports for peripherals are configured in their setup functions
 */
void portSetup(void);
