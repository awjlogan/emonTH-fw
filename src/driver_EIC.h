#pragma once

#include <stdbool.h>

typedef struct EIC_Cfg_ {
  int ch;
  int sense;
  int pin;
  void (*cb)();
} EIC_Cfg_t;

/*! @brief Set the callback function for an EIC channel
 *  @param [in] ch : EIC channel
 *  @param [in] cb : callback function pointer
 */
void eicCallbackSet(const int ch, void (*cb)());

/*! @brief Disable EIC level line
 *  @param [in] ch : EIC channel to disable
 */
void eicChannelDisable(const int ch);

/*! @brief Enable EIC level line with optional callback
 *  @param [in] eiccfg : EIC channel configuration
 */
void eicChannelEnable(const EIC_Cfg_t eiccfg);

/*! @brief Disable EIC */
void eicDisable(void);

/*! @brief Enable EIC */
void eicEnable(void);

/*! @brief Setup the External Interrupt Controller */
void eicSetup(void);

/*! @brief Setup a pin for EIC function
 *  @param [in] pin : (logical) pin to configure
 */
void eicPinSetup(const int pin);

/*! @brief Setup the External Interrupt Controller for the pulse controller  */
void eicSetupPulse(void);
