#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct EIC_Cfg_ {
  size_t  ch;    /* Channel index */
  uint8_t sense; /* Sense direction */
  size_t  pin;   /* Pin to attach */
  void (*cb)();  /* Callback on interrupt */
} EIC_Cfg_t;

/*! @brief Set the callback function for an EIC channel.
 *         Callback runs in EIC ISR context and must be ISR-safe.
 *  @param [in] ch : EIC channel
 *  @param [in] cb : callback function pointer
 */
void eicCallbackSet(const size_t ch, void (*cb)());

/*! @brief Disable EIC level line
 *  @param [in] ch : EIC channel to disable
 */
void eicChannelDisable(const size_t ch);

/*! @brief Enable EIC level line with optional callback.
 *  @param [in] eiccfg : EIC channel configuration
 */
void eicChannelEnable(const EIC_Cfg_t eiccfg);

/*! @brief Disable EIC */
void eicDisable(void);

/*! @brief Enable EIC */
void eicEnable(void);

/*! @brief Setup the External Interrupt Controller */
void eicSetup(void);

/*! @brief Setup a pin for EIC function.
 *  @param [in] pin : logical pin to configure (PORT group 0 index)
 */
void eicPinSetup(const size_t pin);

/*! @brief Setup the External Interrupt Controller for the pulse controller  */
void eicSetupPulse(void);
