#pragma once

#include <stdbool.h>

/*! @brief Set the callback function for an EIC channel
 *  @param [in] ch : EIC channel
 *  @param [in] cb : callback function pointer
 */
void eicCallbackSet(const int ch, void (*cb)());

/*! @brief Disable EIC level line
 *  @param [in] ch : EIC channel to disable
 */
void eicLevelDisable(const int ch);

/*! @brief Enable EIC level line with optional callback
 *  @param [in] ch : EIC channel to enable
 *  @param [in] cb : call back function
 */
void eicLevelEnable(const int ch, void (*cb)());

/*! @brief Setup the External Interrupt Controller */
void eicSetup(void);

/*! @brief Close the EIC controller configuration, runs async until required */
void eicSetupClose(void);

/*! @brief Setup the External Interrupt Controller for the pulse controller  */
void eicSetupPulse(void);
