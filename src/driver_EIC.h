#pragma once

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
