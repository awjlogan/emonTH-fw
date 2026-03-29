#pragma once

/*! @brief Disable EIC */
void eicDisable(void);

/*! @brief Enable EIC */
void eicEnable(void);

/*! @brief Setup the External Interrupt Controller */
void eicSetup(void);

/*! @brief Setup the EIC for the HDC interrupt
 *  @param [in] cb : callback function for pulse edge
 */
void eicSetupHDC(void (*cb)());

/*! @brief Setup the EIC for the pulse controller
 *  @param [in] cb : callback function for pulse edge
 */
void eicSetupPulse(void (*cb)());
