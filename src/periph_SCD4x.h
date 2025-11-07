#pragma once

/*! @brief Discover any SCD4x devices
 *  @param [in] altitude : height above sea level
 */
void scd4xDiscover(const uint16_t altitude);

/*! @brief Measure CO2 level in single shot mode
 *  @return Measured CO2 level (ppm)
 */
uint16_t scd4xMeasureCO2_LP(void);

/*! @brief Indicate if there is an SCD4x present
 *  @return true if present, false otherwise
 */
bool scd4xPresent(void);
