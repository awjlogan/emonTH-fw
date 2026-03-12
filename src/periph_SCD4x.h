#pragma once

/*! @brief Discover any SCD4x devices and apply configuration.
 *         May power-cycle the sensor during discovery.
 *  @param [in] altitude : height above sea level (meters, written to device)
 */
void scd4xDiscover(const uint16_t altitude);

/*! @brief Measure CO2 level.
 *         Blocking; may take multiple seconds depending on device variant.
 *  @return measured CO2 level (ppm)
 */
uint16_t scd4xMeasureCO2(void);

/*! @brief Indicate if there is an SCD4x present.
 *         Valid after scd4xDiscover() has been called.
 *  @return true if present, false otherwise
 */
bool scd4xPresent(void);
