#pragma once

#include <stdbool.h>

/* @brief Clear the overflow interrupt */
void rtcIntOvfClr(void);

/* @brief Get the status of the overflow interrupt
 * @return : true if set, false otherwise
 */
bool rtcIntOvfStat(void);

/* @brief  the RTC module
 * @param [in] wakePeriod_s : the wake up period is seconds
 */
void rtcSetup(int wakePeriod_s);
