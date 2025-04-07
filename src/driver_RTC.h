#pragma once

#include "emonTH_saml.h"

/*! @brief Enable the RTC counter with interrupt on overflow */
void rtcEnable(int period);

/*! @brief Setup the RTC module */
void rtcSetup(void);
