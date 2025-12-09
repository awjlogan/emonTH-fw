#pragma once

#include "emonTH.h"
#include "emonTH_saml.h"

typedef struct RTC_Evt_ {
  int      smpInterval; /* Sample interval (s) */
  EVTSRC_t evt;         /* Event ID */
} RTC_Evt_t;

/*! @brief Enable the RTC counter with interrupt on overflow */
void rtcEnable(int period);

/*! @brief Register a periodic event with the RTC
 *  @param [in] rtcevt : event to be registered
 */
void rtcEvtReg(const RTC_Evt_t rtcevt);

/*! @brief Setup the RTC module */
void rtcSetup(void);
