#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_def.h"
#include "periph_HDC2010.h"

_Static_assert((sizeof(bool) == 1), "bool must be 1 byte");

#define WAKE_PERIOD_DEF 10u

/*********************************
 * Firmware version
 *********************************/

#define VERSION_FW_MAJ 0u
#define VERSION_FW_MIN 1u
#define VERSION_FW_REV 0u

/*********************************
 * Device configuration
 *********************************/

#define NETWORK_GROUP_DEF 210u /* Must match emonBase group */
#define NODE_ID_DEF       27u  /* Default node ID */

typedef struct EmonTHCfg_ {
  uint8_t  RF_Freq;
  uint8_t  networkGroup;
  uint8_t  nodeID;
  bool     idFromNVM;
  int8_t   txType;
  uint8_t  rfPower;
  bool     pulseEnabled;
  uint8_t  pulsePeriod;
  uint8_t  extTempEnabled;
  uint64_t oneWireAddress[TEMP_MAX_ONEWIRE];
} EmonTHCfg_t;

_Static_assert(sizeof(EmonTHCfg_t) < 57, "EmonThCfg_t bigger than 56 bytes");

/*********************************
 * Remaining
 *********************************/

#define TX_BUFFER_W 128u

typedef struct EmonTHDataset_ {
  HDCResultRaw_t hdcResRaw;
  int16_t        tempExternal[TEMP_MAX_ONEWIRE];
  int16_t        battery;
  uint32_t       pulseCnt;
} EmonTHDataset_t;

/* This struct must match the OEM definitions found at:
 * https://docs.openenergymonitor.org/electricity-monitoring/networking/sending-data-between-nodes-rfm.html
 */
typedef struct __attribute__((__packed__)) PackedData_ {
  int16_t  tempInternal;
  int16_t  tempExternal[TEMP_MAX_ONEWIRE];
  int16_t  humidityInternal;
  uint16_t battery;
  uint32_t pulse;
} PackedData_t;

/* Maximum size of RFM69CW buffer is 61 bytes. Node, number, and CRC included.
 */
_Static_assert((sizeof(PackedData_t) + 4) < 62, "PackedData_t > 62 bytes");

/* EVTSRC_t contains all the event/interrupts sources. This value is shifted
 * to provide a vector of set events as bits.
 */
typedef enum EVTSRC_ {
  EVT_DMA             = 0u,
  EVT_TICK_1kHz       = 1u,
  EVT_WAKE_SAMPLE_INT = 2u,
  EVT_UART            = 3u,
  EVT_ADC             = 4u,
  EVT_DMAC_UART_CMPL  = 5u,
  EVT_WAKE_TIMER      = 6u,
  EVT_SAVE_RESET      = 7u,
  EVT_WAKE_SAMPLE_EXT = 8u,
  EVT_TIMER_MC        = 9u,
  EVT_EIC_PULSE       = 10u,
  EVT_TH_SAMPLE_RD    = 12u,
  EVT_SAMPLE_PROCESS  = 13u,
  EVT_ONEWIRE_SAMPLE  = 14u,
  EVT_ONEWIRE_READ    = 15u,
  EVT_ENTER_CONFIG    = 19u,
  EVT_SEND_DATA_RFM   = 20u,
  EVT_SEND_DATA_UART  = 21u
} EVTSRC_t;

/*! @brief Clear a pending event/interrupt flag after the task has been handled
 *  @param [in] Event source in enum
 */
void emonTHEventClr(const EVTSRC_t evt);

/*! @brief Set the pending event/interrupt flag for tasks that are not handled
 *         within an ISR
 *  @param [in] evt : Event source in enum
 */
void emonTHEventSet(const EVTSRC_t evt);

void emonTHInteractiveUartSet(void);

/*! @brief Blocking write of a string to UART.
 *  @param [in] s: pointer to null terminated string
 */
void uartPuts(const char *s);
