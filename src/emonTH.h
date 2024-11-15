#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_def.h"

_Static_assert((sizeof(bool) == 1), "bool must be 1 byte");

/*********************************
 * Firmware version
 *********************************/

#define VERSION_FW_MAJ 0u
#define VERSION_FW_MIN 0u
#define VERSION_FW_REV 1u

/*********************************
 * Device configuration
 *********************************/

#define NETWORK_GROUP 210u /* Must match emonBase group */
#define NODE_ID       17u  /* Node ID for reports */

typedef struct EmonTHCfg_ {
  uint8_t  RF_Freq;
  uint8_t  networkGroup;
  uint8_t  nodeID;
  bool     rfEnabled;
  uint8_t  rfPower;
  bool     pulseEnabled;
  uint8_t  pulsePeriod;
  uint8_t  extTempEnabled;
  uint64_t oneWireAddress[TEMP_MAX_ONEWIRE];
} EmonTHCfg_t;

_Static_assert(sizeof(EmonTHCfg_t) < 65, "EmonThCfg_t bigger than 64 bytes");

/*********************************
 * Remaining
 *********************************/

#define TX_BUFFER_W 512u

/* Configuration key - indicates that the configuration is the default or
 * has been retrieved from non-volatile storage */
#define CONFIG_NVM_KEY 0xca55e77eul

typedef struct Emon32Dataset_ {
  uint32_t     msgNum;
  uint32_t     pulseCnt[NUM_PULSECOUNT];
  float        temp[TEMP_MAX_ONEWIRE];
  unsigned int numTempSensors;
} EmonTHDataset_t;

/* This struct must match the OEM definitions found at:
 * https://docs.openenergymonitor.org/electricity-monitoring/networking/sending-data-between-nodes-rfm.html
 */
typedef struct __attribute__((__packed__)) PackedData_ {
  uint32_t msg;
  int16_t  T[TEMP_MAX_ONEWIRE];
  uint32_t pulse[NUM_PULSECOUNT];
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
  EVT_TCC             = 2u,
  EVT_UART            = 3u,
  EVT_ADC             = 4u,
  EVT_DMAC_UART_CMPL  = 5u,
  EVT_DMAC_SMP_CMPL   = 6u,
  EVT_ECM_CYCLE_CMPL  = 7u,
  EVT_ECM_SET_CMPL    = 8u,
  EVT_SAVE_RESET      = 9u,
  EVT_DMAC_I2C_CMPL   = 10u,
  EVT_TIMER_MC        = 11u,
  EVT_EIC_PULSE       = 12u,
  EVT_EEPROM_TMR      = 13u,
  EVT_TEMP_SAMPLE     = 14u,
  EVT_TEMP_READ       = 15u,
  EVT_CONFIG_CHANGED  = 16u,
  EVT_CONFIG_SAVED    = 17u,
  EVT_SAFE_RESET_REQ  = 18u,
  EVT_PROCESS_CMD     = 19u,
  EVT_PROCESS_DATASET = 20u,
  EVT_EEPROM_STORE    = 21u,
  EVT_CLEAR_ACCUM     = 22u,
  EVT_ECM_PEND_1S     = 23u,
  EVT_ECM_TRIG        = 24
} EVTSRC_t;

/*! @brief Output a string to the debug destination. If the USB CDC is connected
 *         this is the destination, otherwise through hardware UART.
 *  @param [in] s: pointer to null terminated string
 */
void dbgPuts(const char *s);

/*! @brief Clear a pending event/interrupt flag after the task has been handled
 *  @param [in] Event source in enum
 */
void emonTHEventClr(const EVTSRC_t evt);

/*! @brief Set the pending event/interrupt flag for tasks that are not handled
 *         within an ISR
 *  @param [in] evt : Event source in enum
 */
void emonTHEventSet(const EVTSRC_t evt);
