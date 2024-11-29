#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_def.h"

/* Configurable options. All the structs are packed to allow simple write to
 * EEPROM as a contiguous set.
 */
typedef enum DataTx_ {
  DATATX_RFM69 = 0,
  DATATX_UART  = 1,
  DATATX_BOTH  = 2
} TxType_t;

typedef struct __attribute__((__packed__)) BaseCfg_ {
  uint8_t nodeID;      /* ID for report*/
  bool    nodeIDSaved; /* Use the saved ID, rather than the switch value */
  uint8_t dataGrp;     /* Transmission group - default 210 */
  bool    useJson;     /* JSON format for serial output */
  bool    extTempEn;   /* External temperature sensor enabled */
  uint8_t res0[3];
} BaseCfg_t;

typedef struct __attribute__((__packed__)) DataTxCfg_ {
  uint8_t txType;  /* UART, RFM on SPI, or both */
  uint8_t rfmFreq; /* 0: 868 MHz, 1: 915 MHz, 2: 433 MHz. */
  uint8_t rfmPwr;
  uint8_t res0;
} DataTxCfg_t;

typedef struct __attribute__((__packed__)) PulseCfgPacked_ {
  uint8_t timeMask;
  bool    active;
  uint8_t res0[2];
} PulseCfgPacked_t;

typedef struct __attribute__((__packed__)) EmonTHConfigPacked_ {
  BaseCfg_t        baseCfg;
  DataTxCfg_t      dataTxCfg;
  PulseCfgPacked_t pulseCfg;
  uint64_t         oneWireAddress[TEMP_MAX_ONEWIRE];
} EmonTHConfigPacked_t;

_Static_assert(sizeof(EmonTHConfigPacked_t) < 57,
               "EmonTHConfigPacked_t bigger than 56 bytes");

/*! @brief Add a character to the command stream
 *  @param [in] c : character to add
 */
void configCmdChar(const uint8_t c);

/*! @brief Enter the configuration mode */
void configEnter(void);

/*! @brief Print the board and firmware information to serial */
void configFirmwareBoardInfo(void);

/*! @brief This functions loads the default configuration and from NVM
 */
EmonTHConfigPacked_t *configLoadFromNVM(void);

/*! @brief Return one word from the SAML's unique ID
 *  @param[in] idx : index of the word to fetch
 *  @return word idx from the unique ID
 */
uint32_t getUniqueID(int idx);
