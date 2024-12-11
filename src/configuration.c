#include <inttypes.h>
#include <string.h>

#include "emonTH_assert.h"

#include "driver_ADC.h"
#include "driver_NVM.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"

#include "configuration.h"
#include "emonTH.h"
#include "emonTH_build_info.h"
#include "util.h"

/*************************************
 * Types
 *************************************/

typedef enum {
  RCAUSE_SYST  = 0x40,
  RCAUSE_WDT   = 0x20,
  RCAUSE_EXT   = 0x10,
  RCAUSE_BOD33 = 0x04,
  RCAUSE_BOD12 = 0x02,
  RCAUSE_POR   = 0x01
} RCAUSE_t;

typedef struct __attribute__((__packed__)) NVMHeader_ {
  uint32_t watermark;
  uint16_t crc16;
  uint8_t  writeCount;
  uint8_t  res0;
} NVMHeader_t;

/* Configuration key - indicates that the configuration is the default or
 * has been retrieved NVM. */
#define CONFIG_NVM_KEY 0xca55e77eul

/*************************************
 * Prototypes
 *************************************/

static void  configDefault(void);
static void  configurePulse(void);
static bool  configProcessCmd(void);
static void  configSaveToNVM(void);
static char *getLastReset(void);
static void  inBufferClear(int n);
static void  printSettings(void);
static void  putInt(const int i);

/*************************************
 * Local variables
 *************************************/

#define IN_BUFFER_W 16
static char                 inBuffer[IN_BUFFER_W];
static int                  inBufferIdx                 = 0;
static bool                 inConfig                    = false;
static bool                 cmdPending                  = false;
static EmonTHConfigPacked_t config                      = {0};
uint8_t                     pageBuffer[FLASH_PAGE_SIZE] = {0};
static bool                 resetReq                    = false;
static bool                 unsavedChange               = false;

/*! @brief Set all configuration values to defaults */
static void configDefault(void) {
  config.baseCfg.nodeID  = NODE_ID_DEF;       // Node ID
  config.baseCfg.dataGrp = NETWORK_GROUP_DEF; // Group for OEM
  config.baseCfg.useJson = false;             // Not JSON format for serial

  config.dataTxCfg.txType  = (uint8_t)DATATX_RFM69; // RFM only
  config.dataTxCfg.rfmPwr  = 0x19;                  // Maximum power
  config.dataTxCfg.rfmFreq = 2;                     // 433 MHz

  config.pulseCfg.active   = false; // Pulse channel inactive
  config.pulseCfg.timeMask = 100u;  // 100 ms minimum between pulses
}

static void configurePulse(void) {
  /* String format in inBuffer:
   *      [1] -> active;
   *      [3] -> NULL: blank time
   */
  ConvInt_t convI;
  bool      active   = 0;
  int       timeMask = 0;

  convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return;
  }
  active = (bool)convI.val;

  convI = utilAtoi(inBuffer + 3, ITOA_BASE10);
  if (!convI.valid) {
    return;
  }
  timeMask = convI.val;

  /* If inactive, clear active flag, no decode for the rest */
  if (!active) {
    config.pulseCfg.active = false;
    return;
  } else {
    config.pulseCfg.active   = true;
    config.pulseCfg.timeMask = timeMask;
  }
}

/*! @brief Get the last reset cause (16.8.14)
 *  @return : null-terminated string with the last cause.
 */
static char *getLastReset(void) {
  const RCAUSE_t lastReset = (RCAUSE_t)RSTC->RCAUSE.reg;
  switch (lastReset) {
  case RCAUSE_SYST:
    return "Reset request";
    break;
  case RCAUSE_WDT:
    return "Watchdog timeout";
    break;
  case RCAUSE_EXT:
    return "External reset";
    break;
  case RCAUSE_BOD33:
    return "3V3 brownout";
    break;
  case RCAUSE_BOD12:
    return "1V2 brownout";
    break;
  case RCAUSE_POR:
    return "Power on cold reset";
    break;
  }
  return "Unknown";
}

/*! @brief Fetch the SAML's 128bit unique ID
 *  @param [in] idx : index of 32bit word
 *  @return : 32bit word from index
 */
uint32_t getUniqueID(int idx) {
  /* Section 10.3.3 Serial Number */
  const uint32_t id_addr_lut[4] = {0x0080A00C, 0x0080A040, 0x0080A044,
                                   0x0080A048};
  return *(volatile uint32_t *)id_addr_lut[idx];
}

static void inBufferClear(int n) {
  inBufferIdx = 0;
  (void)memset(inBuffer, 0, n);
}

static void printSettings(void) {
  uartPuts("\r\n\r\n==== Settings ====\r\n\r\n");
  uartPuts("Data transmission:         ");
  TxType_t tx = (TxType_t)config.dataTxCfg.txType;
  if ((DATATX_RFM69 == tx) || (DATATX_BOTH == tx)) {
    uartPuts("RFM69, ");
    switch (config.dataTxCfg.rfmFreq) {
    case 0:
      uartPuts("868");
      break;
    case 1:
      uartPuts("915");
      break;
    case 2:
      uartPuts("433");
      break;
    }
    uartPuts(" MHz @ ");
    putInt(config.dataTxCfg.rfmPwr);
    uartPuts("\r\n");
  } else if ((DATATX_UART == tx) || (DATATX_BOTH == tx)) {
    uartPuts("Serial\r\n");
  }
  uartPuts("\r\n");

  uartPuts("Pulse channel ");
  if (config.pulseCfg.active) {
    uartPuts("  - Hysteresis (ms): ");
    putInt(config.pulseCfg.timeMask);
    uartPuts("\r\n");
  } else {
    uartPuts("disabled.");
  }
  uartPuts("\r\n\r\n");

  if (unsavedChange) {
    uartPuts("There are unsaved changes. Command \"s\" to save.\r\n\r\n");
  }
}

static void putInt(const int i) {
  char strBuffer[8];
  (void)utilItoa(strBuffer, i, ITOA_BASE10);
  uartPuts(strBuffer);
}

void configCmdChar(const uint8_t c) {
  if (('\r' == c) || ('\n' == c)) {
    if (inConfig) {
      if (!cmdPending) {
        uartPuts("\r\n");
        cmdPending = true;
      }
    }
  } else if (('\b' == c) && inConfig) {
    uartPuts("\b \b");
    if (0 != inBufferIdx) {
      inBufferIdx--;
      inBuffer[inBufferIdx] = 0;
    }
  } else if ((inBufferIdx < IN_BUFFER_W) && utilCharPrintable(c)) {
    inBuffer[inBufferIdx++] = c;
  } else {
    inBufferClear(IN_BUFFER_W);
    uartPuts("\r\n");
  }
}

void configEnter(void) {
  bool configExit = false;
  inConfig        = true;

  portPinDrv(PIN_LED, PIN_DRV_SET);
  while (!configExit) {
    if (cmdPending) {
      configExit = configProcessCmd();
      __WFI();
    }
  }
  if (resetReq) {
    NVIC_SystemReset();
  }
  portPinDrv(PIN_LED, PIN_DRV_CLR);
}

void configFirmwareBoardInfo(void) {
  uartPuts("\033c==== emonTH ====\r\n\r\n");

  uartPuts("> Board:\r\n");
  uartPuts("  - emonTH3\r\n");
  uartPuts("  - Serial:     ");
  for (int i = 0; i < 4; i++) {
    putInt(getUniqueID(i));
  }
  uartPuts("  - Last reset: ");
  uartPuts(getLastReset());
  uartPuts("\r\n");

  uartPuts("> Firmware:\r\n");
  uartPuts("  - Version: ");
  putInt(VERSION_FW_MAJ);
  uartPuts(".");
  putInt(VERSION_FW_MIN);
  uartPuts(".");
  putInt(VERSION_FW_REV);
  uartPuts("\r\n");
  uartPuts("  - Build:      ");
  uartPuts(emonTH_build_info_string());
  uartPuts("\r\n\r\n");
  uartPuts("  - Distributed under GPL3 license, see COPYING.md\r\n");
  uartPuts("  - emonTH Copyright (C) 2024-25 Angus Logan\r\n");
  uartPuts("  - For Bear and Moose\r\n\r\n");
}

EmonTHConfigPacked_t *configLoadFromNVM(void) {
  NVMHeader_t *header   = (NVMHeader_t *)pageBuffer;
  bool         keyFound = false;
  bool         crc0Bad  = false;
  bool         crc1Bad  = false;

  EmonTHConfigPacked_t *pCfg =
      (EmonTHConfigPacked_t *)(pageBuffer + sizeof(*header));
  uint16_t crc16Calc = 0;

  nvmDataFlashRead(0, (uint32_t *)pageBuffer);

  keyFound = (CONFIG_NVM_KEY == header->watermark);

  crc16Calc = calcCRC16_ccitt(pCfg, sizeof(*pCfg));
  if (header->crc16 != crc16Calc) {
    crc0Bad = true;
    nvmDataFlashRead(1, (uint32_t *)pageBuffer);
    crc16Calc = calcCRC16_ccitt(pCfg, sizeof(*pCfg));
    if (header->crc16 != crc16Calc) {
      crc1Bad = true;
    }
  }

  /* If the watermark is not there, or both CRCs are bad then rewrite */
  if (!keyFound || (crc0Bad && crc1Bad)) {
    configDefault();
    header->watermark  = CONFIG_NVM_KEY;
    header->writeCount = 2;
    header->crc16      = calcCRC16_ccitt(&config, sizeof(config));
    memcpy(pageBuffer + sizeof(*header), &config, sizeof(config));

    // Write out page buffer to page 0 and page 1 as backup
    nvmDataFlashWrite(0, (uint32_t *)pageBuffer);
    nvmDataFlashWrite(1, (uint32_t *)pageBuffer);

    // Read back page 0 to continue check
  }

  return &config;
}

static bool configProcessCmd(void) {
  bool         exitConfig = false;
  unsigned int arglen     = 0;
  bool         termFound  = false;
  ConvInt_t    convI      = {false, 0};

  /* Help text - serves as documentation interally as well */
  const char helpText[] =
      "\r\n"
      "emonTH information and configuration commands\r\n\r\n"
      " - ?           : show this text again\r\n"
      " - b<n>        : set RF band. n = 4: 433 MHz, 8: 868 MHz, 9: 915 MHz\r\n"
      " - e<n>        : maximum number of external temperature sensors\r\n"
      " - g<n>        : set network group (default = 210)\r\n"
      " - j<n>        : JSON serial format. n = 0: OFF, n = 1: ON\r\n"
      " - l           : list settings\r\n"
      " - m <x> <y>   : Pulse counting.\r\n"
      "     - x = 0: OFF, x = 1, ON.\r\n"
      "     - y : minimum period (ms). Ignored if x = 0\r\n"
      " - n<n>        : set node ID [1..60].\r\n"
      " - o<n>        : node ID select. n = 0: slide switches, n = 1: saved\r\n"
      " - p<n>        : set the RF power level\r\n"
      " - r           : restore defaults\r\n"
      " - s           : save settings to NVM\r\n"
      " - t0 <n>      : enable external temperature sensing. n = 0: OFF, n = "
      "1: ON\r\n"
      " - t<x> <yy> <yy> <yy> <yy> <yy> <yy> <yy> <yy>\r\n"
      "   : change an external sensor's position\r\n"
      "     - x: position of sensor in the list (1-based)\r\n"
      "     - yy : hexadecimal bytes, e.g. 28 81 43 31 07 00 00 D9\r\n"
      " - v           : firmware and board information\r\n"
      " - w<n>        : enable wireless and serial.\r\n"
      "                   n = 1: wireless only, n = 2: serial only, n = 3: "
      "both\r\n"
      " - x           : exit, lock, and continue\r\n";

  /* Convert \r or \n to 0, and get the length until then. */
  while (!termFound && (arglen < IN_BUFFER_W)) {
    if (0 == inBuffer[arglen]) {
      termFound = true;
      break;
    }
    arglen++;
  }

  if (!termFound) {
    return false;
  }

  /* Decode on first character in the buffer */
  switch (inBuffer[0]) {
  case '?':
    /* Print help text */
    uartPuts(helpText);
    break;
  case 'j':
    if (2u == arglen) {
      convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
      if (!convI.valid) {
        break;
      }

      config.baseCfg.useJson = (bool)convI.val;
      unsavedChange          = true;
    }
    break;
  case 'l':
    printSettings();
    break;
  case 'm':
    configurePulse();
    unsavedChange = true;
    break;
  case 'p':
    /* Configure RF power */
    resetReq      = true;
    unsavedChange = true;
    break;
  case 'r':
    configDefault();
    uartPuts("> Restored default values.\r\n");

    unsavedChange = true;
    resetReq      = true;
    break;
  case 's':
    /* Save to EEPROM config space after recalculating CRC */
    configSaveToNVM();
    unsavedChange = false;
    break;
  case 'x':
    exitConfig = true;
    break;
  }

  cmdPending = false;
  inBufferClear(arglen + 1);
  return exitConfig;
}

void configSaveToNVM(void) {
  NVMHeader_t *header = (NVMHeader_t *)pageBuffer;

  header->crc16     = calcCRC16_ccitt(&config, sizeof(EmonTHConfigPacked_t));
  header->watermark = CONFIG_NVM_KEY;
  header->writeCount += 2;

  memcpy(pageBuffer + sizeof(*header), &config, sizeof(config));
  nvmDataFlashWrite(0, (uint32_t *)pageBuffer);
  nvmDataFlashWrite(1, (uint32_t *)pageBuffer);
}

/* =======================
 * UART Interrupt handler
 * ======================= */

void SERCOM_UART_HANDLER {
  /* Echo the received character to the TX channel, and send to the command
   * stream.
   */
  if (uartGetcReady()) {
    uint8_t rx_char = uartGetc();
    configCmdChar(rx_char);

    if (utilCharPrintable(rx_char) && !cmdPending) {
      uartPutcBlocking(rx_char);
    }
  }
}
