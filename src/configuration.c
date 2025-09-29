#include <inttypes.h>
#include <string.h>

#include "emonTH_assert.h"

#include "driver_ADC.h"
#include "driver_DMAC.h"
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

/*************************************
 * Prototypes
 *************************************/

static bool  configDatalog(void);
static void  configDefault(void);
static bool  configExtTempMax(void);
static bool  configJSON(void);
static bool  configOneWire(void);
static bool  configProcessCmd(void);
static bool  configPulse(void);
static bool  configRF433(void);
static bool  configRFM(void);
static bool  configRFPower(void);
static void  configSaveToNVM(void);
static bool  configUART(void);
static char *getLastReset(void);
static void  inBufferClear(int n);
static void  printSettings(void);
static void  putInt(const int i);
static void  putUniqueID(void);

/*************************************
 * Local variables
 *************************************/

#define IN_BUFFER_W 16
static char                 inBuffer[IN_BUFFER_W];
static int                  inBufferIdx   = 0;
static bool                 cmdPending    = false;
static EmonTHConfigPacked_t config        = {0};
static bool                 unsavedChange = false;

/* The NVM page buffer must be 4 byte aligned for allow access from DFLASH */
// uint8_t pageBuffer[FLASH_PAGE_SIZE] __attribute__((aligned(16))) = {0};

static bool configDatalog(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  if (convI.val < 5) {
    return false;
  }
  config.baseCfg.reportTime = convI.val;
  return true;
}

/*! @brief Set all configuration values to defaults */
static void configDefault(void) {
  config.baseCfg.nodeID     = NODE_ID_DEF;       // Node ID
  config.baseCfg.dataGrp    = NETWORK_GROUP_DEF; // Group for OEM
  config.baseCfg.reportTime = WAKE_PERIOD_DEF;   // Time between reports
  config.baseCfg.useJson    = true;              // JSON format for serial
  config.baseCfg.extTempEn  = TEMP_NUM_DEF;      // Max num external sensors

  config.dataTxCfg.txType  = (uint8_t)DATATX_RFM69; // RFM only
  config.dataTxCfg.rfmPwr  = 0x18;                  // +12 dBm
  config.dataTxCfg.rfmFreq = 3;                     // 433.92 MHz

  config.pulseCfg.active   = false; // Pulse channel inactive
  config.pulseCfg.timeMask = 100u;  // 100 ms minimum between pulses
}

static bool configExtTempMax(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }

  /* Must be 0, 1 or 4 */
  if ((0 != convI.val) && (1 != convI.val) && (4 != convI.val)) {
    return false;
  }

  config.baseCfg.extTempEn = convI.val;
  return true;
}

static bool configJSON(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }

  config.baseCfg.useJson = (bool)convI.val;
  return true;
}

static bool configOneWire(void) {

  for (int i = 0; i < IN_BUFFER_W; i++) {
    if (0 == inBuffer[i]) {
      break;
    } else if (' ' == inBuffer[i]) {
      inBuffer[i] = 0;
    }
  }

  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);

  if (!convI.valid) {
    return false;
  }

  return false;
}

static bool configNodeID(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  if ((convI.val < 1) || (convI.val > 60)) {
    return false;
  }

  config.baseCfg.nodeID = convI.val;
  return true;
}

static bool configPulse(void) {
  /* String format in inBuffer:
   *      [1] -> active;
   *      [3] -> NULL: blank time
   */
  ConvInt_t convI;
  bool      active   = 0;
  int       timeMask = 0;

  convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  active = (bool)convI.val;

  convI = utilAtoi(inBuffer + 3, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  timeMask = convI.val;

  /* If inactive, clear active flag, no decode for the rest */
  if (!active) {
    config.pulseCfg.active = false;
  } else {
    config.pulseCfg.active   = true;
    config.pulseCfg.timeMask = timeMask;
  }
  return true;
}

static bool configRF433(void) {
  int val = inBuffer[1] - '0';

  if (!((0 == val) || (1 == val))) {
    return false;
  }

  /* Only applies to 433 MHz ISM band */
  if (!((config.dataTxCfg.rfmFreq == 2) || (config.dataTxCfg.rfmFreq == 3))) {
    return false;
  }

  if (val) {
    config.dataTxCfg.rfmFreq = 2;
    uartPuts("> 433.00 MHz (backwards compatible, illegal).\r\n");
  } else {
    config.dataTxCfg.rfmFreq = 3;
    uartPuts("> 433.92 MHz (check Rx frequency).\r\n");
  }
  return true;
}

static bool configRFM(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  if ((0 != convI.val) && (1 != convI.val)) {
    return false;
  }
  if (convI.val) {
    config.dataTxCfg.txType |= (1 << 0);
  } else {
    config.dataTxCfg.txType &= ~(1 << 0);
  }
  return true;
}

static bool configRFPower(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  if ((convI.val < 0) || (convI.val > 31)) {
    return false;
  }

  config.dataTxCfg.rfmPwr = convI.val;
  return true;
}

static bool configUART(void) {
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  if ((convI.val != 0) && (convI.val != 1)) {
    return false;
  }
  if (convI.val) {
    config.dataTxCfg.txType |= (1 << 1);
  } else {
    config.dataTxCfg.txType &= ~(1 << 1);
  }
  return true;
}

/*! @brief Get the last reset cause (21.8.1)
 *  @return null-terminated string with the last cause.
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
 *  @return 32bit word from index
 */
uint32_t getUniqueID(int idx) {
  /* Section 10.3 Serial Number */
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

  uartPuts("Base Node ID      : ");
  putInt(config.baseCfg.nodeID);
  uartPuts("\r\n");

  uartPuts("Report time (s)   : ");
  putInt(config.baseCfg.reportTime);
  uartPuts("\r\n");

  uartPuts("OneWire interface : ");
  if (config.baseCfg.extTempEn) {
    uartPuts("En");
  } else {
    uartPuts("Dis");
  }
  uartPuts("abled\r\n");

  uartPuts("Data transmission : ");
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
      uartPuts("433.00");
      break;
    case 3:
      uartPuts("433.92");
      break;
    }
    uartPuts(" MHz @ ");
    putInt(config.dataTxCfg.rfmPwr - 18);
    uartPuts("dB\r\n");
  } else if ((DATATX_UART == tx) || (DATATX_BOTH == tx)) {
    uartPuts("Serial\r\n");
  }

  uartPuts("Pulse channel     : ");
  if (config.pulseCfg.active) {
    uartPuts("Enabled\r\n  - Hysteresis (ms): ");
    putInt(config.pulseCfg.timeMask);
    uartPuts("\r\n");
  } else {
    uartPuts("Disabled");
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

static void putUniqueID(void) {
  char strBuffer[8];
  for (int i = 0; i < 4; i++) {
    utilItoa(strBuffer, getUniqueID(i), ITOA_BASE16);
    uartPuts(strBuffer);
  }
}

void configCmdChar(const uint8_t c) {
  if (('\r' == c) || ('\n' == c)) {
    if (!cmdPending) {
      uartPuts("\r\n");
      cmdPending = true;
    }
  } else if (('\b' == c)) {
    uartPuts("\b \b");
    if (0 != inBufferIdx) {
      inBufferIdx--;
      inBuffer[inBufferIdx] = 0;
    }
  } else if ((inBufferIdx < (IN_BUFFER_W - 1)) && utilCharPrintable(c)) {
    inBuffer[inBufferIdx++] = c;
  } else {
    inBufferClear(IN_BUFFER_W);
    uartPuts("\r\n");
  }
}

void configEnter(void) {

  portPinDrv(PIN_LED, PIN_DRV_SET);
  inBufferClear(IN_BUFFER_W);

  uartPuts("\033c==== emonTH3 Configuration ====\r\n\r\n");
  uartPuts("'?' to list commands\r\n\r\n");
  while (1) {
    if (cmdPending) {
      cmdPending = false;
      if (configProcessCmd()) {
        break;
      };
    }
    samlSleepEnter();
  }
  if (unsavedChange) {
    uartPuts("> Unsaved changes not written.\r\n");
  }

  uartPuts("\r\n====== End Configuration ======\r\n\r\n");
  portPinDrv(PIN_LED, PIN_DRV_CLR);
}

void configFirmwareBoardInfo(void) {
  uartPuts("\033c==== emonTH3 ====\r\n\r\n");

  uartPuts("> Board:\r\n");
  uartPuts("  - emonTH3\r\n");
  uartPuts("  - Serial:     ");
  putUniqueID();
  uartPuts("\r\n  - Last reset: ");
  uartPuts(getLastReset());
  uartPuts("\r\n");

  uartPuts("> Firmware:\r\n");
  uartPuts("  - Version:    ");
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

  NVMHeader_t *header    = (NVMHeader_t *)nvmPageBuffer();
  bool         keyFound  = false;
  uint16_t     crc16Calc = 0;
  bool         badCRC    = false;

  EmonTHConfigPacked_t *pCfg =
      (EmonTHConfigPacked_t *)(nvmPageBuffer() + sizeof(*header));

  nvmDataFlashRead(NVM_PAGE_CONFIG);

  keyFound  = (CONFIG_NVM_KEY == header->watermark);
  crc16Calc = calcCRC16_ccitt(pCfg, sizeof(*pCfg));
  badCRC    = crc16Calc != header->crc16;

  /* The watermark has not been found or the CRC values do not match. Restore
   * default values to the NVM. */
  if (!keyFound || badCRC) {
    configDefault();
    nvmPageBufferClear();
    header->watermark  = CONFIG_NVM_KEY;
    header->writeCount = 2;
    header->crc16      = calcCRC16_ccitt(&config, sizeof(config));
    memcpy(pCfg, &config, sizeof(config));
    nvmDataFlashWrite(NVM_PAGE_CONFIG);
  }

  memcpy(&config, pCfg, sizeof(*pCfg));
  return &config;
}

static bool configProcessCmd(void) {
  bool         exitConfig = false;
  unsigned int arglen     = 0;
  bool         termFound  = false;
  bool         cmdUnsaved = false;

  /* Help text - serves as documentation interally as well */
  const char helpText[] =
      "\r\n"
      "emonTH information and configuration commands\r\n\r\n"
      " - ?           : show this text again\r\n"
      " - c<n>        : enable UART. n = 0: OFF, n = 1: ON\r\n"
      " - d<n>        : set the data acquisition period\r\n"
      " - e<n>        : number of external temperature sensors (0, 1, or 4)\r\n"
      " - f           : exit, lock, and continue\r\n"
      " - j<n>        : JSON serial format. n = 0: OFF, n = 1: ON\r\n"
      " - l           : list settings\r\n"
      " - m <x> <y>   : Pulse counting.\r\n"
      "     - x = 0: OFF, x = 1, ON.\r\n"
      "     - y : minimum period (ms). Ignored if x = 0\r\n"
      " - n<n>        : set node ID [1..60].\r\n"
      " - p<n>        : set the RF power level\r\n"
      " - r           : restore defaults\r\n"
      " - s           : save settings to NVM\r\n"
      " - t<x> <yy> <yy> <yy> <yy> <yy> <yy> <yy> <yy>\r\n"
      "   : change an external sensor's position\r\n"
      "     - x: position of sensor in the list (1-based)\r\n"
      "     - yy : hexadecimal bytes, e.g. 28 81 43 31 07 00 00 D9\r\n"
      " - v           : firmware and board information\r\n"
      " - w<n>        : enable wireless. n = 0: OFF, n = 1: ON\r\n"
      "both\r\n"
      " - x<n>        : 433 MHz compatibility. n = 0: 433.92 MHz, n = 1: "
      "433.00 MHz\r\n";

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
  case 'c':
    cmdUnsaved = configUART();
    break;
  case 'd':
    cmdUnsaved = configDatalog();
    break;
  case 'e':
    cmdUnsaved = configExtTempMax();
    break;
  case 'f':
    exitConfig = true;
    break;
  case 'j':
    cmdUnsaved = configJSON();
    break;
  case 'l':
    printSettings();
    break;
  case 'm':
    cmdUnsaved = configPulse();
    break;
  case 'n':
    cmdUnsaved = configNodeID();
    break;
  case 'p':
    cmdUnsaved = configRFPower();
    break;
  case 'r':
    configDefault();
    uartPuts("> Restored default values.\r\n");
    cmdUnsaved = true;
    break;
  case 's':
    configSaveToNVM();
    unsavedChange = false;
    break;
  case 't':
    cmdUnsaved = configOneWire();
    break;
  case 'v':
    configFirmwareBoardInfo();
    break;
  case 'w':
    cmdUnsaved = configRFM();
    break;
  case 'x':
    cmdUnsaved = configRF433();
    break;
  }

  if (!unsavedChange) {
    unsavedChange = cmdUnsaved;
  }
  cmdPending = false;
  inBufferClear(arglen + 1);
  return exitConfig;
}

void configSaveToNVM(void) {
  /* Save to EEPROM after calculating CRC for integrity */
  NVMHeader_t *header = (NVMHeader_t *)nvmPageBuffer();

  header->writeCount += 2;
  header->watermark = CONFIG_NVM_KEY;
  header->crc16     = calcCRC16_ccitt(&config, sizeof(EmonTHConfigPacked_t));

  memcpy(nvmPageBuffer() + sizeof(*header), &config, sizeof(config));
  nvmDataFlashWrite(NVM_PAGE_CONFIG);
}

/* =======================
 * UART Interrupt handler
 * ======================= */

void SERCOM_UART_HANDLER_RXC {
  /* Echo the received character to the TX channel, and send to the command
   * stream.
   */

  emonTHInteractiveUartSet();
  if (uartGetcReady()) {
    uint8_t rx_char = uartGetc();
    configCmdChar(rx_char);

    if (utilCharPrintable(rx_char) && !cmdPending) {
      uartPutcBlocking(rx_char);
    }
  }
}
