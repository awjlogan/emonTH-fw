#include <inttypes.h>
#include <string.h>

#include "emonTH_assert.h"

#include "driver_NVM.h"
#include "driver_PORT.h"
#include "driver_SAML.h"
#include "driver_SERCOM.h"

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
static bool  configSCD(void);
static bool  configUART(void);
static char *getLastReset(void);
static void  inBufferClear(void);
static void  printInvalidVal(void);
static void  printSettingJSON(void);
static void  printSettingPeriod(void);
static void  printSettingPulse(void);
static void  printSettingRF(void);
static void  printSettingRFFreq(void);
static void  printSettingUART(void);
static void  printSettings(void);
static void  printSettingsHR(void);
static void  printSettingsKV(void);
static void  putUint(const uint32_t u);
static void  putUniqueID(void);
static void  sepNullBuffer(void);

/*************************************
 * Local variables
 *************************************/

#define IN_BUFFER_W (16u)

static char                 inBuffer[IN_BUFFER_W];
static size_t               inBufferIdx   = 0;
static bool                 cmdPending    = false;
static EmonTHConfigPacked_t config        = {0};
static bool                 unsavedChange = false;

static bool configDatalog(void) {
  ConvUint_t convI = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    printInvalidVal();
    return false;
  }
  if (convI.val.u16 < 5u) {
    uartPuts("> ERROR : sample period must be greater than 4 s\r\n");
    return false;
  }

  config.baseCfg.reportTime = convI.val.u16;
  printSettingPeriod();
  return true;
}

/*! @brief Set all configuration values to defaults */
static void configDefault(void) {
  (void)memset(&config, 0, sizeof(config));

  config.baseCfg.nodeID     = NODE_ID_DEF;       // Node ID
  config.baseCfg.dataGrp    = NETWORK_GROUP_DEF; // Group for OEM
  config.baseCfg.reportTime = WAKE_PERIOD_DEF;   // Time between reports
  config.baseCfg.useJson    = true;              // JSON format for serial
  config.baseCfg.extTempEn  = TEMP_NUM_DEF;      // Max num external sensors

  config.dataTxCfg.txType  = (uint8_t)DATATX_RFM69; // RFM only
  config.dataTxCfg.rfmPwr  = 0x18;                  // +12 dBm
  config.dataTxCfg.rfmFreq = 3;                     // 433.92 MHz

  config.pulseCfg.active   = false; // Pulse channel inactive
  config.pulseCfg.pu       = 1;     // Pull down
  config.pulseCfg.timeMask = 25u;   // 100 ms minimum between pulses

  config.scdCfg.altitude       = 0;   // Sea level
  config.scdCfg.sampleInterval = 600; // 10 minute CO2 sampling
}

static bool configExtTempMax(void) {
  ConvUint_t convI = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    printInvalidVal();
    return false;
  }

  /* Must be 0, 1 or 4 */
  if ((0 != convI.val.u8) && (1u != convI.val.u8) && (4u != convI.val.u8)) {
    uartPuts("> ERROR : must be in [0,1,4]\r\n");
    return false;
  }

  config.baseCfg.extTempEn = convI.val.u8;
  return true;
}

static bool configJSON(void) {
  ConvUint_t convI = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    printInvalidVal();
    return false;
  }

  config.baseCfg.useJson = (bool)convI.val.u8;
  printSettingJSON();
  return true;
}

static bool configOneWire(void) {

  sepNullBuffer();

  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);

  if (!convI.valid) {
    return false;
  }

  return false;
}

static bool configNodeID(void) {
  ConvUint_t convI = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    printInvalidVal();
    return false;
  }
  if ((convI.val.u8 < 1u) || (convI.val.u8 > 60u)) {
    uartPuts("> ERROR : ID must be [1..60]\r\n");
    return false;
  }

  config.baseCfg.nodeID = convI.val.u8;

  printSettingRF();
  return true;
}

static bool configPulse(void) {
  /* String format in inBuffer:
   *      [1] -> active
   *      [3] -> pull configuration
   *      [5] -> NULL: blank time
   */
  ConvUint_t convI;
  bool       active   = 0;
  uint8_t    pu       = 0;
  uint8_t    timeMask = 0;

  convI = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    printInvalidVal();
    return false;
  }
  active = (bool)convI.val.u8;

  if (!active) {
    config.pulseCfg.active = false;
    printSettingPulse();
    return true;
  }

  switch (inBuffer[3]) {
  case 'd':
    pu = 1u;
    break;
  case 'u':
    pu = 2u;
    break;
  case 'n':
  default:
    pu = 0;
  }

  convI = utilAtoui(inBuffer + 5, ITOA_BASE10);
  if (!convI.valid) {
    printInvalidVal();
    return false;
  }
  timeMask = convI.val.u8;

  config.pulseCfg.active   = true;
  config.pulseCfg.pu       = pu;
  config.pulseCfg.timeMask = timeMask;

  printSettingPulse();
  return true;
}

static bool configRF433(void) {
  int val = inBuffer[1] - '0';

  if (!((0 == val) || (1 == val))) {
    printInvalidVal();
    return false;
  }

  /* Only applies to 433 MHz ISM band */
  if (!((config.dataTxCfg.rfmFreq == 2) || (config.dataTxCfg.rfmFreq == 3))) {
    uartPuts("> ERROR : only for 433 MHz ISM\r\n");
    return false;
  }

  printSettingRF();
  return true;
}

static bool configRFM(void) {
  ConvUint_t convI = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    printInvalidVal();
    return false;
  }
  if (convI.val.u8 > 1u) {
    printInvalidVal();
    return false;
  }
  if (convI.val.u8) {
    config.dataTxCfg.txType |= (1u << 0);
  } else {
    config.dataTxCfg.txType &= ~(1u << 0);
  }

  printSettingRF();
  return true;
}

static bool configRFPower(void) {
  ConvUint_t convI = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    printInvalidVal();
    return false;
  }
  if ((convI.val.u8 == 0) || (convI.val.u8 > 31)) {
    uartPuts("> ERROR : power must be in range [1..31]\r\n");
    return false;
  }

  config.dataTxCfg.rfmPwr = convI.val.u8;

  printSettingRF();
  return true;
}

static bool configSCD(void) {

  sepNullBuffer();
  ConvUint_t convI = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }

  config.scdCfg.sampleInterval = convI.val.u16;

  size_t i;
  for (i = 0; i < IN_BUFFER_W; i++) {
    if (0 == inBuffer[i]) {
      break;
    }
  }

  convI = utilAtoui(inBuffer + i, ITOA_BASE10);
  if (!convI.valid) {
    return false;
  }
  config.scdCfg.altitude = convI.val.u16;

  return true;
}

static bool configUART(void) {
  ConvUint_t convI = utilAtoui(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    printInvalidVal();
    return false;
  }
  if (convI.val.u8 > 8u) {
    printInvalidVal();
    return false;
  }
  if (convI.val.u8) {
    config.dataTxCfg.txType |= (1u << 1);
  } else {
    config.dataTxCfg.txType &= ~(1u << 1);
  }

  printSettingUART();
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

/*! @brief Fetch part of the SAML's 128-bit unique ID.
 *  @param [in] idx : index of 32-bit word (0..3)
 *  @return 32-bit word from index
 */
uint32_t getUniqueID(const size_t idx) {
  /* Section 10.3 Serial Number */
  const uint32_t id_addr_lut[4] = {0x0080A00C, 0x0080A040, 0x0080A044,
                                   0x0080A048};
  return *(volatile uint32_t *)id_addr_lut[idx];
}

static void inBufferClear(void) {
  inBufferIdx = 0;
  (void)memset(inBuffer, 0, IN_BUFFER_W);
}

static void printInvalidVal(void) { uartPuts("> ERROR : invalid value\r\n"); }

static void printSettingJSON(void) {
  uartPuts("json = ");
  uartPuts(config.baseCfg.useJson ? "on" : "off");
  uartPuts("\r\n");
}

static void printSettingPeriod(void) {
  uartPuts("report time = ");
  putUint(config.baseCfg.reportTime);
  uartPuts("\r\n");
}

static void printSettingPulse(void) {
  uartPuts("pulse = ");
  uartPuts(config.pulseCfg.active ? "on" : "off");

  const uint8_t pu = config.pulseCfg.pu;
  uartPuts(", pull = ");
  uartPuts((0 == pu) ? "none" : (1 == pu ? "down" : "up"));
  uartPuts(", period = ");
  putUint(config.pulseCfg.timeMask);
  uartPuts("\r\n");
}

static void printSettingRF(void) {
  uartPuts("RF = ");
  uartPuts(config.dataTxCfg.txType & 0x01 ? "on" : "off");
  uartPuts(", rfBand = ");
  printSettingRFFreq();
  uartPuts(" MHz, ");
  uartPuts(", rfGroup = ");
  putUint(config.baseCfg.dataGrp);
  uartPuts(", rfNode = ");
  putUint(config.baseCfg.nodeID);
  uartPuts(", rfPower = ");
  putUint(config.dataTxCfg.rfmPwr);
  uartPuts(", rfFormat = LowPowerLabs\r\n");
}

static void printSettingRFFreq(void) {
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
}

static void printSettingUART(void) {
  uartPuts("serial = ");
  uartPuts((config.dataTxCfg.txType & 0x2u) ? "on" : "off");
  uartPuts("\r\n");
}

static void printSettings(void) {
  if ('h' == inBuffer[1]) {
    printSettingsHR();
  } else {
    printSettingsKV();
  }

  if (unsavedChange) {
    uartPuts("There are unsaved changes. Command \"s\" to save.\r\n\r\n");
  } else {
    uartPuts("All settings saved.\r\n\r\n");
  }
}

static void printSettingsHR(void) {
  uartPuts("\r\n\r\n==== Settings ====\r\n\r\n");

  uartPuts("Base Node ID      : ");
  putUint(config.baseCfg.nodeID);
  uartPuts("\r\n");

  uartPuts("Report time (s)   : ");
  putUint(config.baseCfg.reportTime);
  uartPuts("\r\n");

  uartPuts("OneWire interface : ");
  if (config.baseCfg.extTempEn) {
    uartPuts("En");
  } else {
    uartPuts("Dis");
  }
  uartPuts("abled\r\n");

  uartPuts("Data transmission :\r\n");
  if (config.dataTxCfg.txType & 0x1) {
    uartPuts("  - RFM69, ");
    printSettingRFFreq();
    uartPuts(" MHz @ ");
    putUint(config.dataTxCfg.rfmPwr - 18u);
    uartPuts("dB\r\n");
  }
  if (config.dataTxCfg.txType & 0x2) {
    uartPuts("  - Serial enabled\r\n");
  }

  uartPuts("Pulse channel     : ");
  if (config.pulseCfg.active) {
    const uint8_t pu = config.pulseCfg.pu;
    uartPuts("Enabled\r\n  - Hysteresis (ms): ");
    putUint(config.pulseCfg.timeMask);
    uartPuts("\r\n");
    uartPuts("  - Pull :");
    uartPuts((0 == pu) ? "off" : ((1 == pu) ? "down" : "up"));
  } else {
    uartPuts("Disabled");
  }
  uartPuts("\r\n\r\n");
}

static void printSettingsKV(void) {
  printSettingPeriod();
  printSettingRF();
  printSettingPulse();
  printSettingUART();
  printSettingJSON();
}

static void putUint(const uint32_t u) {
  char strBuffer[8];
  (void)utilUtoa(strBuffer, u, ITOA_BASE10);
  uartPuts(strBuffer);
}

static void putUniqueID(void) {
  char strBuffer[8];
  for (size_t i = 0; i < 4u; i++) {
    utilUtoa(strBuffer, getUniqueID(i), ITOA_BASE16);
    uartPuts(strBuffer);
  }
}

static void sepNullBuffer(void) {
  for (size_t i = 0; i < IN_BUFFER_W; i++) {
    if (0 == inBuffer[i]) {
      break;
    } else if (' ' == inBuffer[i]) {
      inBuffer[i] = 0;
    }
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
    inBufferClear();
    uartPuts("\r\n");
  }
}

void configEnter(void) {
  portPinDrv(PIN_LED, PIN_DRV_SET);
  inBufferClear();

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
  putUint(VERSION_FW_MAJ);
  uartPuts(".");
  putUint(VERSION_FW_MIN);
  uartPuts(".");
  putUint(VERSION_FW_REV);
  uartPuts("\r\n");
  uartPuts("  - Build:      ");
  uartPuts(emonTH_build_info_string());
  uartPuts("\r\n\r\n");
  uartPuts("  - Distributed under GPL3 license, see COPYING.md\r\n");
  uartPuts("  - emonTH Copyright (C) 2024-26 Angus Logan\r\n");
  uartPuts("  - For Bear and Moose\r\n\r\n");
}

EmonTHConfigPacked_t *configLoadFromNVM(void) {
  EmonTHConfigPacked_t *pCfg = (EmonTHConfigPacked_t *)nvmPageBuffer();

  NVMStatus_t nvm = nvmDataFlashRead(NVM_PAGE_CONFIG);

  if (NVM_READ_OK != nvm) {
    configDefault();
    nvmPageBufferClear();
    memcpy(pCfg, &config, sizeof(config));
    nvmDataFlashWrite(NVM_PAGE_CONFIG, sizeof(config));
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
  static const char helpText[] =
      "\r\n"
      "emonTH information and configuration commands\r\n\r\n"
      " - ?             : show this text again\r\n"
      " - a<n> <m>      : Configure SCD4x CO2 sensor\r\n"
      "     -  n : sample interval (s)\r\n"
      "     -  m : altitude above sea level (m)\r\n"
      " - c<n>          : enable UART. n = 0: OFF, n = 1: ON\r\n"
      " - d<n>          : set the data acquisition period\r\n"
      " - e<n>          : number of external temperature sensors (0, 1, or "
      "4)\r\n"
      " - f             : exit, lock, and continue\r\n"
      " - j<n>          : JSON serial format. n = 0: OFF, n = 1: ON\r\n"
      " - l             : list settings (key / value)\r\n"
      " - lh            : list settings (human readable)\r\n"
      " - m <x> <y> <z> : Pulse counting\r\n"
      "     - x = 0: OFF, x = 1, ON\r\n"
      "     - y = n: no pull, y = d : pull down, y = u : pull up. Only for x = "
      "1\r\n"
      "     - z : minimum period (ms). Only for x = 1\r\n"
      " - n<n>          : set node ID [1..60].\r\n"
      " - p<n>          : set the RF power level\r\n"
      " - r             : restore defaults\r\n"
      " - s             : save settings to NVM\r\n"
      " - t<x> <yy> <yy> <yy> <yy> <yy> <yy> <yy> <yy>\r\n"
      "   : change an external sensor's position\r\n"
      "     - x: position of sensor in the list (1-based)\r\n"
      "     - yy : hexadecimal bytes, e.g. 28 81 43 31 07 00 00 D9\r\n"
      " - v             : firmware and board information\r\n"
      " - w<n>          : enable wireless. n = 0: OFF, n = 1: ON\r\n"
      " - x<n>          : 433 MHz compatibility. n = 0: 433.92 MHz, n = 1: "
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
  case 'a':
    if (configSCD()) {
      cmdUnsaved = true;
    }
    break;
  case 'c':
    if (configUART()) {
      cmdUnsaved = true;
    }
    break;
  case 'd':
    if (configDatalog()) {
      cmdUnsaved = true;
    }
    break;
  case 'e':
    if (configExtTempMax()) {
      cmdUnsaved = true;
    }
    break;
  case 'f':
    exitConfig = true;
    break;
  case 'j':
    if (configJSON()) {
      cmdUnsaved = true;
    }
    break;
  case 'l':
    printSettings();
    break;
  case 'm':
    if (configPulse()) {
      cmdUnsaved = true;
    }
    break;
  case 'n':
    if (configNodeID()) {
      cmdUnsaved = true;
    }
    break;
  case 'p':
    if (configRFPower()) {
      cmdUnsaved = true;
    }
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
  inBufferClear();
  return exitConfig;
}

void configSaveToNVM(void) {
  nvmPageBufferClear();
  memcpy(nvmPageBuffer(), &config, sizeof(config));
  nvmDataFlashWrite(NVM_PAGE_CONFIG, sizeof(config));

  uartPuts("> All settings saved.\r\n");
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
