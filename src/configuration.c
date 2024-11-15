#include <inttypes.h>
#include <string.h>

#include "emonTH_assert.h"

#include "driver_ADC.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"

#include "configuration.h"
#include "eeprom.h"
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

static void     configDefault(void);
static void     configInitialiseNVM(void);
static void     configurePulse(void);
static bool     configureSerialLog(void);
static uint32_t getBoardRevision(void);
static char    *getLastReset(void);
static void     inBufferClear(int n);
static void     printSettings(void);
static void     printUptime(void);
static void     putFloat(float val, int flt_len);
static char     waitForChar(void);

/*************************************
 * Local variables
 *************************************/

#define IN_BUFFER_W 64
static EmonTHConfig_t config;
static char           inBuffer[IN_BUFFER_W];
static int            inBufferIdx   = 0;
static bool           cmdPending    = false;
static bool           resetReq      = false;
static bool           unsavedChange = false;

/*! @brief Set all configuration values to defaults */
static void configDefault(void) {
  config.key = CONFIG_NVM_KEY;

  /* Single phase, 50 Hz, 240 VAC, 10 s report period */
  config.baseCfg.nodeID      = NODE_ID; /* Node ID to transmit */
  config.baseCfg.dataGrp     = 210u;
  config.baseCfg.logToSerial = true;
  config.baseCfg.useJson     = false;
  config.dataTxCfg.txType    = (uint8_t)DATATX_UART;
  config.dataTxCfg.rfmPwr    = 0x19;
  config.dataTxCfg.rfmFreq   = 0;

  /* Pulse counters:
   *   - Period: 100 ms
   *   - Rising edge trigger
   *   - All disabled
   */
  for (int i = 0u; i < NUM_PULSECOUNT; i++) {
    config.pulseCfg[i].pulseActive = false;
    config.pulseCfg[i].period      = 100u;
    config.pulseCfg[i].edge        = 0u;
  }

  config.crc16_ccitt = calcCRC16_ccitt(&config, (sizeof(config) - 2u));
}

/*! @brief Write the configuration values to index 0, and zero the
 *         accumulator space to.
 */
static void configInitialiseNVM(void) { configDefault(); }

static void configurePulse(void) {
  /* String format in inBuffer:
   *      [1] -> ch;
   *      [3] -> active;
   *      [5] -> edge (rising, falling, both)
   *      [7] -> NULL: blank time
   */
  ConvInt_t convI;
  int       ch     = 0;
  int       active = 0;
  int       period = 0;
  char      edge   = 0;

  convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
  if (!convI.valid) {
    return;
  }
  ch = convI.val - 1;

  if ((ch < 0) || (ch >= NUM_PULSECOUNT)) {
    return;
  }

  convI = utilAtoi(inBuffer + 3, ITOA_BASE10);
  if (!convI.valid) {
    return;
  }
  active = (bool)convI.val;

  convI = utilAtoi((inBuffer + 7), ITOA_BASE10);
  if (!convI.valid) {
    return;
  }
  period = convI.val;

  edge = inBuffer[5];
  if (!(('r' == edge) || ('f' == edge) || ('b' == edge))) {
    return;
  }

  /* If inactive, clear active flag, no decode for the rest */
  if (0 == active) {
    config.pulseCfg[ch].pulseActive = false;
    // printf_("> Pulse channel %d disabled.\r\n", (ch + 1u));
    return;
  } else {
    config.pulseCfg[ch].pulseActive = true;
    // printf_("> Pulse channel %d: ", (ch + 1u));
    switch (edge) {
    case 'r':
      dbgPuts("Rising, ");
      config.pulseCfg[ch].edge = 0u;
      break;
    case 'f':
      dbgPuts("Falling, ");
      config.pulseCfg[ch].edge = 1u;
      break;
    case 'b':
      dbgPuts("Both, ");
      config.pulseCfg[ch].edge = 2u;
      break;
    }
    config.pulseCfg[ch].period = period;
    // printf_("%d ms\r\n", config.pulseCfg[ch].period);
  }
}

static bool configureSerialLog(void) {
  /* Log to serial output, default TRUE
   * Format: c0 | c1
   */
  ConvInt_t convI = utilAtoi(inBuffer + 1, ITOA_BASE10);

  if (convI.valid) {
    config.baseCfg.logToSerial = (bool)convI.val;
    // printf_("> Log to serial: %c\r\n", config.baseCfg.logToSerial ? 'Y' :
    // 'N');
    return true;
  }
  return false;
}

/*! @brief Get the last reset cause (16.8.14)
 *  @return : null-terminated string with the last cause.
 */
static char *getLastReset(void) {
  const RCAUSE_t lastReset = (RCAUSE_t)PM->RCAUSE.reg;
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

/*! @brief Fetch the SAMD's 128bit unique ID
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
  dbgPuts("\r\n\r\n==== Settings ====\r\n\r\n");
  dbgPuts("Data transmission:         ");
  if (DATATX_RFM69 == (TxType_t)config.dataTxCfg.txType) {
    dbgPuts("RFM69, ");
    switch (config.dataTxCfg.rfmFreq) {
    case 0:
      dbgPuts("868");
      break;
    case 1:
      dbgPuts("915");
      break;
    case 2:
      dbgPuts("433");
      break;
    }
    // printf_(" MHz, power %d\r\n", config.dataTxCfg.rfmPwr);
  } else {
    dbgPuts("Serial\r\n");
  }
  // printf_("Data format:               %s\r\n",
  // config.baseCfg.useJson ? "JSON" : "Key:Value");
  dbgPuts("\r\n");

  for (unsigned int i = 0; i < NUM_PULSECOUNT; i++) {
    bool enabled = config.pulseCfg[i].pulseActive;
    // printf_("Pulse Channel %d (%sactive)\r\n", (i + 1), enabled ? "" : "in");
    // printf_("  - Hysteresis (ms): %d\r\n", config.pulseCfg[i].period);
    dbgPuts("  - Edge:            ");
    switch (config.pulseCfg[i].edge) {
    case 0:
      dbgPuts("Rising");
      break;
    case 1:
      dbgPuts("Falling");
      break;
    case 2:
      dbgPuts("Both");
      break;
    default:
      dbgPuts("Unknown");
    }
    dbgPuts("\r\n\r\n");
  }

  if (unsavedChange) {
    dbgPuts("There are unsaved changes. Command \"s\" to save.\r\n\r\n");
  }
}

static void putFloat(float val, int flt_len) {
  char strBuffer[16];
  int  ftoalen = utilFtoa(strBuffer, val);

  if (flt_len) {
    int fillSpace = flt_len - ftoalen;

    while (fillSpace--) {
      dbgPuts(" ");
    }
  }

  dbgPuts(strBuffer);
}

/*! @brief Blocking wait for a key from the serial link. If the USB CDC is
 *         connected the key will come from here.
 */
static char waitForChar(void) {
  /* Disable the NVIC for the interrupt if needed while waiting for the
   * character otherwise it is handled by the configuration buffer.
   */
  char c;

  int irqEnabled =
      (NVIC->ISER[0] & (1 << ((uint32_t)(SERCOM_UART_INTERACTIVE_IRQn) & 0x1F)))
          ? 1
          : 0;
  if (irqEnabled)
    NVIC_DisableIRQ(SERCOM_UART_INTERACTIVE_IRQn);

  while (0 == (uartInterruptStatus(SERCOM_UART_DBG) & SERCOM_USART_INTFLAG_RXC))
    ;
  c = uartGetc(SERCOM_UART_DBG);

  if (irqEnabled)
    NVIC_EnableIRQ(SERCOM_UART_INTERACTIVE_IRQn);

  return c;
}

void configCmdChar(const uint8_t c) {
  if (('\r' == c) || ('\n' == c)) {
    if (!cmdPending) {
      dbgPuts("\r\n");
      cmdPending = true;
      emonTHEventSet(EVT_PROCESS_CMD);
    }
  } else if ('\b' == c) {
    dbgPuts("\b \b");
    if (0 != inBufferIdx) {
      inBufferIdx--;
      inBuffer[inBufferIdx] = 0;
    }
  } else if ((inBufferIdx < IN_BUFFER_W) && utilCharPrintable(c)) {
    inBuffer[inBufferIdx++] = c;
  } else {
    inBufferClear(IN_BUFFER_W);
    dbgPuts("\r\n");
  }
}

void configFirmwareBoardInfo(void) {
  dbgPuts("\033c==== emonTH ====\r\n\r\n");

  dbgPuts("> Board:\r\n");
  dbgPuts("  - emonTH3\r\n");
  // printf_("  - Serial:     0x%02x%02x%02x%02x\r\n",
  //         (unsigned int)getUniqueID(0), (unsigned int)getUniqueID(1),
  //         (unsigned int)getUniqueID(2), (unsigned int)getUniqueID(3));
  // printf_("  - Last reset: %s\r\n", getLastReset());
  dbgPuts("\r\n");

  dbgPuts("> Firmware:\r\n");
  // printf_("  - Version:    %d.%d.%d\r\n", VERSION_FW_MAJ, VERSION_FW_MIN,
  //         VERSION_FW_REV);
  dbgPuts("  - Build:      ");
  // dbgPuts(emonTH_build_info_string());
  dbgPuts("\r\n\r\n");
  dbgPuts("  - Distributed under GPL3 license, see COPYING.md\r\n");
  dbgPuts("  - emonTH Copyright (C) 2024-25 Angus Logan\r\n");
  dbgPuts("  - For Bear and Moose\r\n\r\n");
}

EmonTHConfig_t *configGetConfig(void) { return &config; }

void configLoadFromNVM(void) {}

void configProcessCmd(void) {
  unsigned int arglen    = 0;
  bool         termFound = false;
  ConvInt_t    convI     = {false, 0};

  /* Help text - serves as documentation interally as well */
  const char helpText[] =
      "\r\n"
      "emonTH information and configuration commands\r\n\r\n"
      " - ?           : show this text again\r\n"
      " - b<n>        : set RF band. n = 4 -> 433 MHz, 8 -> 868 MHz, 9 -> "
      "915 MHz\r\n"
      " - c<n>        : log to serial output. n = 0: OFF, n = 1: ON\r\n"
      " - g<n>        : set network group (default = 210)\r\n"
      " - j<n>        : JSON serial format. n = 0: OFF, n = 1: ON\r\n"
      " - l           : list settings\r\n"
      " - m<w> <x> <y> <z>\r\n"
      "   - Pulse counting.\r\n"
      "     - w : pulse channel index\r\n"
      "     - x = 0: OFF, x = 1, ON.\r\n"
      "     - y : edge sensitivity (r,f,b). Ignored if x = 0\r\n"
      "     - z : minimum period (ms). Ignored if x = 0\r\n"
      " - n<n>        : set node ID [1..60]\r\n"
      " - p<n>        : set the RF power level\r\n"
      " - r           : restore defaults\r\n"
      " - s           : save settings to NVM\r\n"
      " - v           : firmware and board information\r\n";

  /* Convert \r or \n to 0, and get the length until then. */
  while (!termFound && (arglen < IN_BUFFER_W)) {
    if (0 == inBuffer[arglen]) {
      termFound = true;
      break;
    }
    arglen++;
  }

  if (!termFound) {
    return;
  }

  /* Decode on first character in the buffer */
  switch (inBuffer[0]) {
  case '?':
    /* Print help text */
    dbgPuts(helpText);
    break;
  case 'c':
    if (configureSerialLog()) {
      unsavedChange = true;
      emonTHEventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'j':
    if (2u == arglen) {
      convI = utilAtoi(inBuffer + 1, ITOA_BASE10);
      if (!convI.valid) {
        break;
      }

      config.baseCfg.useJson = (bool)convI.val;

      // printf_("> Use JSON: %c\r\n", config.baseCfg.useJson ? 'Y' : 'N');

      unsavedChange = true;
      emonTHEventSet(EVT_CONFIG_CHANGED);
    }
    break;
  case 'l':
    printSettings();
    break;
  case 'm':
    configurePulse();
    unsavedChange = true;
    emonTHEventSet(EVT_CONFIG_CHANGED);
    break;
  case 'p':
    /* Configure RF power */
    resetReq      = true;
    unsavedChange = true;
    emonTHEventSet(EVT_CONFIG_CHANGED);
    break;
  case 'r':
    configDefault();

    dbgPuts("> Restored default values.\r\n");

    unsavedChange = true;
    resetReq      = true;
    emonTHEventSet(EVT_CONFIG_CHANGED);
    break;
  case 's':
    /* Save to EEPROM config space after recalculating CRC and indicate if a
     * reset is required.
     */
    config.crc16_ccitt = calcCRC16_ccitt(&config, (sizeof(config) - 2));

    unsavedChange = false;
    if (!resetReq) {
      emonTHEventSet(EVT_CONFIG_SAVED);
    } else {
      emonTHEventSet(EVT_SAFE_RESET_REQ);
    }
    break;
  }

  cmdPending = false;
  inBufferClear(arglen + 1);
}

/* =======================
 * UART Interrupt handler
 * ======================= */

void SERCOM_UART_INTERACTIVE_HANDLER {
  /* Echo the received character to the TX channel, and send to the command
   * stream.
   */
  if (uartGetcReady(SERCOM_UART_INTERACTIVE)) {
    uint8_t rx_char = uartGetc(SERCOM_UART_INTERACTIVE);
    configCmdChar(rx_char);

    if (utilCharPrintable(rx_char) && !cmdPending) {
      uartPutcBlocking(SERCOM_UART_INTERACTIVE, rx_char);
    }
  }
}
