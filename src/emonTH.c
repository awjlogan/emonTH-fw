#include <stddef.h>
#include <string.h>

#include "emonTH_saml.h"

#include "driver_ADC.h"
#include "driver_CLK.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "driver_RTC.h"
#include "driver_SAML.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"

#include "configuration.h"
#include "dataPack.h"
#include "emonTH.h"
#include "emonTH_assert.h"
#include "periph_DS18B20.h"
#include "periph_HDC2010.h"
#include "periph_rfm69.h"
#include "pulse.h"
#include "temperature.h"
#include "util.h"

typedef struct TransmitOpt_ {
  bool    json;
  bool    useRFM;
  bool    logSerial;
  uint8_t nodeID;
} TransmitOpt_t;

typedef enum THState_ {
  TH_STATE_IDLE,
  TH_STATE_SAMPLE_INT,
  TH_STATE_SAMPLE_INT_WAIT,
  TH_STATE_SAMPLE_INT_RD,
  TH_STATE_SAMPLE_EXT,
  TH_STATE_SAMPLE_EXT_WAIT,
  TH_STATE_SAMPLE_EXT_RD,
  TH_STATE_PROCESS_SEND,
  TH_STATE_CLEAN
} THState_t;

/*************************************
 * Persistent state variables
 *************************************/

static volatile bool     interactiveUart    = false;
static volatile bool     interactiveTimeout = false;
static volatile uint32_t evtPend;
AssertInfo_t             g_assert_info;

/*************************************
 * Static function prototypes
 *************************************/

static void         errorFatal(void);
static bool         evtPending(EVTSRC_t evt);
static void         gpioClr(const int gpio);
static void         gpioSet(const int gpio);
static uint_fast8_t readSlideSW(void);
static void         regEnable(bool dly);
static void         regDisable(void);
static uint32_t     tempSetup(void);
static void transmitData(const EmonTHDataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer);
static void ucSetup(void);
static void interactiveWait(void);

/*************************************
 * Functions
 *************************************/

void uartPuts(const char *s) {
  EMONTH_ASSERT(s);
  uartPutsBlocking(s);
}

void emonTHEventClr(const EVTSRC_t evt) {
  /* Disable interrupts during RMW update of event status */
  uint32_t evtDecode = ~(1u << evt);
  __disable_irq();
  evtPend &= evtDecode;
  __enable_irq();
}

void emonTHEventSet(const EVTSRC_t evt) {
  /* Disable interrupts during RMW update of event status */
  uint32_t evtDecode = (1u << evt);
  __disable_irq();
  evtPend |= evtDecode;
  __enable_irq();
}

static void errorFatal(void) {
  while (1)
    ;
}

/*! @brief Check if an event source is active. Clear on read.
 *  @param [in] : event source to check
 *  @return true if pending, false otherwise
 */
static bool evtPending(EVTSRC_t evt) {
  bool ret = (evtPend & (1u << evt)) ? true : false;
  evtPend &= ~(1u << evt);
  return ret;
}

static void gpioClr(const int gpio) {
  const int pin = (0 == gpio) ? PIN_GPIO0 : PIN_GPIO1;
  // Revisit : use the GPIO pins, won't work in v0.2 board
  portPinDrv(PIN_EXT_EN, PIN_DRV_CLR);
}

static void gpioSet(const int gpio) {
  const int pin = (0 == gpio) ? PIN_GPIO0 : PIN_GPIO1;
  // Revisit : use the GPIO pins, won't work in v0.2 board
  portPinDrv(PIN_EXT_EN, PIN_DRV_SET);
}

static uint_fast8_t readSlideSW(void) {
  uint_fast8_t swVal    = 0;
  uint8_t      swPin[2] = {PIN_SW_NODE0, PIN_SW_NODE1};

  /* Match the pull up/down to match to eliminate current through pull */
  for (int i = 0; i < 2; i++) {
    unsigned int pinVal = portPinValue(swPin[i]);
    swVal |= pinVal << i;
    if (0 == pinVal) {
      portPinDrv(swPin[i], PIN_DRV_CLR);
    }
  }
  return swVal;
}

/*! @brief Disable the external boost regulator */
static void regDisable(void) { portPinDrv(PIN_REG_EN, PIN_DRV_CLR); }

/*! @brief Enable the external boost regulator
 *  @param [in] dly : apply 128 us delay to allow 3V3 to settle (Figure 27)
 */
static void regEnable(bool dly) {
  portPinDrv(PIN_REG_EN, PIN_DRV_SET);
  if (dly) {
    timerDelay_us(128);
  }
}

/*! @brief Initialises the temperature sensors
 *  @return number of temperature sensors found
 */
static uint32_t tempSetup(void) {
  return tempSensorsInit(TEMP_INTF_ONEWIRE, 0);
}

static void transmitData(const EmonTHDataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer) {

  if (pOpt->logSerial) {
    (void)dataPackSerial(pSrc, txBuffer, TX_BUFFER_W, pOpt->json);
    uartPuts(txBuffer);
  }

  if (pOpt->useRFM) {
    dataPackPacked(pSrc, (PackedData_t *)rfmGetBuffer());
    rfmSetAddress(pOpt->nodeID);
    rfmSendBuffer(sizeof(PackedData_t));
  }
}

/*! @brief Setup the microcontroller. This function must be called first. */
static void ucSetup(void) {
  clkSetup();
  portSetup();
  sercomSetup();
  rtcSetup();
  adcSetup();
  timerSetup();
  eicSetup();
  // dmacSetup();
}

static void interactiveWait(void) {
  /* Wait for 5 s with LED flashing @ 2 Hz. Enter configuration mode by pressing
   * any key. */
  int  count     = 0;
  int  remain    = 5;
  char strbuf[4] = {0};

  uartPuts("> Press any key within 5 seconds to enter "
           "configuration.\r\n");
  while ((count < 20) && !interactiveUart) {
    timerDelaySleep_ms(250);
    portPinDrv(PIN_LED, PIN_DRV_TGL);
    if (!(count % 4)) {
      utilItoa(strbuf, remain--, ITOA_BASE10);
      uartPuts(strbuf);
    } else {
      uartPuts(".");
    }
    count++;
  }

  if (interactiveUart) {
    configEnter();
  } else {
    uartPuts("0\r\n");
  }
  portPinDrv(PIN_LED, PIN_DRV_SET);
}

int main(void) {

  THState_t state     = TH_STATE_IDLE;
  THState_t state_nxt = TH_STATE_IDLE;

  EmonTHDataset_t       dataset               = {0};
  HDCResultRaw_t        hdcResultRaw          = {0};
  unsigned int          tempExtNum            = 0;
  EmonTHConfigPacked_t *pConfig               = 0;
  uint_fast8_t          swVal                 = 0;
  char                  txBuffer[TX_BUFFER_W] = {0};
  TransmitOpt_t         txOpt                 = {0};
  RFMOpt_t              rfmOpt                = {0};

  ucSetup();
  regEnable(true);
  portPinDrv(PIN_LED, PIN_DRV_SET);

  configFirmwareBoardInfo();
  int  wakeCount    = 0;
  char strBuffer[8] = {0};

  /* Load stored values (configuration and accumulated energy) from
   * non-volatile memory (NVM). If the NVM has not been used before then
   * store default configuration.
   */
  pConfig = configLoadFromNVM();

  interactiveWait();

  uartPuts("> Node ID: ");
  swVal = readSlideSW();
  utilItoa(strBuffer, (swVal + pConfig->baseCfg.nodeID), ITOA_BASE10);
  uartPuts(strBuffer);
  uartPuts("\r\n");

  uartPuts("> Setting up RFM69... ");
  rfmOpt.freq    = pConfig->dataTxCfg.rfmFreq;
  rfmOpt.group   = NETWORK_GROUP_DEF;
  rfmOpt.nodeID  = (swVal + pConfig->baseCfg.nodeID);
  rfmOpt.paLevel = pConfig->dataTxCfg.rfmPwr;

  if (rfmInit(&rfmOpt)) {
    rfmSetAESKey(RFM_AES_DEF);
    rfmSetAddress(rfmOpt.nodeID);
    uartPuts("Done!\r\n");
  } else {
    uartPuts("Failed :(\r\n");
  }

  uartPuts("> Sample time: ");
  utilItoa(strBuffer, pConfig->baseCfg.reportTime, ITOA_BASE10);
  uartPuts(strBuffer);
  uartPuts("\r\n");

  uartRxDisable();

  rtcEnable(pConfig->baseCfg.reportTime);

  adcSampleTrigger();
  while (!adcSampleReady()) {
    samlSleepEnter();
  }
  setupI2C();
  hdc2010Setup();

  portPinDrv(PIN_LED, PIN_DRV_CLR);
  while (1) {
    if (evtPending(EVT_WAKE_TIMER)) {
      samlSleepIdle();
      gpioSet(0);
      regEnable(true);

      emonTHEventClr(EVT_WAKE_TIMER);

      gpioClr(0);
      hdc2010ConversionStart();
      while (!hdc2010SampleReady()) {
        samlSleepEnter();
      }
      hdc2010SampleGet(&hdcResultRaw);
      gpioSet(0);
      timerDelaySleep_ms(1);

      setupUart();
      uartRxDisable();

      utilItoa(strBuffer, wakeCount++, ITOA_BASE10);
      uartPuts(strBuffer);
      uartPuts(" : RTC wakeup\r\n");

      adcSampleTrigger();
      while (!adcSampleReady()) {
        samlSleepEnter();
      }

      unsigned int vbatt_uv = adcGetResult() * 3226;
      uartPuts("battery: ");
      utilItoa(strBuffer, vbatt_uv / 1000000, ITOA_BASE10);
      uartPuts(strBuffer);
      uartPuts("V");
      utilItoa(strBuffer, (vbatt_uv % 1000000) / 1000, ITOA_BASE10);
      uartPuts(strBuffer);
      uartPuts("\r\n");

      int tmp = ((hdcResultRaw.temp * 1650) / (1 << 16)) - 400;
      uartPuts("Temp: ");
      utilItoa(strBuffer, tmp / 10, ITOA_BASE10);
      uartPuts(strBuffer);
      uartPuts(".");
      utilItoa(strBuffer, tmp % 10, ITOA_BASE10);
      uartPuts(strBuffer);
      uartPuts("°C\r\n");
      unsigned int utmp =
          ((unsigned int)hdcResultRaw.humidity & 0xFFFF) * 1000 / (1 << 16);
      uartPuts("Humidity: ");
      utilItoa(strBuffer, utmp / 10, ITOA_BASE10);
      uartPuts(strBuffer);
      uartPuts(".");
      utilItoa(strBuffer, utmp % 10, ITOA_BASE10);
      uartPuts(strBuffer);
      uartPuts("%\r\n");

      dataset.battery   = vbatt_uv / 10;
      dataset.hdcResRaw = hdcResultRaw;
      dataPackPacked(&dataset, (PackedData_t *)rfmGetBuffer());
      rfmSendBuffer(sizeof(PackedData_t));

      timerDelaySleep_ms(5);
      setupI2C();

      samlSleepStandby();
      regDisable();
      gpioClr(0);
    }

    samlSleepEnter();
  };

  interactiveWait();

  /* Configure the pulse input if in use. */
  if (pConfig->pulseCfg.active) {
    pulseInit(pConfig->pulseCfg.timeMask);
    timerPulseSetup(&pulseTimerCB);
  }

  /* Find any external temperature sensors.  */
  if (pConfig->baseCfg.extTempEn) {
    tempExtNum = tempSetup();
  }

  txOpt.nodeID = pConfig->baseCfg.nodeID + swVal;
  txOpt.json   = pConfig->baseCfg.useJson;

  switch ((TxType_t)pConfig->dataTxCfg.txType) {
  case DATATX_RFM69:
    txOpt.useRFM = true;
    break;
  case DATATX_UART:
    txOpt.logSerial = true;
    break;
  case DATATX_BOTH:
    txOpt.useRFM    = true;
    txOpt.logSerial = true;
    break;
  }

  /* This enables the RTC for wake up and take first sample. */
  if (!hdc2010Setup()) {
    errorFatal();
  }
  rtcEnable(pConfig->baseCfg.reportTime);
  emonTHEventSet(EVT_WAKE_TIMER);

  for (;;) {
    bool stateStep = false; /* Go straight to next state rather than WFI. */

    state_nxt = state;
    switch (state) {

    case TH_STATE_IDLE:
      if (evtPending(EVT_WAKE_TIMER)) {
        emonTHEventClr(EVT_WAKE_TIMER);
        regEnable(true);
        gpioSet(0);
        state_nxt = TH_STATE_SAMPLE_INT;
        stateStep = true;
      }
      break;

    case TH_STATE_SAMPLE_INT:
      /* Trigger internal sensors, and wait in WFI (should be in STANDBY) */
      if (tempExtNum) {
        tempPowerOn();
      }
      adcSampleTrigger();
      hdc2010ConversionStart();
      state_nxt = TH_STATE_SAMPLE_INT_WAIT;
      break;

    case TH_STATE_SAMPLE_INT_WAIT:
      if (evtPending(EVT_WAKE_SAMPLE_INT)) {
        emonTHEventClr(EVT_WAKE_SAMPLE_INT);
        state_nxt = TH_STATE_SAMPLE_INT_RD;
        stateStep = true;
      }
      break;

    case TH_STATE_SAMPLE_INT_RD:
      dataset.battery = adcGetResult() * 322; /* Battery voltage x1000 */
      hdc2010SampleGet(&hdcResultRaw);
      state_nxt = tempExtNum ? TH_STATE_SAMPLE_EXT : TH_STATE_PROCESS_SEND;
      stateStep = true;
      break;

    case TH_STATE_SAMPLE_EXT:
      /* Trigger external sensors, skip if no response */
      if (TEMP_OK == tempSampleStart(TEMP_INTF_ONEWIRE, 0)) {
        state_nxt = TH_STATE_SAMPLE_EXT_WAIT;
      } else {
        tempPowerOff();
        state_nxt = TH_STATE_PROCESS_SEND;
        stateStep = true;
      }
      break;

    case TH_STATE_SAMPLE_EXT_WAIT:
      if (tempSampleReady()) {
        state_nxt = TH_STATE_SAMPLE_EXT_RD;
        stateStep = true;
      }
      break;

    case TH_STATE_SAMPLE_EXT_RD:
      tempSampleRead(TEMP_INTF_ONEWIRE, dataset.tempExternal);
      tempPowerOff();
      state_nxt = TH_STATE_PROCESS_SEND;
      stateStep = true;
      break;

    case TH_STATE_PROCESS_SEND:
      transmitData(&dataset, &txOpt, txBuffer);
      state_nxt = (!txOpt.logSerial || uartDmaComplete()) &&
                          (!txOpt.useRFM || rfmSendComplete())
                      ? TH_STATE_CLEAN
                      : TH_STATE_PROCESS_SEND;
      stateStep = (state_nxt == TH_STATE_CLEAN);
      break;

    case TH_STATE_CLEAN:
      state_nxt = TH_STATE_IDLE;
      gpioClr(0);
      regDisable();
      break;
    }

    state = state_nxt;
    if (!stateStep) {
      samlSleepEnter();
    }
  };
}

void emonTHInteractiveUartSet(void) { interactiveUart = true; }

void emonTHInteractiveTimeout(void) { interactiveTimeout = true; }
