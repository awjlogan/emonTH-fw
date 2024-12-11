#include <stddef.h>
#include <string.h>

#include "emonTH_saml.h"

#include "driver_ADC.h"
#include "driver_CLK.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
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

static bool         evtPending(EVTSRC_t evt);
void                putchar_(char c);
static uint_fast8_t readSlideSW(void);
static uint32_t     tempSetup(void);
static void transmitData(const EmonTHDataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer);
static void ucSetup(void);
static void interactiveLED(void);
static void interactiveWait(void);
static void tplDone(void);

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

/*! @brief Check if an event source is active. Clear on read.
 *  @param [in] : event source to check
 *  @return : true if pending, false otherwise
 */
static bool evtPending(EVTSRC_t evt) {
  bool ret = (evtPend & (1u << evt)) ? true : false;
  evtPend &= ~(1u << evt);
  return ret;
}

static uint_fast8_t readSlideSW(void) {
  uint_fast8_t swVal    = 0;
  uint8_t      swPin[2] = {PIN_SW_NODE0, PIN_SW_NODE1};

  /* Match the pull up/down to match to eliminate current through pull */
  for (int i = 0; i < 2; i++) {
    swVal |= (portPinValue(swPin[i]) << i);
    if (swVal & (1 << i)) {
      portPinDrv(swPin[i], PIN_DRV_CLR);
    }
  }
  return swVal;
}

/*! @brief Initialises the temperature sensors
 *  @return : number of temperature sensors found
 */
static uint32_t tempSetup(void) {
  unsigned int tempExtNum = 0;

  tempExtNum = tempSensorsInit(TEMP_INTF_ONEWIRE, 0);

  return tempExtNum;
}

static void transmitData(const EmonTHDataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer) {

  dataPackConvert(&pSrc->hdcResRaw);
  if (pOpt->logSerial) {
    int pktLength = dataPackSerial(pSrc, txBuffer, TX_BUFFER_W, pOpt->json);
    uartPutsNonBlocking(txBuffer, pktLength);
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
  timerSetup();
  portSetup();
  dmacSetup();
  adcSetup();
  sercomSetup();
}

static void interactiveLED(void) {
  static int ccCount = 0;
  if ((ccCount <= 32) && !(ccCount % 8)) {
    portPinDrv(PIN_LED, PIN_DRV_TGL);
  } else if ((ccCount <= 64) && !(ccCount % 4)) {
    portPinDrv(PIN_LED, PIN_DRV_TGL);
  } else if (!(ccCount % 2)) {
    portPinDrv(PIN_LED, PIN_DRV_TGL);
  }

  if (80 == ccCount) {
    interactiveTimeout = true;
  } else {
    timerDelaySleepAsync_us(UINT16_MAX, SLEEP_MODE_STANDBY, &interactiveLED);
    ccCount++;
  }
}

static void interactiveWait(void) {
  portPinDrv(PIN_LED, PIN_DRV_SET);
  timerDelaySleepAsync_us(UINT16_MAX, SLEEP_MODE_STANDBY, &interactiveLED);
  while (!interactiveTimeout && !interactiveUart) {
    __WFI();
  }
  portPinDrv(PIN_LED, PIN_DRV_CLR);
  timerFlush();
  if (interactiveUart) {
    configEnter();
  }
}

static void tplDone(void) {
  portPinDrv(PIN_TPL_DONE, PIN_DRV_SET);
  timerDelay_us(2u);
  portPinDrv(PIN_TPL_DONE, PIN_DRV_CLR);
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

  ucSetup();

  /* The ADC's first sample is to be discarded. */
  adcTriggerSample();

  /* Load stored values (configuration and accumulated energy) from
   * non-volatile memory (NVM). If the NVM has not been used before then
   * store default configuration.
   */
  pConfig = configLoadFromNVM();

  /* Set up the RFM module put to sleep immediately. The RFM module requires 10
   * ms to wakup, sleep in standby. */
  timerDelaySleep_ms(12, SLEEP_MODE_STANDBY, true);
  if (rfmInit(pConfig->dataTxCfg.rfmFreq)) {
    rfmSetAESKey("89txbe4p8aik5kt3"); /* Default OEM AES key */
  };

  /* Read the slide switches and wait to enter configuration mode. */
  swVal = readSlideSW();
  interactiveWait();

  if (pConfig->pulseCfg.active) {
    pulseInit(pConfig->pulseCfg.timeMask);
    timerPulseSetup(&pulseTimerCB);
  }
  eicSetupClose();

  switch ((TxType_t)pConfig->dataTxCfg.txType) {
  case DATATX_RFM69:
    txOpt.useRFM = true;
    txOpt.nodeID = pConfig->baseCfg.nodeID + swVal;
    break;
  case DATATX_UART:
    txOpt.logSerial = true;
    break;
  case DATATX_BOTH:
    txOpt.useRFM    = true;
    txOpt.logSerial = true;
    txOpt.nodeID    = pConfig->baseCfg.nodeID + swVal;
    break;
  }
  txOpt.json = pConfig->baseCfg.useJson;

  if (pConfig->pulseCfg.active) {
    tempExtNum = tempSetup();
  }

  tplDone();
  for (;;) {
    bool stateStep = false; /* Go straight to next state rather than WFI. */

    state_nxt = state;
    switch (state) {

    case TH_STATE_IDLE:
      if (evtPending(EVT_WAKE_TIMER)) {
        state_nxt = TH_STATE_SAMPLE_INT;
        stateStep = true;
      }
      break;

    case TH_STATE_SAMPLE_INT:
      /* Trigger internal sensors, and wait in WFI (should be in STANDBY) */
      if ((pConfig->baseCfg.extTempEn && tempExtNum)) {
        tempPowerOn();
      }
      adcTriggerSample();
      hdc2010ConversionStart();
      state_nxt = ((INT16_MIN != adcGetResult()) && hdc2010ConversionStarted())
                      ? TH_STATE_SAMPLE_INT_WAIT
                      : TH_STATE_SAMPLE_INT;
      break;

    case TH_STATE_SAMPLE_INT_WAIT:
      if (evtPending(EVT_WAKE_SAMPLE_INT)) {
        state_nxt = TH_STATE_SAMPLE_INT_RD;
        stateStep = true;
      }
      break;

    case TH_STATE_SAMPLE_INT_RD:
      dataset.battery = adcGetResult() * 322; /* Battery voltage x1000 */
      hdc2010SampleGet(&hdcResultRaw);
      state_nxt = (pConfig->baseCfg.extTempEn && tempExtNum)
                      ? TH_STATE_SAMPLE_EXT
                      : TH_STATE_PROCESS_SEND;
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
      // REVISIT go to PL2(?), process data, and send
      transmitData(&dataset, &txOpt, txBuffer);
      state_nxt = (!txOpt.logSerial || uartDmaComplete()) &&
                          (!txOpt.useRFM || rfmSendComplete())
                      ? TH_STATE_CLEAN
                      : TH_STATE_PROCESS_SEND;
      stateStep = (state_nxt == TH_STATE_CLEAN);
      break;

    case TH_STATE_CLEAN:
      state_nxt = TH_STATE_IDLE;
      tplDone();
      break;
    }

    state = state_nxt;
    if (!stateStep) {
      __WFI();
    }
  };
}

void emonTHInteractiveUartSet(void) { interactiveUart = true; }

void emonTHInteractiveTimeout(void) { interactiveTimeout = true; }
