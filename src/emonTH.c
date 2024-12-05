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
  bool json;
  bool useRFM;
  bool logSerial;
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

static bool     evtPending(EVTSRC_t evt);
void            putchar_(char c);
static void     putsDbgNonBlocking(const char *const s, uint16_t len);
static void     readSlideSW(EmonTHConfigPacked_t *pCfg);
static uint32_t tempSetup(void);
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

/*! @brief Allows the printf function to print to the debug console. If the USB
 *         CDC is connected, characters should be routed there.
 */
void putchar_(char c) { uartPutcBlocking(c); }

static void putsDbgNonBlocking(const char *const s, uint16_t len) {
  uartPutsNonBlocking(DMA_CHAN_UART, s, len);
}

static void readSlideSW(EmonTHConfigPacked_t *pCfg) {
  uint8_t swVal    = 0;
  uint8_t swPin[2] = {PIN_SW_NODE0, PIN_SW_NODE1};

  /* Match the pull up/down to match to eliminate current through pull */
  for (int i = 0; i < 2; i++) {
    swVal |= (portPinValue(swPin[i]) << i);
    if (swVal & (1 << i)) {
      portPinDrv(swPin[i], PIN_DRV_CLR);
    }
  }

  if (!pCfg->baseCfg.nodeIDSaved) {
    pCfg->baseCfg.nodeID = NODE_ID_DEF + swVal;
  }
}

/*! @brief Initialises the temperature sensors
 *  @return : number of temperature sensors found
 */
static uint32_t tempSetup(void) {
  unsigned int   tempExtNum = 0;
  DS18B20_conf_t dsCfg      = {0};

  dsCfg.pin       = PIN_ONEWIRE;
  dsCfg.t_wait_us = 5;

  tempExtNum = tempSensorsInit(TEMP_INTF_ONEWIRE, &dsCfg);

  return tempExtNum;
}

static void transmitData(const EmonTHDataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer) {

  if (pOpt->useRFM) {
    PackedData_t packedData = {0};
    dataPackPacked(pSrc, &packedData);
    // REVISIT RFM send using latest LPL
  }

  if (pOpt->logSerial) {
    int pktLength = dataPackSerial(pSrc, txBuffer, TX_BUFFER_W, pOpt->json);
    putsDbgNonBlocking(txBuffer, pktLength);
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
  bool                  tempExtSample         = false;
  EmonTHConfigPacked_t *pConfig               = 0;
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
  rfmInit(pConfig->dataTxCfg.rfmFreq);

  /* Read the slide switches and wait to enter configuration mode. */
  readSlideSW(pConfig);
  interactiveWait();

  if (pConfig->pulseCfg.active) {
    pulseInit(pConfig->pulseCfg.timeMask);
    timerPulseSetup(&pulseTimerCB);
  }
  eicSetupClose();

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
  txOpt.json = pConfig->baseCfg.useJson;

  if (pConfig->pulseCfg.active) {
    tempExtNum = tempSetup();
  }

  tplDone();
  for (;;) {
    bool stateStep = false; /* Go straight to next state rather than WFI. */

    switch (state) {
    case TH_STATE_IDLE:
      state_nxt = evtPending(EVT_WAKE_TIMER)
                      /* Trigger external sensors if enabled and present. */
                      ? ((pConfig->baseCfg.extTempEn && tempExtNum)
                             ? TH_STATE_SAMPLE_EXT
                             : TH_STATE_SAMPLE_INT)
                      : TH_STATE_IDLE;
      stateStep = true;
      break;

    case TH_STATE_SAMPLE_EXT:
      /* Trigger external sensors (recording response) and skip to internal
       * sensor trigger */
      if (TEMP_OK == tempSampleStart(TEMP_INTF_ONEWIRE, 0)) {
        tempExtSample = true;
      }
      state_nxt = TH_STATE_SAMPLE_INT;
      stateStep = true;
      break;

    case TH_STATE_SAMPLE_INT:
      /* Trigger internal sensors, and wait in WFI (should be in STANDBY) */
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
      } else {
        state_nxt = TH_STATE_SAMPLE_INT_WAIT;
      }
      break;

    case TH_STATE_SAMPLE_INT_RD:
      dataset.battery = adcGetResult() * 322;
      hdc2010SampleGet(&hdcResultRaw);
      state_nxt =
          tempExtSample ? TH_STATE_SAMPLE_EXT_WAIT : TH_STATE_PROCESS_SEND;
      stateStep = true;
      break;

    case TH_STATE_SAMPLE_EXT_WAIT:
      if (tempSampleReady()) {
        state_nxt = TH_STATE_SAMPLE_EXT_RD;
        stateStep = true;
      } else {
        state_nxt = TH_STATE_SAMPLE_EXT_WAIT;
      }
      break;

    case TH_STATE_SAMPLE_EXT_RD:

      tempSampleRead(TEMP_INTF_ONEWIRE, dataset.tempExternal);
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
      // REVISIT reset all values
      state_nxt = TH_STATE_IDLE;
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
