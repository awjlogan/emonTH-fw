#include <stddef.h>
#include <string.h>

#include "emonTH_saml.h"

#include "driver_ADC.h"
#include "driver_CLK.h"
#include "driver_DMAC.h"
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
  bool json;
  bool useRFM;
  bool logSerial;
} TransmitOpt_t;

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

static void     boardSetup(EmonTHConfigPacked_t *pCfg, uint32_t *tempNum);
static void     errorFatal(void);
static bool     evtPending(EVTSRC_t evt);
static void     gpioClr(const int gpio);
static void     gpioSet(const int gpio);
static void     measureInternal(EmonTHDataset_t *pData);
static uint8_t  readSlideSW(void);
static void     regEnable(bool dly);
static void     regDisable(void);
static uint32_t tempSetup(void);
static void transmitData(const EmonTHDataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer);
static void txOptions(EmonTHConfigPacked_t *pCfg, TransmitOpt_t *pOpt);
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

static void boardSetup(EmonTHConfigPacked_t *pCfg, uint32_t *tempNum) {
  char strBuffer[8];

  uint8_t swVal = readSlideSW();
  uartPuts("> Node ID: ");
  utilItoa(strBuffer, (swVal + pCfg->baseCfg.nodeID), ITOA_BASE10);
  uartPuts(strBuffer);
  uartPuts("\r\n");

  uartPuts("> Sample time: ");
  utilItoa(strBuffer, pCfg->baseCfg.reportTime, ITOA_BASE10);
  uartPuts(strBuffer);
  uartPuts("\r\n");
  uartPuts("\r\n");

  // RFMOpt_t rfmOpt = {.freq    = pCfg->dataTxCfg.rfmFreq,
  //                    .group   = NETWORK_GROUP_DEF,
  //                    .nodeID  = (swVal + pCfg->baseCfg.nodeID),
  //                    .paLevel = pCfg->dataTxCfg.rfmPwr};

  // uartPuts("> Setting up RFM69... ");
  // /* Initialise RFM69 into sleep mode, regardless of its future use */
  // if (rfmInit(&rfmOpt)) {
  //   rfmSetAESKey(RFM_AES_DEF);
  //   rfmSetAddress(rfmOpt.nodeID);
  //   uartPuts("Done!\r\n");
  // } else {
  //   uartPuts("Failed :(\r\n");
  //   errorFatal();
  // }

  /* Configure the pulse input if in use. */
  if (pCfg->pulseCfg.active) {
    pulseInit(pCfg->pulseCfg.timeMask);
    timerPulseSetup(&pulseTimerCB);
  }

  /* Find any external temperature sensors.  */
  if (pCfg->baseCfg.extTempEn) {
    *tempNum = tempSetup();
  }

  setupI2C();
  i2cEnable();
  if (!hdc2010Setup()) {
    errorFatal();
  }
  i2cDisable();
}

static void errorFatal(void) {
  while (1) {
    timerDelaySleep_ms(100);
    portPinDrv(PIN_LED, PIN_DRV_TGL);
  }
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
  (void)pin;
  portPinDrv(PIN_EXT_EN, PIN_DRV_CLR);
}

static void gpioSet(const int gpio) {
  const int pin = (0 == gpio) ? PIN_GPIO0 : PIN_GPIO1;

  // Revisit : use the GPIO pins, won't work in v0.2 board
  (void)pin;
  portPinDrv(PIN_EXT_EN, PIN_DRV_SET);
}

static void interactiveWait(void) {
  /* Wait for 5 s with LED flashing @ 2 Hz. Enter configuration mode by pressing
   * any key. */
  int  count     = 0;
  int  remain    = 5;
  char strbuf[4] = {0};

  portPinDrv(PIN_LED, PIN_DRV_SET);

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

  portPinDrv(PIN_LED, PIN_DRV_CLR);
}

static void measureInternal(EmonTHDataset_t *pData) {
  HDCResultRaw_t hdcResultRaw = {0};

  /* Start samples in parallel, proceed when complete */
  hdc2010ConversionStart();
  adcSampleTrigger();

  while (!hdc2010SampleReady() && !adcSampleReady()) {
    samlSleepEnter();
  }

  hdc2010SampleGet(&hdcResultRaw);

  pData->battery   = adcGetResult();
  pData->hdcResRaw = hdcResultRaw;
}

static uint8_t readSlideSW(void) {
  uint8_t swPin[2] = {PIN_SW_NODE0, PIN_SW_NODE1};
  uint8_t swVal    = 0;

  /* If the pin has been pulled low, then disable the pull up as the pin already
   * has a defined value. */
  for (int i = 0; i < 2; i++) {
    unsigned int pinVal = portPinValue(swPin[i]);
    swVal |= pinVal << i;
    if (0 == pinVal) {
      portPinCfg(swPin[i], PORT_PINCFG_PULLEN, PIN_CFG_CLR);
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
    uint32_t n = dataPackSerial(pSrc, txBuffer, TX_BUFFER_W, pOpt->json);
    setupUart();
    uartDisableRx();
    uartPutsNonBlocking(txBuffer, n);
  }

  if (pOpt->useRFM) {
    dataPackPacked(pSrc, (PackedData_t *)rfmGetBuffer());
    rfmSendBuffer(sizeof(PackedData_t));
  }

  /* If using the RFM, sleep while the SPI DMA to complete */
  while (!dmacSPIComplete()) {
    samlSleepEnter();
  }
  if (pOpt->useRFM) {
    rfmTxFinish();
  }

  while (!dmacUARTComplete()) {
    samlSleepEnter();
  }
}

static void txOptions(EmonTHConfigPacked_t *pCfg, TransmitOpt_t *pOpt) {
  pOpt->json = pCfg->baseCfg.useJson;

  switch ((TxType_t)pCfg->dataTxCfg.txType) {
  case DATATX_RFM69:
    pOpt->useRFM = true;
    uartDisable();
    break;
  case DATATX_UART:
    pOpt->logSerial = true;
    spiDisable();
    uartDisableRx();
    break;
  case DATATX_BOTH:
    pOpt->useRFM    = true;
    pOpt->logSerial = true;
    uartDisableRx();
    break;
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
  dmacSetup();

  samlSleepConfigure();
  samlSleepStandby();
}

int main(void) {

  EmonTHDataset_t       dataset               = {0};
  EmonTHConfigPacked_t *pConfig               = 0;
  uint32_t              tempExtNum            = 0;
  char                  txBuffer[TX_BUFFER_W] = {0};
  TransmitOpt_t         txOpt                 = {0};
  bool                  vLow                  = false;

  ucSetup();
  regEnable(true);

  configFirmwareBoardInfo();

  /* Load configuration values from non-volatile memory (NVM). If the NVM has
   * not been used before then store default configuration.
   */
  pConfig = configLoadFromNVM();

  interactiveWait();
  portPinDrv(PIN_LED, PIN_DRV_CLR);

  boardSetup(pConfig, &tempExtNum);

  pConfig->dataTxCfg.txType = DATATX_UART;
  txOptions(pConfig, &txOpt);

  adcSampleTrigger(); /* First ADC sample is junk */

  rtcEnable(pConfig->baseCfg.reportTime);
  regDisable();
  samlSleepEnter();

  while (1) {
    if (evtPending(EVT_WAKE_TIMER)) {
      regEnable(true);
      emonTHEventClr(EVT_WAKE_TIMER);

      eicEnable();
      i2cEnable();

      measureInternal(&dataset);

      samlSleepIdle();
      transmitData(&dataset, &txOpt, txBuffer);

      timerDelaySleep_ms(1);
      if (txOpt.logSerial) {
        setupI2C();
      }
      eicDisable();

      if (!vLow && dataset.battery < 1500) {
        vLow = true;
        SUPC->VREG.reg &= ~SUPC_VREG_LPEFF;
      }

      samlSleepStandby();
      regDisable();
    }
    samlSleepEnter();
  }
}

void emonTHInteractiveUartSet(void) { interactiveUart = true; }

void emonTHInteractiveTimeout(void) { interactiveTimeout = true; }
