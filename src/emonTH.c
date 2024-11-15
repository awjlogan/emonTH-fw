#include <stddef.h>
#include <string.h>

#include "emonTH_samd.h"

#include "driver_ADC.h"
#include "driver_CLK.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "driver_RTC.h"
#include "driver_SAMD.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "driver_WDT.h"

#include "configuration.h"
#include "dataPack.h"
#include "eeprom.h"
#include "emonTH.h"
#include "emonTH_assert.h"
#include "periph_DS18B20.h"
#include "periph_rfm69.h"
#include "pulse.h"
#include "temperature.h"
#include "ui.h"
#include "util.h"

typedef struct TransmitOpt_ {
  bool json;
  bool useRFM;
  bool logSerial;
} TransmitOpt_t;

/*************************************
 * Persistent state variables
 *************************************/

static volatile uint32_t evtPend;
static unsigned int      lastStoredWh;
AssertInfo_t             g_assert_info;

/*************************************
 * Static function prototypes
 *************************************/

static void      datasetAddPulse(EmonTHDataset_t *pDst);
static RFMOpt_t *dataTxConfigure(const EmonTHConfig_t *pCfg);
static void      evtKiloHertz(void);
static uint32_t  evtPending(EVTSRC_t evt);
static void      pulseConfigure(const EmonTHConfig_t *pCfg);
void             putchar_(char c);
static void      putsDbgNonBlocking(const char *const s, uint16_t len);
static uint32_t  tempSetup(void);
static void      transmitData(const EmonTHDataset_t *pSrc,
                              const TransmitOpt_t   *pOpt);
static void      ucSetup(void);

/*************************************
 * Functions
 *************************************/

/*! @brief Add pulse counting information to the dataset to be sent
 *  @param [out] pDst : pointer to the data struct
 */
static void datasetAddPulse(EmonTHDataset_t *pDst) {
  EMONTH_ASSERT(pDst);

  pDst->msgNum++;
  for (unsigned int i = 0; i < NUM_PULSECOUNT; i++) {
    pDst->pulseCnt[i] = pulseGetCount(i);
  }
}

/*! @brief Configure the data transmission output.
 *  @param [in] pCfg : pointer to the configuration struct
 *  @return : pointer to an RFM packet if using RFM, 0 if not.
 */
static RFMOpt_t *dataTxConfigure(const EmonTHConfig_t *pCfg) {
  EMONTH_ASSERT(pCfg);

  RFMOpt_t *rfmOpt = 0;
  if (DATATX_RFM69 == (TxType_t)pCfg->dataTxCfg.txType) {
    rfmOpt            = rfmGetHandle();
    rfmOpt->node      = pCfg->baseCfg.nodeID;
    rfmOpt->grp       = pCfg->baseCfg.dataGrp; /* Fixed for OpenEnergyMonitor */
    rfmOpt->rf_pwr    = pCfg->dataTxCfg.rfmPwr;
    rfmOpt->threshold = 0u;
    rfmOpt->timeout   = 1000u;
    rfmOpt->n         = 23u;
    if (sercomExtIntfEnabled()) {
      rfmInit((RFM_Freq_t)pCfg->dataTxCfg.rfmFreq);
    }
  }
  return rfmOpt;
}

void dbgPuts(const char *s) {
  EMONTH_ASSERT(s);

  if (usbCDCIsConnected()) {
    usbCDCPutsBlocking(s);
  } else {
    uartPutsBlocking(SERCOM_UART_DBG, s);
  }
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

/*! @brief This function is called when the 1 ms timer fires.
 *         Latency is not guaranteed, so only non-timing critical things
 *         should be done here (UI update, watchdog etc)
 */
static void evtKiloHertz(void) {
  int                      extEnabled;
  uint32_t                 msDelta;
  static volatile uint32_t msLast = 0;
  int                      ndisable_ext;
  static unsigned int      statLedOff_time = 0;

  /* Feed watchdog - placed in the event handler to allow reset of stuck
   * processing rather than entering the interrupt reliably.
   */
  wdtFeed();

  /* Update the pulse counters, looking on different edges */
  pulseUpdate();

  /* Track milliseconds to indicate uptime */
  msDelta = timerMillisDelta(msLast);
  if (msDelta >= 1000) {
    timerUptimeIncr();
    msLast = timerMillis();
    /* Account for any jitter in the 1 ms tick */
    if (msDelta > 1000) {
      msDelta -= 1000;
      msLast -= msDelta;
    }
  }
}

/*! @brief Check if an event source is active
 *  @param [in] : event source to check
 *  @return : 1 if pending, 0 otherwise
 */
static uint32_t evtPending(EVTSRC_t evt) {
  return (evtPend & (1u << evt)) ? 1u : 0;
}

/*! @brief Configure any pulse counter interfaces
 *  @param [in] pCfg : pointer to the configuration struct
 */
static void pulseConfigure(const EmonTHConfig_t *pCfg) {
  EMONTH_ASSERT(pCfg);

  uint8_t pinsPulse[][2] = {{GRP_PULSE, PIN_PULSE1}};

  for (unsigned int i = 0; i < NUM_PULSECOUNT; i++) {
    PulseCfg_t *pulseCfg = pulseGetCfg(i);

    if (0 != pulseCfg) {
      pulseCfg->edge    = (PulseEdge_t)pCfg->pulseCfg[i].edge;
      pulseCfg->grp     = pinsPulse[i][0];
      pulseCfg->pin     = pinsPulse[i][1];
      pulseCfg->periods = pCfg->pulseCfg[i].period;
      pulseCfg->active  = pCfg->pulseCfg[i].pulseActive;

      pulseInit(i);
    }
  }
}

/*! @brief Allows the printf function to print to the debug console. If the USB
 *         CDC is connected, characters should be routed there.
 */
void putchar_(char c) { uartPutcBlocking(SERCOM_UART_DBG, c); }

static void putsDbgNonBlocking(const char *const s, uint16_t len) {
  usbCDCPutsBlocking(s);
  uartPutsNonBlocking(DMA_CHAN_UART_DBG, s, len);
}

/*! @brief Initialises the temperature sensors
 *  @return : number of temperature sensors found
 */
static uint32_t tempSetup(void) {
  unsigned int   numTempSensors = 0;
  DS18B20_conf_t dsCfg          = {0};

  dsCfg.grp       = GRP_ONEWIRE;
  dsCfg.pin       = PIN_ONEWIRE;
  dsCfg.t_wait_us = 5;

  numTempSensors = tempInitSensors(TEMP_INTF_ONEWIRE, &dsCfg);

  return numTempSensors;
}

static void transmitData(const EmonTHDataset_t *pSrc,
                         const TransmitOpt_t   *pOpt) {
  char txBuffer[TX_BUFFER_W] = {0};

  int pktLength = dataPackSerial(pSrc, txBuffer, TX_BUFFER_W, pOpt->json);

  if (pOpt->useRFM) {
    PackedData_t packedData = {0};
    dataPackPacked(pSrc, &packedData, PACKED_LOWER);
    if (sercomExtIntfEnabled()) {
      /* Try to send in "clean" air. If failed, retry on next loop. Should not
       * reach RFM_FAILED at all. */
      RFMSend_t res = rfmSendReady(5u);
      if (RFM_SUCCESS == res) {
        rfmSend(&packedData);
      }
    }
    if (pOpt->logSerial) {
      putsDbgNonBlocking(txBuffer, pktLength);
    }
  } else {
    putsDbgNonBlocking(txBuffer, pktLength);
  }
}

/*! @brief Setup the microcontoller. This function must be called first. An
 *         implementation must provide all the functions that are called.
 *         These can be empty if they are not used.
 */
static void ucSetup(void) {
  clkSetup();
  timerSetup();
  portSetup();
  dmacSetup();
  sercomSetup();
  adcSetup();
  // wdtSetup    (WDT_PER_4K);
}

int main(void) {

  EmonTHConfig_t *pConfig        = 0;
  EmonTHDataset_t dataset        = {0};
  unsigned int    numTempSensors = 0;
  RFMOpt_t       *rfmOpt         = 0;
  unsigned int    tempCount      = 0;

  ucSetup();
  uiLedOn(LED_STATUS);
  configFirmwareBoardInfo();

  /* Load stored values (configuration and accumulated energy) from
   * non-volatile memory (NVM). If the NVM has not been used before then
   * store default configuration and 0 energy accumulator area.
   */
  configLoadFromNVM();

  pConfig = configGetConfig();

  /* Set up data transmission interfaces and configuration */
  rfmOpt = dataTxConfigure(pConfig);

  /* Set up pulse and temperature sensors, if present */
  pulseConfigure(pConfig);
  numTempSensors         = tempSetup();
  dataset.numTempSensors = numTempSensors;

  for (;;) {

    /* Start ADC and trigger T/H sample */
    /* Read T/H sample */
    /* Process data */
    /* Send processed data */

    /* Enter STANDBY until woken by the RTC overflow interrupt */
    samdSleep(SLEEP_MODE_STANDBY);
    while (!rtcIntOvfStat) {
      __WFI();
    }
  };
}
