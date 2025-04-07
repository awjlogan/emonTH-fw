#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emonTH_saml.h"
#include "periph_rfm69.h"

#include "RFM69.h"

typedef struct RFMRx_ {
  uint16_t targetID;
  uint16_t senderID;
  int16_t  rxRSSI;
  uint8_t  payloadLen;
  uint8_t  dataLen;
  bool     ackRecv;
  bool     ackReq;
} RFMRx_t;

static bool      rfmAckRecv(uint16_t fromId);
static void      rfmPacketHandler(void);
static uint8_t   rfmReadReg(const unsigned int addr);
static int16_t   rfmReadRSSI(void);
static void      rfmRxBegin(void);
static bool      rfmRxDone(void);
static RFMSend_t rfmSendWithRetry(uint8_t n);
static void      rfmSetMode(int_fast8_t mode);
static void      rfmSleep(void);
static bool      rfmTxAvailable(void);
static void      rfmWriteReg(const unsigned int addr, const uint8_t data);
static uint8_t   spiRx(void);
static void      spiTx(const uint8_t b);
static void      timeoutSet(void);

static uint16_t      address          = 0;
static bool          initDone         = false;
static uint8_t       rfmBuffer[64]    = {0};
static int_fast8_t   rfmMode          = 0;
static RFMOpt_t      rfmOpt           = {0};
static RFMRx_t       rfmRx            = {0};
static volatile bool rfmSendInterrupt = false;
static uint8_t       rxData[64]       = {0};
static volatile bool rxRdy            = false;
static const Pin_t   sel              = {PIN_SPI_RFM_SS};
static bool          sendComplete     = false;
static volatile bool timeoutFlag      = false;

static bool rfmAckRecv(uint16_t fromId) {
  if (rfmRxDone()) {
    return (fromId == rfmRx.senderID) && rfmRx.ackRecv;
  }
  return false;
}

static void rfmPacketHandler(void) {
  if ((RFM69_MODE_RX == rfmMode) &&
      (rfmReadReg(REG_IRQFLAGS2) & RFM_IRQFLAGS2_PAYLOADREADY)) {

    uint16_t ctl = 0;
    rfmSetMode(RFM69_MODE_STANDBY);
    spiSelect(sel);
    spiTx(REG_FIFO & 0x7F);
    rfmRx.payloadLen = spiRx();
    /* Prevent any overflow */
    if (rfmRx.payloadLen > 66) {
      rfmRx.payloadLen = 66;
    }
    rfmRx.targetID = spiRx();
    rfmRx.senderID = spiRx();
    ctl            = spiRx();

    rfmRx.targetID |= (ctl & 0x0C) << 6;
    rfmRx.senderID |= (ctl & 0x03) << 8;

    if ((address == rfmRx.targetID) ||
        (RFM69_BROADCAST_ADDR == rfmRx.targetID) || (rfmRx.payloadLen < 3)) {
      rfmRx.payloadLen = 0;
      spiDeSelect(sel);
      rfmRxBegin();
      return;
    }

    rfmRx.dataLen = rfmRx.payloadLen - 3;
    rfmRx.ackRecv = ctl & RFM69_CTL_SENDACK;
    rfmRx.ackReq  = ctl & RFM69_CTL_REQACK;

    for (int i = 0; i < rfmRx.dataLen; i++) {
      rxData[i] = spiRx();
    }
    rxData[rfmRx.dataLen] = 0;
    spiDeSelect(sel);
    rfmSetMode(RFM69_MODE_RX);
  }
  rfmRx.rxRSSI = rfmReadRSSI();
}

static uint8_t rfmReadReg(const unsigned int addr) {
  uint8_t rdByte;
  spiSelect(sel);
  spiTx((uint8_t)addr);
  rdByte = spiRx();
  spiDeSelect(sel);
  return rdByte;
}

static bool rfmTxAvailable(void) {
  bool mode    = (RFM69_MODE_RX == rfmMode);
  bool len     = (0 == rfmRx.payloadLen);
  bool rssi    = (rfmReadRSSI() < RFM69_CSMA_LIMIT);
  bool canSend = mode && len && rssi;

  if (canSend) {
    rfmSetMode(RFM69_MODE_STANDBY);
  }
  return canSend;
}

static void rfmWriteReg(const unsigned int addr, const uint8_t data) {
  spiSelect(sel);
  /* Datasheet 5.2.1, Figure 24: "wnr is 1 for write" */
  spiTx((uint8_t)addr | 0x80);
  spiTx(data);
  spiDeSelect(sel);
}

static uint8_t spiRx(void) { return spiSendByte(SERCOM_SPI, 0x00); }

static void spiTx(const uint8_t b) { (void)spiSendByte(SERCOM_SPI, b); }

static void timeoutSet(void) { timeoutFlag = true; }

void rfmSetAESKey(const char *aes) {

  rfmSetMode(RFM69_MODE_SLEEP);

  bool key = aes;

  if (key) {
    spiSelect(sel);
    spiTx(REG_AESKEY1 | 0x80);
    spiSendBuffer(SERCOM_SPI, aes, 16);
    spiDeSelect(sel);
  }

  rfmWriteReg(REG_PACKETCONFIG2,
              ((rfmReadReg(REG_PACKETCONFIG2) & 0xFE) | key));
}

static int16_t rfmReadRSSI(void) {
  int16_t rssi = -rfmReadReg(REG_RSSIVALUE);
  return rssi >>= 1;
}

static void rfmRxBegin(void) {
  memset(&rfmRx, 0, sizeof(rfmRx));
  // Avoids Rx deadlocks
  if (rfmReadReg(REG_IRQFLAGS2) & RFM_IRQFLAGS2_PAYLOADREADY) {
    rfmWriteReg(REG_PACKETCONFIG2, ((rfmReadReg(REG_PACKETCONFIG2) & 0xFB) |
                                    RFM_PACKET2_RXRESTART));
  }

  rfmWriteReg(REG_DIOMAPPING1, RFM_DIOMAPPING1_DIO0_01); // "PAYLOADREADY" in Rx
  rfmSetMode(RFM69_MODE_RX);
}

static bool rfmRxDone(void) {
  if (rxRdy) {
    rxRdy = false;
    rfmPacketHandler();
  }

  if (RFM69_MODE_RX == rfmMode) {
    if (rfmRx.payloadLen > 0) {
      rfmSetMode(RFM69_MODE_STANDBY);
      return true;
    } else {
      /* Already in Rx, waiting for packet(s) */
      return false;
    }
  }
  rfmRxBegin();
  return false;
}

static RFMSend_t rfmSendWithRetry(uint8_t n) {
  sendComplete = false;

  for (int r = 0; r < RFM_RETRIES; r++) {
    // "send" in LPL
    rfmWriteReg(REG_PACKETCONFIG2, ((rfmReadReg(REG_PACKETCONFIG2) & 0xFB) |
                                    RFM_PACKET2_RXRESTART));
    timerDelaySleepAsync_ms(RFM69_CSMA_LIMIT_MS, SLEEP_MODE_IDLE, &timeoutSet);

    while (!rfmTxAvailable() && !timeoutFlag) {
      (void)rfmRxDone();
    }

    timerFlush();
    if (timeoutFlag) {
      timeoutFlag = false;
      continue;
    }

    // end "send" in LPL
    // "sendframe"
    rfmSetMode(RFM69_MODE_STANDBY); // Turn off Rx while filling FIFO
    while (0 == (rfmReadReg(REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY))
      ;
    spiSelect(sel);
    spiTx(REG_FIFO | 0x80);
    spiTx(n + 3);
    spiTx(5); // from OEM Tx
    spiTx((uint8_t)address);
    spiTx(RFM69_CTL_REQACK); // CTL byte
    spiSendBuffer(SERCOM_SPI, rfmBuffer, n);
    spiDeSelect(sel);

    eicChannelEnable(EIC_CH_RFM, EIC_CONFIG_SENSE0_RISE_Val, 0);
    rfmSetMode(RFM69_MODE_TX);
    while (0 == (rfmReadReg(REG_IRQFLAGS2) & RFM_IRQFLAGS2_PACKETSENT)) {
      __WFI();
    };
    eicChannelDisable(EIC_CH_RFM);
    rfmSetMode(RFM69_MODE_STANDBY);

    // end "sendframe"
    timerDelaySleepAsync_ms(RFM_TIMEOUT, SLEEP_MODE_IDLE, &timeoutSet);
    while (!timeoutFlag) {
      if (rfmAckRecv(5)) {
        timerFlush();
        sendComplete = true;
        return RFM_SUCCESS;
      }
    }
    timeoutFlag = false;
  }
  timerFlush();
  sendComplete = true;
  return RFM_TIMED_OUT;
}

bool rfmSendComplete(void) { return sendComplete; }

static void rfmSetMode(int_fast8_t mode) {
  if (rfmMode == mode) {
    return;
  }

  rfmWriteReg(REG_OPMODE, ((rfmReadReg(REG_OPMODE) & 0xE3) | mode));
  while ((RFM69_MODE_SLEEP == rfmMode) &&
         (0 == (rfmReadReg(REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY)))
    ;
  rfmMode = mode;
}

static void rfmSleep(void) { rfmSetMode(RFM69_MODE_SLEEP); }

uint8_t *rfmGetBuffer(void) { return rfmBuffer; }

RFMOpt_t *rfmGetHandle(void) { return &rfmOpt; }

void rfmInterrupt(void) { rxRdy = true; }

bool rfmInit(RFM_Freq_t freq) {

  /* Configuration parameters */
  const uint8_t config[][2] = {
      {REG_OPMODE, 0x04},     /* OPMODE: Sequencer, standby, listen off */
      {REG_DATAMODUL, 0x00},  /* DataModul: Packet, FSK, no shaping */
      {REG_BITRATEMSB, 0x02}, /* BitRate MSB: ~49.23 Kbps */
      {REG_BITRATELSB, 0x8A}, /* BitRate LSB */
      {REG_FDEVMSB, 0x05},    /* FdevMsb: ~90 kHz */
      {REG_FDEVLSB, 0xC3},    /* FdevLsb */
      {REG_FRFMSB,
       (RFM_FREQ_868MHz == freq)
           ? 0xD9
           : ((RFM_FREQ_915MHz == freq) ? 0xE4 : 0x6C)}, /* FrfMsb */
      {REG_FRFMID, 0x00},                                /* FrfMid */
      {REG_FRFLSB, 0x00},                                /* FrfLsb */
      {REG_PALEVEL, (0x80 | RFM_PALEVEL_DEF)},           /* PaLevel */
      {REG_AFCFEI, 0x2C},
      {REG_DIOMAPPING1, 0x80}, /* DioMapping1 */
      {REG_DIOMAPPING2, 0x03}, /* DioMapping2 */
      {REG_IRQFLAGS2, 0x00},   /* IrqFlags: FIFO overrun */
      {REG_SYNCCONFIG, 0x88},  /* SyncConfig : On, FIFO fill, 2 bytes, Tol */
      {REG_SYNCVALUE1, 0x2D},  /* SyncValue1 */
      {REG_SYNCVALUE2, 210},   /* Group ID, fixed as 210 for OEM */
      {REG_PACKETCONFIG1,
       0x00}, /* PktConfig: fixed, !DC free, !CRC, !CRCClear */
      {0xFF, 0}};

  /* Initialise RFM69 */
  timerDelaySleepAsync_ms(25, SLEEP_MODE_IDLE, &timeoutSet);
  while ((0xAA != rfmReadReg(REG_SYNCVALUE1)) && !timeoutFlag) {
    rfmWriteReg(REG_SYNCVALUE1, 0xAAu);
  }

  while ((0x55u != rfmReadReg(REG_SYNCVALUE1)) && !timeoutFlag) {
    rfmWriteReg(REG_SYNCVALUE1, 0x55u);
  }

  timerFlush();
  if (timeoutFlag) {
    timeoutFlag = false;
    return false;
  }

  /* Configuration */
  for (int idxCfg = 0; (0xFF != config[idxCfg][0]); idxCfg++) {
    rfmWriteReg(config[idxCfg][0], config[idxCfg][1]);
  }

  rfmSetAESKey(0);

  initDone = true;
  rfmSleep();

  // REVISIT configure EIC for external RFM signalling
  // eicConfigureRfmIrq();
  return true;
}

RFMSend_t rfmSendBuffer(const int_fast8_t n) {
  if (n > 61) {
    return RFM_N_TOO_LARGE;
  }

  if (!initDone) {
    return RFM_NO_INIT;
  }

  return rfmSendWithRetry(n);
}

void rfmSetAddress(const uint16_t addr) {
  address = addr;
  rfmWriteReg(REG_NODEADRS, addr);
}
