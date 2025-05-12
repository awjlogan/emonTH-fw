#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board_def.h"
#include "driver_EIC.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emonTH.h"
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

static void      rfmPacketHandler(void);
static uint8_t   rfmReadReg(const unsigned int addr);
static int16_t   rfmReadRSSI(void);
static void      rfmRxBegin(void);
static bool      rfmRxDone(void);
static RFMSend_t rfmSendNoRetry(uint8_t n);
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
static RFMRx_t       rfmRx            = {0};
static volatile bool rfmSendInterrupt = false;
static uint8_t       rxData[64]       = {0};
static volatile bool rxRdy            = false;
static const Pin_t   sel              = {PIN_SPI_RFM_SS};
static bool          sendComplete     = false;
static volatile bool timeoutFlag      = false;

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

    if (!(address == rfmRx.targetID ||
          RFM69_BROADCAST_ADDR == rfmRx.targetID) ||
        (rfmRx.payloadLen < 3)) {
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

static uint8_t spiRx(void) { return spiSendByte(0x00); }

static void spiTx(const uint8_t b) { (void)spiSendByte(b); }

static void timeoutSet(void) { timeoutFlag = true; }

void rfmSetAESKey(const char *aes) {

  rfmSetMode(RFM69_MODE_SLEEP);

  bool key = aes;

  if (key) {
    spiSelect(sel);
    spiTx(REG_AESKEY1 | 0x80);
    spiSendBuffer(aes, 16);
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

static RFMSend_t rfmSendNoRetry(uint8_t n) {
  sendComplete = false;

  // "send" in LPL
  rfmWriteReg(REG_PACKETCONFIG2,
              ((rfmReadReg(REG_PACKETCONFIG2) & 0xFB) | RFM_PACKET2_RXRESTART));
  timerDelaySleepAsync_ms(RFM69_CSMA_LIMIT_MS, SLEEP_MODE_IDLE, &timeoutSet);

  while (!rfmTxAvailable() && !timeoutFlag) {
    (void)rfmRxDone();
  }

  timerFlush();
  if (timeoutFlag) {
    timeoutFlag = false;
    return RFM_TIMED_OUT;
  }

  // end "send" in LPL
  // "sendframe"
  rfmSetMode(RFM69_MODE_STANDBY); // Turn off Rx while filling FIFO
  while (0 == (rfmReadReg(REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY))
    ;
  spiSelect(sel);
  spiTx(REG_FIFO | 0x80);
  spiTx(n + 3);
  spiTx(5u); // from OEM Tx
  spiTx((uint8_t)address);
  spiTx(0); // No ack requested
  spiSendBuffer(rfmBuffer, n);
  spiDeSelect(sel);

  rfmSetMode(RFM69_MODE_TX);
  while (0 == (rfmReadReg(REG_IRQFLAGS2) & RFM_IRQFLAGS2_PACKETSENT))
    ;
  rfmSetMode(RFM69_MODE_SLEEP);

  sendComplete = true;
  return RFM_SUCCESS;
}

bool rfmSendComplete(void) { return sendComplete; }

static void rfmSetMode(int_fast8_t mode) {
  if (rfmMode == mode) {
    return;
  }

  uint8_t rOpMode = rfmReadReg(REG_OPMODE) & 0xE3;

  switch (mode) {
  case RFM69_MODE_TX:
    rOpMode |= RFM_OPMODE_TRANSMITTER;
    break;
  case RFM69_MODE_RX:
    rOpMode |= RFM_OPMODE_RECEIVER;
    break;
  case RFM69_MODE_SYNTH:
    rOpMode |= RFM_OPMODE_SYNTHESIZER;
    break;
  case RFM69_MODE_STANDBY:
    rOpMode |= RFM_OPMODE_STANDBY;
    break;
  case RFM69_MODE_SLEEP:
    rOpMode |= RFM_OPMODE_SLEEP;
    break;
  default:
    return;
  }

  rfmWriteReg(REG_OPMODE, rOpMode);
  /* When coming from SLEEP, wait until FIFO is ready */
  if (RFM69_MODE_SLEEP == rfmMode) {
    while ((rfmReadReg(REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY) == 0)
      ;
  }
  rfmMode = mode;
}

static void rfmSleep(void) { rfmSetMode(RFM69_MODE_SLEEP); }

uint8_t *rfmGetBuffer(void) { return rfmBuffer; }

void rfmInterrupt(void) { rxRdy = true; }

bool rfmInit(RFMOpt_t *pOpt) {

  /* Configuration parameters */
  const uint8_t config[][2] = {
      {REG_OPMODE, 0x04},    /* OPMODE: Sequencer, standby, listen off */
      {REG_DATAMODUL, 0x00}, /* DataModul: Packet, FSK, no shaping */
      {REG_BITRATEMSB, RFM_BITRATEMSB_55555},
      {REG_BITRATELSB, RFM_BITRATELSB_55555},
      {REG_FDEVMSB, RFM_FDEVMSB_50000},
      {REG_FDEVLSB, RFM_FDEVLSB_50000},
      {REG_FRFMSB, (RFM_FREQ_868MHz == pOpt->freq)
                       ? RFM_FRFMSB_868
                       : ((RFM_FREQ_915MHz == pOpt->freq) ? RFM_FRFMSB_915
                                                          : RFM_FRFMSB_433)},
      {REG_FRFMID, (RFM_FREQ_868MHz == pOpt->freq)
                       ? RFM_FRFMID_868
                       : ((RFM_FREQ_915MHz == pOpt->freq) ? RFM_FRFMID_915
                                                          : RFM_FRFMID_433)},
      {REG_FRFLSB, (RFM_FREQ_868MHz == pOpt->freq)
                       ? RFM_FRFLSB_868
                       : ((RFM_FREQ_915MHz == pOpt->freq) ? RFM_FRFLSB_915
                                                          : RFM_FRFLSB_433)},
      {REG_RXBW, (RFM_RXBW_DCCFREQ_010 | RFM_RXBW_MANT_16 | RFM_RXBW_EXP_2)},
      {REG_DIOMAPPING1, RFM_DIOMAPPING1_DIO0_01},
      {REG_DIOMAPPING2, RFM_DIOMAPPING2_CLKOUT_OFF},
      {REG_IRQFLAGS2, RFM_IRQFLAGS2_FIFOOVERRUN},
      {REG_RSSITHRESH, 0xDC},
      {REG_SYNCCONFIG, (RFM_SYNC_ON | RFM_SYNC_FIFOFILL_AUTO | RFM_SYNC_SIZE_2 |
                        RFM_SYNC_TOL_0)},
      {REG_SYNCVALUE1, 0x2D}, /* Make compatible with RFM12B library */
      {REG_SYNCVALUE2, pOpt->group},
      {REG_PACKETCONFIG1, RFM_PACKET1_FORMAT_VARIABLE | RFM_PACKET1_DCFREE_OFF |
                              RFM_PACKET1_CRC_ON | RFM_PACKET1_CRCAUTOCLEAR_ON |
                              RFM_PACKET1_ADRSFILTERING_OFF},
      {REG_PAYLOADLENGTH, 66},
      {REG_FIFOTHRESH,
       (RFM_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RFM_FIFOTHRESH_VALUE)},
      {REG_PACKETCONFIG2,
       (RFM_PACKET2_RXRESTARTDELAY_2BITS | RFM_PACKET2_AUTORXRESTART_OFF |
        RFM_PACKET2_AES_OFF)},
      {REG_TESTDAGC, RFM_DAGC_IMPROVED_LOWBETA0}};

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
  for (size_t idxCfg = 0; idxCfg < (sizeof(config) / sizeof(*config));
       idxCfg++) {
    rfmWriteReg(config[idxCfg][0], config[idxCfg][1]);
  }

  rfmSetAESKey(0);
  rfmWriteReg(REG_PALEVEL, (RFM_PALEVEL_PA0_ON | pOpt->paLevel));

  rfmSetMode(RFM69_MODE_STANDBY);
  initDone = true;
  rfmSleep();

  // eicChannelEnable((EIC_Cfg_t){
  //     .ch = EIC_CH_RFM, .pin = PIN_RFM_IRQ, .sense = EIC_SENSE_RFM, .cb =
  //     0});
  return true;
}

RFMSend_t rfmSendBuffer(const int_fast8_t n) {
  if (n > 61) {
    return RFM_N_TOO_LARGE;
  }

  if (!initDone) {
    return RFM_NO_INIT;
  }

  return rfmSendNoRetry(n);
}

void rfmSetAddress(const uint16_t addr) {
  address = addr;
  rfmWriteReg(REG_NODEADRS, addr);
}
