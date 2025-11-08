#include <stddef.h>

#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emonTH.h"
#include "emonTH_saml.h"
#include "periph_SCD4x.h"

typedef enum RWC_ { C, CF, R, W } RWC_t;
typedef enum SCD_ID_ { SCD40, SCD41, SCD43, SCD4x_NONE } SCD_ID_t;

typedef enum SCD_Reg_ {
  CMD_ALTITUDE_GET   = 0x2322u,
  CMD_ALTITUDE_SET   = 0x2427u,
  CMD_PERIODIC_START = 0x21b1u,
  CMD_PERSIST_CFG    = 0x3615u,
  CMD_SAMPLE_READ    = 0xec05u,
  CMD_SAMPLE_READY   = 0xe4b8u,
  CMD_SAMPLE_SINGLE  = 0x219du,
  CMD_SCD_VARIANT    = 0x202fu
} SCD_Reg_t;

typedef enum SCD_Resp_ {
  SCD_RESP_OK,
  SCD_RESP_I2C_FAILED,
  SCD_RESP_BADCRC
} SCD_Resp_t;

typedef struct SCD_Cmd_ {
  SCD_Reg_t cmd;    /* Command address */
  RWC_t     rwc;    /* Read, write, or command */
  size_t    n;      /* Number of data bytes (0 for command) */
  uint16_t  t_wait; /* Time between command and read transfer (ms) */
} SCD_Cmd_t;

const uint8_t addr7 = 0x62u;

SCD_Cmd_t cmdAltitudeGet = {
    .cmd = CMD_ALTITUDE_GET, .n = 2, .rwc = R, .t_wait = 1};

SCD_Cmd_t cmdAltitudeSet = {
    .cmd = CMD_ALTITUDE_SET, .n = 2, .rwc = W, .t_wait = 1};

SCD_Cmd_t cmdPeriodicStart = {
    .cmd = CMD_PERIODIC_START, .n = 0, .rwc = C, .t_wait = 0};

SCD_Cmd_t cmdPersistCfg = {
    .cmd = CMD_PERSIST_CFG, .n = 0, .rwc = C, .t_wait = 800};

SCD_Cmd_t cmdSampleRead = {
    .cmd = CMD_SAMPLE_READ, .n = 2, .rwc = R, .t_wait = 1};

SCD_Cmd_t cmdSampleReady = {
    .cmd = CMD_SAMPLE_READY, .n = 2, .rwc = R, .t_wait = 1};

SCD_Cmd_t cmdSampleSingle = {
    .cmd = CMD_SAMPLE_SINGLE, .n = 0, .rwc = C, .t_wait = 5000};

SCD_Cmd_t cmdSCDVariant = {
    .cmd = CMD_SCD_VARIANT, .n = 2, .rwc = R, .t_wait = 1};

static void       byteSwap(uint8_t *pBuf);
static SCD_Resp_t cmdExecute(const SCD_Cmd_t cmd, uint8_t *pData);
static uint8_t    crcCalc(const uint8_t *pData, const size_t n);
static void       initSCD(const uint16_t altitude);
static uint16_t   measureSCD40(void);
static void       powerOff(void);
static void       powerOn(void);
static void       printInfo(void);
static void       regRead(const SCD_Cmd_t cmd, uint8_t *pData);
static void       regWrite(const uint8_t *pData);

static SCD_ID_t scdID;

static void byteSwap(uint8_t *pBuf) {
  uint8_t tmp0 = pBuf[0];
  pBuf[0]      = pBuf[1];
  pBuf[1]      = tmp0;
}

static SCD_Resp_t cmdExecute(const SCD_Cmd_t cmd, uint8_t *pData) {
  uint8_t addr8  = addr7 << 1;
  uint8_t regMSB = (cmd.cmd >> 8) & 0xFF;
  uint8_t regLSB = cmd.cmd & 0xFF;

  if (I2CM_SUCCESS != i2cActivate(addr8)) {
    return SCD_RESP_I2C_FAILED;
  }

  i2cDataWrite(regMSB);
  i2cDataWrite(regLSB);

  if (W == cmd.rwc) {
    regWrite(pData);
    timerDelaySleep_ms(cmd.t_wait);
    return SCD_RESP_OK;
  } else if (R == cmd.rwc) {
    regRead(cmd, pData);
  } else if (C == cmd.rwc) {
    i2cAck(I2CM_NACK, I2CM_ACK_CMD_STOP);
    timerDelaySleep_ms(cmd.t_wait);
  }

  return SCD_RESP_OK;
}

static uint8_t crcCalc(const uint8_t *pData, const size_t n) {
  /* See section 3.12 Checksum Calculation in SCD4x datasheet */

  uint8_t crc = 0xFFu;
  for (size_t i = 0; i < n; i++) {
    crc ^= pData[i];
    for (size_t bit = 8; bit > 0; bit--) {
      if (crc & 0x80u) {
        crc = (crc << 1) ^ 0x31u;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

static void powerOff(void) { portPinDrv(PIN_EXT_EN, PIN_DRV_CLR); }

static void powerOn(void) {
  /* Table 7 - requires max. 30 ms power up time */
  portPinDrv(PIN_EXT_EN, PIN_DRV_SET);
  timerDelaySleep_ms(30);
}

static void initSCD(uint16_t altitude) {
  uint8_t dBuf[2];

  /* Check altitude has been set as configured */
  cmdExecute(cmdAltitudeGet, dBuf);
  byteSwap(dBuf);
  if (altitude != *(uint16_t *)dBuf) {
    *(uint16_t *)dBuf = altitude;
    byteSwap(dBuf);
    cmdExecute(cmdAltitudeSet, dBuf);
    cmdExecute(cmdPersistCfg, NULL);
  }

  if (SCD40 == scdID) {
    cmdExecute(cmdPeriodicStart, NULL);
  }
}

static uint16_t measureSCD40(void) {
  uint8_t dbuf[2];

  /* the 11 LSBs of of data ready are 0 when not ready */
  do {
    cmdExecute(cmdSampleReady, dbuf);
  } while (0 == (dbuf[1] & 0x7FF));

  cmdExecute(cmdSampleRead, dbuf);
  byteSwap(dbuf);
  return *(uint16_t *)dbuf;
}

static void printInfo(void) {

  uartPuts("  - SCD4x... ");

  switch (scdID) {
  case SCD40:
    uartPuts("SCD40");
    break;
  case SCD41:
    uartPuts("SCD41");
    break;
  case SCD43:
    uartPuts("SCD43");
    break;
  case SCD4x_NONE:
    uartPuts("None");
  }
  uartPuts("\r\n");
}

static void regRead(const SCD_Cmd_t cmd, uint8_t *pData) {
  uint8_t addr8 = (addr7 << 1) | 0x1;

  timerDelaySleep_ms(cmd.t_wait);
  if (I2CM_SUCCESS == i2cActivate(addr8)) {
    for (size_t i = cmd.n; i > 0; i--) {
      pData[i - 1] = i2cDataRead();
      i2cAck(I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
    }
  }

  /* Revisit : discarding CRC for now. */
  (void)i2cDataRead();
  i2cAck(I2CM_NACK, I2CM_ACK_CMD_STOP);
}

static void regWrite(const uint8_t *pData) {
  /* All commands are 16 bit, MSB first */
  i2cDataWrite(pData[1]);
  i2cDataWrite(pData[0]);
  i2cDataWrite(crcCalc(pData, 2));
  i2cAck(I2CM_ACK, I2CM_ACK_CMD_STOP);
}

void scd4xDiscover(const uint16_t altitude) {
  uint8_t rxBuf[2];
  scdID = SCD4x_NONE;

  powerOn();

  if (SCD_RESP_OK == cmdExecute(cmdSCDVariant, rxBuf))
    switch (rxBuf[1]) {
    case 0x0u:
      scdID = SCD40;
      break;
    case 0x1u:
      scdID = SCD41;
      break;
    case 0x5u:
      scdID = SCD43;
      break;
    default:
      scdID = SCD4x_NONE;
    }

  printInfo();

  if (SCD4x_NONE != scdID) {
    initSCD(altitude);
  }

  /* SCD40 does not support power cycling, leave on */
  if (SCD40 != scdID) {
    powerOff();
  }
}

uint16_t scd4xMeasureCO2(void) {

  /* SCD40 does not support power cycled sampling */
  if (scdID == SCD40) {
    return measureSCD40();
  }

  uint16_t co2;
  bool     i2cIsEnabled = i2cEnabled();

  if (!i2cIsEnabled) {
    i2cEnable();
  }

  powerOn();

  /* In power cycled mode, need to discard the first sample as junk */
  cmdExecute(cmdSampleSingle, NULL);
  cmdExecute(cmdSampleSingle, NULL);
  /* Revisit : check for data ready? */
  cmdExecute(cmdSampleRead, (uint8_t *)&co2);

  if (!i2cIsEnabled) {
    i2cDisable();
  }

  powerOff();
  byteSwap((uint8_t *)&co2);
  return co2;
}

bool scd4xPresent(void) { return scdID != SCD4x_NONE; }
