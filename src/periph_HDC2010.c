#include "periph_HDC2010.h"
#include "board_def.h"
#include "driver_EIC.h"
#include "driver_SAML.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"

#define HDC_ADDR 0x40 /* ADDR tied LOW */

static const uint8_t HDC2010_TEMP_LSB        = 0x00u;
static const uint8_t HDC2010_INT_CFG         = 0x07u;
static const uint8_t HDC2010_DRDYINT_CFG     = 0x0Eu;
static const uint8_t HDC2010_MEASUREMENT_CFG = 0x0Fu;

static volatile bool sampleStarted = false;
static volatile bool sampleReady   = false;

static void hdc2010Interrupt(void);
static void hdc2010RegNRead(const uint8_t ptrStart, void *pDst, const int n);
static bool hdc2010RegWrite(const uint8_t reg, const uint8_t data);

void hdc2010ConversionStart(void) {
  hdc2010RegWrite(HDC2010_MEASUREMENT_CFG, 0x01);
  // samlSetActivity(SLEEP_MODE_STANDBY, PERIPH_IDX_I2CM);
  sampleStarted = true;
}

bool hdc2010ConversionStarted(void) { return sampleStarted; }

static void hdc2010Interrupt(void) { sampleReady = true; }

static void hdc2010RegNRead(const uint8_t ptrStart, void *pDst, const int n) {
  uint8_t *buffer = (uint8_t *)pDst;
  // samlSetActivity(SLEEP_MODE_IDLE, PERIPH_IDX_I2CM);
  if (I2CM_SUCCESS == i2cActivate((HDC_ADDR << 1))) {
    i2cDataWrite(ptrStart);
    i2cAck(I2CM_ACK, I2CM_ACK_CMD_STOP);
  }
  if (I2CM_SUCCESS == (i2cActivate(((HDC_ADDR << 1) + 1)))) {
    for (int i = 0; i < n; i++) {
      *buffer++ = i2cDataRead();
      if (i < (n - 1)) {
        i2cAck(I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
      }
    }
    i2cAck(I2CM_NACK, I2CM_ACK_CMD_STOP);
  }
  // samlSetActivity(SLEEP_MODE_STANDBY, PERIPH_IDX_I2CM);
}

static bool hdc2010RegWrite(const uint8_t reg, const uint8_t data) {
  if (I2CM_SUCCESS == i2cActivate((HDC_ADDR << 1))) {
    i2cDataWrite(reg);
    i2cDataWrite(data);
    i2cAck(I2CM_ACK, I2CM_ACK_CMD_STOP);
    return true;
  } else {
    return false;
  }
}

void hdc2010SampleGet(HDCResultRaw_t *pRes) {
  hdc2010RegNRead(HDC2010_TEMP_LSB, pRes, sizeof(*pRes));
}

bool hdc2010SampleReady(void) { return sampleReady; }

bool hdc2010Setup(void) {
  eicChannelEnable((EIC_Cfg_t){.cb    = &hdc2010Interrupt,
                               .ch    = EIC_CH_HDC,
                               .pin   = PIN_HDC_DRDY,
                               .sense = EIC_SENSE_HDC});

  if (!hdc2010RegWrite(HDC2010_INT_CFG, (1 << 7))) {
    return false;
  }
  /* Interrupt output enabled, active HIGH */
  return hdc2010RegWrite(HDC2010_DRDYINT_CFG, ((1 << 2) | (1 << 1)));
}
