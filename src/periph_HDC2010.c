#include "periph_HDC2010.h"
#include "board_def.h"
#include "driver_EIC.h"
#include "driver_SAML.h"
#include "driver_SERCOM.h"
#include "qfplib.h"

static const uint8_t HDC2010_TEMP_LSB        = 0x00u;
static const uint8_t HDC2010_DRDYINT_CFG     = 0x0Eu;
static const uint8_t HDC2010_MEASUREMENT_CFG = 0x0Fu;

typedef enum HDCState_ {
  HDC_STATE_SETUP,
  HDC_STATE_TRIG,
  HDC_STATE_SAMPLE,
  HDC_STATE_READ
} HDCState_t;

/* Interrupt driven driver for HDC2010 temperature/humidity sensor
 *
 * Usage:
 *   - Setup once at startup.
 *     - 0x07: Interrupt Configuration Register = 0x80 (DRDY_ENABLE)
 *     - 0x0E: Configuration Register = 0x06 (INT_EN | INT_POL)
 *   - Functional:
 *     1. TRIG
 *       - Write: Measurement Configuration Register = 0x01
 *       - Power: IDLE1
 *     2. SAMPLE
 *       - Interrupt: EIC
 *       - Power: STANDBY
 */
static HDCState_t hdcState = HDC_STATE_SETUP;

static uint8_t hdc2010RegRead(const uint8_t reg);
static void    hdc2010RegNRead(const uint8_t ptrStart, void *pDst, const int n);
static void    hdc2010RegWrite(const uint8_t reg, const uint8_t data);
static void    hdc2010SampleRead(void);

void hdc2010InterruptClear(void) {
  /* Clear the level interrupt */
  eicLevelDisable(EIC_CH_HDC);
  (void)hdc2010RegRead(HDC2010_DRDYINT_CFG);
  hdc2010SampleRead();
}

void hdc2010ConversionStart(void) {
  hdc2010RegWrite(HDC2010_MEASUREMENT_CFG, 0x01);

  hdcState = HDC_STATE_SAMPLE;
  samlSetActivity(SLEEP_MODE_STANDBY, PERIPH_IDX_I2CM);
  eicLevelEnable(EIC_CH_HDC, hdc2010InterruptClear);
}

static uint8_t hdc2010RegRead(const uint8_t reg) {
  uint8_t result = 0;
  samlSetActivity(SLEEP_MODE_IDLE1, PERIPH_IDX_I2CM);
  if (I2CM_SUCCESS == i2cActivate(SERCOM_I2CM, (HDC_ADDR << 1))) {
    i2cDataWrite(SERCOM_I2CM, reg);
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);
  }
  if (I2CM_SUCCESS == (i2cActivate(SERCOM_I2CM, ((HDC_ADDR << 1) + 1)))) {
    result = i2cDataRead(SERCOM_I2CM);
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);
  }
  return result;
}

static void hdc2010RegNRead(const uint8_t ptrStart, void *pDst, const int n) {
  uint8_t *buffer = (uint8_t *)pDst;
  samlSetActivity(SLEEP_MODE_IDLE1, PERIPH_IDX_I2CM);
  if (I2CM_SUCCESS == i2cActivate(SERCOM_I2CM, (HDC_ADDR << 1))) {
    i2cDataWrite(SERCOM_I2CM, ptrStart);
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);
  }
  if (I2CM_SUCCESS == (i2cActivate(SERCOM_I2CM, ((HDC_ADDR << 1) + 1)))) {
    i2cEnableSmartMode(SERCOM_I2CM);
    for (int i = 0; i < n; i++) {
      *buffer++ = i2cDataRead(SERCOM_I2CM);
    }
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);
  }
}

static void hdc2010RegWrite(const uint8_t reg, const uint8_t data) {
  samlSetActivity(SLEEP_MODE_IDLE1, PERIPH_IDX_I2CM);
  if (I2CM_SUCCESS == i2cActivate(SERCOM_I2CM, (HDC_ADDR << 1))) {
    i2cDataWrite(SERCOM_I2CM, reg);
    i2cDataWrite(SERCOM_I2CM, data);
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);
  }
}

void hdc2010SampleGet(HDCResultInt_t *pRes) {
  hdc2010RegNRead(HDC2010_TEMP_LSB, pRes, sizeof(*pRes));
}

void hdc2010SampleItoF(const HDCResultInt_t *pInt, HDCResultF_t *pF) {
  /* Datasheet 7.6.2 */
  int inter = (int)pInt->temp * 165;
  pF->temp  = qfp_fsub(qfp_fdiv(qfp_int2float(inter), (float)(1 << 16)), 40.0f);

  /* Datasheet 7.6.4 */
  inter        = (int)pInt->humidity * 100;
  pF->humidity = qfp_fdiv(qfp_int2float(inter), (float)(1 << 16));
}

static void hdc2010SampleRead(void) { /* Trigger the I2C read using DMA */ }

void hdc2010Setup(void) {
  /* Enable DRDY/INT_EN as active HIGH */
  hdc2010RegWrite(HDC2010_DRDYINT_CFG, ((1 << 2) | (1 << 1)));
}
