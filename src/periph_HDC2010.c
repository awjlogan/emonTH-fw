#include <stddef.h>

#include "driver_EIC.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "periph_HDC2010.h"

#define HDC_ADDR (0x40) /* ADDR tied LOW */

static const uint8_t HDC2010_TEMP_LSB        = 0x00u;
static const uint8_t HDC2010_INT_CFG         = 0x07u;
static const uint8_t HDC2010_DRDYINT_CFG     = 0x0Eu;
static const uint8_t HDC2010_MEASUREMENT_CFG = 0x0Fu;

static volatile bool sampleReady = false;

static void hdc2010Interrupt(void);
static void hdc2010RegNRead(const uint8_t ptrStart, void *pDst, const size_t n);
static bool hdc2010RegWrite(const uint8_t reg, const uint8_t data);

void hdc2010ConversionStart(void) {
  sampleReady = false;
  hdc2010RegWrite(HDC2010_MEASUREMENT_CFG, 0x01);
}

static void hdc2010Interrupt(void) { sampleReady = true; }

static void hdc2010RegNRead(const uint8_t ptrStart, void *pDst,
                            const size_t n) {
  uint8_t *buffer = (uint8_t *)pDst;
  if (I2CM_SUCCESS == i2cActivate((HDC_ADDR << 1))) {
    i2cDataWrite(ptrStart);

    if (I2CM_SUCCESS == (i2cActivate(((HDC_ADDR << 1) + 1u)))) {
      for (size_t i = 0; i < n; i++) {
        *buffer++ = i2cDataRead();
        if (i < (n - 1u)) {
          i2cAck(I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
        }
      }
      i2cAck(I2CM_NACK, I2CM_ACK_CMD_STOP);
      return;
    }
  }
  i2cAck(I2CM_NACK, I2CM_ACK_CMD_STOP);
}

static bool hdc2010RegWrite(const uint8_t reg, const uint8_t data) {
  if (I2CM_SUCCESS == i2cActivate((HDC_ADDR << 1))) {
    i2cDataWrite(reg);
    i2cDataWrite(data);
    i2cAck(I2CM_ACK, I2CM_ACK_CMD_STOP);
    return true;
  }

  i2cAck(I2CM_ACK, I2CM_ACK_CMD_STOP);
  return false;
}

void hdc2010SampleGet(HDCResultRaw_t *pRes) {
  hdc2010RegNRead(HDC2010_TEMP_LSB, pRes, sizeof(*pRes));
  sampleReady = false;
}

bool hdc2010SampleReady(void) { return sampleReady; }

bool hdc2010Setup(void) {
  eicSetupHDC(&hdc2010Interrupt);

  /* Soft reset the HDC2010 to clear any outstanding state. Soft reset is
   * treated as power on, requires at most 3 ms to become active
   * (Section 7.5.2). */
  if (!hdc2010RegWrite(HDC2010_DRDYINT_CFG, (1u << 7))) {
    return false;
  }
  timerDelaySleep_ms(3u);

  /* Enable data ready interrupt */
  if (!hdc2010RegWrite(HDC2010_INT_CFG, (1u << 7))) {
    return false;
  }

  /* Interrupt output enabled, active HIGH */
  return hdc2010RegWrite(HDC2010_DRDYINT_CFG, ((1u << 2) | (1u << 1)));
}
