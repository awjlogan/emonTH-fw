#include "emonTH_saml.h"

#include "board_def.h"
#include "configuration.h"
#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "util.h"

#define I2CM_ACTIVATE_TIMEOUT_US 200u /* Time to wait for I2C bus */

/* ======= SERCOM assignment and configuration =======
 * UART:
 *  SERCOM0
 *    Tx (data out):  pin 23, PA24, pad 2 (TXPO = 1)
 *    Rx (data in):   pin 24, PA25, pad 3 (RXPO = 3)
 * SPI:
 *  SERCOM1
 *    !SS : pin 16, PA15
 *    MOSI: pin 17, PA16, pad 0 (DOPO = 3)
 *    MISO: pin 19, PA18, pad 2 (DIPO = 2)
 *    SCK : pin 20, PA18, pad 3 (DOPO = 3)
 * I2C:
 *  SERCOM2 (Alt)
 *    SDA: pin 11, PA08, pad 0 (37.4 Signal Description)
 *    SCL: pin 12, PA09, pad 1
 */

// static void setupI2C(void);
static void setupSPI(void);
// static void setupUart(void);

static volatile bool i2cTimeout = false;

void setupI2C(void) {
  portPinMux(PIN_I2CM_SDA, PMUX_I2CM);
  portPinMux(PIN_I2CM_SCL, PMUX_I2CM);

  MCLK->APBCMASK.reg |= SERCOM_I2CM_APBCMASK;
  GCLK->PCHCTRL[SERCOM_I2CM_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
  GCLK->PCHCTRL[SERCOM_I2CM_GCLK_SLOW_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

  SERCOM_I2CM->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_SWRST;
  while (SERCOM_I2CM->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SWRST)
    ;

  /* For 400 kHz I2C, SCL T_high >= 0.6 us, T_low >= 1.3 us, with
   * (T_high + T_low) <= 2.5 us, and T_low / T_high ~ 1.8.
   * From I2C Host Clock generation (37.6.2.4.1):
   * BAUD.BAUDLOW = (T_low * f_clk) - 5 (1.625 us -> 8 @ 8 MHz)
   * BAUD.BAUD = (T_high * f_clk) - 5 (0.875 us -> 2 @ 8 MHz)
   */
  SERCOM_I2CM->I2CM.BAUD.reg =
      SERCOM_I2CM_BAUD_BAUDLOW(8u) | SERCOM_I2CM_BAUD_BAUD(2u);

  /* Configure I2C SERCOM as host */
  SERCOM_I2CM->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE(5);

  /* Enable SERCOM, with sync */
  SERCOM_I2CM->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;
  while (SERCOM_I2CM->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP)
    ;

  /* After enabling the I2C SERCOM, the bus state is UNKNOWN (Table 28-13)
   * Force into IDLE state, with sync
   */
  SERCOM_I2CM->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(0x1u);
  while (SERCOM_I2CM->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP)
    ;

  SERCOM_I2CM->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB |
                                   SERCOM_I2CM_INTENSET_SB |
                                   SERCOM_I2CM_INTENSET_ERROR;
}

static void setupSPI(void) {

  /* Configure clocks - runs from the OSC16M clock on gen 0 */
  MCLK->APBCMASK.reg |= SERCOM_SPI_APBCMASK;
  GCLK->PCHCTRL[SERCOM_SPI_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;

  portPinDrv(PIN_SPI_RFM_SS, PIN_DRV_SET);
  portPinMux(PIN_SPI_MISO, PMUX_SPI_DATA);
  portPinMux(PIN_SPI_MOSI, PMUX_SPI_DATA);
  portPinMux(PIN_SPI_SCK, PMUX_SPI_DATA);

  /* Table 34-3 - driven @ F_CORE = 8 MHz. BAUD = (F_CORE / 2F_BAUD) - 1
   * RFM69 maximum SCK is 10 MHz, clock SPI SCK @ 4 MHz.
   */
  SERCOM_SPI->SPI.BAUD.reg = (F_CORE / (2 * SPI_DATA_BAUD)) - 1;

  /* SPI mode 0: CPOL == 0, CPHA == 0 */
  SERCOM_SPI->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE(3) | SPI_DOPO | SPI_DIPO;

  /* Enable TX and RX interrupts (complete and empty), not routed to NVIC */
  SERCOM_SPI->SPI.INTENSET.reg |= SERCOM_SPI_INTENSET_RXC |
                                  SERCOM_SPI_INTENSET_TXC |
                                  SERCOM_SPI_INTENSET_DRE;

  /* While disabled, RXEN will be set immediately. When the SPI SERCOM is
   * enabled, this requires synchronisation before the SPI is ready. See
   * field description in 27.8.2
   */
  SERCOM_SPI->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;
  SERCOM_SPI->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
  while (0 != SERCOM_SPI->SPI.SYNCBUSY.reg)
    ;
}

void setupUart(void) {
  uint32_t sampr;
  uint32_t sampc;
  uint32_t baud;

  if (F_CORE >= (16 * UART_BAUD)) {
    sampr = 0;
    sampc = 16;
  } else if (F_CORE >= (8 * UART_BAUD)) {
    sampr = 2;
    sampc = 8;
  } else if (F_CORE >= (3 * UART_BAUD)) {
    sampr = 4;
    sampc = 3;
  }

  baud = 65536u - (uint32_t)(((uint64_t)65536u * sampc * UART_BAUD) / F_CORE);

  portPinMux(PIN_UART_TX, PMUX_UART);
  portPinMux(PIN_UART_RX, PMUX_UART);

  /* Configure clocks - runs from the OSC16M gen 0 @ 8 MHz */
  MCLK->APBCMASK.reg |= SERCOM_UART_APBCMASK;
  GCLK->PCHCTRL[SERCOM_UART_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[SERCOM_UART_GCLK_ID].reg & GCLK_PCHCTRL_CHEN))
    ;

  SERCOM_UART->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
  while (SERCOM_UART->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_SWRST)
    ;

  /* Configure the USART */
  SERCOM_UART->USART.CTRLA.reg =
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_SAMPR(sampr) |
      SERCOM_USART_CTRLA_MODE(1) | UART_RXPO | UART_TXPO;

  /* TX/RX enable requires synchronisation */
  SERCOM_UART->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN |
                                 SERCOM_USART_CTRLB_TXEN |
                                 SERCOM_USART_CTRLB_CHSIZE(0);
  while (SERCOM_UART->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_CTRLB)
    ;

  SERCOM_UART->USART.BAUD.reg = (uint16_t)baud;

  /* Enable requires synchronisation (35.6.6) */
  SERCOM_UART->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
  while (SERCOM_UART->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_ENABLE)
    ;

  SERCOM_UART->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  NVIC_EnableIRQ(SERCOM_UART_RXC_IRQn);
}

void sercomDisable(Sercom *sercom) {
  /* Disable the SERCOM instance and gate GCLK */
  if (SERCOM_SPI == sercom) {
    sercom->SPI.CTRLA.reg &= ~(SERCOM_SPI_CTRLA_ENABLE);
    GCLK->PCHCTRL[SERCOM_SPI_GCLK_ID].reg &= ~GCLK_PCHCTRL_CHEN;
  } else if (SERCOM_UART == sercom) {
    SERCOM_UART->USART.CTRLA.reg &= ~(SERCOM_USART_CTRLA_ENABLE);
    GCLK->PCHCTRL[SERCOM_UART_GCLK_ID].reg &= ~GCLK_PCHCTRL_CHEN;
  }
}

void sercomSetup(void) {
  // setupI2C();
  setupSPI();
  setupUart();
}

/*
 * =====================================
 * UART Functions
 * =====================================
 */

void uartFlush(void) {
  while (!(SERCOM_UART->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE))
    ;
}

void uartPutcBlocking(const char c) {
  while (!(SERCOM_UART->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE))
    ;
  SERCOM_UART->USART.DATA.reg = c;
  SERCOM_UART->USART.INTFLAG.reg |= SERCOM_USART_INTFLAG_DRE;
}

void uartPutsBlocking(const char *s) {
  while (*s)
    uartPutcBlocking(*s++);
}

void uartConfigureDMA(void) {

  volatile DmacDescriptor *dmacDesc = dmacGetDescriptor(DMA_CHAN_UART);
  dmacDesc->BTCTRL.reg = DMAC_BTCTRL_VALID | DMAC_BTCTRL_BLOCKACT_INT |
                         DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC |
                         DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE;

  dmacDesc->DSTADDR.reg  = (uint32_t)&SERCOM_UART->USART.DATA;
  dmacDesc->DESCADDR.reg = 0u;

  dmacChannelConfigure(DMA_CHAN_UART,
                       (DMAC_CHCTRLB_LVL(1u) |
                        DMAC_CHCTRLB_TRIGSRC(SERCOM_UART_DMAC_ID_TX) |
                        DMAC_CHCTRLB_TRIGACT_BEAT));

  dmacEnableChannelInterrupt(DMA_CHAN_UART);
}

void uartPutsNonBlocking(const char *const s, uint16_t len) {
  volatile DmacDescriptor *dmacDesc = dmacGetDescriptor(DMA_CHAN_UART);
  /* Valid bit is cleared when a channel is complete */
  dmacDesc->BTCTRL.reg |= DMAC_BTCTRL_VALID;
  dmacDesc->BTCNT.reg   = len;
  dmacDesc->SRCADDR.reg = (uint32_t)s + len;
  dmacChannelEnable(DMA_CHAN_UART);
}

char uartGetc(void) { return SERCOM_UART->USART.DATA.reg; }

bool uartGetcReady(void) {
  return (bool)(SERCOM_UART->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC);
}

void uartInterruptEnable(const uint32_t interrupt) {
  SERCOM_UART->USART.INTENSET.reg |= interrupt;
}

void uartInterruptDisable(const uint32_t interrupt) {
  SERCOM_UART->USART.INTENCLR.reg |= interrupt;
}

void uartRxDisable(void) {
  SERCOM_UART->USART.CTRLB.reg &= ~SERCOM_USART_CTRLB_RXEN;
  while (SERCOM_UART->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_CTRLB)
    ;
  /* Errata 2.12.6 Overconsumption in Standby mode. Configure RXPO and TXPO to
   * use the same pad to remove GCLK request. */
  SERCOM_UART->USART.CTRLA.reg =
      (SERCOM_UART->USART.CTRLA.reg & ~SERCOM_USART_CTRLA_RXPO_Msk) |
      SERCOM_USART_CTRLA_RXPO(UART_PAD_TX);
  NVIC_DisableIRQ(SERCOM_UART_RXC_IRQn);
}

uint32_t uartInterruptStatus(void) { return SERCOM_UART->USART.INTFLAG.reg; }

void uartInterruptClear(uint32_t interrupt) {
  SERCOM_UART->USART.INTFLAG.reg |= interrupt;
}

/*
 * =====================================
 * I2C Functions
 * =====================================
 */
I2CM_Status_t i2cActivate(uint8_t addr) {
  I2CM_Status_t s = I2CM_SUCCESS;

  SERCOM_I2CM->I2CM.ADDR.reg = SERCOM_I2CM_ADDR_ADDR(addr);
  /* MB: master on bus, SB: slave on bus */
  while (!(SERCOM_I2CM->I2CM.INTFLAG.reg &
           (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB))) {
  }

  /* Check for NoAck response from client (37.6.2.4.2 Transmitting Address
   * Packets) */
  if (SERCOM_I2CM->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK) {
    s = I2CM_NOACK;
  }

  return s;
}

void i2cAck(I2CM_Ack_t ack, I2CM_AckCmd_t cmd) {
  SERCOM_I2CM->I2CM.CTRLB.reg =
      (ack << SERCOM_I2CM_CTRLB_ACKACT_Pos) | SERCOM_I2CM_CTRLB_CMD(cmd);
  while (SERCOM_I2CM->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP)
    ;
}

void i2cDataWrite(uint8_t data) {
  SERCOM_I2CM->I2CM.DATA.reg = data;
  while (!(SERCOM_I2CM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB))
    ;
}

uint8_t i2cDataRead(void) {
  while (!(SERCOM_I2CM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB))
    ;
  return SERCOM_I2CM->I2CM.DATA.reg;
}

void i2cEnableSmartMode(void) {
  SERCOM_I2CM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_SMEN;
}

void i2cSetTimeout(void) { i2cTimeout = true; }

/*
 * =====================================
 * SPI Functions
 * =====================================
 */

void spiDeSelect(const Pin_t nSS) { portPinDrv(nSS.pin, PIN_DRV_SET); }

void spiSelect(const Pin_t nSS) { portPinDrv(nSS.pin, PIN_DRV_CLR); }

void spiSendBuffer(Sercom *sercom, const void *pSrc, int n) {
  uint8_t *pData = (uint8_t *)pSrc;

  while (n--) {
    while (0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_DRE))
      ;
    sercom->SPI.DATA.reg = *pData++;
  }
}

uint8_t spiSendByte(Sercom *sercom, const uint8_t b) {
  while (0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_DRE))
    ;
  sercom->SPI.DATA.reg = b;

  while (0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_RXC))
    ;
  return (uint8_t)sercom->SPI.DATA.reg;
}
