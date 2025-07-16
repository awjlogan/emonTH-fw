#include "driver_DMAC.h"
#include "driver_ADC.h"
#include "driver_PORT.h"
#include "emonTH_saml.h"

#include "emonTH.h"

void irqHandlerADCCommon(void);

static volatile DmacDescriptor dmacs[NUM_CHAN_DMA];
static DmacDescriptor          dmacs_wb[NUM_CHAN_DMA];

static volatile bool dmacComplete[2] = {0};

/* Useful ref: https://aykevl.nl/2019/09/samd21-dma */

void dmacSetup(void) {
  /* Clocking - AHB enabled at reset (19.8.6 AHB Mask) */
  DMAC->BASEADDR.reg = (uint32_t)dmacs;
  DMAC->WRBADDR.reg  = (uint32_t)dmacs_wb;
  DMAC->CTRL.reg     = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xFu);

  /* CRC module - CRC16-CCITT byte wise access from IO */
  DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCSRC_IO;

  dmacComplete[0] = true;
  dmacComplete[1] = true;

  /* Enable the DMAC interrupt in the NVIC, but leave the channel interrupt
   * enable/disable for each channel to the peripheral */
  NVIC_EnableIRQ(DMAC_UART_IRQn);
  NVIC_EnableIRQ(DMAC_SPI_IRQn);
}

volatile DmacDescriptor *dmacGetDescriptor(unsigned int ch) {
  return &dmacs[ch];
}

void dmacChannelDisable(unsigned int ch) {
  DMAC->CHID.reg = ch;
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
}

void dmacChannelEnable(unsigned int ch) {
  DMAC->CHID.reg = ch;
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
  dmacComplete[ch] = false;
}

void dmacEnableChannelInterrupt(unsigned int ch) {
  DMAC->CHID.reg       = ch;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;
}

void dmacClearChannelInterrupt(unsigned int ch) {
  DMAC->CHID.reg      = ch;
  DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
}

void dmacChannelConfigure(unsigned int ch, const uint32_t ctrlb) {
  DMAC->CHID.reg    = ch;
  DMAC->CHCTRLB.reg = ctrlb;
}

uint16_t calcCRC16_ccitt(const void *pSrc, unsigned int n) {
  const uint8_t *pData = (uint8_t *)pSrc;

  /* CCITT is 0xFFFF initial value */
  DMAC->CRCCHKSUM.reg = 0xFFFF;
  DMAC->CTRL.reg |= DMAC_CTRL_CRCENABLE;

  /* Input into CRC data byte wise. Byte beats convert in a single cycle, so
   * using a DSB to ensure the previous store is complete is sufficient.
   */
  while (n--) {
    DMAC->CRCDATAIN.reg = *pData++;
    __DSB();
  }

  /* Clear status and disable CRC module before returning CRC */
  DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
  DMAC->CTRL.reg &= ~DMAC_CTRL_CRCENABLE;

  return DMAC->CRCCHKSUM.reg;
}

bool dmacSPIComplete(void) { return dmacComplete[1]; }

bool dmacUARTComplete(void) { return dmacComplete[0]; }

/* UART DMA handler */
void irq_handler_dmac_0(void) {
  dmacClearChannelInterrupt(DMA_CHAN_UART);
  dmacComplete[0] = true;
}

/* SPI DMA handler */
void irq_handler_dmac_1(void) {
  dmacClearChannelInterrupt(DMA_CHAN_SPI);
  dmacComplete[1] = true;
}
