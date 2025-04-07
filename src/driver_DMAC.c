#include "driver_DMAC.h"
#include "driver_ADC.h"
#include "emonTH_saml.h"

#include "emonTH.h"

void irqHandlerADCCommon(void);

static volatile DmacDescriptor dmacs[NUM_CHAN_DMA];
static DmacDescriptor          dmacs_wb[NUM_CHAN_DMA];
static volatile bool           uartDmaCmpl = false;

/* Useful ref: https://aykevl.nl/2019/09/samd21-dma */

void dmacSetup(void) {
  /* Clocking - AHB enabled at reset (19.8.6 AHB Mask) */
  DMAC->BASEADDR.reg = (uint32_t)dmacs;
  DMAC->WRBADDR.reg  = (uint32_t)dmacs_wb;
  DMAC->CTRL.reg     = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xFu);

  /* CRC module - CRC16-CCITT byte wise access from IO */
  DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCSRC_IO;

  /* Enable the DMAC interrupt in the NVIC, but leave the channel interrupt
   * enable/disable for each channel to the peripheral */
  NVIC_EnableIRQ(DMAC_0_IRQn);
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
  if (DMA_CHAN_UART == ch) {
    uartDmaCmpl = false;
  }
}

void dmacEnableChannelInterrupt(unsigned int ch) {
  DMAC->CHID.reg = ch;
  DMAC->CHINTENSET.reg |= DMAC_CHINTENSET_TCMPL;
}

void dmacDisableChannelInterrupt(unsigned int ch) {
  DMAC->CHID.reg = ch;
  DMAC->CHINTENCLR.reg |= DMAC_CHINTENCLR_TCMPL;
}

void dmacClearChannelInterrupt(unsigned int ch) {
  DMAC->CHID.reg = ch;
  DMAC->CHINTFLAG.reg |= DMAC_CHINTFLAG_TCMPL;
}

void dmacChannelConfigure(unsigned int ch, const uint32_t ctrlb) {
  DMAC->CHID.reg    = ch;
  DMAC->CHCTRLB.reg = ctrlb;
}

void dmacChannelResume(unsigned int ch) {
  DMAC->CHID.reg = ch;
  DMAC->CHCTRLB.reg |= DMAC_CHCTRLB_CMD_RESUME;
}

void dmacChannelSuspend(unsigned int ch) {
  DMAC->CHID.reg = ch;
  DMAC->CHCTRLB.reg |= DMAC_CHCTRLB_CMD_SUSPEND;
}

bool dmacChannelBusy(unsigned int ch) {
  if (0 != (DMAC->BUSYCH.reg & (1u << ch))) {
    return 1u;
  } else {
    return 0;
  }
}

void irq_handler_dmac(void) {
  DMAC->CHID.reg = DMA_CHAN_UART;
  if (DMAC->CHINTFLAG.reg & DMAC_CHINTFLAG_TCMPL) {
    uartDmaCmpl         = true;
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
  }

  DMAC->CHID.reg = DMA_CHAN_SPI;
  if (DMAC->CHINTFLAG.reg & DMAC_CHINTFLAG_TCMPL) {
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
  }
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

bool uartDmaComplete(void) { return uartDmaCmpl; }
