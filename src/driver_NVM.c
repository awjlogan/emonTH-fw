#include <string.h>

#include "driver_NVM.h"
#include "emonTH_assert.h"
#include "emonTH_saml.h"

/* The NVM page buffer must be 4 byte aligned for allow access from DFLASH */
uint8_t pageBuffer[FLASH_PAGE_SIZE] __attribute__((aligned(16))) = {0};

void nvmDataFlashRead(const NVMPage_t page) {
  EMONTH_ASSERT(page < NVMCTRL_DATAFLASH_PAGES);

  uint32_t *pDst = (uint32_t *)pageBuffer;

  const volatile uint32_t *addr =
      (const volatile uint32_t *)((FLASH_PAGE_SIZE * page) + NVMCTRL_DATAFLASH);

  for (size_t i = 0; i < (FLASH_PAGE_SIZE / sizeof(*pDst)); i++) {
    *pDst++ = *addr++;
  }
}

void nvmDataFlashWrite(const NVMPage_t page) {
  const uint32_t    *pBuf       = (const uint32_t *)pageBuffer;
  volatile uint32_t *nvmAddress = (volatile uint32_t *)NVMCTRL_DATAFLASH;

  /* Flush anything outstanding in the page buffer */
  if (NVMCTRL->STATUS.reg & NVMCTRL_STATUS_LOAD) {
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
  }
  while (!(NVMCTRL->STATUS.reg & NVMCTRL_STATUS_READY))
    ;

  /* Set the correct address in DFLASH region */
  NVMCTRL->ADDR.reg = NVMCTRL_ADDR_ARRAY_DATAFLASH |
                      NVMCTRL_ADDR_AOFFSET(page * FLASH_PAGE_SIZE);

  /* Delete the row */
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
  while (!(NVMCTRL->STATUS.reg & NVMCTRL_STATUS_READY))
    ;

  /* Write to the page buffer, and write out complete page when done */
  for (size_t i = 0; i < (FLASH_PAGE_SIZE / sizeof(*pBuf)); i++) {
    *nvmAddress++ = *pBuf++;
  }

  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
  while (!(NVMCTRL->STATUS.reg & NVMCTRL_STATUS_READY))
    ;
}

uint8_t *nvmPageBuffer(void) { return pageBuffer; }

void nvmPageBufferClear(void) { memset(pageBuffer, 0, sizeof(pageBuffer)); }
