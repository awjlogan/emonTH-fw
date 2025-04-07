#include <string.h>

#include "driver_NVM.h"
#include "emonTH_assert.h"
#include "emonTH_saml.h"

void nvmDataFlashRead(const int page, uint32_t *pDst) {
  EMONTH_ASSERT(page < NVMCTRL_DATAFLASH_PAGES);

  const volatile uint32_t *addr =
      (const volatile uint32_t *)((FLASH_PAGE_SIZE * page) + NVMCTRL_DATAFLASH);

  for (size_t i = 0; i < (FLASH_PAGE_SIZE / sizeof(*pDst)); i++) {
    *pDst++ = *addr++;
  }
}

void nvmDataFlashWrite(const int page, const uint32_t *pSrc) {
  const uint32_t    *pBuf       = pSrc;
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

void nvmSetup(void) {}
