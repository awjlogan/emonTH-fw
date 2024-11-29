#include "driver_NVM.h"
#include "emonTH_assert.h"
#include "emonTH_saml.h"

void nvmDataFlashRead(const int page, uint32_t *pDst) {
  EMONTH_ASSERT(page < NVMCTRL_DATAFLASH_PAGES);
  volatile uint32_t *addr =
      (volatile uint32_t *)((FLASH_PAGE_SIZE * page) + NVMCTRL_DATAFLASH);

  for (uint_fast8_t i = 0; i < (FLASH_PAGE_SIZE / sizeof(uint32_t)); i++) {
    *pDst++ = *addr++;
  }
}

void nvmDataFlashWrite(const int page, const uint32_t *pSrc) {
  uint32_t          *pBuf       = (uint32_t *)pSrc;
  volatile uint32_t *nvmAddress = (volatile uint32_t *)FLASH_ADDR;

  /* Flush anything outstanding in the page buffer and wait until ready */
  if (NVMCTRL->STATUS.reg & NVMCTRL_STATUS_LOAD) {
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
  }
  while (!(NVMCTRL->STATUS.reg & NVMCTRL_STATUS_READY))
    ;

  /* Write to the page buffer and finally trigger the write to the array */
  NVMCTRL->ADDR.reg = NVMCTRL_ADDR_ARRAY_DATAFLASH |
                      NVMCTRL_ADDR_AOFFSET(page * FLASH_PAGE_SIZE);
  for (uint_fast8_t i = 0; i < (FLASH_PAGE_SIZE / sizeof(*pBuf)); i++) {
    *nvmAddress++ = *pBuf++;
  }
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
}

void nvmSetup(void) {}
