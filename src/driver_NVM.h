#pragma once

#include <stdint.h>

/* Configuration key - indicates that the configuration is the default or
 * has been retrieved NVM. */
#define CONFIG_NVM_KEY 0xca55e77eul

typedef struct __attribute__((__packed__)) NVMHeader_ {
  uint32_t watermark;
  uint16_t crc16;
  uint8_t  writeCount;
  uint8_t  res0;
} NVMHeader_t;

typedef enum NVMPage_ { NVM_PAGE_CONFIG = 0 } NVMPage_t;

/*! @brief Read a page from the Data Flash region
 *  @param [in] page : the page to read
 */
void nvmDataFlashRead(const NVMPage_t page);

/*! @brief Write a page to the Data Flash region
 *  @param [in] page : page to write to
 */
void nvmDataFlashWrite(const NVMPage_t page);

/*! @brief Get the NVM page buffer address
 *  @return : pointer to the page buffer
 */
uint8_t *nvmPageBuffer(void);

/*! @brief Clear the NVM page buffer */
void nvmPageBufferClear(void);
