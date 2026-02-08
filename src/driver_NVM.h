#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum NVMStatus_ {
  NVM_READ_OK,
  NVM_READ_BAD_CRC,
  NVM_READ_NO_INIT
} NVMStatus_t;

typedef enum NVMPage_ { NVM_PAGE_CONFIG = 0 } NVMPage_t;

/*! @brief Read a page from the Data Flash region
 *  @param [in] page : the page to read
 *  @return status of the read
 */
NVMStatus_t nvmDataFlashRead(const NVMPage_t page);

/*! @brief Write a page to the Data Flash region
 *  @param [in] page : page to write to
 *  @param [in] n : number of data bytes
 */
void nvmDataFlashWrite(const NVMPage_t page, const size_t n);

/*! @brief Get the NVM page buffer address
 *  @return : pointer to the data in the page buffer
 */
uint8_t *nvmPageBuffer(void);

/*! @brief Clear any data in the NVM page buffer */
void nvmPageBufferClear(void);
