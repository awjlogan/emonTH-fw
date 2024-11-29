#pragma once

#include <stdint.h>

/*! @brief Read a page from the Data Flash region
 *  @param [in] page : the page number to read
 *  @param [in] pDst : pointer to the 64 byte destination buffer
 */
void nvmDataFlashRead(const int page, uint32_t *pDst);

/*! @brief Write a page to the Data Flash region
 *  @param [in] page : page number to write to
 *  @param [in] pSrc : pointer to the data source
 */
void nvmDataFlashWrite(const int page, const uint32_t *pSrc);

void nvmSetup(void);
