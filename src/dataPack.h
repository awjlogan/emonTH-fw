#pragma once

#include <stdbool.h>

#include "emonTH.h"

/*! @brief Packs the data packet into serial format.
 *         Returns the number of characters that would have been packed,
 *         regardless of the value of m. If the return value != m, then the
 *         buffer would have overflowed (similar to snprintf). Does not append
 *         a NULL. Clears data buffer in advance.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pDst : pointer to the destination buffer
 *  @param [in] m : width of the destination buffer
 *  @param [in] opt : [0]: json false -> K:V; true -> JSON, [1]: tempEx
 *  @return : the number of the characters that would be packed
 */
int dataPackSerial(const EmonTHDataset_t *restrict pData, char *restrict pDst,
                   int m, int opt);

/*! @brief Pack the temperature (internal and external), humidity, battery level
 * and pulse count into a packed structure for transmission over RFM link
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pPacked : pointer to the destination packet
 */
void dataPackPacked(const EmonTHDataset_t *restrict pData,
                    PackedData_t *restrict pPacked);
