#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "emonTH_saml.h"

/*! @brief Calculate the CRC16 (CCITT - 0x1021, init 0xFFFF)
 *  @param [in] pSrc : pointer to data
 *  @param [in] n : number of bytes in data
 *  @return CRC16 CCITT value
 */
uint16_t calcCRC16_ccitt(const void *pSrc, size_t n);

/*! @brief Setup the DMAC peripheral */
void dmacSetup(void);

/*! @brief Returns a pointer to DMA descriptor for the channel.
 *  @param [in] ch : channel index
 *  @return pointer to the DmacDescriptor struct
 */
volatile DmacDescriptor *dmacGetDescriptor(unsigned int ch);

/*! @brief Enable a DMAC channel
 *  @param [in] ch : channel number
 */
void dmacChannelEnable(unsigned int ch);

/*! @brief Configure a DMA channel
 *  @param [in] ch : channel to configure
 *  @param [in] ctrlb : DMAC CTRLB value
 */
void dmacChannelConfigure(unsigned int ch, const uint32_t ctrlb);

/*! @brief Enable DMAC channel interrupt
 *  @param [in] ch : channel to enable interrupt for
 */
void dmacEnableChannelInterrupt(unsigned int ch);

/*! @brief Clear DMAC channel interrupt flag
 *  @param [in] ch : channel to clear interrupt flag for
 */
void dmacClearChannelInterrupt(unsigned int ch);

/*! @brief Indicate if the UART DMA is complete
 *  @return true if complete, false otherwise
 */
bool dmacUARTComplete(void);
