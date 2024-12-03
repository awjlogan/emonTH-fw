#pragma once

#include <stdbool.h>

#include "emonTH_saml.h"

typedef struct DMACCfgCh {
  uint32_t ctrlb;
} DMACCfgCh_t;

/*! @brief Calculate the CRC16 (CCITT - 0x1021)
 *  @param [in] pData : pointer to data
 *  @param [in] n : number of bytes in data
 *  @return CRC16 CCITT value
 */
uint16_t calcCRC16_ccitt(const void *pSrc, unsigned int n);

/*! @brief Setup the DMAC peripheral */
void dmacSetup(void);

/*! @brief Returns a pointer to DMA descriptor for the channel.
 *  @param [in] ch : channel
 *  @return Pointer to the DmacDescriptor struct
 */
volatile DmacDescriptor *dmacGetDescriptor(unsigned int ch);

/*! @brief Set the callback when the DMA has filled the sample buffer */
void dmacCallbackBufferFill(void (*cb)(void));

/*! @brief Disable a DMAC channel
 *  @param [in] ch : channel number
 */
void dmacChannelDisable(unsigned int ch);

/*! @brief Enable a DMAC channel
 *  @param [in] ch : channel number
 */
void dmacChannelEnable(unsigned int ch);

/*! @brief Get channel transfer status
 *  @param [in] ch : channel number
 *  @return true if channel ch is busy, false otherwise
 */
bool dmacChannelBusy(unsigned int ch);

/*! @brief Configure a DMA channel
 *  @param [in] ch : channel to configure
 *  @param [in] ctrlb : DMAC CTRLB value
 */
void dmacChannelConfigure(unsigned int ch, const uint32_t ctrlb);

/*! @brief Resume a DMA channel
 *  @param [in] ch : channel to resume
 */
void dmacChannelResume(unsigned int ch);

/*! @brief Suspend a DMA channel
 *  @param [in] ch : channel to suspend
 */
void dmacChannelSuspend(unsigned int ch);

/*! @brief Enable DMAC channel interrupt
 *  @param [in] ch : channel to enable interrupt for
 */
void dmacEnableChannelInterrupt(unsigned int ch);

/*! @brief Disable DMAC channel interrupt
 *  @param [in] ch : channel to disable interrupt for
 */
void dmacDisableChannelInterrupt(unsigned int ch);

/*! @brief Clear DMAC channel interrupt flag
 *  @param [in] ch : channel to clear interrupt flag for
 */
void dmacClearChannelInterrupt(unsigned int ch);

bool uartDmaComplete(void);
