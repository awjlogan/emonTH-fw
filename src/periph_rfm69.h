#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RFM_PALEVEL_DEF                                                        \
  0x19 /* Default power level must be below maximum                            \
        * in case there is no antenna - this can                               \
        * destroy the RFM module.                                              \
        */

typedef enum RFM_Freq_ {
  RFM_FREQ_868MHz,
  RFM_FREQ_915MHz,
  RFM_FREQ_433MHz,
  RFM_FREQ_433_92MHz
} RFM_Freq_t;

typedef struct RFMOpt_ {
  RFM_Freq_t freq;
  uint8_t    group;
  uint8_t    nodeID;
  uint8_t    paLevel;
} RFMOpt_t;

typedef enum RFMSend_ {
  RFM_NO_INIT,
  RFM_TIMED_OUT,
  RFM_FAILED,
  RFM_SUCCESS,
  RFM_N_TOO_LARGE
} RFMSend_t;

/*! @brief Get a pointer to the RFM69 transmit buffer.
 *  @return pointer to RFM69 buffer (maximum payload 61 bytes)
 */
uint8_t *rfmGetBuffer(void);

/*! @brief Initialise the RFM69 module.
 *  @param [in] pOpt : pointer to RFM options
 *  @return true if successful, false otherwise
 */
bool rfmInit(RFMOpt_t *pOpt);

/*! @brief Indicate if the RFM69 send is complete
 *  @return true if complete, false otherwise
 */
bool rfmSendComplete(void);

/*! @brief Send data through the RFM69.
 *         Payload is taken from rfmGetBuffer().
 *  @param [in] n : number of bytes to be sent (max 61)
 *  @return result of the attempt to send
 */
RFMSend_t rfmSendBuffer(const size_t n);

/*! @brief Sets the RFM69's address
 *  @param [in] addr : address to set the RFM69
 */
void rfmSetAddress(const uint8_t addr);

/*! @brief Set the AES key for encryption.
 *  @param [in] aes : 16 character AES key; NULL disables encryption.
 */
void rfmSetAESKey(const char *aes);

/*! @brief Put the RFM into retention sleep mode */
void rfmSleep(void);
