#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Packed to ensure the sequential read out from the hardware is correct */
typedef struct __attribute__((__packed__)) HDCResultInt_ {
  int16_t  temp;
  uint16_t humidity;
} HDCResultRaw_t;

/*! @brief Start a conversion for the HDC2010. */
void hdc2010ConversionStart(void);

/*! @brief Fetch the readings from the HDC2010.
 *         Results are raw register values, little-endian as read from device.
 *  @param [out] pRes : pointer to the result structure (raw values)
 */
void hdc2010SampleGet(HDCResultRaw_t *pRes);

/*! @brief Ready flag for the HDC2010's result.
 *         Set by the DRDY interrupt after conversion completes.
 *  @return true if ready, false otherwise
 */
bool hdc2010SampleReady(void);

/*! @brief Setup the HDC2010 T/H sensor
 *  @return true if successful, false otherwise
 */
bool hdc2010Setup(void);
