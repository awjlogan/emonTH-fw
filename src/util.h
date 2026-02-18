#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum ITOA_BASE_ { ITOA_BASE10, ITOA_BASE16 } ITOA_BASE_t;

typedef struct ConvInt_ {
  bool valid; /* true if the value in val is valid */
  union {
    int32_t i32;
    int16_t i16;
    int8_t  i8;
  } val;
} ConvInt_t;

typedef struct ConvUint_ {
  bool valid; /* true if the value in val is valid */
  union {
    uint32_t u32;
    uint16_t u16;
    uint8_t  u8;
  } val;
} ConvUint_t;

/*! @brief Convert null terminated string to integer, returns the value.
 *  @param [in] pBuf : pointer to string buffer
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return valid and value
 */
ConvInt_t utilAtoi(const char *pBuf, ITOA_BASE_t base);

/*! @brief Convert null terminated string to unsigned integer
 *  @param [in] pBuf : pointer to string buffer
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return valid and value
 */
ConvUint_t utilAtoui(const char *pBuf, ITOA_BASE_t base);

/*! @brief Indicate if a character is printable
 *  @param [in] c : character to check
 *  @return true if printable, false otherwise
 */
bool utilCharPrintable(const char c);

/*! @brief Convert integer to null terminated string. Returns the number of
 *         characters (including NULL).
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] val : value to convert
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return number of bytes including NULL
 */
size_t utilItoa(char *pBuf, int32_t val, const ITOA_BASE_t base);

/*! @brief Convert unsigned integer to null terminated string.
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] uval : value to convert
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return number of bytes including NULL
 */
size_t utilUtoa(char *pBuf, uint32_t uval, const ITOA_BASE_t base);
