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

/*! @brief Convert a null-terminated string to a signed integer.
 *         No leading whitespace is skipped. For base-16, no "0x" prefix
 *         is accepted. Overflow is not detected.
 *  @param [in] pBuf : pointer to string buffer (entire string is parsed)
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return validity flag and parsed value
 */
ConvInt_t utilAtoi(const char *pBuf, ITOA_BASE_t base);

/*! @brief Convert a null-terminated string to an unsigned integer.
 *         No leading whitespace is skipped. For base-16, no "0x" prefix
 *         is accepted. Overflow is not detected.
 *  @param [in] pBuf : pointer to string buffer (entire string is parsed)
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return validity flag and parsed value
 */
ConvUint_t utilAtoui(const char *pBuf, ITOA_BASE_t base);

/*! @brief Indicate if a character is printable
 *  @param [in] c : character to check
 *  @return true if printable, false otherwise
 */
bool utilCharPrintable(const char c);

/*! @brief Convert integer to a null-terminated string.
 *  @param [in] pBuf : pointer to string buffer, at least 12 characters
 *                    for base-10 with sign, 11 for base-16.
 *  @param [in] val : value to convert
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return number of bytes including NULL terminator
 */
size_t utilItoa(char *pBuf, int32_t val, const ITOA_BASE_t base);

/*! @brief Convert unsigned integer to a null-terminated string.
 *  @param [in] pBuf : pointer to string buffer, at least 11 characters
 *  @param [in] uval : value to convert
 *  @param [in] base : select base 10 or base 16 conversion
 *  @return number of bytes including NULL terminator
 */
size_t utilUtoa(char *pBuf, uint32_t uval, const ITOA_BASE_t base);
