#include <stdbool.h>

#include "util.h"

static bool isnumeric(const char c);

static bool isnumeric(const char c) {
  if (('0' <= c) && ('9' >= c)) {
    return true;
  }
  return false;
}

size_t utilItoa(char *pBuf, int32_t val, const ITOA_BASE_t base) {
  if ((ITOA_BASE10 == base) && (val < 0)) {
    *pBuf = '-';
    return 1u + utilUtoa(pBuf + 1, (uint32_t)(-val), base);
  }
  return utilUtoa(pBuf, (uint32_t)val, base);
}

size_t utilUtoa(char *pBuf, uint32_t uval, const ITOA_BASE_t base) {
  char  buf[11]; /* 4294967295 = 10 chars + null */
  char *p = &buf[10];

  *p = '\0';

  /* Handle 0 explicitly */
  if (0 == uval) {
    pBuf[0] = '0';
    pBuf[1] = '\0';
    return 2u;
  }

  if (ITOA_BASE10 == base) {
    while (uval != 0) {
      uint32_t q = uval / 10u;
      *--p       = (char)('0' + (uval - q * 10));
      uval       = q;
    }
  } else {
    static const char itohex[] = "0123456789abcdef";

    while (0 != uval) {
      *--p = itohex[uval & 0xFu];
      uval >>= 4;
    }
  }

  /* Copy to output buffer */
  char  *dst = pBuf;
  size_t len = 0;
  while (*p) {
    *dst++ = *p++;
    len++;
  }
  *dst = '\0';

  return len + 1u;
}

ConvInt_t utilAtoi(const char *pBuf, ITOA_BASE_t base) {
  bool isNegative = ('-' == *pBuf);
  if (isNegative) {
    pBuf++;
  }

  ConvUint_t u    = utilAtoui(pBuf, base);
  ConvInt_t  conv = {u.valid,
                     {isNegative ? -(int32_t)u.val.u32 : (int32_t)u.val.u32}};
  return conv;
}

ConvUint_t utilAtoui(const char *pBuf, ITOA_BASE_t base) {
  uint32_t   result = 0;
  ConvUint_t conv   = {false, {0}};

  /* Empty string should be treated as failure */
  if (*pBuf == '\0') {
    return conv;
  }

  /* Process left-to-right, no string reversal needed */
  if (ITOA_BASE10 == base) {
    while (*pBuf) {
      if (!isnumeric(*pBuf)) {
        return conv;
      }
      result = result * 10 + (uint32_t)(*pBuf - '0');
      pBuf++;
    }
  } else {
    while (*pBuf) {
      char     c = *pBuf;
      uint32_t digit;
      if (('a' <= c) && ('f' >= c)) {
        digit = (uint32_t)(c - 'a' + 10);
      } else if (('A' <= c) && ('F' >= c)) {
        digit = (uint32_t)(c - 'A' + 10);
      } else if (isnumeric(c)) {
        digit = (uint32_t)(c - '0');
      } else {
        return conv;
      }
      /* result = result * 16 + digit */
      result = (result << 4) + digit;
      pBuf++;
    }
  }

  conv.val.u32 = result;
  conv.valid   = true;
  return conv;
}

bool utilCharPrintable(const char c) {
  /* Allow any printable character plus \r and \n */
  return (((c >= 32) && (c <= 126)) || ('\r' == c) || ('\n' == c));
}
