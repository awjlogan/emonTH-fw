#include <stdbool.h>
#include <stdio.h>

#include "util.h"

static bool isnumeric(const char c);

static bool isnumeric(const char c) {
  if (('0' <= c) && ('9' >= c)) {
    return true;
  }
  return false;
}

void utilStrReverse(char *pBuf, unsigned int len) {
  char         tmp;
  unsigned int idxEnd = len - 1u;
  for (unsigned int idx = 0; idx < (len / 2); idx++) {
    tmp          = pBuf[idx];
    pBuf[idx]    = pBuf[idxEnd];
    pBuf[idxEnd] = tmp;
    idxEnd--;
  }
}

unsigned int utilStrlen(const char *pBuf) {
  unsigned int charCnt = 0;
  while (*pBuf++) {
    charCnt++;
  }
  return charCnt;
}

unsigned int utilItoa(char *pBuf, int32_t val, ITOA_BASE_t base) {
  unsigned int charCnt    = 0;
  bool         isNegative = false;
  char *const  pBase      = pBuf;

  /* Handle 0 explicitly */
  if (0 == val) {
    *pBuf++ = '0';
    *pBuf   = '\0';
    return 2u;
  }

  /* Base 10 can be signed, and has a divide in */
  if (ITOA_BASE10 == base) {
    if (val < 0) {
      isNegative = true;
      val        = -val;
    }

    while (0 != val) {
      *pBuf++ = (val % 10u) + '0';
      val     = val / 10u;
      charCnt++;
    }

    if (isNegative) {
      *pBuf++ = '-';
      charCnt++;
    }
  } else {
    const char itohex[] = "0123456789abcdef";
    uint32_t   val_u    = (uint32_t)val;

    while (0 != val_u) {
      *pBuf++ = itohex[(val_u & 0xFu)];
      val_u >>= 4;
      charCnt++;
    }
  }

  /* Terminate and return */
  *pBuf = '\0';
  charCnt++;

  utilStrReverse(pBase, charCnt - 1u);
  return charCnt;
}

ConvInt_t utilAtoi(char *pBuf, ITOA_BASE_t base) {
  bool         isNegative = false;
  unsigned int len;
  unsigned int mulCnt = 1;
  ConvInt_t    conv   = {false, 0};

  if ('-' == *pBuf) {
    isNegative = true;
    pBuf++;
  }

  /* Reverse string and convert */
  len = utilStrlen(pBuf);
  utilStrReverse(pBuf, len);

  if (ITOA_BASE10 == base) {
    while (*pBuf) {
      if (!isnumeric(*pBuf))
        return conv;
      conv.val += ((*pBuf++) - '0') * mulCnt;
      mulCnt *= 10;
    }
    if (isNegative) {
      conv.val = -conv.val;
    }
  } else {
    while (*pBuf) {
      if (('a' <= *pBuf) && ('f' >= *pBuf)) {
        conv.val += ((*pBuf) - 'a' + 10u) * mulCnt;
      } else if (isnumeric(*pBuf)) {
        conv.val += ((*pBuf) - '0') * mulCnt;
      } else {
        return conv;
      }
      pBuf++;
      mulCnt *= 16;
    }
  }

  conv.valid = true;
  return conv;
}

bool utilCharPrintable(const char c) {
  /* Allow any printable character plus \r and \n */
  return (((c >= 32) && (c <= 126)) || ('\r' == c) || ('\n' == c));
}

unsigned int utilFtoa(char *pBuf, float val) {
  unsigned int charCnt    = 0;
  bool         isNegative = false;
  char *const  pBase      = pBuf;

  uint16_t decimals;
  int      units;

  if (val < 0.0f) {
    isNegative = true;
    val        = val * -1.0f;
  }
  decimals = (int)(val * 100.0f) % 100;
  units    = (int)val;

  charCnt += 3u;
  *pBuf++  = (decimals % 10) + '0';
  decimals = decimals / 10;
  *pBuf++  = (decimals % 10) + '0';
  *pBuf++  = '.';

  if (0 == units) {
    *pBuf++ = '0';
    charCnt++;
  }

  while (0 != units) {
    *pBuf++ = (units % 10) + '0';
    units   = units / 10;
    charCnt++;
  }

  if (isNegative) {
    *pBuf++ = '-';
    charCnt++;
  }
  utilStrReverse(pBase, charCnt);
  return charCnt;
}
