#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "dataPack.h"
#include "emonTH_assert.h"
#include "util.h"

#define CONV_STR_W 16
#define STR_TEMPEX 0
#define STR_BATT   1
#define STR_HUMID  2
#define STR_PERIOD 3
#define STR_PULSE  4
#define STR_TEMP   5
#define STR_COLON  6
#define STR_CRLF   7
#define STR_DQUOTE 8
#define STR_LCURL  9
#define STR_RCURL  10
#define STR_COMMA  11

/* "Fat" string with current length and buffer size. */
typedef struct StrN {
  char *str; /* Pointer to the string */
  int   n;   /* Length of the string  */
  int   m;   /* Buffer length */
} StrN_t;

static void catId(StrN_t *strD, int id, int field, bool json);
static void initFields(StrN_t *pD, char *pS, const int m);
static int  strnItoa(StrN_t *strD, const uint32_t v);
static int  strnCat(StrN_t *strD, const StrN_t *strS);

static char   tmpStr[CONV_STR_W] = {0};
static StrN_t strConv; /* Fat string for conversions */

/* Strings that are inserted in the transmitted message */
const StrN_t baseStr[] = {
    {.str = "tempex", .n = 6, .m = 7},   {.str = "batt", .n = 4, .m = 5},
    {.str = "humidity", .n = 8, .m = 9}, {.str = ".", .n = 1, .m = 2},
    {.str = "pulse", .n = 5, .m = 6},    {.str = "temp", .n = 4, .m = 5},
    {.str = ":", .n = 1, .m = 2},        {.str = "\r\n", .n = 2, .m = 3},
    {.str = "\"", .n = 1, .m = 2},       {.str = "{", .n = 1, .m = 2},
    {.str = "}", .n = 1, .m = 2},        {.str = ",", .n = 1, .m = 2}};

/*! @brief "Append <field><id>:" to the string
 *  @param [out] strD : pointer to the fat string
 *  @param [in] id : numeric index
 *  @param [in] field : field name index, e.g. "STR_V"
 */
static void catId(StrN_t *strD, int id, int field, bool json) {

  /* No comma for the 1st field */
  if (field != STR_TEMP) {
    strD->n += strnCat(strD, &baseStr[STR_COMMA]);
  }

  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[field]);

  if (id > -1) {
    (void)strnItoa(&strConv, id);
    strD->n += strnCat(strD, &strConv);
  }
  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[STR_COLON]);
}

static void initFields(StrN_t *pD, char *pS, const int m) {
  /* Setup destination string */
  pD->str = pS;
  pD->n   = 0;
  pD->m   = m;
  memset(pD->str, 0, m);

  /* Setup conversion string */
  strConv.str = tmpStr;
  strConv.n   = 0;
  strConv.m   = CONV_STR_W;
}

static int strnItoa(StrN_t *strD, const uint32_t v) {
  /* Zero the destination buffer then convert */
  memset(strD->str, 0, strD->m);

  strD->n = utilItoa(strD->str, v, ITOA_BASE10);
  return strD->n;
}

static int strnCat(StrN_t *strD, const StrN_t *strS) {
  /* Check bounds to make sure it won't go over the end. If so, return the
   * actual number of bytes that are copied.
   */
  int newLen;
  int bytesToCopy;

  bytesToCopy = strS->n;
  newLen      = strS->n + strD->n;
  if (newLen >= strD->m) {
    bytesToCopy = strD->m - strD->n;
  }

  memcpy((strD->str + strD->n), strS->str, bytesToCopy);
  return bytesToCopy;
}

void dataPackPacked(const EmonTHDataset_t *restrict pData,
                    PackedData_t *restrict pPacked) {

  /* T/H 10x value, e.g. 261 = 26.1ºC */
  pPacked->tempInternal     = pData->hdcResRaw.temp * 1650 / (1 << 16) - 400;
  pPacked->humidityInternal = pData->hdcResRaw.humidity * 1000 / (1 << 16);
  pPacked->battery          = (pData->battery * 3226) / 10;
  pPacked->pulse            = pData->pulseCnt;
  for (int i = 0; i < TEMP_MAX_ONEWIRE; i++) {
    pPacked->tempExternal[i] = pData->tempExternal[i];
  }
}

int dataPackSerial(const EmonTHDataset_t *restrict pData, char *restrict pDst,
                   int m, int opt) {
  EMONTH_ASSERT(pData);
  EMONTH_ASSERT(pDst);

  bool json   = opt & 0x1;
  bool tempEx = opt & 0x2;

  uint32_t     battery = pData->battery * 3226;
  int          tempInt = ((pData->hdcResRaw.temp * 1650) / (1 << 16)) - 400;
  unsigned int humInt =
      ((unsigned int)pData->hdcResRaw.humidity & 0xFFFF) * 1000 / (1 << 16);

  StrN_t strn;
  initFields(&strn, pDst, m);

  if (json) {
    strn.n += strnCat(&strn, &baseStr[STR_LCURL]);
  }

  catId(&strn, -1, STR_TEMP, json);
  (void)strnItoa(&strConv, tempInt / 10);
  strn.n += strnCat(&strn, &strConv);
  strn.n += strnCat(&strn, &baseStr[STR_PERIOD]);
  (void)strnItoa(&strConv, tempInt % 10);
  strn.n += strnCat(&strn, &strConv);

  catId(&strn, -1, STR_HUMID, json);
  (void)strnItoa(&strConv, humInt / 10);
  strn.n += strnCat(&strn, &strConv);
  strn.n += strnCat(&strn, &baseStr[STR_PERIOD]);
  (void)strnItoa(&strConv, humInt % 10);
  strn.n += strnCat(&strn, &strConv);

  if (tempEx) {
    for (int i = 0; i < TEMP_MAX_ONEWIRE; i++) {
      catId(&strn, i, STR_TEMPEX, json);
      // REVISIT conversion here is wrong, need to take the raw fixed point.
      (void)strnItoa(&strConv, (pData->tempExternal[i] / 100));
      strn.n += strnCat(&strn, &strConv);
    }
  }

  catId(&strn, -1, STR_BATT, json);
  (void)strnItoa(&strConv, (battery / 1000000));
  strn.n += strnCat(&strn, &strConv);
  strn.n += strnCat(&strn, &baseStr[STR_PERIOD]);
  (void)strnItoa(&strConv, ((battery % 1000000) / 1000));
  strn.n += strnCat(&strn, &strConv);

  catId(&strn, -1, STR_PULSE, json);
  (void)strnItoa(&strConv, pData->pulseCnt);
  strn.n += strnCat(&strn, &strConv);

  /* Terminate with } for JSON and \r\n */
  if (json) {
    strn.n += strnCat(&strn, &baseStr[STR_RCURL]);
  }
  strn.n += strnCat(&strn, &baseStr[STR_CRLF]);
  return strn.n;
}
