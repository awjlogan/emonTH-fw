#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "dataPack.h"
#include "emonTH_assert.h"
#include "util.h"

#define CONV_STR_W (16u)

enum {
  STR_TEMPEX = 0,
  STR_BATT   = 1,
  STR_HUMID  = 2,
  STR_PERIOD = 3,
  STR_PULSE  = 4,
  STR_TEMP   = 5,
  STR_COLON  = 6,
  STR_CRLF   = 7,
  STR_DQUOTE = 8,
  STR_LCURL  = 9,
  STR_RCURL  = 10,
  STR_COMMA  = 11,
  STR_CO2    = 12
};

/* "Fat" string with current length and buffer size. */
typedef struct StrN {
  char  *str; /* Pointer to the string */
  size_t n;   /* Length of the string  */
  size_t m;   /* Buffer length */
} StrN_t;

static void   catId(StrN_t *strD, int id, const size_t field, const bool json);
static void   initFields(StrN_t *pD, char *pS, const size_t m);
static size_t strnItoa(StrN_t *strD, const int32_t v);
static size_t strnCat(StrN_t *strD, const StrN_t *strS);

static char   tmpStr[CONV_STR_W] = {0};
static StrN_t strConv; /* Fat string for conversions */

/* Strings that are inserted in the transmitted message */
const StrN_t baseStr[] = {
    {.str = "tempex", .n = 6, .m = 7},   {.str = "batt", .n = 4, .m = 5},
    {.str = "humidity", .n = 8, .m = 9}, {.str = ".", .n = 1, .m = 2},
    {.str = "pulse", .n = 5, .m = 6},    {.str = "temp", .n = 4, .m = 5},
    {.str = ":", .n = 1, .m = 2},        {.str = "\r\n", .n = 2, .m = 3},
    {.str = "\"", .n = 1, .m = 2},       {.str = "{", .n = 1, .m = 2},
    {.str = "}", .n = 1, .m = 2},        {.str = ",", .n = 1, .m = 2},
    {.str = "co2", .n = 3, .m = 4}};

/*! @brief "Append <field><id>:" to the string
 *  @param [out] strD : pointer to the fat string
 *  @param [in] id : numeric index
 *  @param [in] field : field name index, e.g. "STR_V"
 *  @param [in] json : output in JSON format
 */
static void catId(StrN_t *strD, int id, const size_t field, const bool json) {

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

static void initFields(StrN_t *pD, char *pS, const size_t m) {
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

static size_t strnItoa(StrN_t *strD, const int32_t v) {
  /* Zero the destination buffer then convert */
  memset(strD->str, 0, strD->m);

  strD->n = utilItoa(strD->str, v, ITOA_BASE10);
  return strD->n;
}

static size_t strnCat(StrN_t *strD, const StrN_t *strS) {
  /* Check bounds to make sure it won't go over the end. If so, return the
   * actual number of bytes that are copied.
   */
  size_t newLen;
  size_t bytesToCopy;

  bytesToCopy = strS->n;
  newLen      = strS->n + strD->n;
  if (newLen >= strD->m) {
    bytesToCopy = strD->m - strD->n;
  }

  memcpy((strD->str + strD->n), strS->str, bytesToCopy);
  return bytesToCopy;
}

void dataPackPacked(const EmonTHDataset_t *restrict pData,
                    void *restrict pPacked) {

  const int16_t tInt =
      (int16_t)((int32_t)pData->hdcResRaw.temp * 1650 / (1 << 16) - 400);
  const uint16_t hInt =
      (uint16_t)((uint32_t)pData->hdcResRaw.humidity * 1000 / (1 << 16));
  const uint16_t bInt = (uint16_t)((pData->battery * 3226) / 10000);

  /* T/H 10x value, e.g. 261 = 26.1ºC */
  if (4 == pData->numExtMax) {
    PackedData_4Ext_t *tx = (PackedData_4Ext_t *)pPacked;
    tx->tempInternal      = tInt;
    tx->humidityInternal  = hInt;
    tx->battery           = bInt;
    tx->pulse             = pData->pulseCnt;
    for (int i = 0; i < TEMP_MAX_ONEWIRE; i++) {
      tx->tempExternal[i] = pData->tempExternal[i];
    }
    tx->co2 = pData->co2;
  } else {
    PackedData_1Ext_t *tx = (PackedData_1Ext_t *)pPacked;
    tx->tempInternal      = tInt;
    tx->humidityInternal  = hInt;
    tx->battery           = bInt;
    tx->pulse             = pData->pulseCnt;
    tx->tempExternal      = pData->tempExternal[0];
    tx->co2               = pData->co2;
  }
}

size_t dataPackSerial(const EmonTHDataset_t *restrict pData,
                      char *restrict pDst, const size_t m, const bool json) {
  EMONTH_ASSERT(pData);
  EMONTH_ASSERT(pDst);

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

  for (int i = 0; i < TEMP_MAX_ONEWIRE; i++) {
    tempInt = pData->tempExternal[i] * 62500; /* micro-degrees */
    tempInt = tempInt / 100000;               /* deci-degrees */
    catId(&strn, (i + 1), STR_TEMPEX, json);

    (void)strnItoa(&strConv, tempInt / 10);
    strn.n += strnCat(&strn, &strConv);
    strn.n += strnCat(&strn, &baseStr[STR_PERIOD]);
    (void)strnItoa(&strConv, tempInt % 10);
    strn.n += strnCat(&strn, &strConv);
  }

  catId(&strn, -1, STR_HUMID, json);
  (void)strnItoa(&strConv, (int32_t)(humInt / 10u));
  strn.n += strnCat(&strn, &strConv);
  strn.n += strnCat(&strn, &baseStr[STR_PERIOD]);
  (void)strnItoa(&strConv, (int32_t)(humInt % 10u));
  strn.n += strnCat(&strn, &strConv);

  catId(&strn, -1, STR_BATT, json);
  (void)strnItoa(&strConv, (int32_t)(battery / 1000000u));
  strn.n += strnCat(&strn, &strConv);
  strn.n += strnCat(&strn, &baseStr[STR_PERIOD]);
  (void)strnItoa(&strConv, (int32_t)((battery % 1000000u) / 1000u));
  strn.n += strnCat(&strn, &strConv);

  catId(&strn, -1, STR_PULSE, json);
  (void)strnItoa(&strConv, (int32_t)pData->pulseCnt);
  strn.n += strnCat(&strn, &strConv);

  catId(&strn, -1, STR_CO2, json);
  (void)strnItoa(&strConv, (int32_t)pData->co2);
  strn.n += strnCat(&strn, &strConv);

  /* Terminate with } for JSON and \r\n */
  if (json) {
    strn.n += strnCat(&strn, &baseStr[STR_RCURL]);
  }
  strn.n += strnCat(&strn, &baseStr[STR_CRLF]);
  return strn.n;
}
