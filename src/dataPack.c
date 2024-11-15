#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "dataPack.h"
#include "emonTH_assert.h"
#include "util.h"

#define CONV_STR_W 16
#define STR_MSG    0
#define STR_V      1
#define STR_P      2
#define STR_E      3
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
static void catMsg(StrN_t *strD, int msg, bool json);
static void initFields(StrN_t *pD, char *pS, const int m);
static int  strnFtoa(StrN_t *strD, const float v);
static int  strnItoa(StrN_t *strD, const uint32_t v);
static int  strnCat(StrN_t *strD, const StrN_t *strS);
static int  strnLen(StrN_t *str);

static char   tmpStr[CONV_STR_W] = {0};
static StrN_t strConv; /* Fat string for conversions */

/* Strings that are inserted in the transmitted message */
const StrN_t baseStr[12] = {
    {.str = "MSG", .n = 3, .m = 4},   {.str = "V", .n = 1, .m = 2},
    {.str = "P", .n = 1, .m = 2},     {.str = "E", .n = 1, .m = 2},
    {.str = "pulse", .n = 5, .m = 6}, {.str = "t", .n = 1, .m = 2},
    {.str = ":", .n = 1, .m = 2},     {.str = "\r\n", .n = 2, .m = 3},
    {.str = "\"", .n = 1, .m = 2},    {.str = "{", .n = 1, .m = 2},
    {.str = "}", .n = 1, .m = 2},     {.str = ",", .n = 1, .m = 2}};

/*! @brief "Append <field><id>:" to the string
 *  @param [out] strD : pointer to the fat string
 *  @param [in] id : numeric index
 *  @param [in] field : field name index, e.g. "STR_V"
 */
static void catId(StrN_t *strD, int id, int field, bool json) {
  strD->n += strnCat(strD, &baseStr[STR_COMMA]);
  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[field]);
  (void)strnItoa(&strConv, id);
  strD->n += strnCat(strD, &strConv);
  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[STR_COLON]);
}

static void catMsg(StrN_t *strD, int msg, bool json) {
  /* <{">MSG<">:<"><#><"> */

  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_LCURL]);
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[STR_MSG]);
  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[STR_COLON]);
  (void)strnItoa(&strConv, msg);
  strD->n += strnCat(strD, &strConv);
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

static int strnFtoa(StrN_t *strD, const float v) {

  /* Zero the destination buffer then convert */
  memset(strD->str, 0, strD->m);
  utilFtoa(strD->str, v);
  strD->n = strnLen(strD);

  /* Truncate if it exceeds the length of the string */
  if (-1 == strD->n) {
    strD->n = strD->m;
  }
  return strD->n;
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

static int strnLen(StrN_t *str) {
  int i = 0;
  while (str->str[i++]) {
    /* Terminate if exceeded the maximum length */
    if (i >= str->m) {
      return -1;
    }
  }
  return i;
}

int dataPackSerial(const EmonTHDataset_t *pData, char *pDst, int m, bool json) {
  EMONTH_ASSERT(pData);
  EMONTH_ASSERT(pDst);

  StrN_t strn;
  initFields(&strn, pDst, m);

  catMsg(&strn, pData->msgNum, json);

  /* REVIST : pulse and temperature */

  /* Terminate with } for JSON and \r\n */
  if (json) {
    strn.n += strnCat(&strn, &baseStr[STR_RCURL]);
  }
  strn.n += strnCat(&strn, &baseStr[STR_CRLF]);
  return strn.n;
}

int_fast8_t dataPackPacked(const EmonTHDataset_t *pData, void *pPacked,
                           PackedRange_t range) {

  return 0;
}
