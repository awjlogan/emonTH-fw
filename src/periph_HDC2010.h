#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Packed to ensure the sequential read out from the hardware is correct */
typedef struct __attribute__((__packed__)) HDCResultInt_ {
  int16_t temp;
  int16_t humidity;
} HDCResultRaw_t;

typedef struct HDCResultF_ {
  float temp;
  float humidity;
} HDCResultF_t;

void hdc2010ConversionStart(void);
bool hdc2010ConversionStarted(void);
void hdc2010SampleGet(HDCResultRaw_t *pRes);
void hdc2010Setup(void);
