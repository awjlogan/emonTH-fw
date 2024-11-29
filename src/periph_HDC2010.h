#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Packed to ensure the sequential read out from the hardware is correct */
typedef struct __attribute__((__packed__)) HDCResultInt_ {
  int16_t temp;
  int16_t humidity;
} HDCResultInt_t;

typedef struct HDCResultF_ {
  float temp;
  float humidity;
} HDCResultF_t;

void hdc2010ConversionStart(void);
void hdc2010SampleGet(HDCResultInt_t *pRes);
void hdc2010SampleItoF(const HDCResultInt_t *pInt, HDCResultF_t *pF);
void hdc2010Setup(void);
