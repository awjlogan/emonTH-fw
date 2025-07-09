#include <stdbool.h>

#include "driver_ADC.h"
#include "driver_PORT.h"
#include "driver_SAML.h"
#include "emonTH_saml.h"

#include "emonTH.h"

static void adcSync(void);

static volatile bool adcResRdy = false;

uint16_t adcGetResult(void) { return ADC->RESULT.reg; }

bool adcSampleReady(void) { return adcResRdy; }

void adcSampleTrigger(void) {
  adcResRdy       = false;
  ADC->SWTRIG.reg = ADC_SWTRIG_START;
}

void adcSetup(void) {

  portPinMux(PIN_VBATT, PORT_PMUX_PMUXE_Msk);

  /* APB bus clock is enabled by default (19.8.9). Connect GCLK0 @ 8 MHz */
  GCLK->PCHCTRL[ADC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[ADC_GCLK_ID].reg & GCLK_PCHCTRL_CHEN))
    ;

  /* Reset all the ADC registers */
  ADC->CTRLA.reg = ADC_CTRLA_SWRST;
  while (ADC->CTRLA.reg & ADC_CTRLA_SWRST)
    ;
  ADC->CALIB.reg = ADC_CALIB_BIASREFBUF(samlCalibration(CAL_ADC_BIASREFBUF)) |
                   ADC_CALIB_BIASCOMP(samlCalibration(CAL_ADC_BIASCOMP));

  /* Use nominal 3V3 as reference */
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC2;

  /* Input control - requires synchronisation (41.6.8) */
  ADC->INPUTCTRL.reg = AIN_VBATT | ADC_INPUTCTRL_MUXNEG(0x18);
  adcSync();

  /* Divide the 8 MHz peripheral clock down to 250 kHz */
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV32;
  ADC->CTRLC.reg = ADC_CTRLC_RESSEL_10BIT;
  adcSync();

  /* Set 32 us sampling time, SAMPLEN+1 @ 250 kHz. */
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0x7);

  /* Enable the result ready interrupt and route to NVIC (ADC_1). */
  ADC->INTENSET.reg = ADC_INTENSET_RESRDY;
  NVIC_EnableIRQ(ADC_1_IRQn);

  ADC->CTRLA.reg = ADC_CTRLA_ENABLE | ADC_CTRLA_RUNSTDBY | ADC_CTRLA_ONDEMAND;
  adcSync();
}

static void adcSync(void) {
  while (ADC->SYNCBUSY.reg)
    ;
}

void irq_handler_adc_1(void) {
  if (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY) {
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
    adcResRdy        = true;
  }
}
