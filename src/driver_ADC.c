#include <stdbool.h>

#include "driver_ADC.h"
#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "driver_SAMD.h"
#include "emonTH_samd.h"

#include "emonTH.h"

typedef enum ADCState_ {
  ADC_STATE_SETUP,
  ADC_STATE_ENABLE,
  ADC_STATE_SAMPLE,
  ADC_STATE_DISABLE
} ADCState_t;

/* ADC usage:
 * - Setup once at startup.
 *   - Single sample to put valid data into RESULT.
 *   - Leave disabled.
 * - Functional
 *   1. Enable
 *     - Interrupt: SYNC
 *     - Power: STANDBY
 *   2. Sample
 *     - Interrupt: RESULT
 *     - Power: STANDBY
 *   3. Disable
 *     - Interrupt: NONE
 *     - Power: STANDBY
 */

static volatile ADCState_t adcState = ADC_STATE_SETUP;

static void adcSync(void);

static volatile int16_t adcResult      = 0;
static volatile bool    adcResultValid = false;

int16_t adcGetResult(void) { return adcResultValid ? adcResult : INT16_MIN; }

void adcSetup(void) {

  portPinMux(GRP_BATT_SENSE, PIN_VBATT_DIV4, PORT_PMUX_PMUXE_Msk);

  /* APB bus clock is enabled by default (Table 15-1). Connect GCLK 1 @ 1 MHz */
  GCLK->PCHCTRL[ADC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[ADC_GCLK_ID].reg & GCLK_PCHCTRL_CHEN))
    ;

  /* Reset all the ADC registers */
  ADC->CTRLA.reg = ADC_CTRLA_SWRST;
  while (ADC->CTRLA.reg & ADC_CTRLA_SWRST)
    ;

  ADC->CALIB.reg = (samdCalibration(CAL_ADC_BIAS) << 8u) |
                   samdCalibration(CAL_ADC_LINEARITY);

  /* Enable reference buffer and set to external VREF */
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC0;

  /* Input control - requires synchronisation (33.6.15) */
  ADC->INPUTCTRL.reg = AIN_VBATT | ADC_INPUTCTRL_MUXNEG(0x18);
  adcSync();

  /* Divide the 1 MHz peripheral clock down to 250 kHz */
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4;
  ADC->CTRLC.reg = ADC_CTRLC_RESSEL_10BIT;
  adcSync();

  NVIC_EnableIRQ(ADC_0_IRQn);
  samdSetActivity(SLEEP_MODE_STANDBY, PERIPH_IDX_ADC);
  adcState = ADC_STATE_ENABLE;
}

static void adcSync(void) {
  while (ADC->SYNCBUSY.reg)
    ;
}
void adcTriggerSample(void) {
  adcState          = ADC_STATE_SAMPLE;
  ADC->INTENSET.reg = ADC_INTENSET_RESRDY;
  ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;
  adcSync();
  ADC->SWTRIG.reg = ADC_SWTRIG_START;
}

void irq_handler_adc(void) {
  ADC->INTFLAG.reg = ADC->INTFLAG.reg;
  switch (adcState) {
  case ADC_STATE_SAMPLE:
    adcState = ADC_STATE_DISABLE;
    break;
  case ADC_STATE_DISABLE:
    adcResultValid    = true;
    adcResult         = ADC->RESULT.reg;
    adcState          = ADC_STATE_ENABLE;
    ADC->INTENCLR.reg = ADC_INTENCLR_RESRDY;
    ADC->CTRLA.reg &= ~(ADC_CTRLA_ENABLE);
    break;
  case ADC_STATE_ENABLE:
  case ADC_STATE_SETUP:
    break;
  }
}
