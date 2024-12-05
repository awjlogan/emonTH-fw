#include <stdbool.h>

#include "driver_ADC.h"
#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "driver_SAML.h"
#include "emonTH_saml.h"

#include "emonTH.h"

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

static void adcSync(void);

static volatile int16_t adcResult      = 0;
static volatile bool    adcResultValid = false;

int16_t adcGetResult(void) { return adcResultValid ? adcResult : INT16_MIN; }

void adcSetup(void) {

  portPinMux(PIN_VBATT_DIV4, PORT_PMUX_PMUXE_Msk);

  /* APB bus clock is enabled by default (Table 15-1). Connect GCLK 1 @ 1 MHz */
  GCLK->PCHCTRL[ADC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[ADC_GCLK_ID].reg & GCLK_PCHCTRL_CHEN))
    ;

  /* Reset all the ADC registers */
  ADC->CTRLA.reg = ADC_CTRLA_SWRST;
  while (ADC->CTRLA.reg & ADC_CTRLA_SWRST)
    ;

  ADC->CALIB.reg = (samlCalibration(CAL_ADC_BIAS) << 8u) |
                   samlCalibration(CAL_ADC_LINEARITY);

  /* Enable reference buffer and set to external VREF */
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC0;

  /* Input control - requires synchronisation (41.6.8) */
  ADC->INPUTCTRL.reg = AIN_VBATT | ADC_INPUTCTRL_MUXNEG(0x18);
  adcSync();

  /* Divide the 1 MHz peripheral clock down to 250 kHz */
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4;
  ADC->CTRLC.reg = ADC_CTRLC_RESSEL_10BIT;
  adcSync();

  ADC->INTENSET.reg = ADC_INTENSET_RESRDY;
  NVIC_EnableIRQ(ADC_0_IRQn);
  samlSetActivity(SLEEP_MODE_STANDBY, PERIPH_IDX_ADC);
}

static void adcSync(void) {
  while (ADC->SYNCBUSY.reg)
    ;
}
void adcTriggerSample(void) {
  adcResultValid = false;
  ADC->CTRLA.reg = ADC_CTRLA_ENABLE | ADC_CTRLA_RUNSTDBY;
  adcSync();
  ADC->SWTRIG.reg = ADC_SWTRIG_START;
}

void irq_handler_adc(void) {
  ADC->INTFLAG.reg = ADC->INTFLAG.reg;
  adcResultValid   = true;
  adcResult        = ADC->RESULT.reg;
  ADC->CTRLA.reg   = 0;
}
