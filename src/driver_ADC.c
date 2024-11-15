#include "driver_ADC.h"
#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "driver_SAMD.h"
#include "emonTH_samd.h"

#include "emonTH.h"

static void adcSync(void);

static int16_t adcCalibrateSmp(int pin) {
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_PIN0 | pin;
  ADC->INTFLAG.reg |= ADC_INTFLAG_RESRDY;
  ADC->SWTRIG.reg = ADC_SWTRIG_START;
  while (0 == (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY))
    ;
  return ADC->RESULT.reg;
}

void adcSetup(void) {

  portPinMux(GRP_BATT_SENSE, PIN_VBATT_DIV4, PORT_PMUX_PMUXE_B_Val);

  /* APB bus clock is enabled by default (Table 15-1). Connect GCLK 3 */
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID(ADC_GCLK_ID) | GCLK_CLKCTRL_GEN(3u) | GCLK_CLKCTRL_CLKEN;

  /* Reset all the ADC registers */
  ADC->CTRLA.reg = ADC_CTRLA_SWRST;
  while (ADC->CTRLA.reg & ADC_CTRLA_SWRST)
    ;

  ADC->CALIB.reg = (samdCalibration(CAL_ADC_BIAS) << 8u) |
                   samdCalibration(CAL_ADC_LINEARITY);

  /* Enable reference buffer and set to external VREF */
  ADC->REFCTRL.reg = ADC_REFCTRL_REFCOMP | ADC_REFCTRL_REFSEL_INT1V;

  /* Differential mode, /4 prescale of F_PERIPH, right aligned, enable
   * averaging. Requires synchronisation after write (33.6.15)
   */
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_8BIT;
  adcSync();

  /* Conversion time is 3.5 us, target 6 us total conversion time
   * Setup 2.5 us conversion time: SAMPLEN = (2T * f_clk) - 1
   * (2 * 2.5E-6 * 2E6) - 1 = 9
   */
  ADC->SAMPCTRL.reg = 0x9u;

  /* 2x oversampling */
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_2;

  /* Input control - requires synchronisation (33.6.15) */
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_PIN0 | ADC_INPUTCTRL_MUXNEG_GND;
  adcSync();
}

int16_t adcSingleConversion(const unsigned int ch) {
  /* Save the scan and positive mux positions, do the conversion, and
   * restore the ADC state before returning the result.
   */
  int16_t      result      = 0;
  unsigned int enabledFlag = 0;

  const unsigned int inputCtrl = ADC->INPUTCTRL.reg;

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_PIN0 | ADC_INPUTCTRL_MUXPOS(ch);
  adcSync();

  ADC->INTFLAG.reg |= ADC_INTFLAG_RESRDY;

  if (!(ADC->CTRLA.reg & ADC_CTRLA_ENABLE)) {
    enabledFlag = 1u;
    ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;
    adcSync();
  }

  ADC->SWTRIG.reg = ADC_SWTRIG_START;
  while (0 == (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY))
    ;
  result = ADC->RESULT.reg;

  ADC->INTFLAG.reg |= ADC_INTFLAG_RESRDY;
  ADC->INPUTCTRL.reg = inputCtrl;
  adcSync();

  /* Disable the ADC if it was enabled for a single conversion */
  if (enabledFlag) {
    ADC->CTRLA.reg &= ~ADC_CTRLA_ENABLE;
    adcSync();
  }

  return result;
}

static void adcSync(void) {
  while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY)
    ;
}
