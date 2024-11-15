#include "driver_ADC.h"
#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "driver_SAMD.h"
#include "emonTH_samd.h"

#include "emonTH.h"

static void adcSync(void);

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

  /* Input control - requires synchronisation (33.6.15) */
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_PIN0 | ADC_INPUTCTRL_MUXNEG_GND;
  adcSync();
}

static void adcSync(void) {
  while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY)
    ;
}
