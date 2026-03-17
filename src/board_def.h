#pragma once

/* SAML uses Arm Cortex-M0+ or Cortex-M4 - can place fast functions into RAM
 * to avoid the penalty of loading from flash with wait states.
 */
#define RAMFUNC __attribute__((section(".ramfunc")))

/* Clock frequencies
 *  - Core is on OSC16M @ 8 MHz
 */
#define F_CORE 8000000ul

/* Maximum number of OneWire DS18B20 sensors that can be used */
#define TEMP_MAX_ONEWIRE 4
_Static_assert(((1 == TEMP_MAX_ONEWIRE) || (4 == TEMP_MAX_ONEWIRE)),
               "Max number of external OneWire sensors can only be 1 or 4.");

/* Serial Communication Instances */
#define SERCOM_I2CM SERCOM0
#define SERCOM_SPI  SERCOM1
#define SERCOM_UART SERCOM2

#define SERCOM_I2CM_APBCMASK MCLK_APBCMASK_SERCOM0
#define SERCOM_SPI_APBCMASK  MCLK_APBCMASK_SERCOM1
#define SERCOM_UART_APBCMASK MCLK_APBCMASK_SERCOM2

#define SERCOM_I2CM_GCLK_ID      SERCOM0_GCLK_ID_CORE
#define SERCOM_I2CM_GCLK_SLOW_ID SERCOM0_GCLK_ID_SLOW
#define SERCOM_SPI_GCLK_ID       SERCOM1_GCLK_ID_CORE
#define SERCOM_UART_GCLK_ID      SERCOM2_GCLK_ID_CORE

#define SERCOM_UART_DMAC_ID_TX SERCOM2_DMAC_ID_TX

#define SERCOM_UART_HANDLER_RXC irq_handler_sercom2_2()
#define SERCOM_UART_DRE_IRQn    SERCOM2_0_IRQn /* DRE interrupt */
#define SERCOM_UART_RXC_IRQn    SERCOM2_2_IRQn /* RXC interrupt */

#define DMAC_UART_IRQn DMAC_0_IRQn

/* Timer configurations
 * TC0/1 share the same GCLK source, so group on same low power oscillator.
 * TC2 handles higher resolution time.
 */
#define TC_NUM_INST 3

#define TIMER_LP          TC0
#define TIMER_LP_APBCMASK MCLK_APBCMASK_TC0
#define TIMER_LP_GCLK_ID  TC0_GCLK_ID
#define TIMER_LP_IRQn     TC0_IRQn
#define TIMER_LP_HANDLER  irq_handler_tc0

#define TIMER_PULSE          TC1
#define TIMER_PULSE_APBCMASK MCLK_APBCMASK_TC1
#define TIMER_PULSE_GCLK_ID  TC1_GCLK_ID
#define TIMER_PULSE_IRQn     TC1_IRQn
#define TIMER_PULSE_HANDLER  irq_handler_tc1

#define TIMER_DELAY          TC2
#define TIMER_DELAY_APBCMASK MCLK_APBCMASK_TC2
#define TIMER_DELAY_GCLK_ID  TC2_GCLK_ID
#define TIMER_DELAY_IRQn     TC2_IRQn
#define TIMER_DELAY_HANDLER  irq_handler_tc2

/* EIC channels */
#define EIC_IRQ_HANDLER irq_handler_eic_4

/* Regulator enable */
#define PIN_REG_EN 0u

/* Slide switches */
#define PIN_SW_NODE0 6u
#define PIN_SW_NODE1 7u

/* LED & GPIO */
#define PIN_LED   14u
#define PIN_GPIO0 11u
#define PIN_GPIO1 10u

/* Battery sensing */
#define PIN_VBATT 2u
#define AIN_VBATT ADC_INPUTCTRL_MUXPOS_AIN0

/* OneWire Interface */
#define PIN_ONEWIRE     5u
#define PIN_ONEWIRE_PWR 3u

/* Pulse interface */
#define PIN_PULSE 4u

/* UART related defines */
#define PIN_UART_RX 9u
#define PIN_UART_TX 8u
#define UART_PAD_TX 0u
#define UART_PAD_RX 1u
#define UART_BAUD   115200u
#define UART_TXPO   SERCOM_USART_CTRLA_TXPO(UART_PAD_TX)
#define UART_RXPO   SERCOM_USART_CTRLA_RXPO(UART_PAD_RX)
#define PMUX_UART   PORT_PMUX_PMUXE(3) /* SERCOM-ALT */

/* SPI related defines */
#define PIN_SPI_MISO   18u
#define PIN_SPI_SCK    19u
#define PIN_SPI_MOSI   16u
#define PIN_SPI_RFM_SS 17u
#define SPI_DATA_BAUD  4000000ul
#define SPI_DIPO       SERCOM_SPI_CTRLA_DIPO(2)
#define SPI_DOPO       SERCOM_SPI_CTRLA_DOPO(3)
#define PMUX_SPI_DATA  PORT_PMUX_PMUXE(2) /* SERCOM */
#define PIN_RFM_IRQ    15u

/* I2C related defines */
#define PIN_I2CM_SDA 22u
#define PIN_I2CM_SCL 23u
#define PIN_HDC_DRDY 25u
#define PIN_EXT_EN   24u
#define PMUX_I2CM    PORT_PMUX_PMUXE(2) /* SERCOM */

/* DMA defines */
#define NUM_CHAN_DMA  1u
#define DMA_CHAN_UART 0u
