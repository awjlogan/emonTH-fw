#pragma once

/* SAML uses Arm Cortex-M0+ or Cortex-M4 - can place fast functions into RAM
 * to avoid the penalty of loading from flash with wait states.
 */
#define RAMFUNC __attribute__((section(".ramfunc")))

/* Board identification number. If a custom board is used, this should be
 * added to "dbgPutBoard" in emonTH.c
 */
#define BOARD_ID_LC       0
#define BOARD_ID_STANDARD 1
#define BOARD_ID_EMONPI   2
#define BOARD_ID_EMONTH3  16
#define BOARD_ID_DEV      255
#define BOARD_ID          BOARD_ID_EMONTH3

/* Define the peripherals by index for use in power reduction */
#define NUM_PERIPHERALS 7
typedef enum PeriphIndex_ {
  PERIPH_IDX_CORE = 0,
  PERIPH_IDX_I2CM = 1,
  PERIPH_IDX_SPI  = 2,
  PERIPH_IDX_UART = 3,
  PERIPH_IDX_DMAC = 4,
  PERIPH_IDX_ADC  = 5,
  PERIPH_IDX_TC   = 6
} PeriphIndex_t;

/* Clock frequencies
 *  - Core is on OSC16M @ 8 MHz
 *  - Peripherals are on the OSC16M / 8 -> 1 MHz
 */
#define F_CORE   8000000ul
#define F_PERIPH 1000000ul

/* Maximum number of OneWire DS18B20 sensors that can be used */
#define TEMP_MAX_ONEWIRE 4

#define RFM_TIMEOUT 30

/* Serial Communication Instances */
#define SERCOM_UART SERCOM0
#define SERCOM_SPI  SERCOM1
#define SERCOM_I2CM SERCOM0

#define SERCOM_UART_APBCMASK MCLK_APBCMASK_SERCOM0
#define SERCOM_SPI_APBCMASK  MCLK_APBCMASK_SERCOM1
#define SERCOM_I2CM_APBCMASK MCLK_APBCMASK_SERCOM0

#define SERCOM_UART_GCLK_ID      SERCOM0_GCLK_ID_CORE
#define SERCOM_SPI_GCLK_ID       SERCOM1_GCLK_ID_CORE
#define SERCOM_I2CM_GCLK_ID      SERCOM0_GCLK_ID_CORE
#define SERCOM_I2CM_GCLK_SLOW_ID SERCOM0_GCLK_ID_SLOW

#define SERCOM_SPI_DMAC_ID_TX  SERCOM1_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_TX SERCOM2_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_RX SERCOM2_DMAC_ID_RX
#define SERCOM_UART_DMAC_ID_TX SERCOM0_DMAC_ID_TX

#define SERCOM_UART_HANDLER_RXC irq_handler_sercom0_2()
#define SERCOM_UART_DRE_IRQn    SERCOM0_0_IRQn /* DRE interrupt */
#define SERCOM_UART_RXC_IRQn    SERCOM0_2_IRQn /* RXC interrupt */

/* Timer configuration */
#define TIMER_DELAY          TC1
#define TIMER_DELAY_APBCMASK MCLK_APBCMASK_TC1
#define TIMER_DELAY_GCLK_ID  TC1_GCLK_ID
#define TIMER_DELAY_IRQn     TC1_IRQn

#define TIMER_PULSE          TC2
#define TIMER_PULSE_APBCMASK MCLK_APBCMASK_TC2
#define TIMER_PULSE_GCLK_ID  TC2_GCLK_ID
#define TIMER_PULSE_IRQn     TC2_IRQn

/* Level interrupt EIC channels */
#define EIC_CH_NUM          3
#define EIC_CH_HDC          1
#define EIC_CH_RFM          3
#define EIC_CH_PULSE        4
#define EIC_IRQ_HANDLER_HDC irq_handler_eic_1
#define EIC_IRQn_HDC        EIC_1_IRQn
#define EIC_SENSE_HDC       EIC_CONFIG_SENSE1_RISE
#define EIC_FILTEN_HDC      1
#define EIC_INTENSET_HDC                                                       \
  EIC_INTENSET_EXTINT(                                                         \
      2) // Revisit : bug in header? Think the shift is incorrect
#define EIC_IRQ_HANDLER_RFM irq_handler_eic_3
#define EIC_IRQn_RFM        EIC_3_IRQn
#define EIC_SENSE_RFM       EIC_CONFIG_SENSE3_RISE
#define EIC_FILTEN_RFM      0
#define EIC_INTENSET_RFM    EIC_INTENSET_EXTINT(3)
#define EIC_IRQn_RFM        EIC_3_IRQn

/* Regulator enable */
#define PIN_REG_EN 0

/* Slide switches */
#define PIN_SW_NODE0 6
#define PIN_SW_NODE1 7

/* LED & GPIO */
#define PIN_LED   27
#define PIN_GPIO0 22
#define PIN_GPIO1 23

/* Battery sensing */
#define PIN_VBATT 2
#define AIN_VBATT ADC_INPUTCTRL_MUXPOS_AIN0

/* OneWire Interface */
#define PIN_ONEWIRE     3
#define PIN_ONEWIRE_PWR 5

/* Pulse interface */
#define PIN_PULSE 4

/* UART related defines */
#define PIN_UART_RX 25u
#define PIN_UART_TX 24u
#define UART_PAD_TX 2u
#define UART_PAD_RX 3u
#define UART_BAUD   115200u
#define UART_TXPO   SERCOM_USART_CTRLA_TXPO(1)
#define UART_RXPO   SERCOM_USART_CTRLA_RXPO(3)
#define PMUX_UART   PORT_PMUX_PMUXE(2) /* SERCOM */

/* SPI related defines */
#define PIN_SPI_MISO   18u
#define PIN_SPI_SCK    19u
#define PIN_SPI_MOSI   16u
#define PIN_SPI_RFM_SS 15u
#define SPI_DATA_BAUD  4000000ul
#define SPI_DIPO       SERCOM_SPI_CTRLA_DIPO(2)
#define SPI_DOPO       SERCOM_SPI_CTRLA_DOPO(3)
#define PMUX_SPI_DATA  PORT_PMUX_PMUXE(2) /* SERCOM */
#define PIN_RFM_IRQ    14u

/* I2C related defines */
#define PIN_I2CM_SDA 22u
#define PIN_I2CM_SCL 23u
#define PIN_HDC_DRDY 10
#define PIN_EXT_EN   11
#define PMUX_I2CM    PORT_PMUX_PMUXE(2) /* SERCOM */

/* DMA defines */
#define NUM_CHAN_DMA  2u
#define DMA_CHAN_SPI  1u
#define DMA_CHAN_UART 0u
