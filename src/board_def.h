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
 *  - Core is on OSC16M
 *  - Peripherals are on the OSC16M / 16 -> 1 MHz
 */
#define F_CORE   8000000ul
#define F_PERIPH 1000000ul

/* Maximum number of OneWire DS18B20 sensors that can be used */
#define TEMP_MAX_ONEWIRE 4

#define RFM_TIMEOUT 30
#define RFM_RETRIES 8

/* Serial Communication Instances */
#define SERCOM_SPI  SERCOM0
#define SERCOM_I2CM SERCOM2
#define SERCOM_UART SERCOM1

#define SERCOM_SPI_APBCMASK      MCLK_APBCMASK_SERCOM0
#define SERCOM_I2CM_INT_APBCMASK MCLK_APBCMASK_SERCOM2
#define SERCOM_UART_APBCMASK     MCLK_APBCMASK_SERCOM1

#define SERCOM_SPI_GCLK_ID      SERCOM0_GCLK_ID_CORE
#define SERCOM_I2CM_INT_GCLK_ID SERCOM2_GCLK_ID_CORE
#define SERCOM_UART_GCLK_ID     SERCOM1_GCLK_ID_CORE

#define SERCOM_SPI_DMAC_ID_TX  SERCOM0_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_TX SERCOM2_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_RX SERCOM2_DMAC_ID_RX
#define SERCOM_UART_DMAC_ID_TX SERCOM1_DMAC_ID_TX

#define SERCOM_UART_HANDLER   irq_handler_sercom1()
#define SERCOM_UART_NVIC_IRQn SERCOM1_0_IRQn
#define SERCOM_UART_IRQn      SERCOM1_0_IRQn

/* Timer configuration */
#define TIMER_DELAY          TC1
#define TIMER_DELAY_APBCMASK MCLK_APBCMASK_TC1
#define TIMER_DELAY_GCLK_ID  TC1_GCLK_ID
#define TIMER_DELAY_IRQn     TC1_IRQn

#define TIMER_PULSE          TC2
#define TIMER_PULSE_APBCMASK MCLK_APBCMASK_TC2
#define TIMER_PULSE_GCLK_ID  TC2_GCLK_ID
#define TIMER_PULSE_IRQn     TC2_IRQn

/* Pin Configuration (nb. logical, not physical) */
#define GRP_PINA 0u

/* TPL5111 interface */
#define GRP_TPL      GRP_PINA
#define PIN_TPL_DRV  0u
#define PIN_TPL_DONE 1u

/* Level interrupt EIC channels */
#define EIC_CH_NUM   5
#define EIC_CH_WAKE  0
#define EIC_CH_RFM   4
#define EIC_CH_HDC   1
#define EIC_CH_OW    6
#define EIC_CH_PULSE 7

/* Slide switches */
#define GRP_SLIDE_SW GRP_PINA
#define PIN_SW_NODE0 3
#define PIN_SW_NODE1 4

/* LED */
#define GRP_LED GRP_PINA
#define PIN_LED 14u

/* Battery sensing */
#define GRP_BATT_SENSE GRP_PINA
#define PIN_VBATT_DIV4 2
#define AIN_VBATT      ADC_INPUTCTRL_MUXPOS_AIN0

/* OneWire Interface */
#define GRP_ONEWIRE     GRP_PINA
#define PIN_ONEWIRE     9u
#define PIN_ONEWIRE_PWR 10u

/* Pulse interface */
#define GRP_PULSE GRP_PINA
#define PIN_PULSE 8u

/* Debug UART related defines */
#define GRP_SERCOM_UART GRP_PINA
#define PIN_UART_RX     25u
#define PIN_UART_TX     24u
#define UART_PAD_RX     3u
#define UART_PAD_TX     2u
#define UART_BAUD       115200u
#define PMUX_UART       PORT_PMUX_PMUXE(2) /* SERCOM */

/* SPI related defines */
#define GRP_SERCOM_SPI GRP_PINA
#define PIN_SPI_MISO   15u
#define PIN_SPI_SCK    11u
#define PIN_SPI_MOSI   10u
#define PIN_SPI_RFM_SS 14u
#define SPI_DATA_BAUD  4000000ul
#define PMUX_SPI_DATA  PORT_PMUX_PMUXE(2) /* SERCOM */

/* I2C related defines */
#define HDC_ADDR        0x40 /* ADDR tied LOW */
#define GRP_SERCOM_I2CM GRP_PINA
#define PIN_I2CM_SDA    5u
#define PIN_I2CM_SCL    6u
#define PMUX_I2CM       PORT_PMUX_PMUXE(3) /* SERCOM (Alt) */

/* DMA defines */
#define NUM_CHAN_DMA  2u
#define DMA_CHAN_I2CM 1u
#define DMA_CHAN_UART 0u
