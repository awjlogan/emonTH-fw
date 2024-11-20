#pragma once

/* SAMD uses Arm Cortex-M0+ or Cortex-M4 - can place fast functions into RAM
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
 *  - Core is on OSC8M
 *  - Peripherals are on the OSC8M / 8 -> 1 MHz
 */
#define F_CORE   8000000ul
#define F_PERIPH 1000000ul

/* Pulse count setup */
#define NUM_PULSECOUNT 1

/* Temperature sensors
 * This is the maximum number of OneWire DS18B20 sensors that can be used
 */
#define TEMP_MAX_ONEWIRE 4

/* Serial Communication Instances */
#define SERCOM_SPI      SERCOM0
#define SERCOM_I2CM     SERCOM2
#define SERCOM_UART_DBG SERCOM1

#define SERCOM_SPI_APBCMASK      MCLK_APBCMASK_SERCOM0
#define SERCOM_I2CM_INT_APBCMASK MCLK_APBCMASK_SERCOM2
#define SERCOM_UART_DBG_APBCMASK MCLK_APBCMASK_SERCOM1

#define SERCOM_SPI_GCLK_ID      SERCOM0_GCLK_ID_CORE
#define SERCOM_I2CM_INT_GCLK_ID SERCOM2_GCLK_ID_CORE
#define SERCOM_UART_DBG_GCLK_ID SERCOM1_GCLK_ID_CORE

#define SERCOM_SPI_DMAC_ID_TX      SERCOM0_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_TX     SERCOM2_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_RX     SERCOM2_DMAC_ID_RX
#define SERCOM_UART_DBG_DMAC_ID_TX SERCOM1_DMAC_ID_TX

#define SERCOM_UART_INTERACTIVE_HANDLER irq_handler_sercom1()
#define SERCOM_UART_INTERACTIVE         SERCOM1

#define SERCOM_UART_DBG_NVIC_IRQn    SERCOM1_0_IRQn
#define SERCOM_UART_INTERACTIVE_IRQn SERCOM1_0_IRQn

/* Timer configuration */
#define TIMER_DELAY          TC1
#define TIMER_DELAY_APBCMASK MCLK_APBCMASK_TC1
#define TIMER_DELAY_GCLK_ID  TC1_GCLK_ID
#define TIMER_DELAY_IRQn     TC1_IRQn

/* Pin Configuration (nb. logical, not physical) */
#define GRP_PINA 0u

/* Level interrupt EIC channels */
#define EIC_CH_NUM 3
#define EIC_CH_RFM 0
#define EIC_CH_HDC 1
#define EIC_CH_OW  6

/* Slide switches */
#define GRP_SLIDE_SW GRP_PINA
#define PIN_SW_NODE0 4
#define PIN_SW_NODE1 5
#define PIN_SW_UART  6

/* LED */
#define GRP_LED GRP_PINA
#define PIN_LED 17u

/* Battery sensing */
#define GRP_BATT_SENSE GRP_PINA
#define PIN_VBATT_DIV4 2
#define AIN_VBATT      ADC_INPUTCTRL_MUXPOS_AIN0

/* OneWire Interface */
#define GRP_ONEWIRE     GRP_PINA
#define PIN_ONEWIRE     8u
#define PIN_ONEWIRE_PWR 9u

/* Pulse interface */
#define GRP_PULSE  GRP_PINA
#define PIN_PULSE1 7u

/* Debug UART related defines */
#define GRP_SERCOM_UART_DBG0 GRP_PINA
#define PIN_UART_DBG_RX0     25u
#define PIN_UART_DBG_TX0     24u
#define UART_DBG_PAD_RX      3u
#define UART_DBG_PAD_TX      2u
#define UART_DBG_BAUD        38400u
#define PMUX_UART_DBG0       PORT_PMUX_PMUXE_C /* SERCOM */

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
#define PIN_I2CM_SDA    22u
#define PIN_I2CM_SCL    23u
#define PMUX_I2CM       PORT_PMUX_PMUXE(2) /* SERCOM (Alt) */

/* DMA defines */
#define NUM_CHAN_DMA      2u
#define DMA_CHAN_I2CM     1u
#define DMA_CHAN_UART_DBG 0u
