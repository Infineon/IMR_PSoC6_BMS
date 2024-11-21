/*******************************************************************************
 * File Name: cycfg_pins.h
 *
 * Description:
 * Pin configuration
 * This file was automatically generated and should not be modified.
 * Configurator Backend 3.30.0
 * device-db 4.11.1.5194
 * mtb-pdl-cat1 3.4.0.24948
 *
 *******************************************************************************
 * Copyright 2024 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#if !defined(CYCFG_PINS_H)
#define CYCFG_PINS_H

#include "cycfg_notices.h"
#include "cy_gpio.h"
#include "cycfg_routing.h"

#if defined (CY_USING_HAL)
#include "cyhal_hwmgr.h"
#endif /* defined (CY_USING_HAL) */

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define CYBSP_WCO_IN_ENABLED 1U
#define CYBSP_WCO_IN_PORT GPIO_PRT0
#define CYBSP_WCO_IN_PORT_NUM 0U
#define CYBSP_WCO_IN_PIN 0U
#define CYBSP_WCO_IN_NUM 0U
#define CYBSP_WCO_IN_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_WCO_IN_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_0_HSIOM
    #define ioss_0_port_0_pin_0_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_WCO_IN_HSIOM ioss_0_port_0_pin_0_HSIOM
#define CYBSP_WCO_IN_IRQ ioss_interrupts_gpio_0_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_WCO_IN_HAL_PORT_PIN P0_0
#define CYBSP_WCO_IN P0_0
#define CYBSP_WCO_IN_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_WCO_IN_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_WCO_IN_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_WCO_OUT_ENABLED 1U
#define CYBSP_WCO_OUT_PORT GPIO_PRT0
#define CYBSP_WCO_OUT_PORT_NUM 0U
#define CYBSP_WCO_OUT_PIN 1U
#define CYBSP_WCO_OUT_NUM 1U
#define CYBSP_WCO_OUT_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_WCO_OUT_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_1_HSIOM
    #define ioss_0_port_0_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_WCO_OUT_HSIOM ioss_0_port_0_pin_1_HSIOM
#define CYBSP_WCO_OUT_IRQ ioss_interrupts_gpio_0_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_WCO_OUT_HAL_PORT_PIN P0_1
#define CYBSP_WCO_OUT P0_1
#define CYBSP_WCO_OUT_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_WCO_OUT_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_WCO_OUT_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_DEBUG_UART_RX_ENABLED 1U
#define CYBSP_DEBUG_UART_RX_PORT GPIO_PRT0
#define CYBSP_DEBUG_UART_RX_PORT_NUM 0U
#define CYBSP_DEBUG_UART_RX_PIN 2U
#define CYBSP_DEBUG_UART_RX_NUM 2U
#define CYBSP_DEBUG_UART_RX_DRIVEMODE CY_GPIO_DM_HIGHZ
#define CYBSP_DEBUG_UART_RX_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_2_HSIOM
    #define ioss_0_port_0_pin_2_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_DEBUG_UART_RX_HSIOM ioss_0_port_0_pin_2_HSIOM
#define CYBSP_DEBUG_UART_RX_IRQ ioss_interrupts_gpio_0_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_DEBUG_UART_RX_HAL_PORT_PIN P0_2
#define CYBSP_DEBUG_UART_RX P0_2
#define CYBSP_DEBUG_UART_RX_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_DEBUG_UART_RX_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_DEBUG_UART_RX_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_NONE
#endif /* defined (CY_USING_HAL) */

#define CYBSP_DEBUG_UART_TX_ENABLED 1U
#define CYBSP_DEBUG_UART_TX_PORT GPIO_PRT0
#define CYBSP_DEBUG_UART_TX_PORT_NUM 0U
#define CYBSP_DEBUG_UART_TX_PIN 3U
#define CYBSP_DEBUG_UART_TX_NUM 3U
#define CYBSP_DEBUG_UART_TX_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define CYBSP_DEBUG_UART_TX_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_3_HSIOM
    #define ioss_0_port_0_pin_3_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_DEBUG_UART_TX_HSIOM ioss_0_port_0_pin_3_HSIOM
#define CYBSP_DEBUG_UART_TX_IRQ ioss_interrupts_gpio_0_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_DEBUG_UART_TX_HAL_PORT_PIN P0_3
#define CYBSP_DEBUG_UART_TX P0_3
#define CYBSP_DEBUG_UART_TX_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_DEBUG_UART_TX_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define CYBSP_DEBUG_UART_TX_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#define LED_Green (P10_0)
#define LED_Blue (P10_1)
#endif /* defined (CY_USING_HAL) */

#define LED_Red_ENABLED 1U
#define LED_Red_PORT GPIO_PRT10
#define LED_Red_PORT_NUM 10U
#define LED_Red_PIN 2U
#define LED_Red_NUM 2U
#define LED_Red_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define LED_Red_INIT_DRIVESTATE 1
#ifndef ioss_0_port_10_pin_2_HSIOM
    #define ioss_0_port_10_pin_2_HSIOM HSIOM_SEL_GPIO
#endif
#define LED_Red_HSIOM ioss_0_port_10_pin_2_HSIOM
#define LED_Red_IRQ ioss_interrupts_gpio_10_IRQn

#if defined (CY_USING_HAL)
#define LED_Red_HAL_PORT_PIN P10_2
#define LED_Red P10_2
#define LED_Red_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define LED_Red_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define LED_Red_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#define EPD_BUSY (P10_3)
#define EPD_RST_L (P10_4)
#endif /* defined (CY_USING_HAL) */

#define BMS_LED1_GR_ENABLED 1U
#define BMS_LED1_GR_PORT GPIO_PRT10
#define BMS_LED1_GR_PORT_NUM 10U
#define BMS_LED1_GR_PIN 5U
#define BMS_LED1_GR_NUM 5U
#define BMS_LED1_GR_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define BMS_LED1_GR_INIT_DRIVESTATE 1
#ifndef ioss_0_port_10_pin_5_HSIOM
    #define ioss_0_port_10_pin_5_HSIOM HSIOM_SEL_GPIO
#endif
#define BMS_LED1_GR_HSIOM ioss_0_port_10_pin_5_HSIOM
#define BMS_LED1_GR_IRQ ioss_interrupts_gpio_10_IRQn

#if defined (CY_USING_HAL)
#define BMS_LED1_GR_HAL_PORT_PIN P10_5
#define BMS_LED1_GR P10_5
#define BMS_LED1_GR_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define BMS_LED1_GR_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define BMS_LED1_GR_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif /* defined (CY_USING_HAL) */

#define ADC_2ED4820_CSO_ENABLED 1U
#define ADC_2ED4820_CSO_PORT GPIO_PRT10
#define ADC_2ED4820_CSO_PORT_NUM 10U
#define ADC_2ED4820_CSO_PIN 6U
#define ADC_2ED4820_CSO_NUM 6U
#define ADC_2ED4820_CSO_DRIVEMODE CY_GPIO_DM_ANALOG
#define ADC_2ED4820_CSO_INIT_DRIVESTATE 1
#ifndef ioss_0_port_10_pin_6_HSIOM
    #define ioss_0_port_10_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define ADC_2ED4820_CSO_HSIOM ioss_0_port_10_pin_6_HSIOM
#define ADC_2ED4820_CSO_IRQ ioss_interrupts_gpio_10_IRQn

#if defined (CY_USING_HAL)
#define ADC_2ED4820_CSO_HAL_PORT_PIN P10_6
#define ADC_2ED4820_CSO P10_6
#define ADC_2ED4820_CSO_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define ADC_2ED4820_CSO_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define ADC_2ED4820_CSO_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define ADC_CODING_RES_ENABLED 1U
#define ADC_CODING_RES_PORT GPIO_PRT10
#define ADC_CODING_RES_PORT_NUM 10U
#define ADC_CODING_RES_PIN 7U
#define ADC_CODING_RES_NUM 7U
#define ADC_CODING_RES_DRIVEMODE CY_GPIO_DM_ANALOG
#define ADC_CODING_RES_INIT_DRIVESTATE 1
#ifndef ioss_0_port_10_pin_7_HSIOM
    #define ioss_0_port_10_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define ADC_CODING_RES_HSIOM ioss_0_port_10_pin_7_HSIOM
#define ADC_CODING_RES_IRQ ioss_interrupts_gpio_10_IRQn

#if defined (CY_USING_HAL)
#define ADC_CODING_RES_HAL_PORT_PIN P10_7
#define ADC_CODING_RES P10_7
#define ADC_CODING_RES_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define ADC_CODING_RES_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define ADC_CODING_RES_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define PERI_SPI_MOSI (P11_0)
#define PERI_SPI_MISO (P11_1)
#define PERI_SPI_CLK (P11_2)
#define PERI_SPI_CS_MCP3465 (P11_3)
#define PERI_SPI_CS_2ED4820 (P11_4)
#define PERI_SPI_CS_CY15B (P11_5)
#define EPD_VCC_EN (P12_0)
#define EPD_DISCHARGE (P12_1)
#endif /* defined (CY_USING_HAL) */

#define ioss_0_port_12_pin_6_ENABLED 1U
#define ioss_0_port_12_pin_6_PORT GPIO_PRT12
#define ioss_0_port_12_pin_6_PORT_NUM 12U
#define ioss_0_port_12_pin_6_PIN 6U
#define ioss_0_port_12_pin_6_NUM 6U
#define ioss_0_port_12_pin_6_DRIVEMODE CY_GPIO_DM_ANALOG
#define ioss_0_port_12_pin_6_INIT_DRIVESTATE 1
#ifndef ioss_0_port_12_pin_6_HSIOM
    #define ioss_0_port_12_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_12_pin_6_IRQ ioss_interrupts_gpio_12_IRQn

#if defined (CY_USING_HAL)
#define ioss_0_port_12_pin_6_HAL_PORT_PIN P12_6
#define ioss_0_port_12_pin_6 P12_6
#define ioss_0_port_12_pin_6_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define ioss_0_port_12_pin_6_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define ioss_0_port_12_pin_6_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define ioss_0_port_12_pin_7_ENABLED 1U
#define ioss_0_port_12_pin_7_PORT GPIO_PRT12
#define ioss_0_port_12_pin_7_PORT_NUM 12U
#define ioss_0_port_12_pin_7_PIN 7U
#define ioss_0_port_12_pin_7_NUM 7U
#define ioss_0_port_12_pin_7_DRIVEMODE CY_GPIO_DM_ANALOG
#define ioss_0_port_12_pin_7_INIT_DRIVESTATE 1
#ifndef ioss_0_port_12_pin_7_HSIOM
    #define ioss_0_port_12_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_12_pin_7_IRQ ioss_interrupts_gpio_12_IRQn

#if defined (CY_USING_HAL)
#define ioss_0_port_12_pin_7_HAL_PORT_PIN P12_7
#define ioss_0_port_12_pin_7 P12_7
#define ioss_0_port_12_pin_7_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define ioss_0_port_12_pin_7_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define ioss_0_port_12_pin_7_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define WPn_ENABLED 1U
#define WPn_PORT GPIO_PRT2
#define WPn_PORT_NUM 2U
#define WPn_PIN 3U
#define WPn_NUM 3U
#define WPn_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define WPn_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_3_HSIOM
    #define ioss_0_port_2_pin_3_HSIOM HSIOM_SEL_GPIO
#endif
#define WPn_HSIOM ioss_0_port_2_pin_3_HSIOM
#define WPn_IRQ ioss_interrupts_gpio_2_IRQn

#if defined (CY_USING_HAL)
#define WPn_HAL_PORT_PIN P2_3
#define WPn P2_3
#define WPn_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define WPn_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define WPn_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif /* defined (CY_USING_HAL) */

#define HOLDn_ENABLED 1U
#define HOLDn_PORT GPIO_PRT2
#define HOLDn_PORT_NUM 2U
#define HOLDn_PIN 4U
#define HOLDn_NUM 4U
#define HOLDn_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define HOLDn_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_4_HSIOM
    #define ioss_0_port_2_pin_4_HSIOM HSIOM_SEL_GPIO
#endif
#define HOLDn_HSIOM ioss_0_port_2_pin_4_HSIOM
#define HOLDn_IRQ ioss_interrupts_gpio_2_IRQn

#if defined (CY_USING_HAL)
#define HOLDn_HAL_PORT_PIN P2_4
#define HOLDn P2_4
#define HOLDn_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define HOLDn_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define HOLDn_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CANFD_RX_ENABLED 1U
#define CYBSP_CANFD_RX_PORT GPIO_PRT5
#define CYBSP_CANFD_RX_PORT_NUM 5U
#define CYBSP_CANFD_RX_PIN 0U
#define CYBSP_CANFD_RX_NUM 0U
#define CYBSP_CANFD_RX_DRIVEMODE CY_GPIO_DM_HIGHZ
#define CYBSP_CANFD_RX_INIT_DRIVESTATE 1
#ifndef ioss_0_port_5_pin_0_HSIOM
    #define ioss_0_port_5_pin_0_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CANFD_RX_HSIOM ioss_0_port_5_pin_0_HSIOM
#define CYBSP_CANFD_RX_IRQ ioss_interrupts_gpio_5_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CANFD_RX_HAL_PORT_PIN P5_0
#define CYBSP_CANFD_RX P5_0
#define CYBSP_CANFD_RX_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CANFD_RX_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CANFD_RX_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_NONE
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CANFD_TX_ENABLED 1U
#define CYBSP_CANFD_TX_PORT GPIO_PRT5
#define CYBSP_CANFD_TX_PORT_NUM 5U
#define CYBSP_CANFD_TX_PIN 1U
#define CYBSP_CANFD_TX_NUM 1U
#define CYBSP_CANFD_TX_DRIVEMODE CY_GPIO_DM_STRONG
#define CYBSP_CANFD_TX_INIT_DRIVESTATE 1
#ifndef ioss_0_port_5_pin_1_HSIOM
    #define ioss_0_port_5_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CANFD_TX_HSIOM ioss_0_port_5_pin_1_HSIOM
#define CYBSP_CANFD_TX_IRQ ioss_interrupts_gpio_5_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CANFD_TX_HAL_PORT_PIN P5_1
#define CYBSP_CANFD_TX P5_1
#define CYBSP_CANFD_TX_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CANFD_TX_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#define CYBSP_CANFD_TX_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif /* defined (CY_USING_HAL) */

#define CANFD_STB_ENABLED 1U
#define CANFD_STB_PORT GPIO_PRT5
#define CANFD_STB_PORT_NUM 5U
#define CANFD_STB_PIN 6U
#define CANFD_STB_NUM 6U
#define CANFD_STB_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define CANFD_STB_INIT_DRIVESTATE 0
#ifndef ioss_0_port_5_pin_6_HSIOM
    #define ioss_0_port_5_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define CANFD_STB_HSIOM ioss_0_port_5_pin_6_HSIOM
#define CANFD_STB_IRQ ioss_interrupts_gpio_5_IRQn

#if defined (CY_USING_HAL)
#define CANFD_STB_HAL_PORT_PIN P5_6
#define CANFD_STB P5_6
#define CANFD_STB_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CANFD_STB_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define CANFD_STB_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#define TLE9012_UART_RX (P6_0)
#define TLE9012_UART_TX (P6_1)
#endif /* defined (CY_USING_HAL) */

#define CYBSP_SWITCH_ENABLED 1U
#define CYBSP_SWITCH_PORT GPIO_PRT6
#define CYBSP_SWITCH_PORT_NUM 6U
#define CYBSP_SWITCH_PIN 3U
#define CYBSP_SWITCH_NUM 3U
#define CYBSP_SWITCH_DRIVEMODE CY_GPIO_DM_PULLDOWN
#define CYBSP_SWITCH_INIT_DRIVESTATE 0
#ifndef ioss_0_port_6_pin_3_HSIOM
    #define ioss_0_port_6_pin_3_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_SWITCH_HSIOM ioss_0_port_6_pin_3_HSIOM
#define CYBSP_SWITCH_IRQ ioss_interrupts_gpio_6_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_SWITCH_HAL_PORT_PIN P6_3
#define CYBSP_SWITCH P6_3
#define CYBSP_SWITCH_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_SWITCH_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#define CYBSP_SWITCH_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_PULLDOWN
#define TLE9012_ERR (P6_5)
#endif /* defined (CY_USING_HAL) */

#define ioss_0_port_6_pin_6_ENABLED 1U
#define ioss_0_port_6_pin_6_PORT GPIO_PRT6
#define ioss_0_port_6_pin_6_PORT_NUM 6U
#define ioss_0_port_6_pin_6_PIN 6U
#define ioss_0_port_6_pin_6_NUM 6U
#define ioss_0_port_6_pin_6_DRIVEMODE CY_GPIO_DM_PULLUP
#define ioss_0_port_6_pin_6_INIT_DRIVESTATE 1
#ifndef ioss_0_port_6_pin_6_HSIOM
    #define ioss_0_port_6_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_6_pin_6_IRQ ioss_interrupts_gpio_6_IRQn

#if defined (CY_USING_HAL)
#define ioss_0_port_6_pin_6_HAL_PORT_PIN P6_6
#define ioss_0_port_6_pin_6 P6_6
#define ioss_0_port_6_pin_6_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define ioss_0_port_6_pin_6_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#define ioss_0_port_6_pin_6_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_PULLUP
#endif /* defined (CY_USING_HAL) */

#define ioss_0_port_6_pin_7_ENABLED 1U
#define ioss_0_port_6_pin_7_PORT GPIO_PRT6
#define ioss_0_port_6_pin_7_PORT_NUM 6U
#define ioss_0_port_6_pin_7_PIN 7U
#define ioss_0_port_6_pin_7_NUM 7U
#define ioss_0_port_6_pin_7_DRIVEMODE CY_GPIO_DM_PULLDOWN
#define ioss_0_port_6_pin_7_INIT_DRIVESTATE 1
#ifndef ioss_0_port_6_pin_7_HSIOM
    #define ioss_0_port_6_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_6_pin_7_IRQ ioss_interrupts_gpio_6_IRQn

#if defined (CY_USING_HAL)
#define ioss_0_port_6_pin_7_HAL_PORT_PIN P6_7
#define ioss_0_port_6_pin_7 P6_7
#define ioss_0_port_6_pin_7_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define ioss_0_port_6_pin_7_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#define ioss_0_port_6_pin_7_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_PULLDOWN
#define BUTTON_DISPLAY (P7_0)
#endif /* defined (CY_USING_HAL) */

#define BUTTON_SHUTDOWN_ENABLED 1U
#define BUTTON_SHUTDOWN_PORT GPIO_PRT7
#define BUTTON_SHUTDOWN_PORT_NUM 7U
#define BUTTON_SHUTDOWN_PIN 1U
#define BUTTON_SHUTDOWN_NUM 1U
#define BUTTON_SHUTDOWN_DRIVEMODE CY_GPIO_DM_HIGHZ
#define BUTTON_SHUTDOWN_INIT_DRIVESTATE 0
#ifndef ioss_0_port_7_pin_1_HSIOM
    #define ioss_0_port_7_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define BUTTON_SHUTDOWN_HSIOM ioss_0_port_7_pin_1_HSIOM
#define BUTTON_SHUTDOWN_IRQ ioss_interrupts_gpio_7_IRQn

#if defined (CY_USING_HAL)
#define BUTTON_SHUTDOWN_HAL_PORT_PIN P7_1
#define BUTTON_SHUTDOWN P7_1
#define BUTTON_SHUTDOWN_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define BUTTON_SHUTDOWN_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define BUTTON_SHUTDOWN_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_NONE
#define D_2ED4820_SAVE_STATE_EN (P7_2)
#define D_2ED4820_INT (P7_3)
#define D_2ED4820_ENABLE (P7_4)
#endif /* defined (CY_USING_HAL) */

#define BMS_LED4_BL_ENABLED 1U
#define GPIO_RELAIS_ENABLED BMS_LED4_BL_ENABLED
#define BMS_LED4_BL_PORT GPIO_PRT7
#define GPIO_RELAIS_PORT BMS_LED4_BL_PORT
#define BMS_LED4_BL_PORT_NUM 7U
#define GPIO_RELAIS_PORT_NUM BMS_LED4_BL_PORT_NUM
#define BMS_LED4_BL_PIN 5U
#define GPIO_RELAIS_PIN BMS_LED4_BL_PIN
#define BMS_LED4_BL_NUM 5U
#define GPIO_RELAIS_NUM BMS_LED4_BL_NUM
#define BMS_LED4_BL_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define GPIO_RELAIS_DRIVEMODE BMS_LED4_BL_DRIVEMODE
#define BMS_LED4_BL_INIT_DRIVESTATE 0
#define GPIO_RELAIS_INIT_DRIVESTATE BMS_LED4_BL_INIT_DRIVESTATE
#ifndef ioss_0_port_7_pin_5_HSIOM
    #define ioss_0_port_7_pin_5_HSIOM HSIOM_SEL_GPIO
#endif
#define BMS_LED4_BL_HSIOM ioss_0_port_7_pin_5_HSIOM
#define GPIO_RELAIS_HSIOM BMS_LED4_BL_HSIOM
#define BMS_LED4_BL_IRQ ioss_interrupts_gpio_7_IRQn
#define GPIO_RELAIS_IRQ BMS_LED4_BL_IRQ

#if defined (CY_USING_HAL)
#define BMS_LED4_BL_HAL_PORT_PIN P7_5
#define GPIO_RELAIS_HAL_PORT_PIN BMS_LED4_BL_HAL_PORT_PIN
#define BMS_LED4_BL P7_5
#define GPIO_RELAIS BMS_LED4_BL
#define BMS_LED4_BL_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define GPIO_RELAIS_HAL_IRQ BMS_LED4_BL_HAL_IRQ
#define BMS_LED4_BL_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define GPIO_RELAIS_HAL_DIR BMS_LED4_BL_HAL_DIR
#define BMS_LED4_BL_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#define GPIO_RELAIS_HAL_DRIVEMODE BMS_LED4_BL_HAL_DRIVEMODE
#endif /* defined (CY_USING_HAL) */

#define BMS_LED3_RD_ENABLED 1U
#define BMS_LED3_RD_PORT GPIO_PRT7
#define BMS_LED3_RD_PORT_NUM 7U
#define BMS_LED3_RD_PIN 6U
#define BMS_LED3_RD_NUM 6U
#define BMS_LED3_RD_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define BMS_LED3_RD_INIT_DRIVESTATE 0
#ifndef ioss_0_port_7_pin_6_HSIOM
    #define ioss_0_port_7_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define BMS_LED3_RD_HSIOM ioss_0_port_7_pin_6_HSIOM
#define BMS_LED3_RD_IRQ ioss_interrupts_gpio_7_IRQn

#if defined (CY_USING_HAL)
#define BMS_LED3_RD_HAL_PORT_PIN P7_6
#define BMS_LED3_RD P7_6
#define BMS_LED3_RD_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define BMS_LED3_RD_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define BMS_LED3_RD_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif /* defined (CY_USING_HAL) */

#define BMS_LED2_OR_ENABLED 1U
#define BMS_LED2_OR_PORT GPIO_PRT7
#define BMS_LED2_OR_PORT_NUM 7U
#define BMS_LED2_OR_PIN 7U
#define BMS_LED2_OR_NUM 7U
#define BMS_LED2_OR_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define BMS_LED2_OR_INIT_DRIVESTATE 0
#ifndef ioss_0_port_7_pin_7_HSIOM
    #define ioss_0_port_7_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define BMS_LED2_OR_HSIOM ioss_0_port_7_pin_7_HSIOM
#define BMS_LED2_OR_IRQ ioss_interrupts_gpio_7_IRQn

#if defined (CY_USING_HAL)
#define BMS_LED2_OR_HAL_PORT_PIN P7_7
#define BMS_LED2_OR P7_7
#define BMS_LED2_OR_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define BMS_LED2_OR_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define BMS_LED2_OR_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif /* defined (CY_USING_HAL) */

#define VCC_HOLD_ENABLED 1U
#define VCC_HOLD_PORT GPIO_PRT8
#define VCC_HOLD_PORT_NUM 8U
#define VCC_HOLD_PIN 3U
#define VCC_HOLD_NUM 3U
#define VCC_HOLD_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define VCC_HOLD_INIT_DRIVESTATE 0
#ifndef ioss_0_port_8_pin_3_HSIOM
    #define ioss_0_port_8_pin_3_HSIOM HSIOM_SEL_GPIO
#endif
#define VCC_HOLD_HSIOM ioss_0_port_8_pin_3_HSIOM
#define VCC_HOLD_IRQ ioss_interrupts_gpio_8_IRQn

#if defined (CY_USING_HAL)
#define VCC_HOLD_HAL_PORT_PIN P8_3
#define VCC_HOLD P8_3
#define VCC_HOLD_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define VCC_HOLD_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define VCC_HOLD_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#define SPI_EPD_MOSI (P9_0)
#define SPI_EPD_MISO (P9_1)
#define SPI_EPD_SCLK (P9_2)
#define SPI_EPD_CS (P9_3)
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_WCO_IN_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_WCO_IN_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_WCO_OUT_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_WCO_OUT_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_DEBUG_UART_RX_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_DEBUG_UART_RX_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_DEBUG_UART_TX_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_DEBUG_UART_TX_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t LED_Red_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t LED_Red_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t BMS_LED1_GR_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t BMS_LED1_GR_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t ADC_2ED4820_CSO_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t ADC_2ED4820_CSO_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t ADC_CODING_RES_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t ADC_CODING_RES_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t ioss_0_port_12_pin_6_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t ioss_0_port_12_pin_6_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t ioss_0_port_12_pin_7_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t ioss_0_port_12_pin_7_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t WPn_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t WPn_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t HOLDn_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t HOLDn_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CANFD_RX_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CANFD_RX_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CANFD_TX_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CANFD_TX_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CANFD_STB_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CANFD_STB_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_SWITCH_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_SWITCH_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_6_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t ioss_0_port_6_pin_6_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_7_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t ioss_0_port_6_pin_7_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t BUTTON_SHUTDOWN_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t BUTTON_SHUTDOWN_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t BMS_LED4_BL_config;

#define GPIO_RELAIS_config BMS_LED4_BL_config

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t BMS_LED4_BL_obj;
#define GPIO_RELAIS_obj BMS_LED4_BL_obj
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t BMS_LED3_RD_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t BMS_LED3_RD_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t BMS_LED2_OR_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t BMS_LED2_OR_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t VCC_HOLD_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t VCC_HOLD_obj;
#endif /* defined (CY_USING_HAL) */

void init_cycfg_pins(void);
void reserve_cycfg_pins(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_PINS_H */
