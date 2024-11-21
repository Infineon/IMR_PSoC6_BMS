/*******************************************************************************
 * File Name: cycfg_pins.c
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

#include "cycfg_pins.h"

const cy_stc_gpio_pin_config_t CYBSP_WCO_IN_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_WCO_IN_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_WCO_IN_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_WCO_IN_PORT_NUM,
    .channel_num = CYBSP_WCO_IN_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_WCO_OUT_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_WCO_OUT_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_WCO_OUT_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_WCO_OUT_PORT_NUM,
    .channel_num = CYBSP_WCO_OUT_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_DEBUG_UART_RX_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = CYBSP_DEBUG_UART_RX_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_DEBUG_UART_RX_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_DEBUG_UART_RX_PORT_NUM,
    .channel_num = CYBSP_DEBUG_UART_RX_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_DEBUG_UART_TX_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = CYBSP_DEBUG_UART_TX_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_DEBUG_UART_TX_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_DEBUG_UART_TX_PORT_NUM,
    .channel_num = CYBSP_DEBUG_UART_TX_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t LED_Red_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = LED_Red_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t LED_Red_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = LED_Red_PORT_NUM,
    .channel_num = LED_Red_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t BMS_LED1_GR_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = BMS_LED1_GR_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t BMS_LED1_GR_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = BMS_LED1_GR_PORT_NUM,
    .channel_num = BMS_LED1_GR_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t ADC_2ED4820_CSO_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = ADC_2ED4820_CSO_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t ADC_2ED4820_CSO_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = ADC_2ED4820_CSO_PORT_NUM,
    .channel_num = ADC_2ED4820_CSO_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t ADC_CODING_RES_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = ADC_CODING_RES_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t ADC_CODING_RES_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = ADC_CODING_RES_PORT_NUM,
    .channel_num = ADC_CODING_RES_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t ioss_0_port_12_pin_6_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = ioss_0_port_12_pin_6_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t ioss_0_port_12_pin_6_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = ioss_0_port_12_pin_6_PORT_NUM,
    .channel_num = ioss_0_port_12_pin_6_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t ioss_0_port_12_pin_7_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = ioss_0_port_12_pin_7_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t ioss_0_port_12_pin_7_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = ioss_0_port_12_pin_7_PORT_NUM,
    .channel_num = ioss_0_port_12_pin_7_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t WPn_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = WPn_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t WPn_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = WPn_PORT_NUM,
    .channel_num = WPn_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t HOLDn_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = HOLDn_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t HOLDn_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = HOLDn_PORT_NUM,
    .channel_num = HOLDn_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CANFD_RX_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = CYBSP_CANFD_RX_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CANFD_RX_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CANFD_RX_PORT_NUM,
    .channel_num = CYBSP_CANFD_RX_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CANFD_TX_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom = CYBSP_CANFD_TX_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CANFD_TX_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CANFD_TX_PORT_NUM,
    .channel_num = CYBSP_CANFD_TX_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CANFD_STB_config =
{
    .outVal = 0,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = CANFD_STB_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CANFD_STB_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CANFD_STB_PORT_NUM,
    .channel_num = CANFD_STB_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_SWITCH_config =
{
    .outVal = 0,
    .driveMode = CY_GPIO_DM_PULLDOWN,
    .hsiom = CYBSP_SWITCH_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_SWITCH_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_SWITCH_PORT_NUM,
    .channel_num = CYBSP_SWITCH_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_6_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_PULLUP,
    .hsiom = ioss_0_port_6_pin_6_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t ioss_0_port_6_pin_6_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = ioss_0_port_6_pin_6_PORT_NUM,
    .channel_num = ioss_0_port_6_pin_6_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_7_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_PULLDOWN,
    .hsiom = ioss_0_port_6_pin_7_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t ioss_0_port_6_pin_7_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = ioss_0_port_6_pin_7_PORT_NUM,
    .channel_num = ioss_0_port_6_pin_7_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t BUTTON_SHUTDOWN_config =
{
    .outVal = 0,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = BUTTON_SHUTDOWN_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t BUTTON_SHUTDOWN_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = BUTTON_SHUTDOWN_PORT_NUM,
    .channel_num = BUTTON_SHUTDOWN_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t BMS_LED4_BL_config =
{
    .outVal = 0,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = BMS_LED4_BL_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t BMS_LED4_BL_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = BMS_LED4_BL_PORT_NUM,
    .channel_num = BMS_LED4_BL_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t BMS_LED3_RD_config =
{
    .outVal = 0,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = BMS_LED3_RD_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t BMS_LED3_RD_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = BMS_LED3_RD_PORT_NUM,
    .channel_num = BMS_LED3_RD_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t BMS_LED2_OR_config =
{
    .outVal = 0,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = BMS_LED2_OR_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t BMS_LED2_OR_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = BMS_LED2_OR_PORT_NUM,
    .channel_num = BMS_LED2_OR_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t VCC_HOLD_config =
{
    .outVal = 0,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = VCC_HOLD_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t VCC_HOLD_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = VCC_HOLD_PORT_NUM,
    .channel_num = VCC_HOLD_PIN,
};
#endif /* defined (CY_USING_HAL) */

void init_cycfg_pins(void)
{
    Cy_GPIO_Pin_Init(CYBSP_DEBUG_UART_RX_PORT, CYBSP_DEBUG_UART_RX_PIN, &CYBSP_DEBUG_UART_RX_config);
    Cy_GPIO_Pin_Init(CYBSP_DEBUG_UART_TX_PORT, CYBSP_DEBUG_UART_TX_PIN, &CYBSP_DEBUG_UART_TX_config);
    Cy_GPIO_Pin_Init(LED_Red_PORT, LED_Red_PIN, &LED_Red_config);
    Cy_GPIO_Pin_Init(BMS_LED1_GR_PORT, BMS_LED1_GR_PIN, &BMS_LED1_GR_config);
    Cy_GPIO_Pin_Init(ADC_2ED4820_CSO_PORT, ADC_2ED4820_CSO_PIN, &ADC_2ED4820_CSO_config);
    Cy_GPIO_Pin_Init(ADC_CODING_RES_PORT, ADC_CODING_RES_PIN, &ADC_CODING_RES_config);
    Cy_GPIO_Pin_Init(ioss_0_port_12_pin_6_PORT, ioss_0_port_12_pin_6_PIN, &ioss_0_port_12_pin_6_config);
    Cy_GPIO_Pin_Init(ioss_0_port_12_pin_7_PORT, ioss_0_port_12_pin_7_PIN, &ioss_0_port_12_pin_7_config);
    Cy_GPIO_Pin_Init(WPn_PORT, WPn_PIN, &WPn_config);
    Cy_GPIO_Pin_Init(HOLDn_PORT, HOLDn_PIN, &HOLDn_config);
    Cy_GPIO_Pin_Init(CYBSP_CANFD_RX_PORT, CYBSP_CANFD_RX_PIN, &CYBSP_CANFD_RX_config);
    Cy_GPIO_Pin_Init(CYBSP_CANFD_TX_PORT, CYBSP_CANFD_TX_PIN, &CYBSP_CANFD_TX_config);
    Cy_GPIO_Pin_Init(CANFD_STB_PORT, CANFD_STB_PIN, &CANFD_STB_config);
    Cy_GPIO_Pin_Init(CYBSP_SWITCH_PORT, CYBSP_SWITCH_PIN, &CYBSP_SWITCH_config);
    Cy_GPIO_Pin_Init(ioss_0_port_6_pin_6_PORT, ioss_0_port_6_pin_6_PIN, &ioss_0_port_6_pin_6_config);
    Cy_GPIO_Pin_Init(ioss_0_port_6_pin_7_PORT, ioss_0_port_6_pin_7_PIN, &ioss_0_port_6_pin_7_config);
    Cy_GPIO_Pin_Init(BUTTON_SHUTDOWN_PORT, BUTTON_SHUTDOWN_PIN, &BUTTON_SHUTDOWN_config);
    Cy_GPIO_Pin_Init(BMS_LED4_BL_PORT, BMS_LED4_BL_PIN, &BMS_LED4_BL_config);
    Cy_GPIO_Pin_Init(BMS_LED3_RD_PORT, BMS_LED3_RD_PIN, &BMS_LED3_RD_config);
    Cy_GPIO_Pin_Init(BMS_LED2_OR_PORT, BMS_LED2_OR_PIN, &BMS_LED2_OR_config);
    Cy_GPIO_Pin_Init(VCC_HOLD_PORT, VCC_HOLD_PIN, &VCC_HOLD_config);
}
void reserve_cycfg_pins(void)
{
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&CYBSP_WCO_IN_obj);
    cyhal_hwmgr_reserve(&CYBSP_WCO_OUT_obj);
    cyhal_hwmgr_reserve(&CYBSP_DEBUG_UART_RX_obj);
    cyhal_hwmgr_reserve(&CYBSP_DEBUG_UART_TX_obj);
    cyhal_hwmgr_reserve(&LED_Red_obj);
    cyhal_hwmgr_reserve(&BMS_LED1_GR_obj);
    cyhal_hwmgr_reserve(&ADC_2ED4820_CSO_obj);
    cyhal_hwmgr_reserve(&ADC_CODING_RES_obj);
    cyhal_hwmgr_reserve(&ioss_0_port_12_pin_6_obj);
    cyhal_hwmgr_reserve(&ioss_0_port_12_pin_7_obj);
    cyhal_hwmgr_reserve(&WPn_obj);
    cyhal_hwmgr_reserve(&HOLDn_obj);
    cyhal_hwmgr_reserve(&CYBSP_CANFD_RX_obj);
    cyhal_hwmgr_reserve(&CYBSP_CANFD_TX_obj);
    cyhal_hwmgr_reserve(&CANFD_STB_obj);
    cyhal_hwmgr_reserve(&CYBSP_SWITCH_obj);
    cyhal_hwmgr_reserve(&ioss_0_port_6_pin_6_obj);
    cyhal_hwmgr_reserve(&ioss_0_port_6_pin_7_obj);
    cyhal_hwmgr_reserve(&BUTTON_SHUTDOWN_obj);
    cyhal_hwmgr_reserve(&BMS_LED4_BL_obj);
    cyhal_hwmgr_reserve(&BMS_LED3_RD_obj);
    cyhal_hwmgr_reserve(&BMS_LED2_OR_obj);
    cyhal_hwmgr_reserve(&VCC_HOLD_obj);
#endif /* defined (CY_USING_HAL) */
}
