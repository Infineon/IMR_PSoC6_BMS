/*******************************************************************************
* File Name: cycfg_routing.h
*
* Description:
* Establishes all necessary connections between hardware elements.
* This file was automatically generated and should not be modified.
* Configurator Backend 3.10.0
* device-db 4.11.1.5194
* mtb-pdl-cat1 3.4.0.24948
*
********************************************************************************
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
********************************************************************************/

#if !defined(CYCFG_ROUTING_H)
#define CYCFG_ROUTING_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "cycfg_notices.h"

void init_cycfg_routing(void);

#define ioss_0_port_0_pin_0_ANALOG P0_0_SRSS_WCO_IN
#define ioss_0_port_0_pin_1_ANALOG P0_1_SRSS_WCO_OUT
#define ioss_0_port_0_pin_2_HSIOM P0_2_SCB0_UART_RX
#define ioss_0_port_0_pin_3_HSIOM P0_3_SCB0_UART_TX
#define ioss_0_port_5_pin_0_HSIOM P5_0_CANFD0_TTCAN_RX0
#define ioss_0_port_5_pin_1_HSIOM P5_1_CANFD0_TTCAN_TX0
#define ioss_0_port_6_pin_6_HSIOM P6_6_CPUSS_SWJ_SWDIO_TMS
#define ioss_0_port_6_pin_7_HSIOM P6_7_CPUSS_SWJ_SWCLK_TCLK
#define ioss_0_port_10_pin_6_ANALOG P10_6_PASS_SARMUX_PADS6
#define ioss_0_port_10_pin_7_ANALOG P10_7_PASS_SARMUX_PADS7

#define SAR0_VPLUS0_PIN 7
#define SAR0_VPLUS0_PORT 0
#define SAR0_VPLUS1_PIN 6
#define SAR0_VPLUS1_PORT 0

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_ROUTING_H */
