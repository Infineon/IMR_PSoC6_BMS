/*******************************************************************************
 * File Name: cycfg_routing.c
 *
 * Description:
 * Establishes all necessary connections between hardware elements.
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

#include "cycfg_routing.h"
#include "cy_trigmux.h"
#include "stdbool.h"
#include "cy_device_headers.h"

void init_cycfg_routing(void)
{
    SAR->MUX_SWITCH0 = SAR_MUX_SWITCH0_MUX_FW_P6_VPLUS_Msk |
        SAR_MUX_SWITCH0_MUX_FW_P7_VPLUS_Msk;
    SAR->MUX_SWITCH_SQ_CTRL = SAR_MUX_SWITCH_SQ_CTRL_MUX_SQ_CTRL_P6_Msk |
        SAR_MUX_SWITCH_SQ_CTRL_MUX_SQ_CTRL_P7_Msk;
    SAR->CTRL |= SAR_CTRL_ENABLED_Msk;
}
