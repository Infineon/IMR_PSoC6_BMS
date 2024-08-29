/***********************************************************************************************//**
 * \file mtb_e2200cs021_hw_interface.h
 *
 * Description: This file contains Infineon specific hardware configuration.
 *
 * For the details of E-INK display hardware and driver interface, see the
 * documents available at the following website:
 * https://www.pervasivedisplays.com/product/2-71-e-ink-display/
 *
 ***************************************************************************************************
 * \copyright
 * Copyright 2018-2021 Cypress Semiconductor Corporation
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
 **************************************************************************************************/

#pragma once

#include "cy_result.h"
#include "cyhal_gpio.h"
#include "cyhal_spi.h"
#include "cyhal_system.h"
#include "mtb_e2200cs021.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// Macros used for byte level operations
#define MTB_E2200CS021_BYTE_SIZE       (uint8_t)(0x08u)

// Definitions of pin sates
#define MTB_E2200CS021_PIN_LOW         (uint8_t)(0x00u)
#define MTB_E2200CS021_PIN_HIGH        (uint8_t)(0x01u)

// Push the chip select pin to logic HIGH
#define MTB_E2200CS021_CsHigh()          cyhal_gpio_write(pins->spi_cs, MTB_E2200CS021_PIN_HIGH)

// Pull the chip select pin to logic LOW
#define MTB_E2200CS021_CsLow()           cyhal_gpio_write(pins->spi_cs, MTB_E2200CS021_PIN_LOW)

// Push the reset pin to logic HIGH
#define MTB_E2200CS021_RstHigh()         cyhal_gpio_write(pins->reset, MTB_E2200CS021_PIN_HIGH)

// Pull the reset pin to logic LOW
#define MTB_E2200CS021_RstLow()          cyhal_gpio_write(pins->reset, MTB_E2200CS021_PIN_LOW)

// Push the discharge pin to logic HIGH
#define MTB_E2200CS021_DischargeHigh()   cyhal_gpio_write(pins->discharge, MTB_E2200CS021_PIN_HIGH)

// Pull the discharge pin to logic LOW
#define MTB_E2200CS021_DischargeLow()    cyhal_gpio_write(pins->discharge, MTB_E2200CS021_PIN_LOW)

// Turn on the display by pushing the display enable pin to logic HIGH
#define MTB_E2200CS021_TurnOnVcc()       cyhal_gpio_write(pins->enable, MTB_E2200CS021_PIN_LOW) 	// Inverted High-Side switch logic

// Turn off the display by pulling the display enable pin to logic LOW
#define MTB_E2200CS021_TurnOffVcc()      cyhal_gpio_write(pins->enable, MTB_E2200CS021_PIN_HIGH) 	// Inverted High-Side switch logic

// Check to see if the DISPBUSY pin is logic HIGH
#define MTB_E2200CS021_IsBusy()          cyhal_gpio_read(pins->busy)

#define MTB_E2200CS021_WaitUs(x)         cyhal_system_delay_us(x)

#define MTB_E2200CS021_WaitMs(x)         cyhal_system_delay_ms(x)

cy_rslt_t MTB_E2200CS021_InitDriver(const mtb_e2200cs021_pins_t* pins, cyhal_spi_t* spi_inst);
cy_rslt_t MTB_E2200CS021_WriteSPI(uint8_t data);
cy_rslt_t MTB_E2200CS021_ReadSPI(uint8_t* data);
cy_rslt_t MTB_E2200CS021_WriteReadSPI(uint8_t writeData, uint8_t* readData);
cy_rslt_t MTB_E2200CS021_WriteSPIBuffer(uint8_t* data, uint16_t dataLength);
uint32_t MTB_E2200CS021_GetTimeTick(void);
void MTB_E2200CS021_ScreenIsUpdating(bool updating);
void MTB_E2200CS021_FreeDriver(void);

#if defined(__cplusplus)
}
#endif
