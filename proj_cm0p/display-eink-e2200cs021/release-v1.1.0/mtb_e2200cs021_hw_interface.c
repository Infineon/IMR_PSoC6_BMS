/***********************************************************************************************//**
 * \file mtb_e2200cs021_hw_interface.c
 *
 * Description: This file is the implementation for the Cypress specific port.
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

#include "mtb_e2200cs021_hw_interface.h"

#include "cyhal_timer.h"
#include "cyhal_syspm.h"
#if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
#include "cyabs_rtos.h"
#endif

static bool                         screen_updating = false;
static const mtb_e2200cs021_pins_t* pins;
static cyhal_spi_t*                 spi_ptr;

#if !defined(CY_RTOS_AWARE) && !defined(COMPONENT_RTOS_AWARE)
static const cyhal_timer_cfg_t eink_timer_cfg =
{
    .is_continuous = true,
    .direction     = CYHAL_TIMER_DIR_UP,
    .is_compare    = true,
    .period        = 0xFFFFFFFF,
    .compare_value = 0xFFFFFFFF,
    .value         = 0,
};
static cyhal_timer_t eink_timer;

//--------------------------------------------------------------------------------------------------
// init_timer
//--------------------------------------------------------------------------------------------------
cy_rslt_t init_timer()
{
    cy_rslt_t result = cyhal_timer_init(&eink_timer, NC, NULL);
    if (CY_RSLT_SUCCESS == result)
    {

        result = cyhal_timer_configure(&eink_timer, &eink_timer_cfg);
    }
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_set_frequency(&eink_timer, 32000); // 1/32 millisecond accuracy
    }
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_start(&eink_timer);
    }
    return result;
}


//--------------------------------------------------------------------------------------------------
// syspm_callback
//--------------------------------------------------------------------------------------------------
bool syspm_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode,
                    void* callback_arg)
{
    CY_UNUSED_PARAMETER(state);
    CY_UNUSED_PARAMETER(callback_arg);
    switch (mode)
    {
        case CYHAL_SYSPM_CHECK_READY:
        {
            if (!screen_updating)
            {
                if (state == CYHAL_SYSPM_CB_CPU_DEEPSLEEP)
                {
                    // Since the tcpwm registers are retained over a deep sleep
                    // simply stopping the timer is sufficient here for a
                    // proper power transitions. One issue, however, is that
                    // there is a small, hardware level, delay between setting
                    // the TCPWM stop register and the TCPWM status register
                    // being updated. This is solved with a brief delay here.
                    cyhal_timer_stop(&eink_timer);
                    cyhal_system_delay_ms(1);
                    return true;
                }
                else if (state == CYHAL_SYSPM_CB_SYSTEM_HIBERNATE)
                {
                    cyhal_timer_free(&eink_timer);
                    return true;
                }

                // Should not get here but return true anyway since the
                // timer can run in other syspm states.
                return true;
            }
            else
            {
                return false;
            }

            break;
        }

        // The course of action is the same whether a later syspm check fails
        // or the board succesfully sleeps/hibernates: Restart/reinit the timer
        case CYHAL_SYSPM_CHECK_FAIL:
        case CYHAL_SYSPM_AFTER_TRANSITION:
            if (state == CYHAL_SYSPM_CB_CPU_DEEPSLEEP)
            {
                cyhal_timer_start(&eink_timer);
            }
            else if (state == CYHAL_SYSPM_CB_SYSTEM_HIBERNATE)
            {
                init_timer();
            }
            break;

        case CYHAL_SYSPM_BEFORE_TRANSITION:
        default:
            break;
    }

    return true;
}


static cyhal_syspm_callback_data_t syspm_callback_data =
{
    .callback     = &syspm_callback,
    .states       = (cyhal_syspm_callback_state_t)(CYHAL_SYSPM_CB_CPU_DEEPSLEEP |
                                                   CYHAL_SYSPM_CB_SYSTEM_HIBERNATE),
    .next         = NULL,
    .args         = NULL,
    .ignore_modes = (cyhal_syspm_callback_mode_t)0,
};
#endif // if !defined(CY_RTOS_AWARE) && !defined(COMPONENT_RTOS_AWARE)

//--------------------------------------------------------------------------------------------------
// MTB_E2200CS021_InitDriver
//--------------------------------------------------------------------------------------------------
cy_rslt_t MTB_E2200CS021_InitDriver(const mtb_e2200cs021_pins_t* pin_data, cyhal_spi_t* spi_inst)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    spi_ptr = spi_inst;
    pins    = pin_data;

    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_gpio_init(pins->spi_cs, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
    }
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_gpio_init(pins->reset, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
    }
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_gpio_init(pins->busy, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
    }
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_gpio_init(pins->discharge, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
    }
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_gpio_init(pins->enable, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        MTB_E2200CS021_CsHigh(); // Make the chip select HIGH
    }
    #if !defined(CY_RTOS_AWARE) && !defined(COMPONENT_RTOS_AWARE)
    if (CY_RSLT_SUCCESS == result)
    {
        result = init_timer();
    }

    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_syspm_register_callback(&syspm_callback_data);
    }
    #endif

    return result;
}


//--------------------------------------------------------------------------------------------------
// MTB_E2200CS021_GetTimeTick
//--------------------------------------------------------------------------------------------------
uint32_t MTB_E2200CS021_GetTimeTick(void)
{
    #if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
    uint32_t  rtosTime;
    cy_rslt_t status = cy_rtos_get_time(&rtosTime);
    CY_UNUSED_PARAMETER(status); // CY_ASSERT only processes in DEBUG, ignores for others
    CY_ASSERT(CY_RSLT_SUCCESS == status);
    return rtosTime;
    #else
    return cyhal_timer_read(&eink_timer) / 32; // millisecond accuracy
    #endif
}


//--------------------------------------------------------------------------------------------------
// MTB_E2200CS021_WriteSPI
//--------------------------------------------------------------------------------------------------
cy_rslt_t MTB_E2200CS021_WriteSPI(uint8_t data)
{
    return cyhal_spi_send(spi_ptr, data);
}


//--------------------------------------------------------------------------------------------------
// MTB_E2200CS021_ReadSPI
//--------------------------------------------------------------------------------------------------
cy_rslt_t MTB_E2200CS021_ReadSPI(uint8_t* data)
{
    uint32_t  readData;
    cy_rslt_t rslt = cyhal_spi_recv(spi_ptr, &readData);
    *data = (uint8_t)readData;
    return rslt;
}


//--------------------------------------------------------------------------------------------------
// MTB_E2200CS021_WriteReadSPI
//--------------------------------------------------------------------------------------------------
cy_rslt_t MTB_E2200CS021_WriteReadSPI(uint8_t writeData, uint8_t* readData)
{
    cy_rslt_t result = cyhal_spi_send(spi_ptr, writeData);
    if (CY_RSLT_SUCCESS == result)
    {
        uint32_t read;
        result    = cyhal_spi_recv(spi_ptr, &read);
        *readData = (uint8_t)read;
    }
    return result;
}


//--------------------------------------------------------------------------------------------------
// MTB_E2200CS021_WriteSPIBuffer
//--------------------------------------------------------------------------------------------------
cy_rslt_t MTB_E2200CS021_WriteSPIBuffer(uint8_t* data, uint16_t dataLength)
{
    return cyhal_spi_transfer(spi_ptr, (const uint8_t*)data, dataLength, NULL, 0, 0);
}


//--------------------------------------------------------------------------------------------------
// MTB_E2200CS021_ScreenIsUpdating
//--------------------------------------------------------------------------------------------------
void MTB_E2200CS021_ScreenIsUpdating(bool updating)
{
    screen_updating = updating;
}


//--------------------------------------------------------------------------------------------------
// MTB_E2200CS021_FreeDriver
//--------------------------------------------------------------------------------------------------
void MTB_E2200CS021_FreeDriver(void)
{
    cyhal_gpio_free(pins->spi_cs);
    cyhal_gpio_free(pins->reset);
    cyhal_gpio_free(pins->busy);
    cyhal_gpio_free(pins->discharge);
    cyhal_gpio_free(pins->enable);

    pins    = NULL;
    spi_ptr = NULL;

    #if !defined(CY_RTOS_AWARE) && !defined(COMPONENT_RTOS_AWARE)
    cyhal_timer_free(&eink_timer);
    cyhal_syspm_unregister_callback(&syspm_callback_data);
    #endif
}
