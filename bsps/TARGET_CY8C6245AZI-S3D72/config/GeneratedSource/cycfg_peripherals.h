/*******************************************************************************
* File Name: cycfg_peripherals.h
*
* Description:
* Peripheral Hardware Block configuration
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

#if !defined(CYCFG_PERIPHERALS_H)
#define CYCFG_PERIPHERALS_H

#include "cycfg_notices.h"
#include "cy_canfd.h"
#include "cy_sysclk.h"
#if defined (CY_USING_HAL)
    #include "cyhal_hwmgr.h"
#endif //defined (CY_USING_HAL)
#include "cy_sysanalog.h"
#include "cy_sar.h"
#include "cycfg_routing.h"
#include "cy_scb_uart.h"
#include "cy_rtc.h"
#include "cy_tcpwm_counter.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define CANFD_ENABLED 1U
#define CANFD_HW CANFD0
#define CANFD_CHANNEL CANFD0_CH0
#define CANFD_STD_ID_FILTER_ID_0 0
#define CANFD_DATA_0 0
#define CANFD_DATA_1 1
#define CANFD_DATA_2 2
#define CANFD_DATA_3 3
#define CANFD_DATA_4 4
#define CANFD_DATA_5 5
#define CANFD_DATA_6 6
#define CANFD_DATA_7 7
#define CANFD_DATA_8 8
#define CANFD_DATA_9 9
#define CANFD_DATA_10 10
#define CANFD_DATA_11 11
#define CANFD_DATA_12 12
#define CANFD_DATA_13 13
#define CANFD_DATA_14 14
#define CANFD_DATA_15 15
#define CANFD_IRQ_0 canfd_0_interrupts0_0_IRQn
#define CANFD_IRQ_1 canfd_0_interrupts1_0_IRQn
#define pass_0_aref_0_ENABLED 1U
#define SAR_ENABLED 1U
#define SAR_HW SAR
#define SAR_IRQ pass_interrupt_sar_IRQn
#define SAR_CTL ((uint32_t)CY_SAR_VREF_PWR_100 | (uint32_t)CY_SAR_VREF_SEL_VDDA | (uint32_t)CY_SAR_BYPASS_CAP_ENABLE | (uint32_t)CY_SAR_NEG_SEL_VSSA_KELVIN | (uint32_t)CY_SAR_CTRL_NEGVREF_HW | (uint32_t)CY_SAR_CTRL_COMP_DLY_4 | (uint32_t)CY_SAR_COMP_PWR_100 | (uint32_t)CY_SAR_DEEPSLEEP_SARMUX_OFF | (uint32_t)CY_SAR_SARSEQ_SWITCH_ENABLE)
#define SAR_SAMPLE ((uint32_t)SAR_SAMPLE_CTRL_EOS_DSI_OUT_EN_Msk | (uint32_t)CY_SAR_RIGHT_ALIGN | (uint32_t)CY_SAR_DIFFERENTIAL_SIGNED | (uint32_t)CY_SAR_SINGLE_ENDED_SIGNED | (uint32_t)CY_SAR_AVG_CNT_16 | (uint32_t)CY_SAR_AVG_MODE_SEQUENTIAL_FIXED | (uint32_t)CY_SAR_TRIGGER_MODE_FW_ONLY)
#define SAR_CH0_CONFIG (((uint32_t)SAR0_VPLUS0_PORT << SAR_CHAN_CONFIG_POS_PORT_ADDR_Pos) | (uint32_t)(SAR0_VPLUS0_PIN << SAR_CHAN_CONFIG_POS_PIN_ADDR_Pos) | CY_SAR_CHAN_SINGLE_ENDED | CY_SAR_CHAN_AVG_ENABLE | (uint32_t)CY_SAR_CHAN_SAMPLE_TIME_0)
#define SAR_CH1_CONFIG (((uint32_t)SAR0_VPLUS1_PORT << SAR_CHAN_CONFIG_POS_PORT_ADDR_Pos) | (uint32_t)(SAR0_VPLUS1_PIN << SAR_CHAN_CONFIG_POS_PIN_ADDR_Pos) | CY_SAR_CHAN_SINGLE_ENDED | CY_SAR_CHAN_AVG_ENABLE | (uint32_t)CY_SAR_CHAN_SAMPLE_TIME_1)
#define SAR_VREF_MV 3300UL
#define CYBSP_UART_ENABLED 1U
#define CYBSP_UART_HW SCB0
#define CYBSP_UART_IRQ scb_0_interrupt_IRQn
#define srss_0_rtc_0_ENABLED 1U
#define srss_0_rtc_0_10_MONTH_OFFSET (20U)
#define srss_0_rtc_0_MONTH_OFFSET (16U)
#define srss_0_rtc_0_10_DAY_OFFSET (28U)
#define srss_0_rtc_0_DAY_OFFSET (24U)
#define srss_0_rtc_0_1000_YEAR_OFFSET (12U)
#define srss_0_rtc_0_100_YEAR_OFFSET (8U)
#define srss_0_rtc_0_10_YEAR_OFFSET (4U)
#define srss_0_rtc_0_YEAR_OFFSET (0U)
#define TIMER_1MS_ENABLED 1U
#define TIMER_1MS_HW TCPWM0
#define TIMER_1MS_NUM 0UL
#define TIMER_1MS_MASK (1UL << 0)
#define TIMER_WATCHDOG_ENABLED 1U
#define TIMER_WATCHDOG_HW TCPWM0
#define TIMER_WATCHDOG_NUM 1UL
#define TIMER_WATCHDOG_MASK (1UL << 1)
#define TIMER_WATCHDOG_IRQ tcpwm_0_interrupts_1_IRQn

extern void canfd_rx_callback(bool rxFIFOMsg, uint8_t msgBufOrRxFIFONum, cy_stc_canfd_rx_buffer_t* basemsg);
extern cy_stc_canfd_bitrate_t CANFD_nominalBitrateConfig;
extern cy_stc_canfd_bitrate_t CANFD_dataBitrateConfig;
extern cy_stc_canfd_transceiver_delay_compensation_t CANFD_tdcConfig;
extern cy_stc_id_filter_t CANFD_stdIdFilter_0;
extern cy_stc_id_filter_t CANFD_stdIdFilters[];
extern cy_stc_canfd_sid_filter_config_t CANFD_sidFiltersConfig;
extern cy_stc_extid_filter_t CANFD_extIdFilters[];
extern cy_stc_canfd_extid_filter_config_t CANFD_extIdFiltersConfig;
extern cy_stc_canfd_global_filter_config_t CANFD_globalFilterConfig;
extern cy_en_canfd_fifo_config_t CANFD_rxFifo0Config;
extern cy_en_canfd_fifo_config_t CANFD_rxFifo1Config;
extern cy_stc_canfd_config_t CANFD_config;
extern cy_stc_canfd_t0_t CANFD_T0RegisterBuffer_0;
extern cy_stc_canfd_t1_t CANFD_T1RegisterBuffer_0;
extern uint32_t CANFD_dataBuffer_0[];
extern cy_stc_canfd_tx_buffer_t CANFD_txBuffer_0;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t CANFD_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_sysanalog_config_t pass_0_aref_0_config;
extern const cy_stc_sar_config_t SAR_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t SAR_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_scb_uart_config_t CYBSP_UART_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t CYBSP_UART_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_rtc_config_t srss_0_rtc_0_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t srss_0_rtc_0_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_tcpwm_counter_config_t TIMER_1MS_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t TIMER_1MS_obj;
#endif //defined (CY_USING_HAL)
extern const cy_stc_tcpwm_counter_config_t TIMER_WATCHDOG_config;
#if defined (CY_USING_HAL)
    extern const cyhal_resource_inst_t TIMER_WATCHDOG_obj;
#endif //defined (CY_USING_HAL)

void init_cycfg_peripherals(void);
void reserve_cycfg_peripherals(void);

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_PERIPHERALS_H */
