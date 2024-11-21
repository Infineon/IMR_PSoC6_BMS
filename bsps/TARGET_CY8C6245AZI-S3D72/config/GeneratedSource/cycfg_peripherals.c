/*******************************************************************************
 * File Name: cycfg_peripherals.c
 *
 * Description:
 * Peripheral Hardware Block configuration
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

#include "cycfg_peripherals.h"

#define CANFD_STD_ID_FILTER_0 \
{\
    .sfid2 = 4U, \
    .sfid1 = 4U, \
    .sfec = CY_CANFD_SFEC_STORE_RX_FIFO_0, \
    .sft = CY_CANFD_SFT_RANGE_SFID1_SFID2, \
 }
#define TIMER_1MS_INPUT_DISABLED 0x7U
#define TIMER_WATCHDOG_INPUT_DISABLED 0x7U

void canfd_rx_callback(bool rxFIFOMsg, uint8_t msgBufOrRxFIFONum, cy_stc_canfd_rx_buffer_t* basemsg);
cy_stc_canfd_bitrate_t CANFD_nominalBitrateConfig =
{
    .prescaler = 5U - 1U,
    .timeSegment1 = 2U - 1U,
    .timeSegment2 = 2U - 1U,
    .syncJumpWidth = 2U - 1U,
};
cy_stc_canfd_bitrate_t CANFD_dataBitrateConfig =
{
    .prescaler = 3U - 1U,
    .timeSegment1 = 5U - 1U,
    .timeSegment2 = 2U - 1U,
    .syncJumpWidth = 2U - 1U,
};
cy_stc_canfd_transceiver_delay_compensation_t CANFD_tdcConfig =
{
    .tdcEnabled = false,
    .tdcOffset = 0U,
    .tdcFilterWindow = 0U,
};
cy_stc_id_filter_t CANFD_stdIdFilter_0 =
{
    .sfid2 = 4U,
    .sfid1 = 4U,
    .sfec = CY_CANFD_SFEC_STORE_RX_FIFO_0,
    .sft = CY_CANFD_SFT_RANGE_SFID1_SFID2,
};
cy_stc_id_filter_t CANFD_stdIdFilters[] =
{
    [0] = CANFD_STD_ID_FILTER_0,
};
cy_stc_canfd_sid_filter_config_t CANFD_sidFiltersConfig =
{
    .numberOfSIDFilters = 1U,
    .sidFilter = CANFD_stdIdFilters,
};
cy_stc_extid_filter_t CANFD_extIdFilters[] =
{
};
cy_stc_canfd_extid_filter_config_t CANFD_extIdFiltersConfig =
{
    .numberOfEXTIDFilters = 0U,
    .extidFilter = (cy_stc_extid_filter_t*)&CANFD_extIdFilters,
    .extIDANDMask = 0UL,
};
cy_stc_canfd_global_filter_config_t CANFD_globalFilterConfig =
{
    .nonMatchingFramesStandard = CY_CANFD_REJECT_NON_MATCHING,
    .nonMatchingFramesExtended = CY_CANFD_REJECT_NON_MATCHING,
    .rejectRemoteFramesStandard = false,
    .rejectRemoteFramesExtended = false,
};
cy_en_canfd_fifo_config_t CANFD_rxFifo0Config =
{
    .mode = CY_CANFD_FIFO_MODE_BLOCKING,
    .watermark = 0U,
    .numberOfFIFOElements = 8U,
    .topPointerLogicEnabled = false,
};
cy_en_canfd_fifo_config_t CANFD_rxFifo1Config =
{
    .mode = CY_CANFD_FIFO_MODE_BLOCKING,
    .watermark = 0U,
    .numberOfFIFOElements = 0U,
    .topPointerLogicEnabled = false,
};
cy_stc_canfd_config_t CANFD_config =
{
    .txCallback = NULL,
    .rxCallback = canfd_rx_callback,
    .errorCallback = NULL,
    .canFDMode = false,
    .bitrate = &CANFD_nominalBitrateConfig,
    .fastBitrate = &CANFD_dataBitrateConfig,
    .tdcConfig = &CANFD_tdcConfig,
    .sidFilterConfig = &CANFD_sidFiltersConfig,
    .extidFilterConfig = &CANFD_extIdFiltersConfig,
    .globalFilterConfig = &CANFD_globalFilterConfig,
    .rxBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
    .rxFIFO1DataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
    .rxFIFO0DataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
    .txBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
    .rxFIFO0Config = &CANFD_rxFifo0Config,
    .rxFIFO1Config = &CANFD_rxFifo1Config,
    .noOfRxBuffers = 1U,
    .noOfTxBuffers = 1U,
    .messageRAMaddress = CY_CAN0MRAM_BASE + 0U,
    .messageRAMsize = 4096U,
};
cy_stc_canfd_t0_t CANFD_T0RegisterBuffer_0 =
{
    .id = 0x7FFU,
    .rtr = CY_CANFD_RTR_DATA_FRAME,
    .xtd = CY_CANFD_XTD_STANDARD_ID,
    .esi = CY_CANFD_ESI_ERROR_ACTIVE,
};
cy_stc_canfd_t1_t CANFD_T1RegisterBuffer_0 =
{
    .dlc = 8U,
    .brs = true,
    .fdf = CY_CANFD_FDF_STANDARD_FRAME,
    .efc = false,
    .mm = 0U,
};
uint32_t CANFD_dataBuffer_0[] =
{
    [CANFD_DATA_0] = 0U,
    [CANFD_DATA_1] = 0x08070605U,
};
cy_stc_canfd_tx_buffer_t CANFD_txBuffer_0 =
{
    .t0_f = &CANFD_T0RegisterBuffer_0,
    .t1_f = &CANFD_T1RegisterBuffer_0,
    .data_area_f = CANFD_dataBuffer_0,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CANFD_obj =
{
    .type = CYHAL_RSC_CAN,
    .block_num = 0U,
    .channel_num = 0U,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_sysanalog_config_t pass_0_aref_0_config =
{
    .startup = CY_SYSANALOG_STARTUP_FAST,
    .iztat = CY_SYSANALOG_IZTAT_SOURCE_LOCAL,
    .vref = CY_SYSANALOG_VREF_SOURCE_LOCAL_1_2V,
    .deepSleep = CY_SYSANALOG_DEEPSLEEP_IPTAT_IZTAT_VREF,
};
const cy_stc_sar_config_t SAR_config =
{
    .ctrl = (uint32_t) SAR_CTL,
    .sampleCtrl = (uint32_t) SAR_SAMPLE,
    .sampleTime01 = (1023UL << (uint32_t)CY_SAR_SAMPLE_TIME0_SHIFT) | (1023UL << (uint32_t)CY_SAR_SAMPLE_TIME1_SHIFT),
    .sampleTime23 = (2UL << (uint32_t)CY_SAR_SAMPLE_TIME2_SHIFT) | (2UL << (uint32_t)CY_SAR_SAMPLE_TIME3_SHIFT),
    .rangeThres = (0UL << (uint32_t)CY_SAR_RANGE_HIGH_SHIFT) | (0UL << (uint32_t)CY_SAR_RANGE_LOW_SHIFT),
    .rangeCond = CY_SAR_RANGE_COND_BELOW,
    .chanEn = 3UL,
    .chanConfig = {(uint32_t) SAR_CH0_CONFIG, (uint32_t) SAR_CH1_CONFIG, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL},
    .intrMask = CY_SAR_INTR_EOS,
    .satIntrMask = 0UL,
    .rangeIntrMask = 0UL,
    .configRouting = false,
    .vrefMvValue = SAR_VREF_MV,
    .clock = CY_SAR_CLK_PERI,
    .fifoCfgPtr = NULL,
    .trTimer = false,
    .scanCnt = false,
    .scanCntIntr = false,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t SAR_obj =
{
    .type = CYHAL_RSC_ADC,
    .block_num = 0,
    .channel_num = 0,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_scb_uart_config_t CYBSP_UART_config =
{
    .uartMode = CY_SCB_UART_STANDARD,
    .enableMutliProcessorMode = false,
    .smartCardRetryOnNack = false,
    .irdaInvertRx = false,
    .irdaEnableLowPowerReceiver = false,
    .oversample = 8,
    .enableMsbFirst = false,
    .dataWidth = 8UL,
    .parity = CY_SCB_UART_PARITY_NONE,
    .stopBits = CY_SCB_UART_STOP_BITS_1,
    .enableInputFilter = false,
    .breakWidth = 11UL,
    .dropOnFrameError = false,
    .dropOnParityError = false,
    .receiverAddress = 0x0UL,
    .receiverAddressMask = 0x0UL,
    .acceptAddrInFifo = false,
    .enableCts = false,
    .ctsPolarity = CY_SCB_UART_ACTIVE_LOW,
    .rtsRxFifoLevel = 0UL,
    .rtsPolarity = CY_SCB_UART_ACTIVE_LOW,
    .rxFifoTriggerLevel = 63UL,
    .rxFifoIntEnableMask = 0UL,
    .txFifoTriggerLevel = 63UL,
    .txFifoIntEnableMask = 0UL,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_UART_obj =
{
    .type = CYHAL_RSC_SCB,
    .block_num = 0U,
    .channel_num = 0U,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_rtc_config_t srss_0_rtc_0_config =
{
    .sec = 0U,
    .min = 0U,
    .hour = 12U,
    .amPm = CY_RTC_AM,
    .hrFormat = CY_RTC_24_HOURS,
    .dayOfWeek = CY_RTC_SATURDAY,
    .date = 1U,
    .month = CY_RTC_JANUARY,
    .year = 0U,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t srss_0_rtc_0_obj =
{
    .type = CYHAL_RSC_RTC,
    .block_num = 0U,
    .channel_num = 0U,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_tcpwm_counter_config_t TIMER_1MS_config =
{
    .period = 4294967295,
    .clockPrescaler = CY_TCPWM_COUNTER_PRESCALER_DIVBY_2,
    .runMode = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection = CY_TCPWM_COUNTER_COUNT_UP,
    .compareOrCapture = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0 = 16384,
    .compare1 = 16384,
    .enableCompareSwap = false,
    .interruptSources = CY_TCPWM_INT_NONE,
    .captureInputMode = TIMER_1MS_INPUT_DISABLED & 0x3U,
    .captureInput = CY_TCPWM_INPUT_0,
    .reloadInputMode = TIMER_1MS_INPUT_DISABLED & 0x3U,
    .reloadInput = CY_TCPWM_INPUT_0,
    .startInputMode = TIMER_1MS_INPUT_DISABLED & 0x3U,
    .startInput = CY_TCPWM_INPUT_0,
    .stopInputMode = TIMER_1MS_INPUT_DISABLED & 0x3U,
    .stopInput = CY_TCPWM_INPUT_0,
    .countInputMode = TIMER_1MS_INPUT_DISABLED & 0x3U,
    .countInput = CY_TCPWM_INPUT_1,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t TIMER_1MS_obj =
{
    .type = CYHAL_RSC_TCPWM,
    .block_num = 0U,
    .channel_num = 0U,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_tcpwm_counter_config_t TIMER_WATCHDOG_config =
{
    .period = 500,
    .clockPrescaler = CY_TCPWM_COUNTER_PRESCALER_DIVBY_2,
    .runMode = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection = CY_TCPWM_COUNTER_COUNT_UP,
    .compareOrCapture = CY_TCPWM_COUNTER_MODE_COMPARE,
    .compare0 = 500,
    .compare1 = 16384,
    .enableCompareSwap = false,
    .interruptSources = CY_TCPWM_INT_ON_TC,
    .captureInputMode = TIMER_WATCHDOG_INPUT_DISABLED & 0x3U,
    .captureInput = CY_TCPWM_INPUT_0,
    .reloadInputMode = TIMER_WATCHDOG_INPUT_DISABLED & 0x3U,
    .reloadInput = CY_TCPWM_INPUT_0,
    .startInputMode = TIMER_WATCHDOG_INPUT_DISABLED & 0x3U,
    .startInput = CY_TCPWM_INPUT_0,
    .stopInputMode = TIMER_WATCHDOG_INPUT_DISABLED & 0x3U,
    .stopInput = CY_TCPWM_INPUT_0,
    .countInputMode = TIMER_WATCHDOG_INPUT_DISABLED & 0x3U,
    .countInput = CY_TCPWM_INPUT_1,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t TIMER_WATCHDOG_obj =
{
    .type = CYHAL_RSC_TCPWM,
    .block_num = 0U,
    .channel_num = 1U,
};
#endif /* defined (CY_USING_HAL) */

__WEAK void canfd_rx_callback(bool rxFIFOMsg, uint8_t msgBufOrRxFIFONum, cy_stc_canfd_rx_buffer_t* basemsg)
{
    (void)rxFIFOMsg;
    (void)msgBufOrRxFIFONum;
    (void)basemsg;
}
void init_cycfg_peripherals(void)
{
    Cy_SysClk_PeriphAssignDivider(PCLK_CANFD0_CLOCK_CAN0, CY_SYSCLK_DIV_8_BIT, 1U);
    SAR_MUX_SWITCH0(SAR_HW) |= CY_SAR_MUX_FW_VSSA_VMINUS;
    SAR_MUX_SWITCH_SQ_CTRL(SAR_HW) |= CY_SAR_MUX_SQ_CTRL_VSSA;
    Cy_SysClk_PeriphAssignDivider(PCLK_PASS_CLOCK_SAR, CY_SYSCLK_DIV_16_BIT, 2U);
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB0_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS0, CY_SYSCLK_DIV_16_BIT, 4U);
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS1, CY_SYSCLK_DIV_16_BIT, 4U);
}
void reserve_cycfg_peripherals(void)
{
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&CANFD_obj);
    cyhal_hwmgr_reserve(&SAR_obj);
    cyhal_hwmgr_reserve(&CYBSP_UART_obj);
    cyhal_hwmgr_reserve(&srss_0_rtc_0_obj);
    cyhal_hwmgr_reserve(&TIMER_1MS_obj);
    cyhal_hwmgr_reserve(&TIMER_WATCHDOG_obj);
#endif /* defined (CY_USING_HAL) */
}
