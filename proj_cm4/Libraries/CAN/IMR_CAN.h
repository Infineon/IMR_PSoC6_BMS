/*
 * IMR_CAN.h - Abstraction off all CAN receive and transmit actions.
 * Valid incoming messages set flags that are handled and reset in main.
 *
 *  Created on: 17.07.2023
 *      Author: m. schmidt and r. santeler
 *  Last change: 2025-01-20, S. Detzel
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product").
 * By including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing so
 * agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef IMR_CAN_H_
#define IMR_CAN_H_

#define CAT(x, y) CAT_(x, y)
#define CAT_(x, y) x ## y

#include "cybsp.h"
#include "cy_utils.h"

#include "IMR_CAN_GLOBAL.h"

/*******************************************************************************
* Definitions
*******************************************************************************/
/* IRQ Event Source Name */
#define CAN_IRQ_RX_MESSAGE_HANDLER	canfd_rx_callback
/* Maximum size of the data to be analyzed */
#define CAN_MAX_DATA_LENGTH			8

/*******************************************************************************
* Variables
*******************************************************************************/
extern IMR_CAN_MESSAGE_STRUCT_t IMR_CAN_RX;

extern uint8_t newCanMessageReceived;
extern uint16_t canSendErrorCount;
extern uint16_t canSendSuccessCount;
extern uint8_t receivedCanHandshakeCount;
extern uint8_t receivedCanSwapCount;
extern uint8_t receivedCanFinishCount;
extern uint8_t receivedCanSwitchOnVetoCount;
extern uint8_t receivedCanSwitchOnVetoSoC;
extern uint8_t receivedCanSwitchOnVetoActiveState;

extern uint8_t isCanRequested_RT_Reset;
extern uint8_t isCanRequested_RT_GetLine;
extern uint8_t isCanRequested_RT_GetStoredLineNumber;
extern uint8_t isCanRequested_RT_GetMaxLineNumber;
extern uint8_t isCanRequested_RT_SetSamplingTime;
extern uint8_t isCanRequested_RT_GetSamplingTime;

extern uint16_t CanRequested_LinesStartIndex;
extern uint16_t CanRequested_LinesNumber;
extern uint16_t CanRequested_LinesCurrentIndex;
extern uint16_t CanRequested_LinesCurrentIndexPart;
extern uint32_t CanRequested_SamplingTime;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void CAN_IRQ_RX_MESSAGE_HANDLER(bool msg_valid, uint8_t msg_buf_fifo_num,
		cy_stc_canfd_rx_buffer_t* canfd_rx_buf);

void set_canfd_filter(uint32_t sfid1, uint32_t sfid2);
void resetCanMessageCounters();
void CAN_Reset();

cy_en_canfd_status_t init_canfd();
cy_en_canfd_status_t refresh_canfd(void);

#endif /* IMR_CAN_H_ */

