/*
 * IMR2_CAN_GLOBAL.c - Abstraction off all CAN receive and transmit actions. Valid incoming messages set flags that are handled and reset in main.
 *
 *  Created on: 17.07.2023
 *      Author: m. schmidt and r. santeler
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <stdio.h>

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "ram_public.h"
#include "global_management.h"
#include "memory.h"
#include "IMR2_CAN_GLOBAL.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define CANFD_HW_CHANNEL        0
#define CANFD_BUFFER_INDEX      0

#define CAN_NODE1_ID 7
#define CAN_NODE2_ID 8

#define CANFD_DLC               8

#ifdef COMPONENT_CAT1
#define CANFD_INTERRUPT         canfd_0_interrupts0_0_IRQn
#else
#define CANFD_INTERRUPT         canfd_interrupts0_0_IRQn
#endif

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/* canfd interrupt handler */
static void isr_canfd (void);
void CAN_IRQ_RX_MESSAGE_PARSE(uint8_t id, uint8_t* data, uint8_t size);

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* This is a shared context structure, unique for each canfd channel */
static cy_stc_canfd_context_t canfd_context;

/* Array to store the data bytes of the CANFD frame */
static uint32_t canfd_dataBuffer[] = {
    [CANFD_DATA_0] = 0x04030201U,
    [CANFD_DATA_1] = 0x08070605U
};

/* CAN ID filter setup object (needed because the original object seems to be locked) */
cy_stc_id_filter_t CANFD_stdIdFilters_new[1] = {
    [0] = {
    	    .sfid2 = BMS_CLASS_MASK,
    	    .sfid1 = BMS_CLASS_MASK,
    	    .sfec = CY_CANFD_SFEC_STORE_RX_FIFO_0,
    	    .sft = CY_CANFD_SFT_RANGE_SFID1_SFID2,
    	  }
};
cy_stc_canfd_sid_filter_config_t CANFD_sidFiltersConfig_new = {
    .numberOfSIDFilters = 1U,
    .sidFilter = CANFD_stdIdFilters_new,
};

// Buffer for last received can message
IMR2_CAN_MESSAGE_STRUCT_t IMR2_CAN_RX = { 0 };

// Common flags and error counter
uint8_t newCanMessageReceived = 0;  // Flag to check if a new CAN message is received
uint16_t canSendErrorCount = 0;     // Count of CAN sending errors
uint16_t canSendSuccessCount = 0;   // Needed because the first CAN message send will always be successful. Only if this counter is higher that 1 there really is communication with the target.

// Counter, how often certain can messages are received
uint8_t receivedCanHandshakeCount = 0;
uint8_t receivedCanSwapCount = 0;
uint8_t receivedCanFinishCount = 0;
uint8_t receivedCanSwitchOnVetoCount = 0;
uint8_t receivedCanSwitchOnVetoSoC = 0;
uint8_t receivedCanSwitchOnVetoActiveState = 0;

// Flags to mark that CAN requests are to be handled. NOTE: IF REQUESTS ARE ADDED ONLY A FLAG MUST BE SET DURING INTERRUPT AND THE RESPONSE MUST BE HANDELED IN MAIN manage_externalCommunicationBMS. ALSO NOTE THAT THERE IS A IF TO CHECK IF ANY FLAG IS SET - IT MUST ALSO BE ADDED THERE
uint8_t isCanRequested_RT_Reset = 0;
uint8_t isCanRequested_RT_GetLine = 0;
uint8_t isCanRequested_RT_GetStoredLineNumber = 0;
uint8_t isCanRequested_RT_GetMaxLineNumber = 0;
uint8_t isCanRequested_RT_SetSamplingTime = 0;
uint8_t isCanRequested_RT_GetSamplingTime = 0;

// Parameters for requested CAN requests
uint16_t CanRequested_LinesStartIndex = 0;
uint16_t CanRequested_LinesNumber = 0;
uint16_t CanRequested_LinesCurrentIndex = 0;
uint16_t CanRequested_LinesCurrentIndexPart = 0;
uint32_t CanRequested_SamplingTime = 0;


/***************************************************************************************************************/
/*********************************** DO NOT CHANGE SETTINGS BELOW THIS LINE ************************************/
/***************************************************************************************************************/

/*******************************************************************************
* Function Name: set_canfd_ID - Change the CAN ID the system to Source_CAN_ID
*******************************************************************************/
void set_canfd_ID(uint32_t Source_CAN_ID){
	CANFD_stdIdFilters_new[0].sfid1 = Source_CAN_ID;
	CANFD_stdIdFilters_new[0].sfid2 = Source_CAN_ID;
	Cy_CANFD_SidFiltersSetup(CANFD_HW, CANFD_HW_CHANNEL, &CANFD_sidFiltersConfig_new, &canfd_context);
}
/*******************************************************************************
* Function Name: set_canfd_ID_based_on_slot - Change the CAN ID the system responds to, based on the slot it is inserted in
*******************************************************************************/
void set_canfd_ID_based_on_slot(){
	if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
		CANFD_T0RegisterBuffer_0.id = BMS_CLASS_MASK+1;
		set_canfd_ID(BMS_CLASS_MASK);
	}
	else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
		CANFD_T0RegisterBuffer_0.id = BMS_CLASS_MASK;
		set_canfd_ID(BMS_CLASS_MASK+1);
	}
	else{ // In charger or none the ID of system 1 is used
		CANFD_T0RegisterBuffer_0.id = BMS_CLASS_MASK;
		set_canfd_ID(BMS_CLASS_MASK+1);
	}
}

/*******************************************************************************
* Function Name: CAN_TX_Request
********************************************************************************
* Summary:
* Send a CAN request with a CAN_Command, Target_Data/Length to the device Target_CAN_ID.
* If the transmit is not successful a reset of the CAN peripheral is triggered and the return result differs from CY_CANFD_SUCCESS.
*
* Parameters:
*  Target_CAN_ID		- Destination CAN ID
*  CAN_Command			- CAN command id - see IMR2_CAN_BMS_COMMANDS
*  Target_Data			- Pointer to the data array to be send
*  Target_Data_Length	- Size of the data array to be send
*
* Return:
*  cy_en_canfd_status_t - Result of Cy_CANFD_UpdateAndTransmitMsgBuffer
*
*******************************************************************************/
cy_en_canfd_status_t CAN_TX_Request(uint32_t Target_CAN_ID, uint16_t CAN_Command, uint8_t* Target_Data, uint8_t Target_Data_Length) {
	cy_en_canfd_status_t status;

	// Set target CAN ID
	CANFD_T0RegisterBuffer_0.id = Target_CAN_ID;

	// Determine Command and ID byte of CAN frame
	((uint8_t*)&(canfd_dataBuffer[0]))[0] = CAN_Command;
	((uint8_t*)&(canfd_dataBuffer[0]))[1] = CANFD_stdIdFilters_new[0].sfid1;

	// Set Data Length Code (DLC) to amount of transmit bytes)
	uint8_t CAN_POS = 2U;
	CANFD_txBuffer_0.t1_f->dlc = Target_Data_Length + CAN_POS;

	// Transfer transmit message bytes into CAN data structure
	for (uint16_t i = 0; i < Target_Data_Length; i++)	{
		if(i <= 1)
			((uint8_t*)&(canfd_dataBuffer[0]))[(i+2)%4] = Target_Data[i];
		else if(i <= 5)
			((uint8_t*)&(canfd_dataBuffer[1]))[(i+2)%4] = Target_Data[i];
		//CAN_NODE_TRANSMIT_LMO_NAME.can_data_byte[i] = Target_Data[i - CAN_POS];
	}

	// Write data to buffer
	CANFD_txBuffer_0.data_area_f = canfd_dataBuffer;
	//printf("Send CAN %X %X %X", CAN_Command, ((uint8_t*)&(canfd_dataBuffer[0]))[0], CANFD_txBuffer_0.data_area_f);

	/* Sending CANFD frame to other node */
	status = Cy_CANFD_UpdateAndTransmitMsgBuffer(CANFD_HW,
											CANFD_HW_CHANNEL,
											&CANFD_txBuffer_0,
											CANFD_BUFFER_INDEX,
											&canfd_context);

	// If an error occurred increment error counter. If successful increment success full counter. This is needed because the first send will always be successful. Only if this counter is higher that 1 there really is communication with the target.
	if(status != CY_CANFD_SUCCESS){
		// Count up CAN error
		canSendErrorCount++;
		// Overflow protection, value must never be 0 if errors persist
		if(canSendErrorCount == 0) canSendErrorCount++;
		// Try to reset the error (if bus off error occurred the CAN will be reinitialized. If canSendSuccessCount is not 0 the red led on controller board will light up to signal this)
		CAN_Reset();
		canSendSuccessCount = 0;
	}
	else{
		canSendSuccessCount++;
		//cyhal_gpio_write(LED_Red, 1);
		cyhal_gpio_write(LED_Green, 1);
	}

    return status;
}


/*******************************************************************************
* Function Name: CAN_Reset
********************************************************************************
* Summary:
* If the CAN peripheral stops working this can be called to reinitialize it (if BUS_OFF error is set) and clear all errors. (see MTB CAN implementation documentation)
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void CAN_Reset(){
	uint32_t CAN_Status = Cy_CANFD_GetInterruptStatus(CANFD_HW, CANFD_HW_CHANNEL);
	//printBinary(CAN_Status, UINT32_MAX, 32);
	//printf(" = %ld\r\n", CAN_Status);

	// Reinitialize CANFD Channel
	// Note: Following errors can be set. If any other error might be possible include it in the cause
	//	CY_CANFD_TIMESTAMP_WRAPAROUND
	//	CY_CANFD_MRAM_ACCESS_FAILURE
	//	CY_CANFD_TIMEOUT_OCCURRED
	//	CY_CANFD_BIT_ERROR_CORRECTED
	//	CY_CANFD_BIT_ERROR_UNCORRECTED
	//	CY_CANFD_ERROR_LOG_OVERFLOW
	//	CY_CANFD_ERROR_PASSIVE
	//	CY_CANFD_WARNING_STATUS
	//	CY_CANFD_BUS_OFF_STATUS
	//	CY_CANFD_WATCHDOG_INTERRUPT
	//	CY_CANFD_PROTOCOL_ERROR_ARB_PHASE
	//	CY_CANFD_PROTOCOL_ERROR_DATA_PHASE
	//	CY_CANFD_ACCESS_RESERVED_ADDR
	if((CAN_Status & (CY_CANFD_BUS_OFF_STATUS)) != 0){ //CY_CANFD_ERROR_PASSIVE |
		uint32_t timestamp = TIMER_COUNTER_1MS;

		//cy_en_canfd_status_t status; status =
		Cy_CANFD_Init(CANFD_HW, CANFD_HW_CHANNEL, &CANFD_config,
							   &canfd_context);

		// Set own and default destination CAN ID
		set_canfd_ID_based_on_slot();

		/* Assign the user defined data buffer to CANFD data area */
		CANFD_txBuffer_0.data_area_f = canfd_dataBuffer;

		// If init was successful
		//if (status == CY_CANFD_SUCCESS)	{
			// Toggle green LED on controller board if reinit ok
			//cyhal_gpio_write(LED_Green, 0);
		//}

		// Enable red LED on controller board if reinit fails and there were successful communications between errors (otherwise the led would blink any time open CAN lines are touched)
		if(canSendSuccessCount >= 1)
			cyhal_gpio_write(LED_Green, 0);

		printf("CAN_Reset took %ldms\r\n", TIMER_COUNTER_1MS-timestamp);
	}

	// Clear all status interrupts
	Cy_CANFD_ClearInterrupt(CANFD_HW, CANFD_HW_CHANNEL,(  CY_CANFD_RX_FIFO_0_WATERMARK_REACHED
														| CY_CANFD_RX_FIFO_0_FULL
														| CY_CANFD_RX_FIFO_0_MSG_LOST
														| CY_CANFD_RX_FIFO_1_WATERMARK_REACHED
														| CY_CANFD_RX_FIFO_1_FULL
														| CY_CANFD_RX_FIFO_1_MSG_LOST
														| CY_CANFD_TX_FIFO_1_WATERMARK_REACHED
														| CY_CANFD_TX_FIFO_1_FULL
														| CY_CANFD_TX_FIFO_1_MSG_LOST
														| CY_CANFD_TIMESTAMP_WRAPAROUND
														| CY_CANFD_MRAM_ACCESS_FAILURE
														| CY_CANFD_TIMEOUT_OCCURRED
														| CY_CANFD_BIT_ERROR_CORRECTED
														| CY_CANFD_BIT_ERROR_UNCORRECTED
														| CY_CANFD_ERROR_LOG_OVERFLOW
														| CY_CANFD_ERROR_PASSIVE
														| CY_CANFD_WARNING_STATUS
														| CY_CANFD_BUS_OFF_STATUS
														| CY_CANFD_WATCHDOG_INTERRUPT
														| CY_CANFD_PROTOCOL_ERROR_ARB_PHASE
														| CY_CANFD_PROTOCOL_ERROR_DATA_PHASE
														| CY_CANFD_ACCESS_RESERVED_ADDR));
	//Cy_CANFD_ClearInterrupt(CANFD_HW, CANFD_HW_CHANNEL,(CY_CANFD_TRANSMISSION_COMPLETE  | CY_CANFD_TRANSMISSION_CANCEL_FINISHED | CY_CANFD_TX_FIFO_EMPTY | CY_CANFD_TX_EVENT_FIFO_NEW_ENTRY));
	//Cy_CANFD_ClearInterrupt(CANFD_HW, CANFD_HW_CHANNEL,(CY_CANFD_RX_BUFFER_NEW_MESSAGE | CY_CANFD_RX_FIFO_1_NEW_MESSAGE | CY_CANFD_RX_FIFO_0_NEW_MESSAGE));

}

/*******************************************************************************
* Function Name: CAN_IRQ_RX_MESSAGE_HANDLER
********************************************************************************
* Summary:
* CAN message received callback
*
* Parameters - see MTB CAN implementation documentation
*
* Return:
*  none
*
*******************************************************************************/
void CAN_IRQ_RX_MESSAGE_HANDLER(bool msg_valid, uint8_t msg_buf_fifo_num, cy_stc_canfd_rx_buffer_t* canfd_rx_buf) {
    if (true == msg_valid) {
        /* Checking whether the frame received is a data frame */
        if(CY_CANFD_RTR_DATA_FRAME == canfd_rx_buf->r0_f->rtr) {
			/* Array to hold the data bytes of the CANFD frame */
			uint8_t canfd_data_buffer[CANFD_DLC];
			/* Variable to hold the data length code of the CANFD frame */
			uint32_t canfd_dlc;
			/* Variable to hold the Identifier of the CANFD frame */
			uint32_t canfd_id;

			// Get ID and length
			canfd_dlc = canfd_rx_buf->r1_f->dlc;
			canfd_id  = canfd_rx_buf->r0_f->id;

			// Copy data from CAN FIFO to buffer
			memcpy(canfd_data_buffer, canfd_rx_buf->data_area_f, canfd_dlc);

			// Store command and sender ID
			IMR2_CAN_RX.CAN_COMMAND = canfd_data_buffer[0];
			IMR2_CAN_RX.CAN_SENDER_ID = canfd_data_buffer[1];

			// Transfer received message from CAN data structure into receive buffer
			uint8_t CAN_POS = 2U;
			for (uint8_t i = CAN_POS; i < canfd_dlc; i++)
				IMR2_CAN_RX.CAN_DATA[i] = canfd_data_buffer[CAN_POS];

			// Trigger parsing of the received data for further processing
			CAN_IRQ_RX_MESSAGE_PARSE(canfd_id, canfd_data_buffer, canfd_dlc);
        }
    }
}

/*******************************************************************************
* Function Name: CAN_IRQ_RX_MESSAGE_PARSE
********************************************************************************
* Summary:
* CAN message handler to parse received information and store it so that the main loop can use it. Called from interrupt.
* IF REQUESTS ARE ADDED ONLY A FLAG MUST BE SET DURING INTERRUPT AND THE RESPONSE MUST BE HANDELED IN MAIN manage_externalCommunicationBMS.
* ALSO NOTE THAT THERE IS A IF TO CHECK IF ANY FLAG IS SET - IT MUST ALSO BE ADDED THERE
*
* Parameters:
*  id   - CAN ID of the sender (unused)
*  data - Data bytes of the received package
*  size - Number of bytes in the received package
*
* Return:
*  none
*
*******************************************************************************/
void CAN_IRQ_RX_MESSAGE_PARSE(uint8_t id, uint8_t* data, uint8_t size){
	// Print received package data - ONLY USE THIS SHORTLY FOR VERIFICATION, DEBUG PRINT INSIDE INTERRUPT MAY CAUSE CATASTROPHIC ERRORS
	//printf("\t%d bytes received from Node-%d with identifier %d: ", (int)size, (int)id, (int)data[1]);
	//for (uint8_t msg_idx = 0U; msg_idx < size ; msg_idx++) {
	//	printf(" %d ", data[msg_idx]);
	//}

	// Count receive of specific message
	if(data[0] == BMS_HS_Handshake){
		receivedCanHandshakeCount++;
		//printf("receivedCanSwapCount %d", receivedCanHandshakeCount);
	}
	else if(data[0] == BMS_HS_Swap){
		receivedCanSwapCount++;
		//printf("receivedCanSwapCount %d", receivedCanSwapCount);
	}
	else if(data[0] == BMS_HS_Finish){
		receivedCanFinishCount++;
		//printf("receivedCanFinishCount %d", receivedCanFinishCount);
	}
	else if(data[0] == BMS_SwitchOn_Veto){
		receivedCanSwitchOnVetoCount++;
		receivedCanSwitchOnVetoSoC = data[2];
		receivedCanSwitchOnVetoActiveState = data[3];
		//printf("BMS_RT_Data_Restart %d", receivedCanFinishCount);
	}
	else if(data[0] == BMS_RT_Data_Restart){
		isCanRequested_RT_Reset = 1;
		//printf("isCanRequested_RT_Reset %d", isCanRequested_RT_Reset);
	}
	else if(data[0] == BMS_RT_Data_GetLines){
		if(isCanRequested_RT_GetLine == 0){
			isCanRequested_RT_GetLine = 1;
			CanRequested_LinesStartIndex = ((uint16_t*)&data[2])[0];
			CanRequested_LinesNumber  = ((uint16_t*)&data[4])[0];
			CanRequested_LinesCurrentIndex = 0;
			CanRequested_LinesCurrentIndexPart = 0;

			// Limit check
			if(CanRequested_LinesStartIndex >= MEM_SIZE_MAX_LINES) CanRequested_LinesStartIndex = MEM_SIZE_MAX_LINES;
			if(CanRequested_LinesNumber >= MEM_SIZE_MAX_LINES) CanRequested_LinesNumber = MEM_SIZE_MAX_LINES;
		}
	}
	else if(data[0] == BMS_RT_Data_GetStoredLineNumber){
		isCanRequested_RT_GetStoredLineNumber = 1;
		CanRequested_LinesStartIndex = data[1];
		//printf("isCanRequested_RT_GetStoredLineNumber %d", isCanRequested_RT_GetStoredLineNumber);
	}
	else if(data[0] == BMS_RT_Data_GetMaxLineNumber){
		isCanRequested_RT_GetMaxLineNumber = 1;
		//printf("isCanRequested_RT_GetMaxLineNumber %d", isCanRequested_RT_GetMaxLineNumber);
	}
	else if(data[0] == BMS_RT_Data_SetSamplingTime){
		isCanRequested_RT_SetSamplingTime = 1;
		CanRequested_SamplingTime = ((uint32_t*)&data[2])[0];
		isCanRequested_RT_GetSamplingTime = 1;
		//printf("isCanRequested_RT_SetSamplingTime %d", isCanRequested_RT_SetSamplingTime);
	}
	else if(data[0] == BMS_RT_Data_GetSamplingTime){
		isCanRequested_RT_GetSamplingTime = 1;
		//printf("isCanRequested_RT_GetSamplingTime %d", isCanRequested_RT_GetSamplingTime);
	}
	//printf("\r\n");

	newCanMessageReceived++;
}


/*******************************************************************************
* Function Name: resetCanMessageCounters
********************************************************************************
* Summary:
* Reset all information used by the main loop.
*******************************************************************************/
void resetCanMessageCounters(){
	newCanMessageReceived = 0;
	canSendErrorCount = 0;
	canSendSuccessCount = 0;
	receivedCanHandshakeCount = 0;
	receivedCanSwapCount = 0;
	receivedCanFinishCount = 0;
	receivedCanSwitchOnVetoCount = 0;
	receivedCanSwitchOnVetoActiveState = 0;
	receivedCanSwitchOnVetoSoC = 0;
}


/*******************************************************************************
* Function Name: init_canfd
********************************************************************************
* Summary:
* Initialize the CANFD channel and interrupt.
*
* Parameters:
*  none
*
* Return:
*  cy_en_canfd_status_t
*
*******************************************************************************/
cy_en_canfd_status_t init_canfd(){
	cy_en_canfd_status_t status;

	/* Populate the configuration structure for CANFD Interrupt */
	cy_stc_sysint_t canfd_irq_cfg =
	{
		/* Source of interrupt signal */
		.intrSrc      = CANFD_INTERRUPT,
		/* Interrupt priority */
		.intrPriority = 1U,
	};

	/* Hook the interrupt service routine and enable the interrupt */
	(void) Cy_SysInt_Init(&canfd_irq_cfg, &isr_canfd);

	NVIC_EnableIRQ(CANFD_INTERRUPT);

	/* Enable global interrupts */
	//__enable_irq();

	/* Initialize CANFD Channel */
	status = Cy_CANFD_Init(CANFD_HW, CANFD_HW_CHANNEL, &CANFD_config,
						   &canfd_context);

	// If init was successful set ID and data buffer
	if (status == CY_CANFD_SUCCESS)
	{
		// Set own and default destination CAN ID
		set_canfd_ID_based_on_slot();

		/* Assign the user defined data buffer to CANFD data area */
		CANFD_txBuffer_0.data_area_f = canfd_dataBuffer;
	}

    // Return status
	//printf("===============================================================\r\n");
    //printf("CANFD Status %ld\r\n", (uint32_t)status);
    //printf("===============================================================\r\n\n");
	return status;
}


/*******************************************************************************
* Function Name: isr_canfd
********************************************************************************
* Summary:
* This is the interrupt handler function for the canfd interrupt.
*******************************************************************************/
static void isr_canfd(void)
{
    /* Just call the IRQ handler with the current channel number and context */
    Cy_CANFD_IrqHandler(CANFD_HW, CANFD_HW_CHANNEL, &canfd_context);
}


// Debug: Code to get a specific CAN error code
//printBinary((uint32_t)CY_CANFD_RX_FIFO_0_WATERMARK_REACHED , UINT32_MAX, 32); printf("CY_CANFD_RX_FIFO_0_WATERMARK_REACHED \r\n");
//printBinary((uint32_t)CY_CANFD_RX_FIFO_0_FULL              , UINT32_MAX, 32); printf("CY_CANFD_RX_FIFO_0_FULL              \r\n");
//printBinary((uint32_t)CY_CANFD_RX_FIFO_0_MSG_LOST          , UINT32_MAX, 32); printf("CY_CANFD_RX_FIFO_0_MSG_LOST          \r\n");
//printBinary((uint32_t)CY_CANFD_RX_FIFO_1_WATERMARK_REACHED , UINT32_MAX, 32); printf("CY_CANFD_RX_FIFO_1_WATERMARK_REACHED \r\n");
//printBinary((uint32_t)CY_CANFD_RX_FIFO_1_FULL              , UINT32_MAX, 32); printf("CY_CANFD_RX_FIFO_1_FULL              \r\n");
//printBinary((uint32_t)CY_CANFD_RX_FIFO_1_MSG_LOST          , UINT32_MAX, 32); printf("CY_CANFD_RX_FIFO_1_MSG_LOST          \r\n");
//printBinary((uint32_t)CY_CANFD_TX_FIFO_1_WATERMARK_REACHED , UINT32_MAX, 32); printf("CY_CANFD_TX_FIFO_1_WATERMARK_REACHED \r\n");
//printBinary((uint32_t)CY_CANFD_TX_FIFO_1_FULL              , UINT32_MAX, 32); printf("CY_CANFD_TX_FIFO_1_FULL              \r\n");
//printBinary((uint32_t)CY_CANFD_TX_FIFO_1_MSG_LOST          , UINT32_MAX, 32); printf("CY_CANFD_TX_FIFO_1_MSG_LOST          \r\n");
//printBinary((uint32_t)CY_CANFD_TIMESTAMP_WRAPAROUND        , UINT32_MAX, 32); printf("CY_CANFD_TIMESTAMP_WRAPAROUND        \r\n");
//printBinary((uint32_t)CY_CANFD_MRAM_ACCESS_FAILURE         , UINT32_MAX, 32); printf("CY_CANFD_MRAM_ACCESS_FAILURE         \r\n");
//printBinary((uint32_t)CY_CANFD_TIMEOUT_OCCURRED            , UINT32_MAX, 32); printf("CY_CANFD_TIMEOUT_OCCURRED            \r\n");
//printBinary((uint32_t)CY_CANFD_BIT_ERROR_CORRECTED         , UINT32_MAX, 32); printf("CY_CANFD_BIT_ERROR_CORRECTED         \r\n");
//printBinary((uint32_t)CY_CANFD_BIT_ERROR_UNCORRECTED       , UINT32_MAX, 32); printf("CY_CANFD_BIT_ERROR_UNCORRECTED       \r\n");
//printBinary((uint32_t)CY_CANFD_ERROR_LOG_OVERFLOW          , UINT32_MAX, 32); printf("CY_CANFD_ERROR_LOG_OVERFLOW          \r\n");
//printBinary((uint32_t)CY_CANFD_ERROR_PASSIVE               , UINT32_MAX, 32); printf("CY_CANFD_ERROR_PASSIVE               \r\n");
//printBinary((uint32_t)CY_CANFD_WARNING_STATUS              , UINT32_MAX, 32); printf("CY_CANFD_WARNING_STATUS              \r\n");
//printBinary((uint32_t)CY_CANFD_BUS_OFF_STATUS              , UINT32_MAX, 32); printf("CY_CANFD_BUS_OFF_STATUS              \r\n");
//printBinary((uint32_t)CY_CANFD_WATCHDOG_INTERRUPT          , UINT32_MAX, 32); printf("CY_CANFD_WATCHDOG_INTERRUPT          \r\n");
//printBinary((uint32_t)CY_CANFD_PROTOCOL_ERROR_ARB_PHASE    , UINT32_MAX, 32); printf("CY_CANFD_PROTOCOL_ERROR_ARB_PHASE    \r\n");
//printBinary((uint32_t)CY_CANFD_PROTOCOL_ERROR_DATA_PHASE   , UINT32_MAX, 32); printf("CY_CANFD_PROTOCOL_ERROR_DATA_PHASE   \r\n");
//printBinary((uint32_t)CY_CANFD_ACCESS_RESERVED_ADDR        , UINT32_MAX, 32); printf("CY_CANFD_ACCESS_RESERVED_ADDR        \r\n");
//printBinary((uint32_t)CY_CANFD_INTERRUPT_LINE_0_EN  , UINT32_MAX, 32); printf("CY_CANFD_INTERRUPT_LINE_0_EN \r\n");
//printBinary((uint32_t)CY_CANFD_INTERRUPT_LINE_1_EN  , UINT32_MAX, 32); printf("CY_CANFD_INTERRUPT_LINE_1_EN \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_LEC_POS          , UINT32_MAX, 32); printf("CY_CANFD_PSR_LEC_POS         \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_LEC_MASK         , UINT32_MAX, 32); printf("CY_CANFD_PSR_LEC_MASK        \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_ACT_POS          , UINT32_MAX, 32); printf("CY_CANFD_PSR_ACT_POS         \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_ACT_MASK         , UINT32_MAX, 32); printf("CY_CANFD_PSR_ACT_MASK        \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_EP               , UINT32_MAX, 32); printf("CY_CANFD_PSR_EP              \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_EW               , UINT32_MAX, 32); printf("CY_CANFD_PSR_EW              \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_BO               , UINT32_MAX, 32); printf("CY_CANFD_PSR_BO              \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_DLEC_POS         , UINT32_MAX, 32); printf("CY_CANFD_PSR_DLEC_POS        \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_DLEC_MASK        , UINT32_MAX, 32); printf("CY_CANFD_PSR_DLEC_MASK       \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_RESI             , UINT32_MAX, 32); printf("CY_CANFD_PSR_RESI            \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_RBRS             , UINT32_MAX, 32); printf("CY_CANFD_PSR_RBRS            \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_RFDF             , UINT32_MAX, 32); printf("CY_CANFD_PSR_RFDF            \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_PXE              , UINT32_MAX, 32); printf("CY_CANFD_PSR_PXE             \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_TDCV_POS         , UINT32_MAX, 32); printf("CY_CANFD_PSR_TDCV_POS        \r\n");
//printBinary((uint32_t)CY_CANFD_PSR_TDCV_MASK        , UINT32_MAX, 32); printf("CY_CANFD_PSR_TDCV_MASK       \r\n");

/* [] END OF FILE */
