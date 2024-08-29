/*
 * tle9012_target.c - Target specific includes, definitions and functions
 *
 *	Library for TLE9012 Li-Ion battery monitoring and balancing system IC. See main file Lib_TLE9012.c for how to use guide
 *	Created by R. Santeler @ Infineon 2024
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cycfg.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "tle9012_target.h"
#include "tle9012_interpreter.h"

// TARGET SPECIFICATION FOR CY8C6245
#if defined (CY8C6245)

	// Allocate context for UART operation -> Only needed for high level access (Do not use here - will break low level access)
	//cy_stc_scb_uart_context_t uartContext;

	// UART transmit buffer
	uint32_t tle9012_tx_buffer_size;
	uint8_t  tle9012_tx_buffer[6];


	/* UART configuration structure */
	const cy_stc_scb_uart_config_t uartConfig = {
		.uartMode                   = CY_SCB_UART_STANDARD,
		.enableMutliProcessorMode   = false,
		.smartCardRetryOnNack       = false,
		.irdaInvertRx               = false,
		.irdaEnableLowPowerReceiver = false,

		.oversample                 = 12UL,

		.enableMsbFirst             = true,
		.dataWidth                  = 8UL,
		.parity                     = CY_SCB_UART_PARITY_NONE,
		.stopBits                   = CY_SCB_UART_STOP_BITS_1,
		.enableInputFilter          = false,
		.breakWidth                 = 11UL,
		.dropOnFrameError           = false,
		.dropOnParityError          = false,

		.receiverAddress            = 0x0UL,
		.receiverAddressMask        = 0x0UL,
		.acceptAddrInFifo           = false,

		.enableCts                  = false,
		.ctsPolarity                = CY_SCB_UART_ACTIVE_LOW,
		.rtsRxFifoLevel             = 0UL,
		.rtsPolarity                = CY_SCB_UART_ACTIVE_LOW,

		.rxFifoTriggerLevel         = 0UL,  /* Level triggers when at least one element is in FIFO */
		.rxFifoIntEnableMask        = CY_SCB_UART_RX_TRIGGER, //  | CY_SCB_UART_RX_FULL

		.txFifoTriggerLevel         = 0UL,//(CY_SCB_FIFO_SIZE/2 - 1), /* Level triggers when half-fifo is half empty */
		.txFifoIntEnableMask        = 0x0UL
	};

	//****************************************************************************
	// calcCRC3 - Calculates the crc3 sum of 5bit data
	// Returns the checksum
	// NOTE: THIS CODE DOES NOT EXECUTE THE CRC3 AS NEEDED
	// TODO Implement fast working code or use uC peripheral
	//****************************************************************************
	uint8_t calcCRC3(uint16_t data) {
	//	uint8_t i, crc, poly = 0x3;
	//	crc = 0x0; // 0x7
	//	data &= 0x3FF;
	//	for (i = 0; i < 6; i++) {
	//		if (crc & 0x04) {
	//			crc <<= 1;
	//			if (data & 1) crc |= 0x01;
	//			crc ^= poly;
	//		}
	//		else {
	//			crc <<= 1;
	//			if (data & 1) crc |= 0x01;
	//		}
	//		data >>= 1;
	//	}
	//	return crc & 0x7;
		return 0;
	}

	void UART_Isr(void)
	{
		// Only needed for high level access (Do not use here - will break low level access)
		//Cy_SCB_UART_Interrupt(SCB3, &uartContext);

		// Verbose debug message - When used will break many other things because ISR takes to long - only use for debug when really needed
		//DEBUGPRINTF("\r\n\tUART ISR: Bytes in FIFO %ld, FIFO status %ld, Receive status %ld\r\n", Cy_SCB_UART_GetNumInRxFifo(SCB3), Cy_SCB_UART_GetRxFifoStatus(SCB3), Cy_SCB_UART_GetReceiveStatus(SCB3, &uartContext));

		// Receive every byte in buffer and add it to the ring buffer
		for(uint16_t i = 0; (Cy_SCB_UART_GetNumInRxFifo(SCB3) > 0); i++){
			uint8_t byte = Cy_SCB_UART_Get(SCB3);

			// If add to buffer failed, mark it in global rx_buffer_state flag
			if(addToReceiveRingBuffer_tle9012(byte) != InterpreterSuccess)
				rx_buffer_state = InterpreterFail;
		}

		// Clear interrupt (without this the interrupt will be triggered immediately forever)
		Cy_SCB_UART_ClearRxFifoStatus(SCB3, CY_SCB_UART_RX_TRIGGER);
	}


	//--------------------------------------------------------------------------------------------------
	// uartInit
	//
	// initialize the UART device. Must be done at start of main.
	// Returns 0 if successful
	//--------------------------------------------------------------------------------------------------
	cy_rslt_t uartInit()
	{
		cy_rslt_t result = CY_RSLT_SUCCESS;

		// Debug - following parameters are constructed from many parts. To see the actual error code they are displayed at runtime when needed
		//DEBUGPRINTF("CY_SCB_UART_BAD_PARAM: %d\r\n", CY_SCB_UART_BAD_PARAM);
		//DEBUGPRINTF("CY_SCB_UART_RECEIVE_BUSY: %d\r\n", CY_SCB_UART_RECEIVE_BUSY);
		//DEBUGPRINTF("CY_SCB_UART_TRANSMIT_BUSY: %d\r\n", CY_SCB_UART_TRANSMIT_BUSY);

		/* Configure UART to operate */
		cy_en_scb_uart_status_t res_init = Cy_SCB_UART_Init(SCB3, &uartConfig, NULL); //&uartContext
		if (res_init != CY_SCB_UART_SUCCESS) {DEBUGPRINTF("\tFailed UART SCB TLE9012: %d\r\n", (int)(uint8_t)res_init); }


		/* Assign pins for UART on SCB3: P6[0], P6[1] */
		#define UART_PORT       P6_0_PORT //TLE9012_UART_RX_PORT
		#define UART_RX_NUM     P6_0_NUM  //TLE9012_UART_RX_NUM
		#define UART_TX_NUM     P6_1_NUM  //TLE9012_UART_TX_NUM
		/* Connect SCB3 UART function to pins */
		Cy_GPIO_SetHSIOM(UART_PORT, UART_RX_NUM, P6_0_SCB3_UART_RX);
		Cy_GPIO_SetHSIOM(UART_PORT, UART_TX_NUM, P6_1_SCB3_UART_TX);
		/* Configure pins for UART operation */
		Cy_GPIO_SetDrivemode(UART_PORT, UART_RX_NUM, CY_GPIO_DM_HIGHZ);
		Cy_GPIO_SetDrivemode(UART_PORT, UART_TX_NUM, CY_GPIO_DM_STRONG_IN_OFF);



		/* Assign divider type and number for UART */
		#define UART_CLK_DIV_TYPE     (CY_SYSCLK_DIV_24_5_BIT)
		#define UART_CLK_DIV_NUMBER   (0U)
		/* Connect assigned divider to be a clock source for UART */
		cy_en_sysclk_status_t res_clk = Cy_SysClk_PeriphAssignDivider(PCLK_SCB3_CLOCK, UART_CLK_DIV_TYPE, UART_CLK_DIV_NUMBER);
		if (res_clk != CY_SYSCLK_SUCCESS) { DEBUGPRINTF("\tFailed UART SCB TLE9012 ASSIGN CLOCK: %d\r\n", (int)(uint8_t)res_clk); }



		/* UART desired baud rate is 115200 bps (Standard mode).
		* The UART baud rate = (clk_scb / Oversample).
		* For clk_peri = 50 MHz, select divider value 36 and get SCB clock = (50 MHz / 36) = 1,389 MHz.
		* Select Oversample = 12. These setting results UART data rate = 1,389 MHz / 12 = 115750 bps.
		*/
		//Cy_SysClk_PeriphDisableDivider(UART_CLK_DIV_TYPE, 0U);
		res_clk = Cy_SysClk_PeriphSetFracDivider(UART_CLK_DIV_TYPE, UART_CLK_DIV_NUMBER, 5U, 8U);   //Cy_SysClk_PeriphSetDivider   (UART_CLK_DIV_TYPE, UART_CLK_DIV_NUMBER, 35UL);
		if (res_clk != CY_SYSCLK_SUCCESS) { DEBUGPRINTF("\tFailed UART SCB TLE9012 ASSIGN CLOCK SetFracDivider: %d\r\n", (int)(uint8_t)res_clk); }
		res_clk = Cy_SysClk_PeriphEnableDivider(UART_CLK_DIV_TYPE, UART_CLK_DIV_NUMBER);
		if (res_clk != CY_SYSCLK_SUCCESS) { DEBUGPRINTF("\tFailed UART SCB TLE9012 CLOCK EnableDivider: %d\r\n", (int)(uint8_t)res_clk); }

		/* Assign UART interrupt number and priority */
		#define UART_INTR_NUM        ((IRQn_Type) scb_3_interrupt_IRQn)
		#define UART_INTR_PRIORITY   (7U)
		/* Populate configuration structure (code specific for CM4) */
		cy_stc_sysint_t uartIntrConfig =
		{
			.intrSrc      = UART_INTR_NUM,
			.intrPriority = UART_INTR_PRIORITY,
		};
		/* Hook interrupt service routine and enable interrupt */
		//Cy_SCB_UART_RegisterCallback(SCB3, CY_SCB_UART_TX_TRIGGER , &uartContext);

		(void) Cy_SysInt_Init(&uartIntrConfig, &UART_Isr);
		NVIC_EnableIRQ(UART_INTR_NUM);

		/* Enable UART to operate */
		Cy_SCB_UART_Enable(SCB3);

		//DEBUGPRINTF("\tFifo size: %d\r\n", (int)uartContext.rxBufSize);

		return result;
	}

#endif
