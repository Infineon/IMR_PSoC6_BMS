/*
 * tle9012_target.h - Target specific includes, definitions and functions
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

#ifndef TLE9012_TARGET_H_
#define TLE9012_TARGET_H_

// TARGET SPECIFICATION
#define CY8C6245
//#define TEST

/* While the following lines make things a lot easier like automatically compiling the code for the target you are compiling for, */
/* a few things are expected to be taken care of beforehand. */
/* - setting up the Pins and their functions */
/* - setting up Peripherals - e.g. UART*/
/* - setting up the Interrupt routine in main code */
/* - setting up target specific components (see target definition below) */

uint8_t calcCRC3(uint16_t data);

#pragma once

	#if defined (CY8C6245)
		// Target definition for PSoC 6 CY8C6245 on IMR BMS set in Device Configurator on Modus Toolbox
		// 		APP				Name					Comment
		//		UART-3.0		SCB3 					enableMsbFirst=true,

		// MTB includes
		#include <stdint.h>
		#include "cycfg.h"
		#include "cy_pdl.h"
		#include "cyhal.h"
		#include "cybsp.h"
		#include "cy_retarget_io.h"
		#include "BMS_TLE9012.h"

		// Settings
		#define UART_COMMAND_DELAY 5 // in ms
		#define TIMEOUT_COMMAND 500 //ms

		// UART transmit buffer
		extern uint32_t tle9012_tx_buffer_size;
		extern uint8_t  tle9012_tx_buffer[6];

		// A counter incrementing each millisecond must be implemented externally!
		#define TIMER_COUNTER_1MS Cy_TCPWM_Counter_GetCounter(TIMER_1MS_HW, TIMER_1MS_NUM)

		// A write debug function must be implemented externally!
		#define DEBUGPRINTF(p_frm, ...) printf(p_frm, ##__VA_ARGS__) // redefinition needed to allow for multiple targets

		// A delay function must be implemented externally!
		#define DELAY_MS(ms) cyhal_system_delay_ms(ms) // redefinition needed to allow for multiple targets

		// Init UART with PDL
		//#define TLE_UART_USE_PDL
		// Allocate context for UART operation
		//extern cy_stc_scb_uart_context_t uartContext;



		//****************************************************************************
		// uartInit
		//
		// Must be called at beginning of main (through BMS_TLE9012.h)
		//****************************************************************************
		cy_rslt_t uartInit();

		//****************************************************************************
		// isEnabled
		//
		// Checks if the tle9012 is currently ready to be written to
		// Return 0 of Disabled, or 1 is Enabled
		//****************************************************************************
		static inline uint8_t isEnabled(void) {
			return 1; // Not implemented
		}

		//****************************************************************************
		// uartWrite
		//
		// writes a given number of bytes to UART
		// The actual data must be written to "tx_buffer" before call of this.
		// The number of bytes must match exactly.
		// In the end all transmitted bytes and the matching received bytes
		// will end up in received during uartRead.
		//****************************************************************************
		static inline void uartWrite(uint32_t size, uint8_t waitTxComplete)
		{
			// Put bytes in transmit buffer
			Cy_SCB_UART_PutArrayBlocking(SCB3, tle9012_tx_buffer,size);
			//Blocking wait for transfer completion
			if(waitTxComplete == 1){
				while (!Cy_SCB_UART_IsTxComplete(SCB3)){}
			}
		}

		//****************************************************************************
		// crc8 - Platform specific CRC 8 calculation
		// TODO get faster code and document reference
		//****************************************************************************
		static inline unsigned char crc8(unsigned char *data ,int length) {
		    unsigned long crc;
		    int i,bit;

		    crc = 0xFF;
		    for ( i=0 ; i<length ; i++ ) {
		        crc ^= data[i];
		        for ( bit=0 ; bit<8 ; bit++ ) {
		            if ( (crc & 0x80)!=0 ) {
		                crc <<= 1;
		                crc ^= 0x1D;
		            }
		            else {
		                crc <<= 1;
		            }
		        }
		    }

		    return (~crc)&0xFF;
		}

	#endif /* CY8C6245 */



	#if defined (TEST)
		// Target definition build test
		#include <stdint.h>
		#include <stdio.h>

		// A write debug function must be implemented externally!
		//extern void debugPrintf(const char *p_frm, ...);
		#define DEBUGPRINTF(p_frm, ...) printf(p_frm, ##__VA_ARGS__) // redefinition needed to allow for multiple targets

		// A delay function must be implemented externally!
		#define DELAY_MS(ms) printf("Delay %dms", ms) // redefinition needed to allow for multiple targets




		static inline void enable_setHIGH(void) {
			printf("enable_setHIGH");
		}

		static inline void enable_setLOW(void) {
			printf("enable_setLOW");
		}

		static inline uint8_t isEnabled(void) {
			printf("isEnabled");
			// Always assume device to be enabled if the supply pins of the TLE9012 are not controlable in hardware
			return 1;
		}


		static inline void saveStateEn_setHIGH(void) {
			printf("saveStateEn_setHIGH");
		}

		static inline void saveStateEn_setLOW(void) {
			printf("saveStateEn_setLOW");
		}

		static inline uint8_t getSaveState(void) {
			printf("getSaveState");
			return 0;
		}





		static inline void enable_interruptPin(void) {
			printf("enable_interruptPin");
		}

		static inline void disable_interruptPin(void) {
			printf("disable_interruptPin");
		}


		//****************************************************************************
		// uartRead
		//
		// sends read command for register address
		// gets data of previous UART access
		//****************************************************************************
		static inline void uartRead(uint16_t adr, uint16_t* data)
		{
			printf("uartRead");
		}

		//****************************************************************************
		// uartWrite
		//
		// writes 16 bit data to register address
		//****************************************************************************
		static inline void uartWrite(uint16_t adr, uint16_t data)
		{
			printf("uartWrite");
		}

	#endif /* TEST */
#endif /* TARGET_H_ */
