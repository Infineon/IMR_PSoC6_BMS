/*
@file    MCP3x6x_Target.c
@brief   target specific functions
@version 0.1
@date    2023-14-11
@author  R. Santeler @ Infineon 2023

@section LICENSE
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef MCP3X6X_TARGET_H_
#define MCP3X6X_TARGET_H_

#define CY8C6245

/* While the following lines make things a lot easier like automatically compiling the code for the target you are compiling for, */
/* a few things are expected to be taken care of beforehand. */
/* - setting up the Pins and their functions */
/* - setting up Current measurement via ADC */
/* - setting up the Interrupt routine in main code */
/* - setting up target specific components (see target definition below) */
/* - setting up the SPI */

#pragma once

	#if defined (CY8C6245)
		// Target definition for PSoC 6 CY8C6245 on IMR2.0 BMS set in Device Configurator on Modus Toolbox
		// 		APP				Name					Comment
		//		SPI-3.0			mSPI					SCB5 master set to 3125kHz and SCLK mode ....TODO
		// 		PIN-3.0 		D_2ED4820_ENABLE		Enable pin APP
		// 		PIN-3.0	 		D_2ED4820_SAVE_STATE_EN	Save state enable pin APP
		// 					 	D_2ED4820_INT			Interrupt pin APP (connected to 2ED INT pin)
		// MTB includes
		//#include "cy_pdl.h"
		#include "cyhal.h"
		#include "cycfg.h"
		#include "cybsp.h"
		#include "cy_retarget_io.h"

		extern cyhal_spi_t mSPI;
		extern cyhal_spi_mode_t mSPI_curMode;

		#define SPI_MCP_ADC_FREQ_HZ (1000000UL)
		#define TARGET_CS_Pin PERI_SPI_CS_MCP3465


		// A write debug function must be implemented externally!
		//extern void debugPrintf(const char *p_frm, ...);
		#define DEBUGPRINTF(p_frm, ...) printf(p_frm, ##__VA_ARGS__) // redefinition needed to allow for multiple targets

		// A delay function must be implemented externally!
		//extern void delay_ms(uint32_t ms);
		#define DELAY_MS(ms) cyhal_system_delay_ms(ms) // redefinition needed to allow for multiple targets
        //vTaskDelay(DELAY_AFTER_STARTUP_SCREEN_MS);

		static inline void spiInit(void) {
			if(mSPI_curMode != CYHAL_SPI_MODE_00_MSB){
				// Reinitialize SPI
				cyhal_spi_clear(&mSPI);
				cyhal_spi_free(&mSPI);
				cy_rslt_t result = cyhal_spi_init(&mSPI,PERI_SPI_MOSI,
										PERI_SPI_MISO,
										PERI_SPI_CLK,
										NC,NULL,8,CYHAL_SPI_MODE_00_MSB,false);
				// Set the SPI baud rate
				if (CY_RSLT_SUCCESS == result) {
					result = cyhal_spi_set_frequency(&mSPI, SPI_MCP_ADC_FREQ_HZ);
					if (CY_RSLT_SUCCESS != result)
						DEBUGPRINTF("Failed to set SPI ADC Frequency %d\r\n", (int)(uint8_t)result);
				}
				else{
					DEBUGPRINTF("Failed to init SPI ADC %d\r\n", (int)(uint8_t)result);
				}
				mSPI_curMode = CYHAL_SPI_MODE_00_MSB;
			}
		}

		static inline void set_CS_SPI_HIGH(void) {
			cyhal_gpio_write(TARGET_CS_Pin, 1);
		}

		static inline void set_CS_SPI_LOW(void) {
			// Reinitialize SPI in needed
			spiInit();

			// Set channel select
			cyhal_gpio_write(TARGET_CS_Pin, 0);
		}

		//****************************************************************************
		// spiTransfer
		//
		// sends read command for register address
		// gets data of previous SPI access
		//****************************************************************************
		static inline uint8_t spiTransfer(uint8_t data)
		{
			uint8_t rx_buf; // receive buffer
			cy_rslt_t result;

			/// Transfer spi message
			result = cyhal_spi_transfer(&mSPI, &data, 1, &rx_buf, 1, 0);

			if(result > 0)
				DEBUGPRINTF("[ERROR]: cyhal_spi_transfer failed");

			return rx_buf;
		}

	#endif /* CY8C6245 */



	#if defined (__GNUC__)

//		#if defined (XMC4500_F100x1024)
//			// Target definition for XMC4500 on BMS_SystemBoard
//			// 		APP				Name					Comment
//			//		SPI_MASTER		SPI_DRIVER				Main SPI_MASTER APP. Config: Full_Duplex, bus speed = 4MHz, TX/RX mode = direct, number of SS lines = 1, word = 8, Frame = 16, leading/trailing delay = 1, enable frame end mode, Mode: MSB first, [low if inactive, TX on rising, RX on falling], FIFO enabled 16bit
//			// 		DIGITAL_IO  	IO_DRIVER_ENABLE		Enable pin APP
//			// 		DIGITAL_IO 		IO_DRIVER_SAVESTATEEN	Save state enable pin APP
//			// 		PIN_INTERRUPT 	PIN_INT_DRIVER			Interrupt pin APP (connected to 2ED INT pin)
//			// Note: Use Dave Apps for IO/SPI and name accordingly!
//			#include <DAVE.h>
//
//
//			// A write debug function must be implemented externally!
//			extern void debugPrintf(const char *p_frm, ...);
//			#define DEBUGPRINTF(p_frm, ...) debugPrintf(p_frm, ##__VA_ARGS__) // redefinition needed to allow for multiple targets
//
//			// A delay function must be implemented externally!
//			extern void delay_ms(uint32_t ms);
//			#define DELAY_MS(ms) delay_ms(ms) // redefinition needed to allow for multiple targets
//
//
//
//			static inline void enable_setHIGH(void) {
//				DIGITAL_IO_SetOutputHigh(&IO_DRIVER_ENABLE);
//			}
//
//			static inline void enable_setLOW(void) {
//				DIGITAL_IO_SetOutputLow(&IO_DRIVER_ENABLE);
//			}
//
//			static inline uint8_t isEnabled(void) {
//				return (uint8_t)DIGITAL_IO_GetInput(&IO_DRIVER_ENABLE);
//			}
//
//
//			static inline void saveStateEn_setHIGH(void) {
//				// Save state is disabled!
//				DIGITAL_IO_SetOutputHigh(&IO_DRIVER_SAVESTATEEN);
//			}
//
//			static inline void saveStateEn_setLOW(void) {
//				// Save state is enabled!
//				DIGITAL_IO_SetOutputLow(&IO_DRIVER_SAVESTATEEN);
//			}
//
//			static inline uint8_t getSaveState(void) {
//				return (uint8_t)DIGITAL_IO_GetInput(&IO_DRIVER_SAVESTATEEN);
//			}
//
//
//
//			static inline void enable_interruptPin(void) {
//				PIN_INTERRUPT_SetEdgeSensitivity(&PIN_INT_DRIVER, PIN_INTERRUPT_EDGE_RISING);
//				// PIN_INTERRUPT_Enable(&PIN_INT);
//			}
//
//			static inline void disable_interruptPin(void) {
//				PIN_INTERRUPT_SetEdgeSensitivity(&PIN_INT_DRIVER, PIN_INTERRUPT_EDGE_NONE);
//				// PIN_INTERRUPT_Disable(&PIN_INT);
//			}
//
//
//			//****************************************************************************
//			// spiRead
//			//
//			// sends read command for register address
//			// gets data of previous SPI access
//			//****************************************************************************
//			static inline void spiRead(uint8_t adr, uint8_t *data)
//			{
//			    uint8_t tx_buf[2]; // transmit buffer
//			    uint8_t rx_buf[2]; // receive buffer
//
//			    *data = 0;
//
//			    // read only if driver is enabled (ENABLE pin high)
//			    if(isEnabled())
//			    {
//			        if(adr >=0 && adr <= 10)
//			        {
//			            tx_buf[0] = adr; // R/W bit is 0 anyway with address <= 10
//			            tx_buf[1] = 0;   // data is "don't care"
//
//			            SPI_MASTER_Transfer(&SPI_DRIVER, tx_buf, rx_buf, 2);
//
//			            *data = rx_buf[1];
//			        }
//			        else
//			        {
//			        	DEBUGPRINTF("[ERROR]: Can't read register - driver is disabled");
//			        }
//			    }
//			}
//
//			//****************************************************************************
//			// spiWrite
//			//
//			// writes 8 bit data to register address
//			//****************************************************************************
//			static inline void spiWrite(uint8_t adr, uint8_t data)
//			{
//			    uint8_t tx_buf[2]; // transmit buffer
//
//			    // write only if driver is enabled (ENABLE pin high)
//			    if(isEnabled())
//			    {
//
//			        if(adr >=0 && adr <= 10)
//			        {
//			            tx_buf[0] = adr;
//			            tx_buf[0] |= (1<<7); // set R/W bit to "1"
//			            tx_buf[1] = data;
//
//			            SPI_MASTER_Transmit(&SPI_DRIVER, tx_buf, 2);
//			        }
//			    }
//			    else
//			    {
//			    	DEBUGPRINTF("\r\n"); // new line
//			        DEBUGPRINTF("[ERROR]: Can't write register - driver is disabled");
//			    }
//			}
//
//
//			//static inline void spi_transmit(uint8_t data) {
//			//	// Transmit a byte and wait till its finished
//			//	SPI_MASTER_Transmit(&SPI_MASTER_0, &data, sizeof(data));
//			//	while (XMC_SPI_CH_GetStatusFlag(SPI_MASTER_0.channel) & XMC_SPI_CH_STATUS_FLAG_MSLS) __NOP();
//			//}
//			//
//			//static inline uint8_t spi_receive(uint8_t data) {
//			//	// Receive a byte and wait till its finished
//			//	ReadData = 42;
//			//	SPI_MASTER_Receive(&SPI_MASTER_0, &ReadData, sizeof(ReadData)); //SPI_ReceiveByte(data);
//			//	while(SPI_MASTER_0.runtime->rx_busy){}
//			//
//			//	// Return byte
//			//	return (uint8_t) ReadData;
//			//}
//
//		#endif /* XMC4500_F100x1024 */

	#endif /* __GNUC__ */

#endif /* TARGET_H_ */
