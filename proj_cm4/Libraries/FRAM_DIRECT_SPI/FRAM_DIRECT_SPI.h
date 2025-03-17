/***************************************************************************//**
* \file FRAM_DIRECT_SPI.c
* \version 1.0
*
* \brief
* Objective:
*    Library for direct HAL SPI access to Infineon/Cypress F-RAM memory
*    without SMIF(QSPI) peripheral.
*    Based on internal validation code from MPL team,
*    packaged and adapted by PSS R. Santeler
*
* Usage:
* 	Select memory size in .h file,
* 	implement target specific function in .c and .h file,
* 	run FRAM_init() at startup. To test the memory and configuration,
* 	activate ENABLE_TERMINAL_FRAM 1 and use FRAM_testTerminal()
* 	to run a terminal. Also check the code of this function for further
* 	information on how to use this library.
* 	Use functions ending with '_4Kb' if the chosen memory IC
* 	has a 1Byte address range.
* 	For all other memory IC (2Byte address range) use normal functions
* 	Use FRAM_Write/Read	to access memory directly -
* 						this is limited by SPI packet size - see datasheet
* 	Use FRAM_Burst_Write/Read 	to access memory page-by-page -
* 								no size limit, but slower
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


#include <stdint.h>


/***************************************************************************
* GLOBAL CONSTANTS AND SETTINGS
***************************************************************************/
// Memory size
#define MEM_SIZE			(32768u)  	/* 256Kb- The memory Read/Write packet */
//#define MEM_SIZE			(8192u)   	/* 64Kb - The memory Read/Write packet */
//#define MEM_SIZE			(2048u)   	/* 16Kb - The memory Read/Write packet */
//#define MEM_SIZE			(512u)    	/* 4Kb  - The memory Read/Write packet */
//#define MEM_SIZE			(128u)    	/* 1Kb  - The memory Read/Write packet */

// External GPIO commands
#define FRAM_SET_HOLD_LOW       Cy_GPIO_Write(HOLDn_PORT, HOLDn_PIN, 0u)
#define FRAM_SET_HOLD_HIGH      Cy_GPIO_Write(HOLDn_PORT, HOLDn_PIN, 1u)
#define FRAM_SET_WP_LOW         Cy_GPIO_Write(WPn_PORT, WPn_PIN, 0u)
#define FRAM_SET_WP_HIGH        Cy_GPIO_Write(WPn_PORT, WPn_PIN, 1u)

// Settings
#define ENABLE_DEBUG_FRAM    (0u)		// 1 for enabling debug print
#define ENABLE_TERMINAL_FRAM (1u)		// 1 to enable terminal functionality
#define UART_PACKET_SIZE     (256u)    	/* The memory Read/Write packet */
#define N_BYTE_ADDRESS       (2u)
#define SPI_4KB              (MEM_SIZE==512u)
#define STATE_SUCCESS        (0u)
#define STATE_ERROR          (1u)
#define N_DELAY_CYCLE      	 (10u)
#define MEMORY_SPI_SPEED 	 1000000UL

// Opcodes
#define WRSR                (0x01)
#define WRITE               (0x02)
#define READ                (0x03)
#define WRDI                (0x04)
#define RDSR                (0x05)
#define WREN                (0x06)
#define READ1               (0x0B)
#define WRITE1              (0x0A)
#define RDID	            (0b10011111)


/***************************************************************************
* GLOBAL VARIABLES
***************************************************************************/
extern uint8_t spiReinitSuccessful; // 0 = STATE_SUCCESS, 1 = STATE_ERROR

extern uint8_t bufferTx [MEM_SIZE+3u];
extern uint8_t bufferRx [MEM_SIZE+3u];
extern uint8_t bufferTx_ID [16];
extern uint8_t bufferRx_ID [16];
extern uint8_t strPtr[2*UART_PACKET_SIZE];
extern uint8_t Data[UART_PACKET_SIZE];
extern uint32_t cmdLen;
extern uint32_t failResult;


/***************************************************************************
* FUNCTION PROTOTYPES
***************************************************************************/
void FRAM_init(void);

void FRAM_WREN();

// Functions to directly write to FRAM
// (size limited by SPI packet size - see datasheet)
void 	 FRAM_Write(uint8_t* wData, uint32_t startAddress, uint32_t datasize);
void 	 FRAM_Write_4Kb(uint8_t OPCODE, uint8_t* wData, uint32_t startAddress,
		uint32_t datasize);
uint32_t FRAM_Read(uint8_t* rData, uint32_t startAddress, uint32_t datasize);
uint32_t FRAM_Read_4Kb(uint8_t OPCODE, uint8_t* rData, uint32_t startAddress,
		uint32_t datasize);
// Functions to write to FRAM in pages (no size limit, but slower)
void 	 FRAM_Burst_Write     (uint8_t *userData, uint32_t startAddress,
		uint32_t burstLen);
void 	 FRAM_Burst_Write_4Kb (uint8_t *userData, uint32_t startAddress,
		uint32_t burstLen);
void 	 FRAM_Burst_Read      (uint8_t *readData, uint32_t startAddress,
		uint32_t burstLen);
void 	 FRAM_Burst_Read_4Kb  (uint8_t *readData, uint32_t startAddress,
		uint32_t burstLen);

#if(ENABLE_TERMINAL_FRAM)
int  FRAM_testTerminal(void);
#endif
