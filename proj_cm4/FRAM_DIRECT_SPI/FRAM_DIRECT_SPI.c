/***************************************************************************//**
* \file FRAM_DIRECT_SPI.c
* \version 1.0
*
* \brief
* Objective:
*    Library for direct HAL SPI access to Infineon/Cypress F-RAM memory without SMIF(QSPI) peripheral.
*    Based on internal validation code from MPL team, packaged and adapted by PSS R. Santeler
*
* Usage:
* 	Select memory size in .h file, implement target specific function in .c and .h file, run
* 	FRAM_init() at startup. To test the memory and configuration, activate ENABLE_TERMINAL_FRAM 1 and
* 	use FRAM_testTerminal() to run a terminal. Also check the code of this function for further
* 	information on how to use this library.
* 	Use functions ending with '_4Kb' if the chosen memory IC has a 1Byte address range. For all other memory IC (2Byte address range) use normal functions
* 	Use FRAM_Write/Read       to access memory directly - this is limited by SPI packet size - see datasheet
* 	Use FRAM_Burst_Write/Read to access memory page-by-page - no size limit, but slower
*
* \copyright
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/


#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "FRAM_DIRECT_SPI.h"

// Target specific includes
#include "cy_syslib.h"
#include "cycfg.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_peripherals.h"
#include "cy_scb_spi.h"



/**********************************************************************************************
* GLOBAL CONSTANTS
**********************************************************************************************/
#define PRINTF_MEMORY_DEBUG(p_frm, ...)		printf(p_frm, ##__VA_ARGS__) // Comment out right side (... //printf(...)) to deactivate debug messages

/**********************************************************************************************
* GLOBAL VARIABLES
**********************************************************************************************/
uint8_t spiReinitSuccessful = STATE_SUCCESS;  // 0 = STATE_SUCCESS, 1 = STATE_ERROR

//uint32_t  port12_p0_MOSI_DM;
//uint32_t  port12_p1_MISO_DM;
//uint32_t  port12_p2_SCK_DM;
//uint32_t  port12_p3_SS_DM;
//uint32_t  port12_p0_MOSI_HSIO;
//uint32_t  port12_p1_MISO_HSIO;
//uint32_t  port12_p2_SCK_HSIO;
//uint32_t  port12_p3_SS_HSIO;

// The SPI object and the marker which mode is currently set must be defined in main and is referenced here
extern cyhal_spi_t mSPI;				// SPI used for safety switch 2ED4820, ADC MCP3465 and Memory CY15B256Q-SXA. Use "extern" if needed in other files
extern cyhal_spi_mode_t mSPI_curMode;	// Marks currently set SPI mode. Needed because SPI devices require different SPI modes and must reinitialize SPI if mode is not correct for them. Use "extern" if needed in other files



/**********************************************************************************************
* ENVIRONMENT SPECIFIC DEFINITIONS AND FUNCTIONS
**********************************************************************************************/
// Internal GPIO commands
#define WP_READ					Cy_GPIO_Read(WPn_PORT, WPn_PIN)
#define HOLD_READ				Cy_GPIO_Read(HOLDn_PORT, HOLDn_PIN)

#define LED_RED_TOGGLE_STATE 	Cy_GPIO_Inv(LED_Red_PORT, LED_Red_PIN);

// SPI commands
#define SPI_CLEAR_FIFO 			cyhal_spi_clear(&mSPI)

// General
#define DELAY_MS(ms) 			CyDelay(ms)
#define DELAY_US(us) 			CyDelayUs(us)
#define DELAY_CYCLES(num)		CyDelayCycles(num)
#define TIMER_COUNTER_1MS 		Cy_TCPWM_Counter_GetCounter(TIMER_1MS_HW, TIMER_1MS_NUM) // 32bit value that counts up every millisecond to measure execution time in terminal

#define UART_GET_CHAR_AND_CHECK	(cyhal_uart_getc(&cy_retarget_io_uart_obj, &receivedCharacter, 1) == CY_RSLT_SUCCESS) //(0UL != Cy_SCB_UART_GetNumInRxFifo(UART_HW))
#define UART_CLEAR 				cyhal_uart_clear(&cy_retarget_io_uart_obj); //Cy_SCB_UART_ClearRxFifo(UART_HW);

/**********************************************************************************************
* FUNCTION PROTOTYPES
**********************************************************************************************/
void PrintArray(char* msg, uint8_t* buff, uint32_t size);
void Zero2FF(uint8_t *wrArray, uint32_t arraySize);
void RandDataPattern(uint8_t*, uint32_t);
void UserDataPattern(uint8_t *wrArray, uint32_t arraySize, uint8_t dataByte, uint8_t dataBytebar);
uint32_t FRAM_Burst_Read_And_Compare(uint8_t* expectedData, uint32_t startAddress, uint32_t burstLen, uint8_t* bufferRx);
void WP_HOLD(uint8_t status);
void WP_HOLD_Check (void);

/*******************************************************************************
* Function Name: spi_cs_enable - Enables the chip select for the Memory
*******************************************************************************/
static inline void spi_cs_enable(){
	cy_rslt_t result;
	spiReinitSuccessful = STATE_ERROR;

	// Reinitialize SPI in correct mode if it is not set correct
	if(mSPI_curMode != CYHAL_SPI_MODE_00_MSB){
		// Reinitialize SPI
		cyhal_spi_clear(&mSPI);
		cyhal_spi_free(&mSPI);
		result = cyhal_spi_init(&mSPI,PERI_SPI_MOSI,
								PERI_SPI_MISO,
								PERI_SPI_CLK,
								NC,NULL,8,CYHAL_SPI_MODE_00_MSB,0);

		// Set the SPI baud rate
		if (CY_RSLT_SUCCESS == result) {
			result = cyhal_spi_set_frequency(&mSPI, MEMORY_SPI_SPEED);
			if (CY_RSLT_SUCCESS == result){
				spiReinitSuccessful = STATE_SUCCESS;
				//PRINTF_MEMORY_DEBUG("\tReinit SPI for MEM %d\r\n", (int)(uint8_t)result);
			}
			else{
				PRINTF_MEMORY_DEBUG("\tFailed to set SPI MEM Frequency %d\r\n", (uint8_t)result);
			}
		}
		else{
			PRINTF_MEMORY_DEBUG("\tFailed to init SPI MEM %d\r\n", (uint8_t)result);
		}
		mSPI_curMode = CYHAL_SPI_MODE_00_MSB;
	}

	// Set chip select
	cyhal_gpio_write(PERI_SPI_CS_CY15B, 0);
}
/*******************************************************************************
* Function Name: spi_cs_disable - Disables the chip select for the Memory
*******************************************************************************/
static inline void spi_cs_disable(){
	// Clear chip select
	cyhal_gpio_write(PERI_SPI_CS_CY15B, 1);
}

static inline void FRAM_Write_array_blocking(void *buffer, uint16_t size){
	/// PSoC 6 - HDL implementation
	cyhal_spi_transfer(&mSPI, buffer, size, NULL, 0, 0);
	//for(uint16_t i = 0; i < size; i++){
	//	cyhal_spi_send(&mSPI, ((uint8_t*)buffer)[i]);
	//}
	//if(CY_RSLT_SUCCESS == result)
	//	return size_read;
	//else
	//	return STATE_ERROR;

	/// PSoC 6 - PDL implementation
	//Cy_SCB_FRAM_WriteArrayBlocking(SPI_FRAM_MASTER_HW, buffer, size);
	//while (Cy_SCB_SPI_IsBusBusy(SPI_FRAM_MASTER_HW));
}

static inline uint16_t FRAM_Read_array_blocking(void *buffer, uint16_t size){
	/// PSoC 6 - HDL implementation
	cy_rslt_t result = 1;
	uint16_t size_read = (uint16_t)size;
	result = cyhal_spi_transfer(&mSPI, NULL, 0, buffer, size, 0);
	//for(uint16_t i = 0; i < size; i++){
	//	result = cyhal_spi_recv(&mSPI, (uint32_t *)&(((uint8_t*)buffer)[i]));
	//}
	if(CY_RSLT_SUCCESS == result)
		return size_read;
	else
		return STATE_ERROR;

	/// PSoC 6 - PDL implementation
	//uint32_t success;
	//success = Cy_SCB_FRAM_ReadArray(SPI_FRAM_MASTER_HW, buffer, size);
	//while (Cy_SCB_SPI_IsBusBusy(SPI_FRAM_MASTER_HW));
	//return success;
}

/**********************************************************************************************
* FUNCTIONS
**********************************************************************************************/

/*******************************************************************************
*
* FRAM Special Functions
*
****************************************************************************/

void FRAM_WREN(void) {
	uint8_t transmitBuf = WREN;

	SPI_CLEAR_FIFO;

	spi_cs_enable();
	FRAM_Write_array_blocking(&transmitBuf, 0x01);
	spi_cs_disable();

	if (ENABLE_DEBUG_FRAM)
		PRINTF_MEMORY_DEBUG("\r\nExecuted WREN Loop");
}

void SPIFRAM_WRDI(void) {
	uint8_t transmitBuf[1];

	transmitBuf[0] = WRDI;

	SPI_CLEAR_FIFO;

	spi_cs_enable();
	FRAM_Write_array_blocking(transmitBuf, 0x01);
	spi_cs_disable();

	if (ENABLE_DEBUG_FRAM)
		PRINTF_MEMORY_DEBUG("\r\nExecuted WRDI Loop");
}

void SPIFRAM_WRSR(uint8_t wData) {
	uint8_t transmitBuf[2];
	transmitBuf[0] = WRSR;
	transmitBuf[1] = wData;

	SPI_CLEAR_FIFO;

	spi_cs_enable();
	FRAM_Write_array_blocking(&transmitBuf[0], 0x02);
	spi_cs_disable();

	if (ENABLE_DEBUG_FRAM)
		PRINTF_MEMORY_DEBUG("\r\nExecuted WRSR Loop %02X %02X ", transmitBuf[0], transmitBuf[1]);
}

uint32_t SPIFRAM_RDSR(uint8_t* rdSR) {
	uint32_t success;
	uint8_t debugprint[4];
	uint8_t transmitBuf[1];
	uint8_t receiveBuf[2];

	transmitBuf[0] = RDSR;

	SPI_CLEAR_FIFO;
	//Cy_SCB_SPI_ClearRxFifo(SPI_FRAM_MASTER_HW);


	spi_cs_enable();
	FRAM_Write_array_blocking(transmitBuf, 0x01);  //0x02
	success = FRAM_Read_array_blocking(receiveBuf, 0x01); //0x02
	spi_cs_disable();

	if (ENABLE_DEBUG_FRAM) {
		//PRINTF_MEMORY_DEBUG("\r\nExecuted Memory Read");
		PrintArray("\r\n Read RX FIFO-", receiveBuf, 1); //2
		debugprint[0] = success << 24;
		debugprint[1] = success << 16;
		debugprint[2] = success << 8;
		debugprint[3] = success;
		PrintArray("\r\n Print success value RX FIFO-", debugprint, 0x04);
		PRINTF_MEMORY_DEBUG("\r\nExecuted RDSR loop");
	}

	rdSR[0] = receiveBuf[0];

	return (success);
}

uint32_t SPIFRAM_RDID(uint8_t* rdID_9B) {
	uint32_t readBytes = 0;
	uint8_t transmitBuf[2];
	uint8_t receiveBuf[12] = {0};

	// Send read command for ID
	transmitBuf[0] = RDID;

	SPI_CLEAR_FIFO;
	//Cy_SCB_SPI_ClearRxFifo(SPI_FRAM_MASTER_HW);

	for (int  i = 0; i < 9; i++)
		receiveBuf[i] = 0x00;

	//DELAY_MS(1);

	spi_cs_enable();
	FRAM_Write_array_blocking(&transmitBuf[0], 1);
	readBytes = FRAM_Read_array_blocking(&receiveBuf[0], 9);
	spi_cs_disable();

	if(ENABLE_DEBUG_FRAM){
		PRINTF_MEMORY_DEBUG("\r\n Executed Memory ID Read for %ld bytes", readBytes);
		PrintArray("\r\n ID - ", &receiveBuf[0], 9);
	}

	// If a pointer to a extern buffer is given copy ID there
	if(rdID_9B != NULL){
		for (uint8_t i = 0; i < 9; i++)
			rdID_9B[i] = receiveBuf[i];
	}

	return (readBytes);
}

/***************************************************************************
* Function Name: FRAM_init.c
***************************************************************************/
void FRAM_init(void) {
	/* Before executing this function - Set up internal routing, pins, and clock-to-peripheral connections */
	//init_cycfg_all();

	/* Init SPI_SCB on PDL (do not use for direct SPI access) ----- */
	//Cy_SCB_SPI_Init(SPI_FRAM_MASTER_HW, &SPI_FRAM_MASTER_config, &SPI_context);
	//Cy_SCB_SPI_SetActiveSlaveSelect(SPI_FRAM_MASTER_HW, CY_SCB_SPI_SLAVE_SELECT0);
	//Cy_SCB_SPI_SetActiveSlaveSelectPolarity(SPI_FRAM_MASTER_HW,
	//	CY_SCB_SPI_SLAVE_SELECT0,
	//	CY_SCB_SPI_ACTIVE_HIGH);

	// Set initial GPIO states
	FRAM_SET_WP_HIGH;
	FRAM_SET_HOLD_HIGH;
	//VDDU1_HIGH; // Enable voltage - drive high in case VDD jumper is disconnected

	/* After executing this function - Enable interrupts, and the CM4 */
	//__enable_irq();

	/* Configure SPI_SCB interrupt if needed (example for PDL) ----- */
	//cy_stc_sysint_t spiscbIntConfig =
	//{
	//	.intrSrc = scb_6_interrupt_IRQn,     /* SCB_SPI interrupt */
	//	.intrPriority = 1u       /* SCB_SPI interrupt priority */
	//};
	/* SCB_SPI interrupt initialization status */
	//cy_en_sysint_status_t intr_init_status;
	//intr_init_status = Cy_SysInt_Init(&spiscbIntConfig, SPI_Isr);
	//CheckStatus("\r\n\r\nSPI interrupt initialization failed\r\n", (uint32)intr_init_status);
	//NVIC_EnableIRQ((IRQn_Type)spiscbIntConfig.intrSrc);

	/* After executing this function - Enable SPI on PDL (do not use for direct SPI access) */
	//Cy_SCB_SPI_Enable(SPI_FRAM_MASTER_HW);
}


/*******************************************************************************
*
* FRAM Functions to directly write to FRAM (size limited by SPI packet size - see datasheet)
*
****************************************************************************/
void FRAM_Write(uint8_t* wData, uint32_t startAddress, uint32_t datasize) {
	uint32_t i;
	uint8_t transmitBuf[132];

	transmitBuf[0] = WRITE;
	transmitBuf[1] = startAddress >> 8;
	transmitBuf[2] = startAddress;

	SPI_CLEAR_FIFO;

	if (datasize > 128u) //restricts burst length to 128 byte
		datasize = 128;

	for (i = 0; i < datasize; i++)
		transmitBuf[i + 3] = wData[i];

	spi_cs_enable();
	FRAM_Write_array_blocking(transmitBuf, datasize + 0x03);
	spi_cs_disable();

	if (ENABLE_DEBUG_FRAM) {
		PRINTF_MEMORY_DEBUG("\r\nExecuted Memory Write");
		PrintArray("\r\n Memory Write Address-", &transmitBuf[1], 2u);
		PrintArray("\r\n Write Data-", wData, datasize);
	}
}


void FRAM_Write_4Kb(uint8_t OPCODE, uint8_t* wData, uint32_t startAddress, uint32_t datasize) {
	uint32_t i;
	uint8_t transmitBuf[132];

	transmitBuf[0] = OPCODE;
	transmitBuf[1] = startAddress;

	SPI_CLEAR_FIFO;

	if (datasize > 128u)//restricts burst length to 128 bye
		datasize = 128;

	for (i = 0; i < datasize; i++)
		transmitBuf[i + 2] = wData[i];

	spi_cs_enable();
	FRAM_Write_array_blocking(transmitBuf, datasize + 0x02);
	spi_cs_disable();

	if (ENABLE_DEBUG_FRAM) {
		PRINTF_MEMORY_DEBUG("\r\nExecuted Memory Write");
		PrintArray("\r\n Memory Write Address-", &transmitBuf[1], 2u);
		PrintArray("\r\n Write Data-", wData, datasize);
	}
}

uint32_t FRAM_Read(uint8_t* rData, uint32_t startAddress, uint32_t datasize) {
	uint32_t i;
	uint32_t success = 0;
	uint8_t debugprint[4];
	uint8_t transmitBuf[132];
	uint8_t receiveBuf[132];

	transmitBuf[0] = READ;
	transmitBuf[1] = startAddress >> 8;
	transmitBuf[2] = startAddress;

	SPI_CLEAR_FIFO;
	//Cy_SCB_SPI_ClearRxFifo(SPI_FRAM_MASTER_HW);

	if (datasize > 128u) //restricts burst length to 128 byte
		datasize = 128;

	//DELAY_MS(1);

	for (i = 0; i < datasize + 3; i++)
		receiveBuf[i] = 0x00;

	spi_cs_enable();
	FRAM_Write_array_blocking(transmitBuf, 0x03);
	success = FRAM_Read_array_blocking(receiveBuf, datasize);
	spi_cs_disable();

	if (ENABLE_DEBUG_FRAM) {
		PRINTF_MEMORY_DEBUG("\r\nExecuted Memory Read");
		PrintArray("\r\n Memory Read Address-", &transmitBuf[1], 2u);
		PrintArray("\r\n Read RX FIFO-", &receiveBuf[0], datasize);

		debugprint[0] = success << 24;
		debugprint[1] = success << 16;
		debugprint[2] = success << 8;
		debugprint[3] = success;
		PrintArray("\r\n Print success value RX FIFO-", debugprint, 0x04);
	}

	for (i = 0; i < datasize; i++)
		rData[i] = receiveBuf[i];

	return (success);
}

uint32_t FRAM_Read_4Kb(uint8_t OPCODE, uint8_t* rData, uint32_t startAddress, uint32_t datasize) {
	uint32_t i;
	uint32_t success = 0;
	uint8_t debugprint[4];
	uint8_t transmitBuf[132];
	uint8_t receiveBuf[132];

	transmitBuf[0] = OPCODE;
	transmitBuf[1] = startAddress;

	SPI_CLEAR_FIFO;
	//Cy_SCB_SPI_ClearRxFifo(SPI_FRAM_MASTER_HW);

	if (datasize > 128u)//restricts burst length to 128 bye
		datasize = 128;

	for (i = 0; i < datasize + 2; i++)
		receiveBuf[i] = 0x00;

	spi_cs_enable();
	FRAM_Write_array_blocking(transmitBuf, 0x02);
	success = FRAM_Read_array_blocking(receiveBuf, datasize); // datasize + 0x02
	spi_cs_disable();

	if (ENABLE_DEBUG_FRAM) {
		PRINTF_MEMORY_DEBUG("\r\nExecuted Memory Read");
		PrintArray("\r\n Memory Read Address-", &transmitBuf[1], 1u);
		PrintArray("\r\n Read RX FIFO-", &receiveBuf[0], datasize);

		debugprint[0] = success << 24;
		debugprint[1] = success << 16;
		debugprint[2] = success << 8;
		debugprint[3] = success;
		PrintArray("\r\n Print success value RX FIFO-", debugprint, 0x04);
	}

	for (i = 0; i < datasize; i++)
		rData[i] = receiveBuf[i + 2];

	return (success);
}


/*******************************************************************************
*
* FRAM Functions to write to FRAM in pages (no size limit, but slower) UNTESTED
*
****************************************************************************/
void FRAM_Burst_Write(uint8_t* userData, uint32_t startAddress, uint32_t burstLen) {
	uint32_t index;
	uint32_t nPageSize = 64u; //max 128

	// Copy Tx buffer
	//for (index = 0; index < burstLen; index++)
	//	bufferTx[index] = userData[index];
	uint8_t* txBuf = userData;

	for (index = 0; index < burstLen / nPageSize; index++) {
		FRAM_WREN();
		FRAM_Write(&txBuf[index * nPageSize], (startAddress + index * nPageSize), nPageSize);
	}

	if ((burstLen / nPageSize) < nPageSize) {
		FRAM_WREN();
		FRAM_Write(&txBuf[index * nPageSize], (startAddress + index * nPageSize), burstLen % nPageSize);
	}

#if(ENABLE_DEBUG_FRAM)
		PRINTF_MEMORY_DEBUG(" \r\n Burst Write - std SCB SPI Complete\r\n ");
#endif
}

void FRAM_Burst_Write_4Kb(uint8_t* userData, uint32_t startAddress, uint32_t burstLen) {
	uint32_t index;
	uint32_t nPageSize = 64u; //max 128

	// Copy Tx buffer
	//for (index = 0; index < burstLen; index++)
	//	bufferTx[index] = userData[index];
	uint8_t* txBuf = userData;

	for (index = 0; index < burstLen / nPageSize; index++) {
		FRAM_WREN();

		if ((startAddress + index * nPageSize) < 0xFF)
			FRAM_Write_4Kb(WRITE, &txBuf[index * nPageSize], (startAddress + index * nPageSize), nPageSize);
		else
			FRAM_Write_4Kb(WRITE1, &txBuf[index * nPageSize], (startAddress + index * nPageSize), nPageSize);

	}

	if ((index * (burstLen / nPageSize)) < burstLen) {
		FRAM_WREN();
		if ((startAddress + index * nPageSize) <= 0xFF)
			FRAM_Write_4Kb(WRITE, &txBuf[index * nPageSize], (startAddress + index * nPageSize), burstLen % nPageSize);
		else
			FRAM_Write_4Kb(WRITE1, &txBuf[index * nPageSize], (startAddress + index * nPageSize), burstLen % nPageSize);
	}

#if(ENABLE_DEBUG_FRAM)
	PRINTF_MEMORY_DEBUG(" \r\n Burst Write - std SCB SPI Complete\r\n ");
#endif
}

void FRAM_Burst_Read(uint8_t* readData, uint32_t startAddress, uint32_t burstLen) {
	uint32_t index;
	uint32_t nPageSize = 64u; //max 128u

	// Clear Rx buffer
	//for (index = 0; index < burstLen; index++)
	//	bufferRx[index] = 0x00;

	for (index = 0; index < burstLen / nPageSize; index++)
		FRAM_Read(&readData[index * nPageSize], index * nPageSize, nPageSize);

	if ((burstLen / nPageSize) < nPageSize) {
		FRAM_WREN();
		FRAM_Read(&readData[index * nPageSize], (startAddress + index * nPageSize), burstLen % nPageSize);
	}
#if(ENABLE_DEBUG_FRAM)
	uint8_t memPrint[2];
	memPrint[0] = startAddress;
	memPrint[1] = startAddress >> 8;
	PrintArray("\r\n Memory read address = ", &memPrint[0], 2);
	PrintArray("\r\n Memory read data = ", readData, burstLen);
	PRINTF_MEMORY_DEBUG("\r\n Memory reading std SCB SPI complete");
#endif
}

void FRAM_Burst_Read_4Kb(uint8_t* readData, uint32_t startAddress, uint32_t burstLen) {
	uint32_t index;
	uint32_t nPageSize = 64u; //max 128u

	// Clear Rx buffer
	//for (index = 0; index < burstLen; index++)
	//	bufferRx[index] = 0x00;

	for (index = 0; index < burstLen / nPageSize; index++) {
		if ((startAddress + index * nPageSize) <= 0xFF)
			FRAM_Read_4Kb(READ, &readData[index * nPageSize], (startAddress + index * nPageSize), nPageSize);
		else
			FRAM_Read_4Kb(READ1, &readData[index * nPageSize], (startAddress + index * nPageSize), nPageSize);
	}

	if ((index * (burstLen / nPageSize)) < burstLen) {
		if ((startAddress + index * nPageSize) <= 0xFF)
			FRAM_Read_4Kb(READ, &readData[startAddress + index * nPageSize], (startAddress + index * nPageSize), burstLen % nPageSize);
		else
			FRAM_Read_4Kb(READ1, &readData[startAddress + index * nPageSize], (startAddress + index * nPageSize), burstLen % nPageSize);
	}

#if(ENABLE_DEBUG_FRAM)
	uint8_t memPrint[2];
	memPrint[0] = startAddress;
	memPrint[1] = startAddress >> 8;
	PrintArray("\r\n Memory read address = ", &memPrint[0], 2);
	PrintArray("\r\n Memory read data = ", readData, burstLen);
	PRINTF_MEMORY_DEBUG("\r\n Memory reading std SCB SPI complete");
#endif
}

uint32_t FRAM_Burst_Read_And_Compare(uint8_t* expectedData, uint32_t startAddress, uint32_t burstLen, uint8_t* bufferRx) {
	uint32_t readTimeMs = 0;
	uint32_t index;
	uint32_t nPageSize = 64u;//max 128u
	uint8_t status = 0;

	uint32_t timeBefore = TIMER_COUNTER_1MS;

	// Clear Rx buffer
	//for (index = 0; index < burstLen; index++)
	//	bufferRx[index] = 0x00;

	for (index = 0; index < MEM_SIZE / nPageSize; index++)
		FRAM_Read(&bufferRx[index * nPageSize], index * nPageSize, nPageSize);

	if ((burstLen / nPageSize) < nPageSize) {
		FRAM_WREN();
		FRAM_Read(&bufferRx[index * nPageSize], (startAddress + index * nPageSize), burstLen % nPageSize);
	}

	readTimeMs = TIMER_COUNTER_1MS-timeBefore;

	// Compare
	uint8_t memPrint[2];
	memPrint[0] = startAddress;
	memPrint[1] = startAddress >> 8;
	for (index = 0; index < burstLen; index++) {
		if (bufferRx[index] != expectedData[index])	{
			PRINTF_MEMORY_DEBUG(" \r\n GF Failed");
			memPrint[1] = index;
			memPrint[0] = index >> 8;
			PrintArray("\r\n Failing Address = ", memPrint, 2u);

			PrintArray("\r\n  Expected data = ", &expectedData[index], 1u);
			PrintArray("\r\n  Read data = ", &bufferRx[index], 1u);

			status = 1;
			return readTimeMs;
		}
		else
			status = 0;
	}

	if (!status) {
		PRINTF_MEMORY_DEBUG(" \r\n GF Passed");
		memPrint[1] = index;
		memPrint[0] = index >> 8;
		PrintArray("\r\n Last Address = ", memPrint, 2u);
	}

	if (ENABLE_DEBUG_FRAM) {
		PrintArray("\r\n Memory read address = ", &memPrint[0], 2);
		PrintArray("\r\n Memory read data = ", bufferRx, burstLen);
		PRINTF_MEMORY_DEBUG("\r\n Full Memory reading in bit bang complete");
	}

	return readTimeMs;
}






/*******************************************************************************
* Function Name: PrintArray
****************************************************************************//**
*
* This function prints the content of the RX buffer to the UART console.
*
* \param msg - message print before array output
*
* \param  rxBuffer - The buffer to the console output.
*
* \param  size - The size of the buffer to the console output.
*
*******************************************************************************/
void PrintArray(char* msg, uint8_t* buff, uint32_t size) {
	PRINTF_MEMORY_DEBUG("%s", msg);

	for (uint32_t index = 0; index < size; index++) {
		PRINTF_MEMORY_DEBUG("0x%02X ", (unsigned int)buff[index]);
	}
	PRINTF_MEMORY_DEBUG("\r\n=======================\r\n");
}





/*******************************************************************************
*
* FRAM TERMINAL TEST Functions
*
****************************************************************************/
#if(ENABLE_TERMINAL_FRAM)
/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* This function processes unrecoverable errors
*******************************************************************************/
void handle_error(void) {
	/* Disable all interrupts */
	//__disable_irq();
	//while (1u) {}
}

//void SPI_Isr(void) {
//	Cy_SCB_SPI_Interrupt(SPI_FRAM_MASTER_HW, &SPI_context);
//
//}

void PrintInteger(uint32_t* intVal, uint32_t size) {
	uint8_t int2char[4];
	uint32_t number, index;

	for (number = 0; number < size; number++) {
		int2char[0] = (uint8_t)intVal[number];
		int2char[1] = intVal[number] >> 8;
		int2char[2] = intVal[number] >> 16;
		int2char[3] = intVal[number] >> 24;

		for (index = 3; index < 0; index--) {
			PRINTF_MEMORY_DEBUG("%02X ", (unsigned int)int2char[index]);
		}

		PRINTF_MEMORY_DEBUG("\r\n=======================\r\n");
	}
}

/*******************************************************************************
* Function Name: InitBuffers
****************************************************************************//**
*
* This function initializes the transfer buffers.
*
* \param txBuffer - The buffer for Write data.
*
* \param rxBuffer - The buffer for Read data.
*
*******************************************************************************/
void InitBuffers(uint8_t txBuffer[], uint8_t rxBuffer[], uint32_t bufferSize) {
	for (uint32_t index = 0; index < bufferSize; index++) {
		txBuffer[index] = 0x00;
		rxBuffer[index] = 0x00;
	}
}

/*******************************************************************************
* Function Name: CheckStatus
****************************************************************************//**
*
* Check if status is SUCCES and call handle error function
*
*******************************************************************************/
void CheckStatus(char* msg, uint32_t status) {
	if (STATE_SUCCESS != status) {
		PRINTF_MEMORY_DEBUG("%s", msg);
		handle_error();
	}
}

/*******************************************************************************
* Function Name: TxRxEqualCheck
****************************************************************************//**
*
* This function checks if the transmitted and received arrays are equal
*
* \param txBuffer - The buffer for Write data.
*
* \param rxBuffer - The buffer for Read data.
*
*******************************************************************************/

uint8_t TxRxEqualCheck(uint8_t txBuffer[], uint8_t rxBuffer[], uint32_t size) {
	return(0U == memcmp(txBuffer, rxBuffer, size));
}

void makeBytes(uint8_t* strPtr, uint8_t* Data, uint32_t* cmdLen)  // Converts ASCII to Hex and combine the nibbles to make byte
{
	uint32_t i = 0;
	for (i = 0; i < cmdLen[0]; i++) {
		if ((strPtr[i] >= 0x41) && (strPtr[i] <= 0x46))
			strPtr[i] = strPtr[i] + 0x09;

		else if ((strPtr[i] >= 0x61) && (strPtr[i] <= 0x66))
			strPtr[i] = strPtr[i] + 0x09;

		else if ((strPtr[i] >= 0x30) && (strPtr[i] <= 0x39))
			strPtr[i] = strPtr[i] + 0x00;
		else {
			PRINTF_MEMORY_DEBUG("Data is not a valid Hex value");
			return;
		}
	}

	for (i = 0; i < cmdLen[0] / 2; i++) {
		Data[i] = ((strPtr[2 * i] & 0x0f) << 4) | ((strPtr[2 * i + 1]) & 0x0f);
		if(ENABLE_DEBUG_FRAM)
			PRINTF_MEMORY_DEBUG("\t\tAdded command byte 0x%02X\r\n", Data[i]);
	}
}
/***************************************************************************
* Function Name: FRAM_testTerminal.c - This function will emulate a test terminal for multiple memory functions and print debug messages. Once entered it will never be left (use this for reference)
***************************************************************************/
int FRAM_testTerminal(void) {
	uint8_t bufferTx[MEM_SIZE + 3u];
	uint8_t bufferRx[MEM_SIZE + 3u];
	uint8_t strPtr[2 * UART_PACKET_SIZE];
	uint8_t Data[UART_PACKET_SIZE];
	uint32_t cmdLen = 0;
	uint8_t receivedCharacter = 0;

	uint8_t dSR[2];
	uint32_t  mrwAddress;
	uint32_t j = 0;
	uint8_t tmpData[MEM_SIZE];
	uint8_t memPrint[4];
	uint8_t opcode;

	// Initialize F-RAM
	FRAM_init();

	// Print test debug
	PRINTF_MEMORY_DEBUG("\r\n Code initialization complete");
	PRINTF_MEMORY_DEBUG("\r\n ******** 1/4/16/64/256 Kb FRAM Tester (selected MEM_SIZE = %d Kb) *************\r\n", (MEM_SIZE*8)/1024);
	PRINTF_MEMORY_DEBUG("\r\n Send following command codes with parameters to perform test");
	PRINTF_MEMORY_DEBUG("\r\n\t Examples: ");
	PRINTF_MEMORY_DEBUG("\r\n\t\t 020001AABB => command 0x02, address 0x0001, byte1 0xAA, byte2 0xBB");
	PRINTF_MEMORY_DEBUG("\r\n\t\t 03000102   => command 0x03, address 0x0001, read 2 byte");
	PRINTF_MEMORY_DEBUG("\r\n\t\t F17702     => Write test with time measurement");
	PRINTF_MEMORY_DEBUG("\r\n\t\t F27702     => Read test with time measurement");
	PRINTF_MEMORY_DEBUG("\r\n case 0x00: // Read status register and Chip ID");
	PRINTF_MEMORY_DEBUG("\r\n case 0x01: // Write to the status register");
	PRINTF_MEMORY_DEBUG("\r\n case 0x02: // Opcode for Write");
	PRINTF_MEMORY_DEBUG("\r\n case 0x03: // Opcode for Read");
	PRINTF_MEMORY_DEBUG("\r\n case 0x04: // Reset Write Enable Latch");
	PRINTF_MEMORY_DEBUG("\r\n case 0x06: // Set Write Enable Latch");
	PRINTF_MEMORY_DEBUG("\r\n case 0x0A: // Opcode for SPI Write (4Kb) - Upper half of memory array");
	PRINTF_MEMORY_DEBUG("\r\n case 0x0B: // Opcode for SPI Read (4Kb)  - Upper half of memory array");
	//PRINTF_MEMORY_DEBUG("\r\n case 0xA2: // Toggle VDDU2_00 off; !0x00 ON");
	//PRINTF_MEMORY_DEBUG("\r\n case 0xB1: // Full memory write with Data in BB (bit bang ~@2.2 MHz) mode");
	//PRINTF_MEMORY_DEBUG("\r\n case 0xDD: // SPI transfer on SI as well SO read, bit banging - DUT on U1");
	//PRINTF_MEMORY_DEBUG("\r\n case 0xDE: // SPI transfer on SI as well SO read, bit banging - DUT on U2");
	//PRINTF_MEMORY_DEBUG("\r\n            // DD or DE, 11 adds 10 sec delay before access, at the beginning");
	//PRINTF_MEMORY_DEBUG("\r\n            // DD or DE, 22 adds 10 sec delay after the access, at the end");
	//PRINTF_MEMORY_DEBUG("\r\n            // DD or DE only no (std) delays");
	//PRINTF_MEMORY_DEBUG("\r\n case 0xDF: // SPI transfer on SI as well SO read, bit banging; partial data byte");
	PRINTF_MEMORY_DEBUG("\r\n case 0xF1: // Full memory WRITE and time measurement ");
	PRINTF_MEMORY_DEBUG("\r\n            // 0x77 0x01 generates random data pattern, 0x77 0x02 generates 00 to FF data pattern,");
	PRINTF_MEMORY_DEBUG("\r\n            // any other is used direct");
	PRINTF_MEMORY_DEBUG("\r\n case 0xF2: // Full memory READ and COMPARE with expected Data and time measurement");
	PRINTF_MEMORY_DEBUG("\r\n            // Use this with same parameters as 0xF1 to verify write");
	PRINTF_MEMORY_DEBUG("\r\n            // 0x77 0x02 generates 00 to FF data pattern any other is used direct");
	PRINTF_MEMORY_DEBUG("\r\n case 0xF0: // Full memory write, read and compare  (GF Test)");
	PRINTF_MEMORY_DEBUG("\r\n case 0xF4: // Full memory write, read and compare  (GF Test) 4Kb");
	//PRINTF_MEMORY_DEBUG("\r\n case 0xF8: // Full memory write in SCB mode, read and verify in SCB and BB mode");
	//PRINTF_MEMORY_DEBUG("\r\n case 0xFA: // Power Cycle Test");
	//PRINTF_MEMORY_DEBUG("\r\n case 0xFB: // Full memory write with Data-4Kb");
	//PRINTF_MEMORY_DEBUG("\r\n case 0xFE: // Memory Density Check");
	PRINTF_MEMORY_DEBUG("\r\n case 0xA0: // Set WP/HOLD HI/LO. 0x00 (L/L); 0x0F(L/H); 0xF0/(H/L); 0xFF (H/H); others (H/H)");
	PRINTF_MEMORY_DEBUG("\r\n case 0xA1: // Read WP/HOLD pin status");
	PRINTF_MEMORY_DEBUG("\r\n case 0xCC: // SPI User Command");
	PRINTF_MEMORY_DEBUG("\r\n case 0xD0: // Invalid opcode test");

	PRINTF_MEMORY_DEBUG("\r\n>>");

	// Test main loop
	while (1) {
		InitBuffers(bufferTx, bufferRx, MEM_SIZE); //(bufferTx, bufferTx, MEM_SIZE);
		PRINTF_MEMORY_DEBUG("\r\n>>");

		// Read UART input and store result to be checked by switch
		j = 0;
		while (1){
			// Try to get next character and store it if successful
			if(UART_GET_CHAR_AND_CHECK) {
				strPtr[j] = receivedCharacter; //receivedCharacter = Cy_SCB_UART_Get(UART_HW);
				j++;

				//PRINTF_MEMORY_DEBUG("\t Received character %c\r\n", receivedCharacter);
			}
			// If \r was received store command and stop loop
			if (strPtr[j - 1] == 0x0d) {
				cmdLen = j - 1;
				UART_CLEAR;
				break;
			}
		}

		// Convert command
		makeBytes(strPtr, Data, &cmdLen);
		//PrintArray("\r\n UART RX bytes (hex)", &Data[0], cmdLen/2);

		// Main state machine
		switch (Data[0]){
			case 0x00: // Read Id and Status Reg
				uint8_t rdid_buffer[9];

				PRINTF_MEMORY_DEBUG("\r\n Read Memory ID");
				SPIFRAM_RDID(&rdid_buffer[0]);
				PrintArray("\r\n ID = ", &rdid_buffer[0], 9);

				PRINTF_MEMORY_DEBUG("\r\n Read Status Register");
				SPIFRAM_RDSR(&dSR[0]);
				PrintArray("\r\n SR = ", &dSR[0], 0x01);
				break;

			case 0x01: // Write to the status register
				FRAM_WREN();
				SPIFRAM_WRSR(Data[1]);
				break;

			case 0x02: // Opcode for Write
				if (cmdLen / 2 < 3) {
					PRINTF_MEMORY_DEBUG("\r\n Insufficient Data to execute write operation - Please enter again\r\n");
					break;
				}

				FRAM_WREN();
				mrwAddress = Data[1];

				if (!SPI_4KB) {
					mrwAddress = (mrwAddress << 8 | Data[2]);
					FRAM_Write(&Data[3], mrwAddress, (cmdLen / 2 - 3));
					PrintArray("\r\n Write SPI FRAM data -", &Data[3], (cmdLen / 2 - 3));
				}
				else {
					FRAM_Write(&Data[2], mrwAddress, (cmdLen / 2 - 2));
					PrintArray("\r\n Write SPI FRAM data (<=4KB - ADDR WIDTH 1B) -", &Data[2], (cmdLen / 2 - 2));
				}
				break;

			case 0x03: //Opcode for Read
				if (cmdLen / 2 < 3) {
					PRINTF_MEMORY_DEBUG("\r\n Insufficient Data to execute read operation - Please enter again\r\n");
					break;
				}
				mrwAddress = Data[1];

				if (!SPI_4KB) {
					mrwAddress = (mrwAddress << 8 | Data[2]);
					FRAM_Burst_Read(bufferRx, mrwAddress, Data[3]);
					PrintArray("\r\n SPI FRAM read data -", bufferRx, Data[3]);
				}
				else {
					FRAM_Burst_Read_4Kb(bufferRx, mrwAddress, Data[2]);
					PrintArray("\r\n SPI FRAM read data (<=4KB - ADDR WIDTH 1B) -", bufferRx, Data[2]);
				}
				break;

			case 0x04: //Reset Write Enable Latch
				SPIFRAM_WRDI();
				PRINTF_MEMORY_DEBUG("\r\n Reset Write Enable Latch (WRDI)");
				break;

			case 0x06: //Set Write Enable Latch
				FRAM_WREN();
				PRINTF_MEMORY_DEBUG("\r\n Set Write Enable Latch (WREN)");
				break;

			case 0x0A: // Opcode for SPI Write (4Kb) - Upper half of memory array
				if (cmdLen / 2 < 3) {
					PRINTF_MEMORY_DEBUG("\r\n Insufficient Data to execute write operation - Please enter again\r\n");
					break;
				}
				FRAM_WREN();
				PRINTF_MEMORY_DEBUG("\r\n 4Kb SPI FRAM burst write with opcode 0x0A (upper half)\r\n");

				mrwAddress = Data[1];
				FRAM_Burst_Write_4Kb(&Data[2], mrwAddress, (cmdLen / 2 - 2));
				PrintArray("\r\n Write SPI FRAM data -", &Data[2], (cmdLen / 2 - 2));
				break;

			case 0xA0: //Set WP/HOLD HI/LO. 0x00 (L/L); 0x0F(L/H); 0xF0/(H/L); 0xFF (H/H); others (H/H)
				WP_HOLD(Data[1]);
				break;

			case 0xA1: //Read WP/HOLD pin status
				WP_HOLD_Check();
				break;

			case 0x0B: // Opcode for SPI Read (4Kb) - Upper half of memory array
				if (cmdLen / 2 < 3) {
					PRINTF_MEMORY_DEBUG("\r\n Insufficient Data to execute read operation - Please enter again\r\n");
					break;
				}

				PRINTF_MEMORY_DEBUG("\r\n 4Kb SPI FRAM burst read with opcode 0x0B (upper half)\r\n");
				mrwAddress = Data[1];
				FRAM_Burst_Read_4Kb(bufferRx, mrwAddress, Data[2]);
				PrintArray("\r\n SPI FRAM read data -", bufferRx, Data[2]);
				break;

			case 0xB1: //Full memory write with Data in BB mode
				PRINTF_MEMORY_DEBUG("\r\n BB not implemented, see commented out section in FRAM_DIRECT_SPI.c\r\n");
				//PrintArray("\n\r Full memory write with Data DUT on U1 ", &Data[1], 2);
				//if ((Data[1] == 0x77) && (Data[2] == 0x01)) {
				//	PRINTF_MEMORY_DEBUG("\n\r Full memory read and compare with Data 0x77 0x11 generates random data pattern");
				//	RandDataPattern(tmpData, MEM_SIZE);
				//}
				//else if ((Data[1] == 0x77) && (Data[2] == 0x02)) {
				//	PRINTF_MEMORY_DEBUG("\n\r Full memory read and compare with Data 0x77 0x11 generates 00 to FF data pattern");
				//	Zero2FF(tmpData, MEM_SIZE);
				//}
				//else
				//	UserDataPattern(tmpData, MEM_SIZE, Data[1], Data[2]);
				//
				//SPI_BB_Burst_Write(tmpData, 0u, MEM_SIZE);
				break;

			case 0xCC: //SPI User Command

				PRINTF_MEMORY_DEBUG("\n\r    ");

				spi_cs_enable();
				FRAM_Write_array_blocking(&Data[1], cmdLen / 2 - 1);
				PrintArray("\r\n SPI Tx = ", &Data[1], cmdLen / 2 - 1);
				//DELAY_MS(1);
				FRAM_Read_array_blocking(bufferRx, cmdLen / 2 - 1);
				spi_cs_disable();

				PrintArray("\r\n SPI Rx = ", &bufferRx[0], cmdLen / 2 - 1);
				//DELAY_MS(1);
				break;

			case 0xD0: //Invalid opcode test
				PRINTF_MEMORY_DEBUG("\n\r Invalid opcode test   ");
				for (opcode = 0x00; opcode < 0xFF; opcode++) {
					bufferTx[1] = 0x04;
					bufferTx[0] = opcode;

					PrintArray("\r\n Transmitted opcode 0x= ", bufferTx, 2);

					spi_cs_enable();
					FRAM_Write_array_blocking(&opcode, 2);
					//DELAY_MS(1);
					FRAM_Read_array_blocking(bufferRx, 2);
					bufferRx[0] = 0x00;
					bufferRx[1] = 0x00;
					bufferTx[0] = 0x05;
					FRAM_Write_array_blocking(&bufferTx[0], 2);
					//DELAY_MS(1);
					FRAM_Read_array_blocking(&bufferRx[0], 2);
					spi_cs_disable();

					PrintArray("\r\n SPI Status Reg Read = ", &bufferRx[0], 2);

					//DELAY_MS(1);
				}
				break;

			case 0xF0: //Full memory write, read and compare  (GF Test)

				PRINTF_MEMORY_DEBUG("\n\r Gross functional test ");
				UserDataPattern(bufferTx, MEM_SIZE, Data[1], Data[2]);

				for (j = 0; j < MEM_SIZE; j++)
					tmpData[j] = bufferTx[j];

				FRAM_Burst_Write(bufferTx, 0u, MEM_SIZE);
				FRAM_Burst_Read_And_Compare(tmpData, 0u, MEM_SIZE, bufferRx);

				//SPI_BB_Burst_Write(bufferTx, 0u, MEM_SIZE);
				//SPI_BB_Burst_Read_And_Compare(tmpData, 0u, MEM_SIZE);
				break;

			case 0xF4: //Full memory write, read and compare  (GF Test) 4Kb

				PRINTF_MEMORY_DEBUG("\n\r Gross functional test 4Kb");
				UserDataPattern(bufferTx, MEM_SIZE, Data[1], Data[2]);

				for (j = 0; j < MEM_SIZE; j++)
					tmpData[j] = 0x00;

				FRAM_Burst_Write_4Kb(bufferTx, 0u, MEM_SIZE);
				FRAM_Burst_Read_4Kb(tmpData, 0u, MEM_SIZE);

				for (j = 0; j < MEM_SIZE; j++) {
					if (tmpData[j] != bufferTx[j]) {
						PRINTF_MEMORY_DEBUG(" \r\n GF Failed");
						memPrint[1] = j;
						memPrint[0] = j >> 8;
						PrintArray("\r\n Failing Address = ", memPrint, 2u);

						PrintArray("\r\n  Expected data = ", &bufferTx[j], 1u);
						PrintArray("\r\n  Read data = ", &tmpData[j], 1u);
					}
				}
				break;

			case 0xF1: //Full memory write with Data

				PrintArray("\n\r Full memory write with Data ", &Data[1], 2);

				if ((Data[1] == 0x77) && (Data[2] == 0x01))	{
					PRINTF_MEMORY_DEBUG("\n\r\t 0x77 0x01 generates random data pattern");
					RandDataPattern(tmpData, MEM_SIZE);
				}
				else if ((Data[1] == 0x77) && (Data[2] == 0x02)) {
					PRINTF_MEMORY_DEBUG("\n\r\t 0x77 0x02 generates 00 to FF data pattern");
					Zero2FF(tmpData, MEM_SIZE);
				}
				else
					UserDataPattern(tmpData, MEM_SIZE, Data[1], Data[2]);

				uint32_t timeBefore = TIMER_COUNTER_1MS;
				FRAM_Burst_Write(tmpData, 0u, MEM_SIZE);
				PRINTF_MEMORY_DEBUG("\n\r\t Burst Write took %ldms", TIMER_COUNTER_1MS-timeBefore);
				//SPI_BB_Burst_Write (tmpData, 0u, MEM_SIZE);
				break;

			case 0xF2: //Full memory read and compare with expected Data

				PrintArray("\n\r Full memory read and compare with Data ", &Data[1], 2);
				if ((Data[1] == 0x77) && (Data[2] == 0x01))	{
					PRINTF_MEMORY_DEBUG("\n\r\t 0x77 0x01 generates random data pattern");
					RandDataPattern(tmpData, MEM_SIZE);
				}

				else if ((Data[1] == 0x77) && (Data[2] == 0x02)) {
					PRINTF_MEMORY_DEBUG("\n\r\t 0x77 0x02 generates 00 to FF data pattern");
					Zero2FF(tmpData, MEM_SIZE);
				}
				else
					UserDataPattern(tmpData, MEM_SIZE, Data[1], Data[2]);

				uint32_t readTimeMs = FRAM_Burst_Read_And_Compare(tmpData, 0u, MEM_SIZE, bufferRx);
				PRINTF_MEMORY_DEBUG("\n\r\t Burst Read took %ldms", readTimeMs);
				break;

			case 0xFB: //Full memory write with Data-4Kb

				PRINTF_MEMORY_DEBUG("\n\r Full memory write 4Kb");
				UserDataPattern(bufferTx, MEM_SIZE, Data[1], Data[2]);

				FRAM_Burst_Write_4Kb(bufferTx, 0u, MEM_SIZE);
				break;

			case 0xFC: //Full (4Kb) memory read and compare with expected Data

				PrintArray("\n\r Full memory read and compare with Data ", &Data[1], 2);
				UserDataPattern(tmpData, MEM_SIZE, Data[1], Data[2]);


				for (j = 0; j < MEM_SIZE; j++) { bufferRx[j] = 0x00; }

				FRAM_Burst_Read_4Kb(bufferRx, 0u, MEM_SIZE);

				for (j = 0; j < MEM_SIZE; j++) {
					if (bufferRx[j] != tmpData[j])

					{
						PRINTF_MEMORY_DEBUG(" \r\n GF Failed");
						memPrint[1] = j;
						memPrint[0] = j >> 8;
						PrintArray("\r\n Failing Address = ", memPrint, 2u);

						PrintArray("\r\n  Expected data = ", tmpData, 1u);
						PrintArray("\r\n  Read data = ", bufferRx, 1u);

					}
				}
				break;

			case 0xFE:
				PRINTF_MEMORY_DEBUG("\r\n Memory Density Check not implemented");
				//MemDensityCheck();
				break;

			default:
				PRINTF_MEMORY_DEBUG(" \r\n Not a recognizable SPI FRAM opcode \r\n ");
				break;
		} // end of switch

	}

	for (;;) {
		LED_RED_TOGGLE_STATE;/* toggle the LED */
	}
}


void RandDataPattern(uint8_t* wrArray, uint32_t arraySize) {
	uint32_t c, n;

	for (c = 0; c < arraySize / 4; c++) {
		n = rand() % 100000000 + 1;
		wrArray[4 * c] = n;
		wrArray[4 * c + 1] = n >> 8;
		wrArray[4 * c + 2] = n >> 16;
		wrArray[4 * c + 3] = n >> 24;
	}

}

void UserDataPattern(uint8_t* wrArray, uint32_t arraySize, uint8_t dataByte, uint8_t dataBytebar) {
	uint32_t n;

	for (n = 0; n < arraySize / 2; n++) {
		wrArray[2 * n] = dataByte;
		wrArray[2 * n + 1] = dataBytebar;
	}
}

void Zero2FF(uint8_t* wrArray, uint32_t arraySize) {
	uint32_t n;

	for (n = 0; n < arraySize; n++){
		wrArray[n] = (uint8_t)n;
	}
}

void WP_HOLD(uint8_t status) {
	if (status == 0x00) {
		FRAM_SET_WP_LOW;
		FRAM_SET_HOLD_LOW;

		PRINTF_MEMORY_DEBUG("FRAM_SET_WP_LOW; FRAM_SET_HOLD_LOW;");
	}

	else if (status == 0x0F) {
		FRAM_SET_WP_LOW;
		FRAM_SET_HOLD_HIGH;
		PRINTF_MEMORY_DEBUG("FRAM_SET_WP_LOW; FRAM_SET_HOLD_HIGH;");
	}

	else if (status == 0xF0) {
		FRAM_SET_WP_HIGH;
		FRAM_SET_HOLD_LOW;
		PRINTF_MEMORY_DEBUG("FRAM_SET_WP_HIGH; FRAM_SET_HOLD_LOW;");
	}

	else {
		FRAM_SET_WP_HIGH;
		FRAM_SET_HOLD_HIGH;
		PRINTF_MEMORY_DEBUG("FRAM_SET_WP_HIGH; FRAM_SET_HOLD_HIGH;");
	}
}

void WP_HOLD_Check(void) {
	uint8_t value = WP_READ;
	PRINTF_MEMORY_DEBUG("\r\n WP pin status   = %d", value);
	value = HOLD_READ;
	PRINTF_MEMORY_DEBUG("\r\n HOLD pin status = %d", value);
}

#endif //#if(ENABLE_TERMINAL_FRAM)














