/*
 * memory.h - Abstraction off all memory related functions that need to be accessed from main.
 * 			  This way the memory library does not need to be implemented hard into main file.
 * 			  Check init_memory and manage_memory for details.
 *
 *  Created on: 21 May 2024
 *      Author: r. santeler
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef MEMORY_H_
#define MEMORY_H_

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
typedef struct{
	uint16_t TimeDiff;
	int16_t CapacityDiff;
	uint16_t BlockVoltage;
	uint16_t CellImbalance;
	int16_t TempAvg;
} memoryRtData_struct;


/*******************************************************************************
* MEMORY ADDRESS DEFINITIONS
*******************************************************************************/
// Definition of stored system data (settings and status)
#define MEM_ADDR_STORED_MEMORY_VALID		0x0000	// Address where the memory valid value is stored
#define MEM_SIZE_STORED_MEMORY_VALID		1		// Size of the memory valid value
#define MEM_COMP_STORED_MEMORY_VALID		42		// Compare value for the memory valid value. If the byte in MEM_ADDR_STORED_DATA_READY is set to this valid the memory is considered valid (not the first write and simple check if memory working)
#define MEM_ADDR_LAST_SOC					0x0001	// Address where the last SoC value is stored
#define MEM_SIZE_LAST_SOC					1		// Size of the last SoC value
#define MEM_ADDR_LAST_SOH					0x0002	// Address where the last SoH value is stored
#define MEM_SIZE_LAST_SOH					1		// Size of the last SoH value
#define MEM_ADDR_NEXT_RT_DATA_ADDR			0x0003	// Address where the next written index for runtime data storage is stored
#define MEM_SIZE_NEXT_RT_DATA_ADDR			2		// Size of the next written index for runtime data storage


// Definition of runtime data storage
#define MEM_SIZE_RT_DATA					10		// Size of one RT data line
													// NAME					SIZE	SIGNED	UNIT		RANGE			TIME LIMIT
#define MEM_INDX_RT_DATA_LINE_TIME_DIFF		0		// Time difference		2B		US		ms			0 to 65535ms	dt max 65.535s
#define MEM_INDX_RT_DATA_LINE_CAPACITY_DIFF	2  	 	// Capacity difference 	2B		S		As*10000	+/- 3.2767As	dt max 0.54s@6A or 109.22s@30mA
#define MEM_INDX_RT_DATA_LINE_BLOCK_VOLTAGE	4   	// Voltage				2B		US		mV			0 to 65.535V
#define MEM_INDX_RT_DATA_LINE_CELL_IMBAL	6   	// Imbalance			2B		US		mV			0 to 65.535V
#define MEM_INDX_RT_DATA_LINE_TEMP_AVG		8   	// Temperature			2B		S		°C*100		+/- 327.67°C

#define MEM_ADDR_RT_DATA_START				0x000A	// Address where the stored settings and status values start and the storing of runtime data starts
#define MEM_ADDR_RT_DATA_STOP				0x7FFF	// Address where the storing of runtime data ends
													// 32757 	Bytes for storage
													// 10 		Bytes per line
													// 3275		Lines possible		with 0.54s@6A->1768.5s=29.475min		with 109.22s@30mA->99,36h
#define MEM_SIZE_MAX_LINES					((MEM_ADDR_RT_DATA_STOP-MEM_ADDR_RT_DATA_START)/MEM_SIZE_RT_DATA)


/*******************************************************************************
 * VARIABLES
*******************************************************************************/
extern uint8_t isMemoryValid;
extern uint8_t memoryLastStoredSoC;
extern uint8_t memoryLastStoredSoH;
extern uint16_t nextMemoryRtLineAddressOffset;
extern uint8_t  memoryRtDataLineBuffer[MEM_SIZE_RT_DATA];


/*******************************************************************************
* FUNCTIONS
*******************************************************************************/
void init_memory();
void manage_memory();
void memoryResetRtData();
void memoryGetRtData(memoryRtData_struct* dataArray, uint16_t numLines, uint16_t dataStartLineIdx, uint8_t onlyPrintData);
void memoryGetRtData_byteArray(uint8_t* receiveBuf, uint16_t numLines, uint16_t dataStartLineIdx);

void memoryWrite(uint8_t* wData, uint32_t startAddress, uint32_t datasize);
void memoryRead(uint8_t* rData, uint32_t startAddress, uint32_t datasize);
void memoryWriteBurst(uint8_t *userData, uint32_t startAddress, uint32_t burstLen);
void memoryReadBurst(uint8_t *readData, uint32_t startAddress, uint32_t burstLen);

#endif /* MEMORY_H_ */
