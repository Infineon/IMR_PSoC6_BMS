/*
 * memory.c - Abstraction off all memory related functions that need to be accessed from main.
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

#include <stdio.h>

#include "cycfg.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "memory.h"
#include "global_management.h"
#include "FRAM_DIRECT_SPI/FRAM_DIRECT_SPI.h"

/*******************************************************************************
 * DEFINITIONS
*******************************************************************************/
#define PRINTF_MEMORY(p_frm, ...)						printf(p_frm, ##__VA_ARGS__)
#define PRINTF_MEMORY_SYSTEM_DATA_UPDATE(p_frm, ...)	printf(p_frm, ##__VA_ARGS__)
#define PRINTF_MEMORY_RT_DATA_UPDATE(p_frm, ...)		//printf(p_frm, ##__VA_ARGS__)
#define PRINTF_MEMORY_RT_DATA_UPDATE_RATE(p_frm, ...)	//printf(p_frm, ##__VA_ARGS__)

/*******************************************************************************
 * VARIABLES
*******************************************************************************/
uint8_t isMemoryValid = 0;
uint8_t memoryLastStoredSoC = 0;
uint8_t memoryLastStoredSoH = 0;

uint16_t nextMemoryRtLineAddressOffset = 0;
uint32_t lastRtDataLineWrittenTimestamp = 0;
int64_t  accumulatedRtDataCapacity_integer = 0;
uint8_t  memoryRtDataLineBuffer[MEM_SIZE_RT_DATA];	// Buffer for one line used to transmit RT data via CAN


/*******************************************************************************
* FUNCTIONS
*******************************************************************************/

//****************************************************************************
// init_memory - Startup and setup of the memory IC
//****************************************************************************
void init_memory(){
	PRINTF_MEMORY("Init Memory CY15B256\r\n");

	// Init FRAM pins
	FRAM_init();

	// Check if stored data is valid. If the byte in MEM_ADDR_STORED_MEMORY_VALID is set to MEM_COMP_STORED_MEMORY_VALID, the memory is considered valid (not the first write and simple check if memory working)
	uint8_t memory_valid_byte = 0;
	memoryRead(&memory_valid_byte, MEM_ADDR_STORED_MEMORY_VALID, MEM_SIZE_STORED_MEMORY_VALID);
	if(memory_valid_byte != MEM_COMP_STORED_MEMORY_VALID){
		// If byte was not valid write correct valid byte
		PRINTF_MEMORY("\tMemory invalid (stored value %d), will try to rewrite memory\r\n", memory_valid_byte);
		isMemoryValid = 0;
		memory_valid_byte = MEM_COMP_STORED_MEMORY_VALID;
		memoryWrite(&memory_valid_byte, MEM_ADDR_STORED_MEMORY_VALID, MEM_SIZE_STORED_MEMORY_VALID);
	}
	else{
		isMemoryValid = 1;
		PRINTF_MEMORY("\tMemory valid (stored value %d)\r\n", memory_valid_byte);

		// Get stored system data
		memoryRead(&memoryLastStoredSoC, MEM_ADDR_LAST_SOC, MEM_SIZE_LAST_SOC);
		memoryRead(&memoryLastStoredSoH, MEM_ADDR_LAST_SOH, MEM_SIZE_LAST_SOH);
		PRINTF_MEMORY_SYSTEM_DATA_UPDATE("\t Last SoC = %d\r\n", memoryLastStoredSoC);
		PRINTF_MEMORY_SYSTEM_DATA_UPDATE("\t Last SoH = %d\r\n", memoryLastStoredSoH);
	}

	// Set initial RT data sampling time
	memoryRtDataRefreshTime = MEMORY_RT_DATA_DEF_REFRESH_TIME;

	// Set initial address offset where the next line must be stored from memory
	memoryRead((uint8_t*)&nextMemoryRtLineAddressOffset, MEM_ADDR_NEXT_RT_DATA_ADDR, MEM_SIZE_NEXT_RT_DATA_ADDR);

	// Reset RT data offset to 0 if system is started in a system slot
	if(statusBMS.slotState == SLOT_IN_SYSTEM_1 || statusBMS.slotState == SLOT_IN_SYSTEM_2){
		memoryResetRtData();
	}
}

//****************************************************************************
// memoryResetRtData - Resets the current line counter to zero which will restart measurement and overwrite data
//****************************************************************************
void memoryResetRtData(){
	nextMemoryRtLineAddressOffset = 0; //MEM_ADDR_RT_DATA_STOP-MEM_SIZE_RT_DATA-100; // Test stop at end of FRAM
	memoryWrite((uint8_t*)&nextMemoryRtLineAddressOffset, MEM_ADDR_NEXT_RT_DATA_ADDR, MEM_SIZE_NEXT_RT_DATA_ADDR);
	memoryRtDataRefreshTimestamp = TIMER_COUNTER_1MS;
	PRINTF_MEMORY("\tMemory reset RT data offset. Recording will start from 0\r\n");
}

//****************************************************************************
// memoryGetRtData - Receive a given number of lines numLines starting from an index dataStartLineIdx and store them in the dataArray or prints them to debug.
//					 If onlyPrintData is set to 1 or the dataArray is NULL the lines data will be printed to debug in CSV format.
//					 NOTE: When using this function to get data, make sure the array behind size of dataArray is at least numLines.
//****************************************************************************
void memoryGetRtData(memoryRtData_struct* dataArray, uint16_t numLines, uint16_t dataStartLineIdx, uint8_t onlyPrintData){
	uint8_t receiveBuf[10];
	memoryRtData_struct printRtStruct;
	memoryRtData_struct* usedRtDataStruct;
	uint16_t arrayIndex = 0;
	uint8_t arrayIndexIncrement = 1;

	// If requested or if passed data array was null, print Header for CSV and prepare to not store data externally
	if(onlyPrintData || dataArray == NULL){
		printf("Line;TimeDiff;CapacityDiff;BlockVoltage;CellImbalance;TempAvg\r\n");
		usedRtDataStruct = &printRtStruct;
		arrayIndexIncrement = 0;
	}
	// Else use the given array
	else{
		usedRtDataStruct = dataArray;
		arrayIndexIncrement = 1;
	}

	// Read and store every line
	for(uint16_t i = 0; i < numLines; i++){
		// Read line from memory
		memoryRead(receiveBuf, MEM_ADDR_RT_DATA_START + ((dataStartLineIdx+i)*MEM_SIZE_RT_DATA), 10);
		// Store it
		usedRtDataStruct[arrayIndex].TimeDiff = ((uint16_t*)&receiveBuf[MEM_INDX_RT_DATA_LINE_TIME_DIFF])[0];
		usedRtDataStruct[arrayIndex].CapacityDiff = (( int16_t*)&receiveBuf[MEM_INDX_RT_DATA_LINE_CAPACITY_DIFF])[0];
		usedRtDataStruct[arrayIndex].BlockVoltage = ((uint16_t*)&receiveBuf[MEM_INDX_RT_DATA_LINE_BLOCK_VOLTAGE])[0];
		usedRtDataStruct[arrayIndex].CellImbalance = ((uint16_t*)&receiveBuf[MEM_INDX_RT_DATA_LINE_CELL_IMBAL])[0];
		usedRtDataStruct[arrayIndex].TempAvg = (( int16_t*)&receiveBuf[MEM_INDX_RT_DATA_LINE_TEMP_AVG])[0];

		// Print CSV line if requested
		if(onlyPrintData){
			printf("%5d;%5u;%5i;%5u;%5u;%5i\r\n", (dataStartLineIdx+i), usedRtDataStruct[arrayIndex].TimeDiff, usedRtDataStruct[arrayIndex].CapacityDiff, usedRtDataStruct[arrayIndex].BlockVoltage, usedRtDataStruct[arrayIndex].CellImbalance, usedRtDataStruct[arrayIndex].TempAvg);
		}

		// Increment index is used with external array
		arrayIndex += arrayIndexIncrement;
	}

}

//****************************************************************************
// memoryGetRtData_byteArray - Receive a given number of lines numLines starting from an index dataStartLineIdx and store them as they are in the memory in the receiveBuf.
//					 		   IMPORTANT NOTE: When using this function, make sure receiveBuf is at least numLines*MEM_SIZE_RT_DATA big or the data will overflow.
//****************************************************************************
void memoryGetRtData_byteArray(uint8_t* receiveBuf, uint16_t numLines, uint16_t dataStartLineIdx){
	memoryRead(receiveBuf, MEM_ADDR_RT_DATA_START + (dataStartLineIdx*MEM_SIZE_RT_DATA), numLines*MEM_SIZE_RT_DATA);
}

//****************************************************************************
// manage_memory - Continuous check and write to memory function
//****************************************************************************
void manage_memory(){
	// DEBUG: Write SOH to be 100%
	//statusBMS.SoH = 100.0; memoryLastStoredSoH = (uint8_t)statusBMS.SoH; memoryWrite(&memoryLastStoredSoH, MEM_ADDR_LAST_SOH, MEM_SIZE_LAST_SOH);

	/// Store system data (settings and status) if needed
	if(TIMER_COUNTER_1MS > (memorySystemDataRefreshTimestamp + MEMORY_SYSTEM_DATA_REFRESH_TIME)){
		memorySystemDataRefreshTimestamp = TIMER_COUNTER_1MS;

		// Update variables if needed
		if(((uint8_t)statusBMS.SoC) != memoryLastStoredSoC){
			memoryLastStoredSoC = (uint8_t)statusBMS.SoC;
			memoryWrite(&memoryLastStoredSoC, MEM_ADDR_LAST_SOC, MEM_SIZE_LAST_SOC);
			PRINTF_MEMORY_SYSTEM_DATA_UPDATE("\t Update memory: SoC = %3u. \r\n", memoryLastStoredSoC);
		}
		if(((uint8_t)statusBMS.SoH) != memoryLastStoredSoH){
			memoryLastStoredSoH = (uint8_t)statusBMS.SoH;
			memoryWrite(&memoryLastStoredSoH, MEM_ADDR_LAST_SOH, MEM_SIZE_LAST_SOH);
			PRINTF_MEMORY_SYSTEM_DATA_UPDATE("\t Update memory: SoH = %3u. \r\n", memoryLastStoredSoH);
		}
	}

	/// Calculate dynamic sampling time
	//  How fast can we sample to not run out of memory before the battery is empty, based on remaining battery runtime/current
	if(memoryRtDataRefresh_isTimeFixed == 0){
		memoryRtDataRefreshTime = batteryState.remaining_runtime_ms / (uint32_t)(MEM_SIZE_MAX_LINES-(nextMemoryRtLineAddressOffset/MEM_SIZE_RT_DATA));
	}
	// If BMS is not discharging always used fixed time
	else if(memoryRtDataRefreshTime == 0){
		memoryRtDataRefresh_isTimeFixed = 1;
		memoryRtDataRefreshTime = MEMORY_RT_DATA_DEF_REFRESH_TIME;
	}
	// Limit refresh rate only for dynamic recording
	if(memoryRtDataRefreshTime > MEMORY_RT_DATA_MAX_REFRESH_TIME && memoryRtDataRefresh_isTimeFixed == 0) memoryRtDataRefreshTime = MEMORY_RT_DATA_MAX_REFRESH_TIME;
	if(memoryRtDataRefreshTime < MEMORY_RT_DATA_MIN_REFRESH_TIME && memoryRtDataRefresh_isTimeFixed == 0) memoryRtDataRefreshTime = MEMORY_RT_DATA_MIN_REFRESH_TIME;


	/// Store real time data if refresh time exceeded, at least one measurement was in between and the system is not in precharge phase
	if(rt_data_time_since_last_record >= memoryRtDataRefreshTime && statusBMS.switchState == SWITCH_IDLE){
	//if(TIMER_COUNTER_1MS > (memoryRtDataRefreshTimestamp + memoryRtDataRefreshTime) && rt_data_time_since_last_record != 0 && statusBMS.switchState == SWITCH_IDLE){
		memoryRtDataRefreshTimestamp = TIMER_COUNTER_1MS;

		// Read address where the next line must be stored from memory
		memoryRead((uint8_t*)&nextMemoryRtLineAddressOffset, MEM_ADDR_NEXT_RT_DATA_ADDR, MEM_SIZE_NEXT_RT_DATA_ADDR);

		// Check if address offset is plausible and stop recording
		if(MEM_ADDR_RT_DATA_START+nextMemoryRtLineAddressOffset > (MEM_ADDR_RT_DATA_STOP-MEM_SIZE_RT_DATA)){
			//PRINTF_MEMORY("\t RT data address offset %5u is to high. Memory is full - will stop recording RT data!\r\n, ", nextMemoryRtLineAddressOffset);
			//memoryRtDataRefreshTimestamp = UINT32_MAX - memoryRtDataRefreshTime;
			return;

			//PRINTF_MEMORY_RT_DATA_UPDATE("\t RT data address offset %d was invalid and is set to 0!\r\n, ", nextMemoryRtLineAddressOffset);
			//nextMemoryRtLineAddressOffset = 0;
		}
		//PRINTF_MEMORY_RT_DATA_UPDATE("\t Update RT data line %5u: Address %5u with offset %5u, ", nextMemoryRtLineAddressOffset/MEM_SIZE_RT_DATA, MEM_ADDR_RT_DATA_START+nextMemoryRtLineAddressOffset, nextMemoryRtLineAddressOffset);


		/// Calculate time difference ******
		//    - It will only be updated when measurement of current is finished (triggered on interval of analog frontend TLE9012_MEASURE_TIME or TLE9012_MEASURE_CHARGE_TIME).
		uint32_t lastRtDataLineWrittenTime = rt_data_time_since_last_record; //TIMER_COUNTER_1MS - lastRtDataLineWrittenTimestamp;
		// Show a Time difference of 0 for the first data line signalizes that the system was restarted
		if(lastRtDataLineWrittenTimestamp == 0){
			lastRtDataLineWrittenTime = 0;
			PRINTF_MEMORY_RT_DATA_UPDATE("lastRtDataLineWrittenTime = %ld\r\n", lastRtDataLineWrittenTime);
		}
		// If fixed sampling time is used only store the time offset from the set time. This way the interval can be much higher
		else if(memoryRtDataRefresh_isTimeFixed == 1){
			//printf("lastRtDataLineWrittenTime %ld - memoryRtDataRefreshTime %ld = ", lastRtDataLineWrittenTime, memoryRtDataRefreshTime);
			lastRtDataLineWrittenTime = lastRtDataLineWrittenTime - memoryRtDataRefreshTime;
			//printf("%ld\r\n", lastRtDataLineWrittenTime);
		}
		else{
			// Limit time to size of used variable -> result would still be unusable, but defect lines can be detected this way!
			if(lastRtDataLineWrittenTime > UINT16_MAX) lastRtDataLineWrittenTime = UINT16_MAX;
		}
		lastRtDataLineWrittenTimestamp = TIMER_COUNTER_1MS;

		/// Calculate capacity difference based on measurement ******
		//    - It will only be updated when measurement of current is finished (triggered on interval of analog frontend TLE9012_MEASURE_TIME or TLE9012_MEASURE_CHARGE_TIME). Note that this limits the sampling time to the measurement
		// 	  - Due to float to integer conversion a part of the value is always lost which is compensated by adding the remainder the next time data is recorded.
		//	  - The data is multiplied by 1000 to move 3 decimals from floating to fixed point representation
		float rtDataCapacityDiff = rt_data_charge_since_last_record + rt_Data_dC_lostDecimals;
		int32_t rtDataCapacityDiff_integer = (int32_t)((rtDataCapacityDiff)*MEMORY_RT_DATA_C_DIF_MULTIPLIER);
		// Limit time to size of used variable -> result would still be unusable, but defect lines can be detected this way!
		if(rtDataCapacityDiff_integer > INT16_MAX) rtDataCapacityDiff_integer = INT16_MAX;
		// Calculate remainder to be added next time
		rt_Data_dC_lostDecimals = rtDataCapacityDiff - ((float)rtDataCapacityDiff_integer)/MEMORY_RT_DATA_C_DIF_MULTIPLIER;

		//printf(" < dt %lu, I %.3f, +%ld, C %.3f> ", lastRtDataLineWrittenTime, ((float)rtDataCapacityDiff_integer/1000.0)/((float)lastRtDataLineWrittenTime/1000.0), rtDataCapacityDiff_integer, (batteryState.capacity_init - batteryState.capacity_t));
		PRINTF_MEMORY_RT_DATA_UPDATE("fDiff %10.5f = %10.5f+%.5f, iDiff %5ld, ", rtDataCapacityDiff, rt_data_charge_since_last_record, rt_Data_dC_lostDecimals, rtDataCapacityDiff_integer);

		// Reset time and charge counter between records
		rt_data_time_since_last_record = 0;
		rt_data_charge_since_last_record = 0.0;

		// Store data
		uint8_t transmitBuf[10];
		((uint16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_TIME_DIFF])[0] 	  = (uint16_t)lastRtDataLineWrittenTime;
		(( int16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_CAPACITY_DIFF])[0] = (uint16_t)rtDataCapacityDiff_integer;
		((uint16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_BLOCK_VOLTAGE])[0] = (uint16_t)(statusBMS.voltage *1000.0);
		((uint16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_CELL_IMBAL])[0] 	  = statusBMS.imbalance;
		(( int16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_TEMP_AVG])[0] 	  = (int16_t)(statusBMS.avgTemp *100.0);
		memoryWrite(transmitBuf, MEM_ADDR_RT_DATA_START+nextMemoryRtLineAddressOffset, MEM_SIZE_RT_DATA);
		PRINTF_MEMORY_RT_DATA_UPDATE("\tTime %02X %02X = %3dms - CapDiff %02X %02X = %5dAs/1000 - Volt %02X %02X = %5umV - Imbal %02X %02X = %5umV - TempAvg %02X %02X = %5dC*100, \r\n", transmitBuf[0], transmitBuf[1], ((uint16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_TIME_DIFF])[0], transmitBuf[2], transmitBuf[3], ((int16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_CAPACITY_DIFF])[0], transmitBuf[4], transmitBuf[5], ((uint16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_BLOCK_VOLTAGE])[0], transmitBuf[6], transmitBuf[7], ((uint16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_CELL_IMBAL])[0], transmitBuf[8], transmitBuf[9], (( int16_t*)&transmitBuf[MEM_INDX_RT_DATA_LINE_TEMP_AVG])[0]);

		// DEBUG Read written memory
		//memoryRtData_struct dataA;
		//memoryGetRtData(&dataA, 1, nextMemoryRtLineAddressOffset/MEM_SIZE_RT_DATA, 1);

		// Write address where the next line must be stored to memory
		nextMemoryRtLineAddressOffset += MEM_SIZE_RT_DATA;
		//PRINTF_MEMORY_RT_DATA_UPDATE("Next Address %5d\r\n", nextMemoryRtLineAddressOffset);
		memoryWrite((uint8_t*)&nextMemoryRtLineAddressOffset, MEM_ADDR_NEXT_RT_DATA_ADDR, MEM_SIZE_NEXT_RT_DATA_ADDR);

		// Print Refresh rate
		PRINTF_MEMORY_RT_DATA_UPDATE_RATE("\tRemaining runtime   (%.2fh %.2fmin %.2fms) = %.2fAs/%.2fA*1000 (rem cap = %.2fAs)\r\n", (float)batteryState.remaining_runtime_ms/1000.0/60.0/60.0, (float)batteryState.remaining_runtime_ms/1000.0/60.0, (float)batteryState.remaining_runtime_ms, batteryState.capacity_usable_remaining, (statusBMS.current)/1000.0, (batteryState.capacity_init - batteryState.capacity_t));
		PRINTF_MEMORY_RT_DATA_UPDATE_RATE("\tRefresh rate %ld ms (dynamic %.0lfms/%ldlines remaining)\r\n", memoryRtDataRefreshTime, (float)batteryState.remaining_runtime_ms, (uint32_t)MEM_SIZE_MAX_LINES-(nextMemoryRtLineAddressOffset/MEM_SIZE_RT_DATA));
	}
}


//****************************************************************************
// Functions to directly write to FRAM (size limited to 128 byte by SPI packet size - see datasheet)
//****************************************************************************
void memoryWrite(uint8_t* wData, uint32_t startAddress, uint32_t datasize){
	FRAM_WREN();
	FRAM_Write(wData, startAddress, datasize);
}
void memoryRead(uint8_t* rData, uint32_t startAddress, uint32_t datasize){
	FRAM_Read(rData, startAddress, datasize);
}

//****************************************************************************
// Functions to write to FRAM in pages (no size limit, but slower) - Note these functions are untested but should work
//****************************************************************************
//void memoryWriteBurst(uint8_t *userData, uint32_t startAddress, uint32_t burstLen){
//	FRAM_Burst_Write(userData, startAddress, burstLen);
//}
//void memoryReadBurst(uint8_t *readData, uint32_t startAddress, uint32_t burstLen){
//	FRAM_Burst_Read(readData, startAddress, burstLen);
//}
