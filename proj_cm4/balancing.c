/*
 * balancing.c - Abstraction off all battery balancing related functions that need to be accessed from main.
 * 				 The implemented algorithm sorts the cells by voltage and returns a threshold (determineBalancingThreshold()).
 * 				 All cells above this must be balanced. The function manage_balancing() determines the a cell map, which is used in main.
 *
 *  Created on: 23 May 2024
 *      Author: r. santeler
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <stdio.h>
#include <math.h>

#include "cycfg.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "global_management.h"

/*******************************************************************************
 * DEFINITIONS
*******************************************************************************/
#define PRINTF_BALANCING(p_frm, ...)		printf(p_frm, ##__VA_ARGS__)

/*******************************************************************************
 * VARIABLES AND STRUCTS
*******************************************************************************/
typedef struct {
	uint8_t idx;		// Index
	int16_t voltage;	// Voltage in mV (Range -32.768 to 32.767 Volt)
} sortBB_struct;


/*******************************************************************************
* FUNCTIONS
*******************************************************************************/

// Function to swap cell structure items
void swap(sortBB_struct* xp, sortBB_struct* yp) {
	sortBB_struct temp;

	// Cache xp info to temp
	temp.idx = xp->idx;
	temp.voltage = xp->voltage;
	// Copy yp to xp
	xp->idx = yp->idx;
	xp->voltage = yp->voltage;
	// Copy xp to yp with temp
	yp->idx = temp.idx;
	yp->voltage = temp.voltage;
}

// Function to prepare selection sort objects
void prepareCellSort(uint16_t* BBs, sortBB_struct* sortBB, uint8_t n){
	static uint8_t initDone = 0;

	// At first execution initialize struct
	if(initDone == 0){
		for (uint8_t i = 0; i < n; i++) {
			sortBB[i].idx = i;
			sortBB[i].voltage = 0;
		}
	}

	// Get current voltage of each cell
	for (uint8_t i = 0; i < n; i++) {
		sortBB[i].voltage = (int16_t)BBs[sortBB[i].idx];
	}
}

// Function to perform selection Sort
void selectionCellSort(sortBB_struct* sortBB, uint8_t n) {
    int i, j, min_idx;

    // One by one move boundary of unsorted subarray
    for (i = 0; i < n - 1; i++) {

        // Find the minimum element in unsorted array
        min_idx = i;
        for (j = i + 1; j < n; j++)
            if (sortBB[j].voltage < sortBB[min_idx].voltage)
                min_idx = j;

        // Swap the found minimum element with the first element
        swap(&sortBB[min_idx], &sortBB[i]);
    }
}

//****************************************************************************
// determineBalancingThreshold - Calculate current balancing threshold and return lowest cell voltage
//****************************************************************************
uint16_t determineBalancingThreshold(uint16_t* batteryArray){
	// Array of cell addresses sorted by voltage
	static sortBB_struct sortBB[NUM_CELLS];

	// Recalculate cell imbalance at a given rate and filter it to minimize flicker
	static uint32_t lastImbalanceCalcTimestamp = 0;
	if((TIMER_COUNTER_1MS - lastImbalanceCalcTimestamp) >= BALANCING_REFRESH_TIME){
		// Store current time as last refresh time
		lastImbalanceCalcTimestamp = TIMER_COUNTER_1MS;

		// Prepare sort (store current voltage values in sortBB) and sort cells
		prepareCellSort(batteryArray, sortBB, setupBMS.NumCells);
		selectionCellSort(sortBB, setupBMS.NumCells);

		// Filter imbalance ad store it
		measure_movAvgFilter_fast(&avgFilter_Imbalance, sortBB[setupBMS.NumCells-1].voltage - sortBB[0].voltage);
		statusBMS.imbalance = (uint16_t)roundf(avgFilter_Imbalance.filterResult);

		// Note for future development: If the system should only balance a certain number B of cells at a time, a slope must be implemented to raise the BalVolt to the cell voltage of the B'est cell in sorted array

		// Debug: Print info
		static uint8_t i = 0;
		if(i % 15 == 0){
			PRINTF_BALANCING("%4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\r\n", sortBB[0].idx, sortBB[1].idx, sortBB[2].idx, sortBB[3].idx, sortBB[4].idx, sortBB[5].idx, sortBB[6].idx, sortBB[7].idx, sortBB[8].idx, sortBB[9].idx, sortBB[10].idx, sortBB[11].idx);
			PRINTF_BALANCING("%4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\r\n", sortBB[0].voltage, sortBB[1].voltage, sortBB[2].voltage, sortBB[3].voltage, sortBB[4].voltage, sortBB[5].voltage, sortBB[6].voltage, sortBB[7].voltage, sortBB[8].voltage, sortBB[9].voltage, sortBB[10].voltage, sortBB[11].voltage);
			//PRINTF_BALANCING("Min/Max Diff: %dmV, State: %d\r\n", statusBMS.imbalance, statusBMS.bms_state);
		}
		i++;
	}

	// If system is charging and balancing requirements are met calculate balancing parameters
	if(statusBMS.bms_state == BMS_CHARGING && statusBMS.imbalance > BALANCING_MIN_MAX_THRESHOLD){
		setupBMS.BalVolt = sortBB[11].voltage;
	}
	// Else deactivate balancing by setting the value very high
	else{
		setupBMS.BalVolt = BALANCING_DEACTIVATED_VALUE;
	}

	// Return calculated threshold
	return setupBMS.BalVolt;
}

//****************************************************************************
// manage_balancing - Check which cells need to be balanced. NOTE: The actual
// 					 write of the map is inside processMeasurement() because
//					 every measurement will stop balancing, and must be
//                   written again as fast as possible afterwards.
//****************************************************************************
void manage_balancing(){
	requestResult tle9012_result;

	// The calculation which cells must be balanced is executed after each TLE9012 measurement
	if(isTLE9012ResultReceivedThisCycle == 1){
		// Determine balancing state and get lowest cell voltage
		uint16_t lowestCellVoltage = determineBalancingThreshold(tle9012_nodes[0].cell_voltages);

		/// BALANCING TLE9012:
		// Balancing is only necessary in charging mode, when the register write isn't already queued and the minimal cell voltage is not below threshold
		if( statusBMS.bms_state == BMS_CHARGING &&
		  ( checkRegisterQueued_tle9012(1, BAL_SETTINGS, 0) == 0) &&
		  ( lowestCellVoltage > setupBMS.LimitUnderVoltageCell))
		{
			// BALANCING ALGORITHM: Mark every cell which voltage is between the highest (BalVolt) and a offset limit (BalLimit)
			uint16_t bal_cell_map_new = 0;
			for(uint8_t i = 0; i < NUM_CELLS; i++){
				if(tle9012_nodes[0].cell_voltages[i] > (setupBMS.BalVolt-setupBMS.BalLimit) && tle9012_nodes[0].cell_voltages[i] > setupBMS.LimitUnderVoltageCell){
					bal_cell_map_new |= 1 << i;
				}
				// DEBUG: BALANCE EVERY CELL
				//statusBMS.bal_cell_map |= 1 << i;
			}

			// Store map of cells marked to balance
			statusBMS.bal_cell_map = bal_cell_map_new;
		}
		// If system is not charging and balancing settings are not 0 deactivate balancing
		else if ((getRegisterCacheDataByIndex_tle9012(1, BAL_SETTINGS) != 0) | ((statusBMS.message_map & (uint16)MSG_CAF_BAL_ACTIVE) != 0)){
			PRINTF_BALANCING("Reset balancing bitmap 0 \r\n");
			statusBMS.bal_cell_map = 0;
			tle9012_result = tle9012_writeRegisterByIndex(1, BAL_SETTINGS, 0, 0, 0);
			if(tle9012_result >= REQUEST_ERROR){ PRINTF_BALANCING("[ERROR]: Deactivate balancing returned %d\r\n", tle9012_result);}
			tle9012_result = tle9012_readRegisterByIndex(1, BAL_SETTINGS, 0, 0);
			if(tle9012_result >= REQUEST_ERROR){ PRINTF_BALANCING("[ERROR]: Read balancing register returned %d\r\n", tle9012_result);}
		}
	}
}
