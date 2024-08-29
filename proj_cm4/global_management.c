/*
 * global_management.c - Abstraction off all settings and objects that can be accessed from every part of the system.
 *
 *  Created on: 1 Jun 2023
 *      Author: r. santeler
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <stdint.h>
#include <math.h>

#include "cycfg.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "ipc_def.h"
#include "cy_retarget_io.h"

#include "global_management.h"
#include "safety_switch.h"
#include "BMS_TLE9012/BMS_TLE9012.h"
#include "battery_state.h"

/*******************************************************************************
* Definitions
*******************************************************************************/
#define PRINTF_GLOBAL_DEBUG(p_frm, ...)		printf(p_frm, ##__VA_ARGS__)

//********************************************************************************
// SYSTEM STRUCTURES - "statusBMS & setupBMS" the initial state is to be set here!
//********************************************************************************
// The status of the whole system // Debug: set initial values to zero for release build - just for show case during development
volatile statusBMS_struct statusBMS = {
	.returnValue = 0,
	.bms_state = BMS_ERROR,
	.current = 0,
	.voltage = 0,
	.avgTemp = 0,
	.SoC = 0,
	.SoH = 0,
	.error_map = 0,
	.message_map = 0,
	.ol_map = 0,
	.uv_map = 0,
	.ov_map = 0,
	.bal_uc_map = 0,
	.bal_oc_map = 0,
	.imbalance = 0,
	.avgCellVolt_mV = 0,
	.bal_cell_map = 0,
	.measurementState = MEASUREMENT_WAIT_RESULT,
	.switchState = SWITCH_IDLE,
	.outputState = OUTPUT_OFF,
	.slotState = SLOT_NONE,
	.extLoadVoltage = 0.0,
	.hotSwapRole = 42,
	.giverHotSwapState = 0,
	.takerHotSwapState = 0,
	.voltageChangeRate_ms = 0
};
uint32_t latched_error_map_switch_on = 0;
uint32_t latched_error_map_ext_com = 0;
uint32_t latched_message_map = 0;

battery_state_struct batteryState = {
	.SoC_t0 = 0.0, 						// initial state of charge
	.SoC_t = 0.0,						// current state of charge
	.SoH_t0 = 0.0,						// initial state of health
	.SoH_t = 0.0,						// current state of health
	.DoD_t0 = 0.0,						// initial depth of discharge
	.DoD_t = 0.0,						// current depth of discharge
	.DoD_delta = 0.0,					// delta depth of discharge
	.voltage_init = 0.0,				// initial voltage
	.capacity_init = 0.0,				// initial capacity
	.capacity_t = 0.0,					// current capacity of the battery (summed)
	.last_state = BATTERY_OTHER,		// initial state (will be changed to charge or discharge at first state change)
	.init_capacity_SoH_at_thres = 0.0,	// initial capacity at state change (will be changed to charge or discharge at first state change)
	.capacity_usable_remaining = 0.0,	// in As. Amount of capacity that is still usable (see REMAINING_RUNTIME_SOC_PERCENT)
	.remaining_runtime_ms = 0,			// in ms. Momentary remaining ON time if the system keeps the currently drawn current till the end.
	.capacity_state_delta = 0.0,		// in As. The capacity change since last state change
	.capacity_range_SoH = BATTERY_SOH_CAPACITY_RANGE,		// in As. Amount of capacity that should change between SOH_UPPER_THRESHOLD and SOH_LOWER_THRESHOLD voltage to get 100% SoH (If zero it will be estimated from OCV curve. Write correct value after first estimation with expected current here)
	.SoH_upper_threshold_V = 0.0,		// in V. Will be estimated from OCV curve and SOH_UPPER_THRESHOLD at init_battery_state() (battery_state.h)
	.SoH_lower_threshold_V = 0.0		// in V. Will be estimated from OCV curve and SOH_LOWER_THRESHOLD at init_battery_state() (battery_state.h)
};

// Default setup values of the System (to be overwritten by load setup step)
volatile setupBMS_struct setupBMS ={
	.NumCells = NUM_CELLS,								// Number of cells in this cell pack
	.CellPackID = { 0xFF, 0xFF, 0xFF, 0xFF },			// First 4 bytes of the the BLE address that must be the same throughout all BLE devices in a pack
	.MeasCalib_Offset_I = 0,			    			// Offset of used for current calculation (if calibration is perfect this should not be needed)
	.LimitUnderVoltageCell = LIMIT_CELL_UNDER_VOLTAGE_mV,	// Undervoltage limit for each cell. If any cell goes under this the safety switch will be activated (MOSFETs blocking)
	.LimitOverVoltageCell  = LIMIT_CELL_OVER_VOLTAGE_mV,	// Overvoltage  limit for each cell. If any cell goes over this the safety switch will be activated (MOSFETs blocking)
	.LimitUnderVoltage = LIMIT_SYS_UNDER_VOLTAGE_mV,	// Undervoltage limit for the whole battery. If the pack voltage goes under this the safety switch will be activated (MOSFETs blocking)
	.LimitOverVoltage  = LIMIT_SYS_OVER_VOLTAGE_mV,		// Overvoltage  limit for the whole battery. If the pack voltage goes over this the safety switch will be activated (MOSFETs blocking)
	.BalVolt = BALANCING_DEACTIVATED_VALUE,				// Initial balancing voltage (reference voltage => lowest cell V in system)
	.BalLimit = BALANCING_CELL_TH_LIMIT,				// Initial balancing limit (cell starts balancing if the own V is BalLimit over BalTol)
	.BatteryRatedCapacity = BATTERY_RATED_CAPACITY		// Rated Capacity = 2.6Ah * 1Cells parallel * 3600 to get As
};


//********************************************************************************
// SYSTEM VARIABLES
//********************************************************************************
// Interrupt variables
volatile uint8_t errorInterruptDetected_2ED4820 = 0; // Safety switch driver error interrupt detection
volatile uint8_t errorInterruptDetected_TLE9012 = 0; // BMS analog front end error interrupt detection

/// Timestamps for time interval control
// Slot detection
uint32_t slotPlugDebounceTimestamp = 0;
// Memory
uint32_t memorySystemDataRefreshTimestamp = 0;
uint32_t memoryRtDataRefreshTimestamp = 0;
uint32_t memoryRtDataRefreshTime = 0;
uint8_t  memoryRtDataRefresh_isTimeFixed = MEMORY_RT_DATA_FIX_REFRESH_T;
// Safety switch
uint32_t switchPrechargeStartTimestamp = 0;
uint32_t lastSwitchOnTimestamp = 0;					// Time stamp when the switch where last tried to switch conducting
uint8_t  safety_switch_retry_counter = 0;
// Main loop intervals
uint32_t lastExternalCommunicationTimestamp = 0;	// Time stamp when the communication from and to the BMS was last managed
uint32_t lastBatterySateCalcTimestamp = 0;			// Time stamp when the battery state was last calculated
uint32_t canLastMessageTimestamp = 0;
uint32_t canSwitchOnVetoRefreshTimestamp = 0;
uint32_t canDataRefreshTimestamp = 0;
uint32_t shutdownTriggeredTimestamp = 0;
uint32_t batteryLowWarningTimestamp = 0;
uint32_t updatePublicDisplayDataTimestamp = 0;
// BMS Management IC TLE9012
uint32_t lastTLE9012MeasurementTimestamp = 0;			 // Time stamp when the measurement on the TLE9012 was last triggered
uint32_t lastTLE9012IntervalComErrorReduceTimestamp = 0; // Time stamp when numComErrors_TLE9012 was last reduced by 1 // See description of TLE9012_MAX_COM_ERRORS
// LED blink
uint16_t blinkTimeLedOrange = 0;
uint32_t blinkTimeLedRedTimestamp = 0;
// Hotswap
uint32_t giverHotSwapTimestamp = 0;
uint32_t takerHotSwapTimestamp = 0;
uint32_t switchOnVetoTimestamp = 0;
// ADC
uint32_t adcZeroingTimestamp = 0;
uint32_t adcExtLastReadSuccessTimestamp = 0;
uint32_t adcLastVoltageChangeRateTimestamp = 0;
// Deep discharge protection
uint32_t deepDischargeTimestamp = 0;

/// Flags
uint8_t isCurrentOffsetCalibrating = 0;
uint8_t isShutdownTriggered = 0;
uint8_t isTLE9012ResultReceivedThisCycle = 0;
uint8_t TriggerCurrentMeasurement = 1; // The trigger of the analog frontend TLE9012 measurement sets this flag and leads the measurement of the current, so that they are done simultaneously

/// TLE9012 com errors
uint16_t numComErrors_TLE9012 = 0;
requestResult tle9012_print_if_error_result;
QueueResult lastQueueResult_tle9012;

/// Watchdog
uint32_t mainLoopCounter = 0;
uint32_t mainLoopCounter_lastWatchdog = UINT32_MAX;
uint8_t isWatchdogActive = 0;
/// ADC
// External ADC
activeChannelExternelADCStates activeChannelExternalADC = 0;    // Currently checked channel of external ADC MCP3465 (default off)
float lastExtVoltageMeasurement = ADC_EXT_V_CHANGE_RATE_MAX;
// Internal ADC (coding resistor and low resolution current measurement)
int32_t adc_coding_res_mV;
int32_t adc_safety_switch_current_mV;
int32_t adc_safety_switch_current;

// RT data recording
uint32_t rt_data_time_since_last_record = 0;		// THIS IS ONLY UPDATED AFTER EACH CURRENT MEASUREMENT (TRIGGERED BY ANALOG FRONTEND MEASUREMENT)! If rt data is recorded faster than measurement this might be 0
float rt_data_charge_since_last_record = 0;	// THIS IS ONLY UPDATED AFTER EACH CURRENT MEASUREMENT (TRIGGERED BY ANALOG FRONTEND MEASUREMENT)! If rt data is recorded faster than measurement this might be 0
float rt_Data_dC_lostDecimals = 0;

/// Filter
#define FILTER_I_BUF_SIZE (SYSTEM_FILTER_INTERVAL_I+2)
int32_t   avgFilter_I_raw   [FILTER_I_BUF_SIZE] = { 0 }; // all elements 0
//float avgFilter_I_filter[FILTER_I_BUF_SIZE] = { 0.0 };
volatile avgFilter avgFilter_I = {
	.bufIdx = 0,
	.lastIdx = 0,
	.bufMaxIdx = FILTER_I_BUF_SIZE-1,
	.bufRaw    = (int32_t*)&avgFilter_I_raw,
	.filterResult = 0.0,
	.avgFilterInterval = SYSTEM_FILTER_INTERVAL_I,
	.avgFilterSum = 0.0,
	.initFilterCompesation = SYSTEM_FILTER_INTERVAL_I-1
};
#define FILTER_V_BUF_SIZE (SYSTEM_FILTER_INTERVAL_V+2)
int32_t   avgFilter_V_raw   [FILTER_V_BUF_SIZE] = { 0 }; // all elements 0
//float avgFilter_V_filter[FILTER_V_BUF_SIZE] = { 0.0 };
volatile avgFilter avgFilter_V = {
	.bufIdx = 0,
	.lastIdx = 0,
	.bufMaxIdx = FILTER_V_BUF_SIZE-1,
	.bufRaw    = (int32_t*)&avgFilter_V_raw,
	.filterResult = 0.0,
	.avgFilterInterval = SYSTEM_FILTER_INTERVAL_V,
	.avgFilterSum = 0.0,
	.initFilterCompesation = SYSTEM_FILTER_INTERVAL_V-1
};
#define FILTER_IMBALANCE_BUF_SIZE (SYSTEM_FILTER_INTERVAL_IMBALANCE+2)
int32_t   avgFilter_imbalance_raw   [FILTER_IMBALANCE_BUF_SIZE] = { 0 }; // all elements 0
//float avgFilter_V_filter[FILTER_V_BUF_SIZE] = { 0.0 };
volatile avgFilter avgFilter_Imbalance = {
	.bufIdx = 0,
	.lastIdx = 0,
	.bufMaxIdx = FILTER_IMBALANCE_BUF_SIZE-1,
	.bufRaw    = (int32_t*)&avgFilter_imbalance_raw,
	.filterResult = 0.0,
	.avgFilterInterval = SYSTEM_FILTER_INTERVAL_IMBALANCE,
	.avgFilterSum = 0.0,
	.initFilterCompesation = SYSTEM_FILTER_INTERVAL_IMBALANCE-1
};
#define FILTER_TEMPCELL_BUF_SIZE (SYSTEM_FILTER_INTERVAL_TEMPCELL+2)
int32_t   avgFilter_tempCell_raw   [FILTER_TEMPCELL_BUF_SIZE] = { 0 }; // all elements 0
volatile avgFilter avgFilter_tempCell = {
	.bufIdx = 0,
	.lastIdx = 0,
	.bufMaxIdx = FILTER_TEMPCELL_BUF_SIZE-1,
	.bufRaw    = (int32_t*)&avgFilter_tempCell_raw,
	.filterResult = 0.0,
	.avgFilterInterval = SYSTEM_FILTER_INTERVAL_TEMPCELL,
	.avgFilterSum = 0.0,
	.initFilterCompesation = SYSTEM_FILTER_INTERVAL_TEMPCELL-1
};


//********************************************************************************
// SYSTEM FUNCTIONS
//********************************************************************************

//****************************************************************************
// measure_movAvgFilter_fast - Implementation of an moving average filter on an
// ring-buffer. This version is used for fast execution.
//****************************************************************************
void measure_movAvgFilter_fast(volatile avgFilter* filter, int32_t newValue){
    // Increment current Buffer index and set back to 0 if greater than size of array
    filter->bufIdx++;
    if(filter->bufIdx > filter->bufMaxIdx)
        filter->bufIdx = 0;

    // Set new value in buffer
    filter->bufRaw[filter->bufIdx] = newValue;

    // Get index of oldest element, which shall be removed (current index minus filter interval with roll-over check)
    int32_t oldIdx = (int32_t)filter->bufIdx - (int32_t)filter->avgFilterInterval;
    if(oldIdx < 0) oldIdx += filter->bufMaxIdx+1;

    // Subtract oldest element and add newest to sum
    filter->avgFilterSum += (float)filter->bufRaw[filter->bufIdx];
    filter->avgFilterSum -= (float)filter->bufRaw[oldIdx];

    // Calculate average (at system init the buffer is empty and the divider must be compensated till buffer is filled the first time)
    if(filter->initFilterCompesation > 0){
        filter->filterResult = filter->avgFilterSum / (float)(filter->avgFilterInterval - filter->initFilterCompesation);
        filter->initFilterCompesation--;
    }
    else{
        filter->filterResult = filter->avgFilterSum / (float)filter->avgFilterInterval;
    }
}

//****************************************************************************
// mapRangeToScale - Converts a range of numbers to a percentage scale
//****************************************************************************
float mapRangeToScale(float n, float lRange, float hRange, float scale){
	// n       number to convert
	// lRange  lowest number of the range
	// hRange  highest number in the range
	// scale   percentage scale

	// Input validation
	if (n < lRange || n > hRange) { return -1.0; }

	// Map to scale
	return (n - lRange)/(hRange - lRange) * scale;
}

//****************************************************************************
// printBinary - Print a value in binary. Use mask to extract certain bits.
//****************************************************************************
void printBinary(uint32_t data, uint32_t mask, uint8 sizeBits){
	/// Print data in binary
	// Get the MSB of mask
	uint32_t compare = 1 << (sizeBits-1); //0b1000000000000000;
	// Run backward through bits and print bit if it is in mask
	for(int8_t numBit = sizeBits-1; numBit >= 0; numBit--){
		// Extract bit
		uint32_t selectedBit = compare & mask;

		// If the current bit is part of mask
		if( selectedBit >= 1){
			// If bit is a 1 write a one
			if((data & selectedBit) == 0){
				PRINTF_GLOBAL_DEBUG("0");
			}
			else{
				PRINTF_GLOBAL_DEBUG("1");
			}
		}

		// Double compare to select next bit
		compare = compare>>1;
	}
}

//****************************************************************************
// polyEvalLinear - Convert a adc value with a polynomial of second order
// TODO: Currently this function can only be used for ADC_I_DRIVER_CONV. Make coefficients as parameter if this is needed more often
//****************************************************************************
float polyEvalLinear(float acd_reading)
{
	register float result = (ADC_I_DRIVER_CONV_P1 * acd_reading) + ADC_I_DRIVER_CONV_P0;
	register float powtemp = acd_reading * acd_reading;

	result += powtemp * ADC_I_DRIVER_CONV_P2;
//	powtemp = powtemp * acd_reading;
//	result += powtemp * ADC_I_DRIVER_CONV_P3;
//	powtemp = powtemp * acd_reading;
//	result += powtemp * ADC_I_DRIVER_CONV_P4;
//	powtemp = powtemp * acd_reading;
//	result += powtemp * ADC_I_DRIVER_CONV_P5;
//	powtemp = powtemp * acd_reading;
//	result += powtemp * ADC_I_DRIVER_CONV_P6;
//	powtemp = powtemp * acd_reading;
//	result += powtemp * ADC_I_DRIVER_CONV_P7;

	return result;
}
