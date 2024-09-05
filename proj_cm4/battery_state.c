/*
 * battery_state.c - Abstraction off all battery state related functions that need to be accessed from main.
 * 				 	 The implemented algorithm is a simple implementation of coloumb counting based on code from p. temsamani
 * 				 	 To implement other algorithms rewrite init_battery_state() to be executed at the start of the system,
 * 				 	 manage_battery_state to be executed every main loop cycle and
 * 				 	 update_battery_state executed after every current measurement.
 *
 *  Created on: 24 Nov 2022
 *      Author: r. santeler, (p. temsamani)
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/


#include <math.h>
#include <stdint.h>

#include "cycfg.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "battery_state.h"
#include "memory.h"
#include "global_management.h"

//********************************************************************************
// DEFINITIONS
//********************************************************************************
#define PRINTF_BATTERY_STATE(p_frm, ...)						printf(p_frm, ##__VA_ARGS__)
#define PRINTF_DEBUG_BTN_RESET(p_frm, ...)						printf(p_frm, ##__VA_ARGS__)

//********************************************************************************
// VARIABLES
//********************************************************************************
// Points on OCV curve: SoC[%], V_C[V]
float ocv_curve[OCV_POINTS][2] = {
	{0.5 ,  2.7},	// -> 0 mAh
	{1.269, 2.728},	// -> 44 mAh
	{2.300, 2.806},	// -> 81 mAh
	{4.511, 2.924},	// -> 158 mAh
	{7.826, 3.041},	// -> 274 mAh
	{9.742, 3.099},	// -> 341 mAh
	{11.068, 3.143},	// -> 387 mAh
	{12.689, 3.193},	// -> 444 mAh
	{14.531, 3.236},	// -> 509 mAh
	{18.657, 3.299},	// -> 653 mAh
	{24.478, 3.358},	// -> 857 mAh
	{29.562, 3.395},	// -> 1035 mAh
	{34.646, 3.434},	// -> 1213 mAh
	{39.361, 3.466},	// -> 1378 mAh
	{43.856, 3.499},	// -> 1535 mAh
	{48.350, 3.538},	// -> 1692 mAh
	{52.476, 3.575},	// -> 1837 mAh
	{57.192, 3.618},	// -> 2002 mAh
	{63.528, 3.672},	// -> 2223 mAh
	{67.360, 3.703},	// -> 2358 mAh
	{71.412, 3.727},	// -> 2499 mAh
	{75.023, 3.759},	// -> 2626 mAh
	{77.675, 3.785},	// -> 2719 mAh
	{80.991, 3.827},	// -> 2835 mAh
	{84.454, 3.861},	// -> 2956 mAh
	{87.622, 3.883},	// -> 3067 mAh
	{90.716, 3.896},	// -> 3175 mAh
	{93.000, 3.903},	// -> 3255 mAh
	{95.800, 3.924},	// -> 3353 mAh
	{97.421, 3.939},	// -> 3410 mAh
	{98.821, 3.963},	// -> 3459 mAh
	{99.0  , 4.15},		// -> 3495 mAh  // The last 3 Points are manually adapted because slope is steep at this point
	{99.4  , 4.18},		// ->
	{100.0 , 4.20},		// -> 3500 mAh

	//{93.000, 3.903},	// -> 3255 mAh
	//{95.800, 3.924},	// -> 3353 mAh
	//{97.421, 3.939},	// -> 3410 mAh
	//{98.821, 3.963},	// -> 3459 mAh
	//{99.632, 3.994},	// -> 3487 mAh
	//{99.853, 4.031},	// -> 3495 mAh
	//{100.00, 4.20},		// -> 3500 mAh
};


//********************************************************************************
// FUNCTIONS
//********************************************************************************
/******************************************************************
 *	init_battery_state - Used at the startup to initialize the SoC and SoH estimation
 *								capacity_rated 	... in As. The usable capacity of the connected battery (between over- and under-voltage condition)
 *								voltage_batt	... in V.  The current voltage of the whole battery (measured over the battery)
 *								numBB           ... The number of cells in he battery
 ******************************************************************/
float init_battery_state(battery_state_struct* bss, float capacity_rated, float voltage_batt, uint16_t numBB){
	// Measure initial battery voltage
	bss->voltage_init = voltage_batt;

	// Determine initial SoC - if usable use last stored SoH, if not set it to 100%
	float initalSoC_fromOCV = get_initial_SoC(bss->voltage_init, numBB, BATTERY_STATE_MINMAX_TOL);
	if((isMemoryValid == 1) && ((bss->voltage_init/(float)numBB) > SOC_UPPER_TH_USE_LAST_VALUE || (bss->voltage_init/numBB) < SOC_LOWER_TH_USE_LAST_VALUE) && (fabs(initalSoC_fromOCV-memoryLastStoredSoC) < SOC_LAST_TO_OCV_MAX_DIFF)){
		bss->SoC_t0 = (float)memoryLastStoredSoC;
		PRINTF_BATTERY_STATE("\t Using SoC = %.2f percent from FRAM (cell voltage %.2f was below %.2f or over %.2f)\r\n", bss->SoC_t0, bss->voltage_init/(float)numBB, SOC_LOWER_TH_USE_LAST_VALUE, SOC_UPPER_TH_USE_LAST_VALUE);
	}
	else{
		bss->SoC_t0 = initalSoC_fromOCV;
		PRINTF_BATTERY_STATE("\t Using SoC = %.2f percent from OCV curve (last SoC in FRAM invalid (valid=%d), cell voltage %.2f between %.2f and %.2f, or difference of stored %d to ocv %.0f SoC is higher than %.0f)\r\n", bss->SoC_t0, isMemoryValid, bss->voltage_init/numBB, SOC_LOWER_TH_USE_LAST_VALUE, SOC_UPPER_TH_USE_LAST_VALUE, memoryLastStoredSoC, initalSoC_fromOCV, SOC_LAST_TO_OCV_MAX_DIFF);
	}

	// Determine SoH - if usable use last stored SoH, if not set it to 100%
	if(isMemoryValid == 1){
		// SoH can never be 0
		if(memoryLastStoredSoH < 1)
			bss->SoH_t0 = SOH_INIT;
		else{
			// Used stored SoH
			bss->SoH_t0 = (float)memoryLastStoredSoH;
		}
		PRINTF_BATTERY_STATE("\t Using SoH = %.2f percent from FRAM\r\n", bss->SoH_t0);
	}
	else{
		// Initialize battery with 100% (as if battery is new, SoH = 100%)
		bss->SoH_t0 = SOH_INIT;
		PRINTF_BATTERY_STATE("\t Using SoH = %.2f percent (new) because last SoH in FRAM not valid %d\r\n", bss->SoH_t0, isMemoryValid);
	}

	// Reset SoH if debug button is pressed at startup
	if(cyhal_gpio_read(CYBSP_SWITCH) == 1){
		bss->SoH_t0 = SOH_INIT;
		memoryLastStoredSoH = (uint8_t)bss->SoH_t0;
		memoryWrite(&memoryLastStoredSoH, MEM_ADDR_LAST_SOH, MEM_SIZE_LAST_SOH);
		PRINTF_DEBUG_BTN_RESET("\t Debug button pressed - Reset SoH to %3u. \r\n", memoryLastStoredSoH);
	}

	// The initial SoC must never be 0
	if(bss->SoC_t0 < 1) bss->SoC_t0 = 1;

	// Initialize Depth of Discharge and related capacity
	bss->SoC_t  = bss->SoC_t0;
	bss->DoD_t0 = 100 - bss->SoC_t0; // DoD_t0 = SoH_t - SoC_t0;
	bss->capacity_init = (bss->SoC_t0 * capacity_rated) / 100;

	// SoH estimation: If capacity_range_SoH is not predefined, approximate it by upper/lower threshold and OCV curve
	if(bss->capacity_range_SoH == 0)
		bss->capacity_range_SoH = capacity_rated/100*(SOH_UPPER_THRESHOLD-SOH_LOWER_THRESHOLD);

	// SoH estimation: Estimate voltage at upper and lower threshold
	// Note: With get_OCV_voltage() the thresholds will be the correlation to the not loaded OCV curve.
	//       In order to get best results it would be better to use the OCV curve for the charge current and temperature at which the system is charged!
	//       To compensate this effect the thresholds are set in the region where the curve is almost linear, and a bit closer together than needed, to allow for shift up and down.
	bss->SoH_lower_threshold_V = get_OCV_voltage(SOH_LOWER_THRESHOLD);
	bss->SoH_upper_threshold_V = get_OCV_voltage(SOH_UPPER_THRESHOLD);
	bss->SoH_t = bss->SoH_t0;

	// Return initial SoC
	return bss->SoC_t0;
}

//****************************************************************************
// manage_battery_state - Check and manage the state of SoC and SoH calculation, this can run every cycle
//****************************************************************************
void manage_battery_state(){
	// SoC & SoH calculations with coulomb counting
	if(statusBMS.current >= 0.0){ // statusBMS.bms_state == BMS_DISCHARGING || statusBMS.bms_state == BMS_IDLE
		statusBMS.SoC = manageDischarge(&batteryState, setupBMS.BatteryRatedCapacity, statusBMS.voltage, (float)(setupBMS.LimitUnderVoltage)/1000.0, ETA_D);
	}
	else if(statusBMS.current < 0.0){ // statusBMS.bms_state == BMS_CHARGING
		statusBMS.SoC =    manageCharge(&batteryState, setupBMS.BatteryRatedCapacity, statusBMS.voltage, (float)(setupBMS.LimitOverVoltage)/1000.0, ETA_C);
	}
	statusBMS.SoH = estimate_SoH(&batteryState, (battery_states)statusBMS.bms_state, (float)setupBMS.BatteryRatedCapacity, ((float)statusBMS.avgCellVolt_mV)/1000.0f);

	// Fake SoC by mapping voltage range to 100 percent
	//statusBMS.SoC = mapRangeToScale((float)statusBMS.voltage*1000.0, SOC_FAKE_MAP_0_PERCENT_mV, SOC_FAKE_MAP_100_PERCENT_mV, 100.0); // (float)setupBMS.LimitUnderVoltageCell*numBatteryBoards, (float)setupBMS.LimitOverVoltageCell*numBatteryBoards, 100.0);

	/// Calculate remaining runtime. For charging, this shows how long the charging will take, for any other state it shows when the battery will be empty
	//static float lastCapacity;
	//if(batteryState.capacity_t != lastCapacity){
	//	lastCapacity = batteryState.capacity_t;
		if(statusBMS.bms_state == BMS_CHARGING && statusBMS.current < 0.0){
			// Calculate capacity difference until system is full
			batteryState.capacity_usable_remaining = (BATTERY_RATED_CAPACITY*(statusBMS.SoH/100.0))-(batteryState.capacity_init - batteryState.capacity_t);
			// Limit usable capacity
			if(batteryState.capacity_usable_remaining < 0.0) batteryState.capacity_usable_remaining = 0.0;
			// Calculate remaining time based on usable capacity
			batteryState.remaining_runtime_ms = (int64_t)(batteryState.capacity_usable_remaining/(-1.0*statusBMS.current/1000.0)*1000.0);
		}
		else{
			// Calculate usable capacity
			batteryState.capacity_usable_remaining = (batteryState.capacity_init - batteryState.capacity_t)-(BATTERY_RATED_CAPACITY*REMAINING_RUNTIME_SOC_PERCENT);
			// Limit usable capacity
			if(batteryState.capacity_usable_remaining < 0.0) batteryState.capacity_usable_remaining = 0.0;
			// Calculate remaining time based on usable capacity
			batteryState.remaining_runtime_ms = (int64_t)(batteryState.capacity_usable_remaining/(statusBMS.current/1000.0)*1000.0);
		}
		//Limit remaining runtime
		if(batteryState.remaining_runtime_ms < 1){
			batteryState.remaining_runtime_ms = 1;
		}
	//}

	// Limit SoC display
	if(statusBMS.SoC < 0)
		statusBMS.SoC = 0;
	if(statusBMS.SoC > 100)
		statusBMS.SoC = 100;
	// Limit SoH display
	if(statusBMS.SoH < 0)
		statusBMS.SoH = 0;
	if(statusBMS.SoH > 100)
		statusBMS.SoH = 100;

	// Direct capacity and rt data estimation on interval. This is moved to be executed not on a fixed interval but only whenever a new current measurement is done - prevents counting errors.
	update_time_capacity_change_for_rt_recording();
	//static uint32_t batteryStateCapacityCalcTimestamp;
	//if(TIMER_COUNTER_1MS > (batteryStateCapacityCalcTimestamp + 50)){
	//	batteryStateCapacityCalcTimestamp = TIMER_COUNTER_1MS;
	//	update_batteryState_currentMeasured();
	//}
}

//****************************************************************************
// update_batteryState_currentMeasured - This performs the integration of the current to get the passed charge.
//                                       Needs to be executed after every current measurement
//****************************************************************************
void update_batteryState_currentMeasured(){
	// Determine time difference in milliseconds
	uint32_t dt = TIMER_COUNTER_1MS - lastBatterySateCalcTimestamp;
	// Perform coulomb counting only if there is a time difference
	if(dt >= 1){
		lastBatterySateCalcTimestamp = TIMER_COUNTER_1MS;
		measurement_battery_state(&batteryState, (battery_states)statusBMS.bms_state, statusBMS.current/1000.0, dt); // current in A
		//PRINTF_MAIN_DEBUG("Update capacity with %f * %f = %f (time %lf)\r\n", statusBMS.current, dt, batteryState.capacity_t, (float)(TIMER_COUNTER_1MS - lastBatterySateCalcTimestamp));
	}
}

/******************************************************************
 *	measurement_battery_state - Use this at every current measurement event to sum up capacity changes.
 *								current_t ... in A. Current reading at this time step
 *								dt        ... in s. Time since last measurement
 ******************************************************************/
void measurement_battery_state(battery_state_struct* bss, battery_states state, float current_t, uint32_t dt){
	// Calculate charge difference
	float dC = current_t * (((float)dt)/1000.0);

	// Do not integrate before init function is called
	if(bss->SoC_t0 != 0){
		bss->capacity_t += dC;
	}

	// RT data recording
	//rt_data_time_since_last_record += dt;
	//rt_data_charge_since_last_record += dC;

	// Debug message to check if integration and rt data value estimation is correct
	//printf(" < dt %.3f, I %.3f, +%.3f, C_di %.3f, C %.3f> \r\n", dt, current_t, current_t * dt, bss->capacity_t, (batteryState.capacity_init - batteryState.capacity_t));
}

/******************************************************************
 *	update_time_capacity_change_for_rt_recording - Every time this is called the stored time and capacity difference used for RT data recording is updated.
 *												   For precise but not well formated data call this from update_batteryState_currentMeasured() (accumulated time errors due to 1ms inaccuracy)
 *												   For well formated graphs call this more often to avoid missing or catching a measurement in  the same time and having jumpy results
 ******************************************************************/
void update_time_capacity_change_for_rt_recording(){
	static uint32_t lastRtDataTimeCapDiffTimestamp;

	if(TIMER_COUNTER_1MS > (lastRtDataTimeCapDiffTimestamp + 10)){
		// Determine time difference in milliseconds
		uint32_t dt = TIMER_COUNTER_1MS - lastRtDataTimeCapDiffTimestamp;
		// Perform adding of recorded time and charge only if there is a time difference
		if(dt >= 1){
			lastRtDataTimeCapDiffTimestamp = TIMER_COUNTER_1MS;

			// Calculate charge difference
			float dC = statusBMS.current/1000.0 * (((float)dt)/1000.0);

			// Perform adding of recorded time and charge only if the change of charge is higher than what can be stored minimum
			if(fabs(dC) >= (1.0/MEMORY_RT_DATA_C_DIF_MULTIPLIER)){
				// RT data recording
				rt_data_time_since_last_record += dt;
				rt_data_charge_since_last_record += dC;

				// Debug message to check if integration and rt data value estimation is correct
				//printf(" < dt %.3f, I %.3f, +%.3f, C_di %.3f, C %.3f> \r\n", dt, current_t, current_t * dt, batteryState->capacity_t, (batteryState.capacity_init - batteryState.capacity_t));
			}
		}
	}
}

/******************************************************************
 *	estimate_SoH - Use this at every current measurement event to sum up capacity changes
 *				   state 			... state of the charge direction (manageDischarge, manageCharge or other)
 *				   capacity_rated 	... in As. The usable capacity of the connected battery (between over- and under-voltage condition)
 *				   avgCellVolt		... in V. Voltage average of all cells
 ******************************************************************/
float estimate_SoH(battery_state_struct* bss, battery_states state,  float capacity_rated, float avgCellVolt){
	// Note: SoH estimation at discharge is deactivated because this yields different results for different discharge currents. Only use this if discharge current is known and adjusted for (e.g.use different OCV curves)!

	// If charge direction changes during SoH estimation discard SoH estimation cycle
	if((bss->last_state == BATTERY_CHARGING    && state == BATTERY_DISCHARGING)
	){
		bss->init_capacity_SoH_at_thres = 0;
		bss->last_state = BATTERY_OTHER;
	}

	// Check initialization condition for SoH estimation -> going below or above estimation range
	if((state == BATTERY_CHARGING    && avgCellVolt <= bss->SoH_lower_threshold_V && statusBMS.current > SOH_INITIAL_CURRENT)
	){
		bss->init_capacity_SoH_at_thres = bss->capacity_init - bss->capacity_t;
		bss->last_state = state;
	}

	// Check finalization condition of SoH estimation -> going above or below estimation range
	if((bss->last_state == BATTERY_CHARGING    && avgCellVolt >  bss->SoH_upper_threshold_V && bss->init_capacity_SoH_at_thres != 0)
	){
		// Calculate state of health in charge mode
		float SoH = (fabsf(bss->capacity_state_delta) / bss->capacity_range_SoH ) * 100.0;

		// Only use the calculated value if the temperature is in range. Note that this is only tested at the end to see the effects of different temperatures in debug log. Depending on system specifications this should stop the estimation at any point
		if(statusBMS.avgTemp > SOH_MIN_TEMPERATURE && statusBMS.avgTemp < SOH_MAX_TEMPERATURE){
			// Damping of value (average between last and current) and limit of final value
			bss->SoH_t = (bss->SoH_t + SoH) / 2;
			if(bss->SoH_t > 100)    bss->SoH_t = 100;
			else if(bss->SoH_t <= 1) bss->SoH_t = 1;
		}
		else{
			PRINTF_BATTERY_STATE("Temperature (%.2fdegC) was outside limits (%.2fdegC to %.2fdegC) for SoH estimation. Calculated value will not be used!\r\n", statusBMS.avgTemp, SOH_MIN_TEMPERATURE, SOH_MAX_TEMPERATURE);
		}
		PRINTF_BATTERY_STATE("*******************************************************************************************\r\n*******************************************************************************************\r\n*******************************************************************************************\r\n");
		PRINTF_BATTERY_STATE("Set SoH_t: %.3f (calculated %.3f), capacity_state_delta %.3f, capacity_range_SoH  %.3f, init_capacity_SoH_at_thres %.3f, last_state %d \r\n", bss->SoH_t, SoH, bss->capacity_state_delta, bss->capacity_range_SoH, bss->init_capacity_SoH_at_thres, (uint8_t)bss->last_state );
		PRINTF_BATTERY_STATE("*******************************************************************************************\r\n*******************************************************************************************\r\n*******************************************************************************************\r\n");

		// Reset SoH variables
		bss->init_capacity_SoH_at_thres = 0;
		bss->last_state = BATTERY_OTHER;
	}

	return bss->SoH_t;
}

/******************************************************************
 *	manageDischarge - Used to calculate changes of the battery state while it is discharging
 ******************************************************************/
float manageDischarge(battery_state_struct* bss, float capacity_rated, float voltage_batt, float v_min, float eta_d) {
	// Calculate capacity change since last change of charge direction
	if(bss->init_capacity_SoH_at_thres != 0)
		bss->capacity_state_delta = bss->init_capacity_SoH_at_thres - (bss->capacity_init - bss->capacity_t);

	//if (bss->SoC_t <= 1 && 100/capacity_rated*bss->capacity_state_delta > SOH_MIN_CAPACITY_DELTA) {
	//	// Calculate state of health in discharge mode
	//	bss->SoH_t = ((fabsf(bss->capacity_t) + (capacity_rated - bss->capacity_init)) / capacity_rated) * 100;
	//}
	//else {
	//	bss->DoD_delta = ((bss->capacity_t ) / (capacity_rated)) * 100;
	//	bss->DoD_t = bss->DoD_t0 + eta_d * bss->DoD_delta;
	//	bss->SoC_t = bss->SoH_t0 - bss->DoD_t;
	//}

	bss->DoD_delta = ((bss->capacity_t ) / (capacity_rated)) * 100;
	bss->DoD_t = bss->DoD_t0 + eta_d * bss->DoD_delta;
	bss->SoC_t = bss->SoH_t0 - bss->DoD_t;

	// Return SoC
	return bss->SoC_t;
}

/******************************************************************
 *	manageCharge - Used to calculate changes of the battery state while it is charging
 ******************************************************************/
float manageCharge(battery_state_struct* bss, float capacity_rated, float voltage_batt, float v_max, float eta_c) {
	// Calculate capacity change since last change of charge direction
	if(bss->init_capacity_SoH_at_thres != 0)
		bss->capacity_state_delta = (bss->capacity_init - bss->capacity_t) - bss->init_capacity_SoH_at_thres;

	//if (bss->SoC_t >= 99 && 100/capacity_rated*bss->capacity_state_delta > SOH_MIN_CAPACITY_DELTA) {
	//	// Calculate state of health in charge mode
	//	bss->SoH_t = ((fabsf(bss->capacity_t) + bss->capacity_init) / capacity_rated) * 100;
	//} else {
	//	bss->DoD_delta = ((bss->capacity_t ) / capacity_rated) * 100;
	//	bss->DoD_t = bss->DoD_t0 + eta_c * bss->DoD_delta;
	//	bss->SoC_t = bss->SoH_t0 - bss->DoD_t;
	//}

	bss->DoD_delta = ((bss->capacity_t ) / capacity_rated) * 100;
	bss->DoD_t = bss->DoD_t0 + eta_c * bss->DoD_delta;
	bss->SoC_t = bss->SoH_t0 - bss->DoD_t;

	// Return SoC
	return bss->SoC_t;
}

/******************************************************************
 *	get_eta_ah - Calculate coulomb efficiency
 ******************************************************************/
float get_eta_ah(float eta_c, float eta_d){
	return eta_c / eta_d;
}

/******************************************************************
 *	get_initial_SoC - Voltage to initial SoC via Open Circuit Voltage OCV estimation
 *	Input:
 *		voltage_batt ... in V.
 *		numCells     ... number of cells
 *		tolerance    ... in V. Tolerance on how far below/above the cell is still recognized as 0%/100%
 *	Returns SoC in % or -1.0f if undervoltage error or -2.0f if overvoltage error
 ******************************************************************/
float get_initial_SoC(float voltage_batt, uint16_t numCells, float tolerance) {
	// Calculate cell voltage
	float voltage_cell = voltage_batt / numCells;
	PRINTF_BATTERY_STATE("\tVbat: %.2f, Vcell %.2f, NumCells %d\r\n", voltage_batt, voltage_cell, numCells);

	// Return undervoltage error, if cell voltage is below value of first OCV
	if(voltage_cell < ocv_curve[0][1]){
	    if(voltage_cell < ocv_curve[0][1]-tolerance){
		    return -1.0f;
	    }
	    else{
	        return 0.0f;
	    }
	}

	// Determine SoC
	for(uint8_t i = 1; i < OCV_POINTS; i++){
		if(voltage_cell < ocv_curve[i][1]){
			// Linearize function between current last and and current OCV
			float p0_V = ocv_curve[i-1][1];
			float p0_P = ocv_curve[i-1][0];
			float p1_V = ocv_curve[i][1];
			float p1_P = ocv_curve[i][0];
			float k = (p1_P-p0_P)/(p1_V-p0_V);
			float d = (p1_V*p0_P-p0_V*p1_P)/(p1_V-p0_V);

			PRINTF_BATTERY_STATE("\tCell < %.2fV (%.2f percent) - Found point\r\n",ocv_curve[i][1], ocv_curve[i][0]);
            //PRINTF_BATTERY_STATE("LastP:%.2f,%.2f CurrentP:%.2f,%.2f; k=%.2f d=%.2f, x=%.2f  \r\n", p0_V, p0_P, p1_V, p1_P, k, d,voltage_cell);

			// Get actual SoC
			return k*voltage_cell + d;
		}
		else{
			//PRINTF_BATTERY_STATE("\tOCV: Cell > %.2f (%.2f)\r\n",ocv_curve[i][1],ocv_curve[i][0]);
		}

	}

	// Return overvoltage error - no OCV found that was higher than cell
	if(voltage_cell > ocv_curve[OCV_POINTS-1][1]+tolerance){
	    return -2.0f;
    }
    else{
        return 100.0f;
    }
}

/******************************************************************
 *	get_OCV_voltage - SoC to voltage via Open Circuit Voltage OCV estimation
 *	Input:
 *		percent ... in % charged
 *	Returns voltage in V or -1.0f if input not between 0% and 100%
 ******************************************************************/
float get_OCV_voltage(float percent) {
    // Return value for 100% if input too high
    if(percent == 100){
        return ocv_curve[OCV_POINTS-1][1];
    }
    else if(percent < 0 || percent > 100){
        return -1.0f;
    }

	// Determine SoC
	for(uint8_t i = 1; i < OCV_POINTS; i++){
		if(percent < ocv_curve[i][0]){
			// Linearize function between current last and and current OCV
			float p0_V = ocv_curve[i-1][1];
			float p0_P = ocv_curve[i-1][0];
			float p1_V = ocv_curve[i][1];
			float p1_P = ocv_curve[i][0];
			float k = (p1_V-p0_V)/(p1_P-p0_P);
			float d = (p1_P*p0_V-p0_P*p1_V)/(p1_P-p0_P);

			//PRINTF_BATTERY_STATE("Cell < %.2f (%.2f) - Found point\r\n",ocv_curve[i][1], ocv_curve[i][0]);
			//PRINTF_BATTERY_STATE("LastP:%.2f,%.2f CurrentP:%.2f,%.2f; k=%.2f d=%.2f, x=%.2f  \r\n", p0_V, p0_P, p1_V, p1_P, k, d,percent);

			// Get actual voltage for percent charged
			return k*percent + d;
		}
		else{
			//PRINTF_BATTERY_STATE("Cell > %.2f (%.2f)\r\n",ocv_curve[i][1],ocv_curve[i][0]);
		}

	}

	// Estimation failed
    return -1.0f;
}

