/*
 * battery_state.h - Abstraction off all battery state related functions that need to be accessed from main.
 * 				 	 The implemented algorithm is a simple implementation of coloumb counting based on code from p. temsamani
 * 				 	 To implement other algorithms rewrite init_battery_state() to be executed at the start of the system,
 * 				 	 manage_battery_state to be executed every main loop cycle and
 * 				 	 update_battery_state executed after every current measurement.
 *
 *  Created on: 24 Nov 2022
 *      Author: RNSANTELER
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef BATTERY_STATE_H_
#define BATTERY_STATE_H_

#include <stdint.h>
#include "cycfg.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


//#define SOH_MIN_CAPACITY_DELTA 20	// in %. Minimum charged/discharged capacity for SoH calculation to be executed

// Points on OCV curve: SoC[%], V_C[V]
#define OCV_POINTS 34
extern float ocv_curve[OCV_POINTS][2];

typedef enum {
	BATTERY_OTHER = 0,
	BATTERY_CHARGING = 5,
	BATTERY_DISCHARGING = 6
} battery_states;

typedef struct {
	float SoC_t0; 							// initial state of charge
	float SoC_t;							// current state of charge
	float SoH_t0;							// initial state of health
	float SoH_t;							// current state of health
	float DoD_t0;							// initial depth of discharge
	float DoD_t;							// current depth of discharge
	float DoD_delta;						// delta depth of discharge
	float voltage_init;						// initial voltage
	float capacity_init;					// initial capacity
	float capacity_t;						// current capacity of the battery (summed)
	battery_states last_state;				// last state of the battery used to determine change of charge direction and therefore SoH calculation conditions
	float init_capacity_SoH_at_thres;		// in As. The capacity the battery had at the last change of last_state
	float capacity_state_delta;				// in As. The capacity change since last state change
	float capacity_range_SoH;				// in As. Amount of capacity that should change between SOH_UPPER_THRESHOLD and SOH_LOWER_THRESHOLD voltage to get 100% SoH (If zero it will be estimated from OCV curve. Write correct value after first estimation with expected current here)
	float capacity_usable_remaining;		// in As. Amount of capacity that is still usable (see REMAINING_RUNTIME_SOC_PERCENT)
	int64_t remaining_runtime_ms;			// in ms. Momentary remaining ON time if the system keeps the currently drawn current till the end.
	float SoH_upper_threshold_V;			// in V. Will be estimated from OCV curve and SOH_UPPER_THRESHOLD at init_battery_state()
	float SoH_lower_threshold_V;			// in V. Will be estimated from OCV curve and SOH_LOWER_THRESHOLD at init_battery_state()
} battery_state_struct;

float init_battery_state(battery_state_struct* bss, float capacity_rated, float voltage_batt, uint16_t numBB);
void manage_battery_state();
void update_batteryState_currentMeasured();
void update_time_capacity_change_for_rt_recording();

void measurement_battery_state(battery_state_struct* bss, battery_states state, float current_t, uint32_t dt);
float estimate_SoH(battery_state_struct* bss, battery_states state,  float capacity_rated, float avgCellVolt);
float manageDischarge(battery_state_struct* bss, float capacity_rated, float voltage_batt, float v_min, float eta_d);
float manageCharge   (battery_state_struct* bss, float capacity_rated, float voltage_batt, float v_max, float eta_c);
float get_initial_SoC(float voltage_batt, uint16_t numCells, float tolerance);
float get_OCV_voltage(float percent);
float get_eta_ah(float eta_c, float eta_d);


#endif /* BATTERY_STATE_H_ */
