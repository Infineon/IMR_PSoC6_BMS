/*
 * diagnostic.c
 * 		Abstraction of the diagnostic system, see doSystemDiagnostic for details
 *
 *  Created on: 4 Apr 2022
 *      Author: r. santeler
 *
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

#include "diagnostic.h"

#include <stdint.h>
#include <global_management.h>
#include "BMS_2ED4820EM/Driver_2ED4820.h"
#include "BMS_TLE9012/BMS_TLE9012.h"
#include "IMR_CAN.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define PRINTF_DIAGNOSTIC(p_frm, ...)		printf(p_frm, ##__VA_ARGS__)


/*******************************************************************************
* VARIABLES
*******************************************************************************/
diagnostic_states diagnostic_state = DIAGNOSTIC_EVAL_CLEAR;


/*******************************************************************************
* FUNCTIONS
*******************************************************************************/
// getErrorBit - Returns 1 if error was found and 0 if error not found
uint8_t getErrorBit(errors_bitmask errBit){
	return (statusBMS.error_map & errBit) != 0;
}
// getErrorBit - Set the error bits errBit bit to 1
void setErrorBit(errors_bitmask errBit){
	statusBMS.error_map |= errBit;
}
// clearErrorBit - Set the error bits errBit bit to 0
void clearErrorBit(errors_bitmask errBit){
	statusBMS.error_map &= ~errBit;
}

// getMsgBit - Returns 1 if message bit was found and 0 if message bit not found
uint8_t getMsgBit(messages_bitmask msgBit){
	return (statusBMS.message_map & msgBit) != 0;
}
// setMsgBit - Set the warning bits errBit bit to 1
void setMsgBit(messages_bitmask msgBit){
	statusBMS.message_map |= msgBit;
}
// clearMsgBit - Set the warning bits errBit bit to 0
void clearMsgBit(messages_bitmask msgBit){
	statusBMS.message_map &= ~msgBit;
}

// getErrorBitmap - Return the error bitmap
uint32_t getErrorBitmap(){
	return statusBMS.error_map;
}
// getMsgBitmap - Return the warning bitmap
uint32_t getMsgBitmap(){
	return statusBMS.message_map;
}

//****************************************************************************
// resetAndSetErrorsLatchedTLE9012 - The error bits of the analog frontend TLE9012
// need to be latched for some time because the result might wrongly be 0 when clear
// was executed but round robin cycle did not run yet.
//****************************************************************************
void resetAndSetErrorsLatchedTLE9012(){
	// Store time of last recorded error for each TLE9012 ERROR
	static uint32_t time_last_SYS_PS_SLEEP;
	static uint32_t time_last_SYS_ADC_ERR;
	static uint32_t time_last_SYS_OL_ERR;
	static uint32_t time_last_INT_IC_ERR;
	static uint32_t time_last_REG_CRC_ERR;
	static uint32_t time_last_EXT_T_ERR;
	static uint32_t time_last_INT_OT;
	static uint32_t time_last_CELL_UV;
	static uint32_t time_last_CELL_OV;
	static uint32_t time_last_BAL_UC;
	static uint32_t time_last_BAL_OC;

	static uint32_t time_last_TLE9012_QUEUE_FAILED;
	static uint32_t time_last_TLE9012_INTERRUPT;

	// Get currently received error bitmap of TLE9012
	uint32_t currentErrorBitmap = tle9012_getFailureBitmap(1);
	//PRINTF_DIAGNOSTIC("CEB: %ld", currentErrorBitmap);

	/// For each TLE9012 Error in global error_map:
	// If the error is gone and the time since the error was last recorded is longer than the latch time clear the error
	// Else if the error is there refresh last error time
	if ( ((currentErrorBitmap & ERR_CAF_PS_SLEEP) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_SYS_PS_SLEEP + TLE9012_COMMON_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_PS_SLEEP);
	}
	else if((currentErrorBitmap & ERR_CAF_PS_SLEEP) != 0){
		time_last_SYS_PS_SLEEP = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_ADC_ERR) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_SYS_ADC_ERR + TLE9012_COMMON_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_ADC_ERR);
	}
	else if((currentErrorBitmap & ERR_CAF_ADC_ERR) != 0){
		time_last_SYS_ADC_ERR = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_OL_ERR) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_SYS_OL_ERR + TLE9012_COMMON_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_OL_ERR);
	}
	else if((currentErrorBitmap & ERR_CAF_OL_ERR) != 0){
		time_last_SYS_OL_ERR = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_INT_IC_ERR) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_INT_IC_ERR + TLE9012_COMMON_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_INT_IC_ERR);
	}
	else if((currentErrorBitmap & ERR_CAF_INT_IC_ERR) != 0){
		time_last_INT_IC_ERR = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_REG_CRC_ERR) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_REG_CRC_ERR + TLE9012_COMMON_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_REG_CRC_ERR);
	}
	else if((currentErrorBitmap & ERR_CAF_REG_CRC_ERR) != 0){
		time_last_REG_CRC_ERR = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_EXT_T_ERR) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_EXT_T_ERR + TLE9012_OVERTMP_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_EXT_T_ERR);
	}
	else if((currentErrorBitmap & ERR_CAF_EXT_T_ERR) != 0){
		time_last_EXT_T_ERR = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_INT_OT) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_INT_OT + TLE9012_OVERTMP_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_INT_OT);
	}
	else if((currentErrorBitmap & ERR_CAF_INT_OT) != 0){
		time_last_INT_OT = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_CELL_UV) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_CELL_UV + TLE9012_COMMON_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_CELL_UV);
	}
	else if((currentErrorBitmap & ERR_CAF_CELL_UV) != 0){
		time_last_CELL_UV = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_CELL_OV) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_CELL_OV + TLE9012_COMMON_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_CELL_OV);
	}
	else if((currentErrorBitmap & ERR_CAF_CELL_OV) != 0){
		time_last_CELL_OV = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_BAL_UC) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_BAL_UC + TLE9012_COMMON_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_BAL_UC);
	}
	else if((currentErrorBitmap & ERR_CAF_BAL_UC) != 0){
		time_last_BAL_UC = TIMER_COUNTER_1MS;
	}

	if ( ((currentErrorBitmap & ERR_CAF_BAL_OC) == 0) &&
			(TIMER_COUNTER_1MS > (time_last_BAL_OC + TLE9012_COMMON_ERR_LATCH_TIME))){
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_BAL_OC);
	}
	else if((currentErrorBitmap & ERR_CAF_BAL_OC) != 0){
		time_last_BAL_OC = TIMER_COUNTER_1MS;
	}

	/// Check if general error occurred
	// Queue errors
	if(lastQueueResult_tle9012 != QUEUE_SUCCESS && lastQueueResult_tle9012 != QUEUE_FINISHED){
		//PRINTF_DIAGNOSTIC("[ERROR]: manageQueue returned %d\r\n", lastQueueResult_tle9012);
		time_last_TLE9012_QUEUE_FAILED = TIMER_COUNTER_1MS;
		setErrorBit(ERR_INT_TLE9012_QUEUE_FAILED);
	}
	else if (TIMER_COUNTER_1MS > (time_last_TLE9012_QUEUE_FAILED +
								  TLE9012_COMMON_ERR_LATCH_TIME)){
		clearErrorBit(ERR_INT_TLE9012_QUEUE_FAILED);
	}

	// TLE9012 error pin interrupt
	if(errorInterruptDetected_TLE9012 == 1 || cyhal_gpio_read(TLE9012_ERR) == 1){
		time_last_TLE9012_INTERRUPT = TIMER_COUNTER_1MS;
		setErrorBit(ERR_CAF_TLE9012_INTERRUPT);
	}
	else if (TIMER_COUNTER_1MS > (time_last_TLE9012_INTERRUPT +
								  TLE9012_COMMON_ERR_LATCH_TIME)){
		clearErrorBit(ERR_CAF_TLE9012_INTERRUPT);
	}

	// Get and set all bits of TLE9012
	statusBMS.error_map |= currentErrorBitmap;
}


//****************************************************************************
// diagnosticSafetySwitch - Reset and set errors of 2ED4820
//****************************************************************************
void diagnosticSafetySwitch(){
	/// Update errors of 2ED4820 driver to BMS error register
	// Reset all driver bits
	statusBMS.error_map &= ~ERRORS_SSW_MASK;
	// Get all set driver bits
	uint32_t error_map_safety_switch = getFailureBitmap(USE_REG_RENEW);
	statusBMS.error_map |= error_map_safety_switch;

	/// Get warnings and info's from driver
	// Reset all driver bits
	statusBMS.message_map &= ~MSG_SSW_MASK;
	// Set all driver bits
	statusBMS.message_map |= getWarningBitmap(USE_REG_CACHED);
	statusBMS.message_map |= getInfoBitmap(USE_REG_CACHED);

	// If an 2ED4820 error is detected
	// (error interrupt or internal error detected) try to reset
	if( error_map_safety_switch != 0 || errorInterruptDetected_2ED4820 == 1 ||
			cyhal_gpio_read(D_2ED4820_INT) == 1){
		if(errorInterruptDetected_2ED4820 == 1 || cyhal_gpio_read(D_2ED4820_INT) == 1){
			// Mark error
			setErrorBit(ERR_SSW_INTERRUPT);
			PRINTF_DIAGNOSTIC("[ERROR]: Safety switch - Interrupt %d, "
					"Pin %d, ErrorMap %ld\r\n", errorInterruptDetected_2ED4820,
					cyhal_gpio_read(D_2ED4820_INT), error_map_safety_switch);
		}
		checkSwitchErrWarnState(1);
		errorInterruptDetected_2ED4820 = 0;
	}
}

/****************************************************************************
 * diagnosticTLE9012 - Reset and set errors of TLE9012.
 * Handles the read and reset of diagnostic registers of
 * the analog front-end TLE9012
 *
 * Note: Currently following the registers are read:
 * 		General diagnosis GEN_DIAG
 * 		Cell voltage supervision warning flag CELL_UV
 * 		Cell voltage supervision warning flag CELL_OV
 * 		Diagnosis OPENLOAD DIAG_OL
 * 		Passive balancing diagnosis OVERCURRENT BAL_DIAG_OC
 * 			(only if balancing function is active)
 * 		Passive balancing diagnosis UNDERCURRENT BAL_DIAG_UC
 * 			(only if balancing function is active)
 *
 * The following diagnostics registers would available (see datasheet):
 * 		External over-temperature warning flags EXT_TEMP_DIAG
 * 		Cell voltage supervision warning flags CELL_UV_DAC_COMP
 * 		Cell voltage supervision warning flags CELL_OV_DAC_COMP
 ***************************************************************************/
void diagnosticTLE9012(measurementStates measureState){
	requestResult tle9012_result;

	// DIAGNOSTIC TLE9012:
	// Check if diagnostic register is queued, if not read it and reset errors
	if(checkRegisterQueued_tle9012(1, GEN_DIAG, 0) == 0){
		if(diagnostic_state == DIAGNOSTIC_EVAL_CLEAR){
			/// Update errors of TLE9012 to BMS error register.
			// Only use data if no error is detected
			if(checkRegisterError_tle9012(1, GEN_DIAG, 0) == 0){
				// The error bits of the analog front-end TLE9012
				// need to be latched for some time because the result
				// might wrongly be 0 when clear was executed
				// but round robin cycle did not run yet.
				resetAndSetErrorsLatchedTLE9012();

				/// Get warnings and info's of TLE9012 to BMS error register
				// Reset all bits of TLE9012
				statusBMS.message_map &= ~MSG_CAF_MASK;
				// Set all bits of TLE9012
				uint16_t data_gen_diag = getRegisterCacheDataByIndex_tle9012(1, GEN_DIAG);
				if(data_gen_diag & BAL_ACTIVE__ON){
					statusBMS.message_map |= MSG_CAF_BAL_ACTIVE;
				}

				/// If any error is set or error interrupt is detected reset errors
				if((data_gen_diag & GEN_DIAG_RESETABLE_FLAGS) != 0){

					/// If an error that is cell specific was found
					/// store and read its cell map
					/// (the cell map will effectively be right in the next turn)

					// Open load
					if(statusBMS.error_map & ERR_CAF_OL_ERR){
						statusBMS.ol_map |= getRegisterCacheDataByIndex_tle9012(1, DIAG_OL);
						TLE9012_DETECT_COM_ERROR(
								tle9012_readRegisterByIndex(1, DIAG_OL, 1, 0),
								tle9012_readRegisterByIndex_DIAG_OL,
								SUCCESS_FUNCTION_NONE);
					}
					else{
						statusBMS.ol_map = 0;
					}
					// Under voltage
					if(statusBMS.error_map & ERR_CAF_CELL_UV){
						statusBMS.uv_map |= getRegisterCacheDataByIndex_tle9012(1, CELL_UV);
						TLE9012_DETECT_COM_ERROR(
								tle9012_readRegisterByIndex(1, CELL_UV, 1, 0),
								tle9012_readRegisterByIndex_CELL_UV,
								SUCCESS_FUNCTION_NONE);
					}
					else{
						statusBMS.uv_map = 0;
					}
					// Over voltage
					if(statusBMS.error_map & ERR_CAF_CELL_OV){
						statusBMS.ov_map |= getRegisterCacheDataByIndex_tle9012(1, CELL_OV);
						TLE9012_DETECT_COM_ERROR(
								tle9012_readRegisterByIndex(1, CELL_OV, 1, 0),
								tle9012_readRegisterByIndex_CELL_OV,
								SUCCESS_FUNCTION_NONE);
					}
					else{
						statusBMS.ov_map = 0;
					}
					// Balancing under current
					if(statusBMS.error_map & ERR_CAF_BAL_UC){
						statusBMS.bal_uc_map |=
								getRegisterCacheDataByIndex_tle9012(1, BAL_DIAG_UC);
						TLE9012_DETECT_COM_ERROR(
								tle9012_readRegisterByIndex(1, BAL_DIAG_UC, 1, 0),
								tle9012_readRegisterByIndex_BAL_DIAG_UC,
								SUCCESS_FUNCTION_NONE);
					}
					else{
						statusBMS.bal_uc_map = 0;
					}
					// Balancing over current
					if(statusBMS.error_map & ERR_CAF_BAL_OC){
						statusBMS.bal_oc_map |=
								getRegisterCacheDataByIndex_tle9012(1, BAL_DIAG_OC);
						TLE9012_DETECT_COM_ERROR(
								tle9012_readRegisterByIndex(1, BAL_DIAG_OC, 1, 0),
								tle9012_readRegisterByIndex_BAL_DIAG_OC,
								SUCCESS_FUNCTION_NONE);
					}
					else{
						statusBMS.bal_oc_map = 0;
					}
					// TODO: EXT_TEMP_DIAG could also be read on temperature error

					/*if(errorInterruptDetected_TLE9012 == 1 ||
							cyhal_gpio_read(TLE9012_ERR) == 1){
						PRINTF_DIAGNOSTIC("[ERROR]: "
								"errorInterruptDetected_TLE9012 detected %d, "
								"pin state was %d, GEN_DIAG ",
								errorInterruptDetected_TLE9012,
								cyhal_gpio_read(TLE9012_ERR));
						printBinary(data_gen_diag, UINT16_MAX, 16);
						PRINTF_DIAGNOSTIC("\r\n");
					}*/

					// Clear all errors
					tle9012_clearProblemDetected();
					tle9012_result = tle9012_clearStatusFlags(1, 0, 0);
					if(tle9012_result >= REQUEST_ERROR){
						PRINTF_DIAGNOSTIC("[ERROR]: tle9012_clearStatusFlags "
								"returned %d\r\n", tle9012_result);}
				}
			}
			// Change state to read
			diagnostic_state = DIAGNOSTIC_READ;
		}
		else if((diagnostic_state == DIAGNOSTIC_READ &&
				measureState == MEASUREMENT_READ_RESULT) ||
				errorInterruptDetected_TLE9012){
			//PRINTF_DIAGNOSTIC("Read GEN_DIAG\r\n");
			tle9012_result = tle9012_readRegisterByIndex(1, GEN_DIAG, 0, 0); // 0xE001
			if(tle9012_result >= REQUEST_ERROR){
				PRINTF_DIAGNOSTIC("[ERROR]: Trigger next GEN_DIAG "
						"read failed %d\r\n", tle9012_result);}

			// Change state to evaluate and clear
			diagnostic_state = DIAGNOSTIC_EVAL_CLEAR;
		}
	}
	errorInterruptDetected_TLE9012 = 0;
}

//****************************************************************************
// diagnosticInternal
// Reset and set internal system errors
// that are not from safety switch driver or analog front-end
//****************************************************************************
void diagnosticInternal(){
	/// Determine errors and messages from rest of system
	// Reset internal (not from TLE9012 or 2ED4820) error bits
    statusBMS.error_map &= ~ERRORS_INT_MASK;
    statusBMS.message_map &= ~MSG_INT_MASK;

    /// ERRORS
	// Check over/under voltage conditions
	if(statusBMS.voltage < (float)(setupBMS.LimitUnderVoltage)/1000.0){
		setErrorBit(ERR_INT_VBAT_UV);
	}
	else if(statusBMS.voltage > (float)(setupBMS.LimitOverVoltage)/1000.0){
		setErrorBit(ERR_INT_VBAT_OV);
	}

	// Check over and under current conditions
	if((statusBMS.current > (float)OVERCURRENT_THRESHOLD_POS_mA) ||
			(statusBMS.current < (float)OVERCURRENT_THRESHOLD_NEG_mA)){
		setErrorBit(ERR_INT_OVERCURRENT);
	}

	// Check if communication with external ADC is still active
	if(TIMER_COUNTER_1MS > (adcExtLastReadSuccessTimestamp + ADC_EXT_READ_TIMEOUT)){
		setErrorBit(ERR_INT_ADC_COM_TIMEOUT);
	}

	// Check if number of internal TLE9012 errors exceeded threshold
	if(numComErrors_TLE9012 >= TLE9012_MAX_COM_ERRORS){
		setErrorBit(ERR_INT_TLE9012_COM_ERROR);
	}
	// Reduce the com error counter by 1 every interval if it is not already 0. Prevents accumulation of errors over time
	if(TIMER_COUNTER_1MS > (lastTLE9012IntervalComErrorReduceTimestamp +
								TLE9012_INTERVAL_COM_ERRORS)){
		lastTLE9012IntervalComErrorReduceTimestamp = TIMER_COUNTER_1MS;
		if(numComErrors_TLE9012 != 0){
			numComErrors_TLE9012--;
		}
	}

	// Check if a switch ON veto was received where the other BMS
	// send value above 100 to signal that it has its output active.
	if(receivedCanSwitchOnVetoCount > 0 &&
			receivedCanSwitchOnVetoActiveState == 1 &&
			statusBMS.hotSwapRole != HOTSWAP_TAKER){ // && statusBMS.slotState == SLOT_IN_SYSTEM_2
		setErrorBit(ERR_INT_SWITCH_ON_VETO_RECEIVED);
	}

	/// WARNINGS and MESSAGES
	// Check if external voltage is detected
	if(statusBMS.extLoadVoltage > EXT_LOAD_VOLTAGE_THRESHOLD_V){ // && (statusBMS.outputState != OUTPUT_ON && statusBMS.outputState != OUTPUT_PRECHARGE)
		setMsgBit(MSG_INT_EXT_LOAD_V_ON);
	}

	// Check if SoC is below warning point
	if(statusBMS.SoC < BATTERY_LOW_WARNING_SOC){
		setMsgBit(MSG_INT_BATTERY_LOW);
	}

	/// Special conditions that will clear errors
	// Charge from under-voltage: If system is in a charger and
	// the external load voltage is higher than main voltage and
	// not below minimum - clear undervoltage errors
	if( ((statusBMS.slotState == SLOT_IN_CHARGER) &&
			((statusBMS.extLoadVoltage+CHARGE_FROM_UV_EXT_OFFSET_V) > statusBMS.voltage)) &&
			(statusBMS.extLoadVoltage > CHARGE_FROM_UV_EXT_MIN_V)){
		clearErrorBit(ERR_INT_VBAT_UV);
		clearErrorBit(ERR_CAF_CELL_UV);
	}
}

//****************************************************************************
// Receive (when possible) errors and warnings from safety switch,
// analog front-end TLE9012 and rest of system.
// Set bms_state to BMS_ERROR if needed or to BMS_IDLE if error cleared.
// Also controls red LED, and error/warning latch.
// Uses measureState to only manage
// analog front-end TLE9012 error between measurements
//****************************************************************************
void doSystemDiagnostic(){
	// Determine errors and messages from 2ED4820
	diagnosticSafetySwitch();

	// Read or queue the diagnostic registers of the analog frontend TLE9012
	diagnosticTLE9012(statusBMS.measurementState);

	// Determine errors and messages from rest of system
    diagnosticInternal();


    // DEBUG: Ignore errors that are present when no cells are inserted
	#if (ACTIVATE_CELL_ERROR_IGNORE == 1)
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_TLE9012_INTERRUPT);
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_ADC_ERR);
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_OL_ERR);
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_CELL_OV);
		statusBMS.error_map &= ~((uint32_t)ERR_CAF_CELL_UV);
		statusBMS.error_map &= ~((uint32_t)ERR_INT_VBAT_UV);
	#endif

    // DEBUG: Check if bitmaps are aligned properly
//	printBinary(ERRORS_SSW_MASK, UINT32_MAX, 32); PRINTF_DIAGNOSTIC("\r\n");
//	printBinary(ERRORS_CAF_MASK, UINT32_MAX, 32); PRINTF_DIAGNOSTIC("\r\n");
//	printBinary(ERRORS_INT_MASK, UINT32_MAX, 32); PRINTF_DIAGNOSTIC("\r\n");
//	printBinary(MSG_SSW_MASK, UINT32_MAX, 32); PRINTF_DIAGNOSTIC("\r\n");
//	printBinary(MSG_CAF_MASK, UINT32_MAX, 32); PRINTF_DIAGNOSTIC("\r\n");
//	printBinary(MSG_INT_MASK, UINT32_MAX, 32); PRINTF_DIAGNOSTIC("\r\n");


    /// Manage red LED and BMS_ERROR state
	// If any errors were found, set state to ERROR ...
	// if state is ERROR and no errors are present set to IDLE ...
	// else check if charge state is below warning limit and blink red LED
	if(statusBMS.error_map != 0){
		cyhal_gpio_write(BMS_LED3_RD, 1);
		statusBMS.bms_state = BMS_ERROR;
		//statusBMS.hotSwapRole = HOTSWAP_ERROR;
	}
	else if(statusBMS.error_map == 0 && statusBMS.bms_state == BMS_ERROR){
		// No Errors found - disable LED and set state to idle (will be corrected later)
		cyhal_gpio_write(BMS_LED3_RD, 0);
		statusBMS.bms_state = BMS_IDLE;
	}
	else{
		// Blink red LED if warning bit is set, else keep it off
		if(getMsgBit(MSG_INT_BATTERY_LOW) == 1 &&
				(TIMER_COUNTER_1MS > (batteryLowWarningTimestamp +
									  BATTERY_LOW_WARNING_DELAY_TIME))){
			// Blink red LED with different on and off times.
			// At beginning the LED is witch on (first if),
			// after BLINK_TIME_LED_RED_ON_TIME the LED is kept off (second if)
			// until the cycle time BLINK_TIME_LED_RED has passed
			if(TIMER_COUNTER_1MS > (blinkTimeLedRedTimestamp + BLINK_TIME_LED_RED)){
				blinkTimeLedRedTimestamp = TIMER_COUNTER_1MS;
				cyhal_gpio_write(BMS_LED3_RD, 1);
			}
			if(TIMER_COUNTER_1MS > (blinkTimeLedRedTimestamp + (BLINK_TIME_LED_RED_ON_TIME))){
				cyhal_gpio_write(BMS_LED3_RD, 0);
			}
		}
		else if(getMsgBit(MSG_INT_BATTERY_LOW) == 0){
			/*PRINTF_DIAGNOSTIC("map %ld, getMSG %d, Bit %d",
					statusBMS.message_map, getMsgBit(MSG_INT_BATTERY_LOW),
					MSG_INT_BATTERY_LOW);*/
			// In normal operation this keeps the timestamp updated.
			// This way the above cause will only trigger if
			// the SoC stays to low for BATTERY_LOW_WARNING_DELAY_TIME
			batteryLowWarningTimestamp = TIMER_COUNTER_1MS;
			cyhal_gpio_write(BMS_LED3_RD, 0);
		}
	}

	// Latch errors and messages till next CAN/PC transmission and till next switch on
	latched_error_map_switch_on |= statusBMS.error_map;
	latched_error_map_ext_com |= statusBMS.error_map;
	latched_message_map |= statusBMS.message_map;
}

//****************************************************************************
// diagnostic_printStatusFlags
// Check error and/or warning bitmap and report failures to debug
//****************************************************************************
void diagnostic_printStatusFlags(uint32_t bitmap_error, uint32_t bitmap_warning,
		uint8_t newLineForEachError, uint8_t onlyPrintIfDataChanged)
{
	static uint8_t last_error_map;
	static uint8_t last_warning_map;

	// Ignore certain flags
	bitmap_warning &= ~((uint32_t)MSG_SSW_INFO_CHARGEPUMPR_READY);

	if(bitmap_error != 0 && (onlyPrintIfDataChanged == 0 ||
			(onlyPrintIfDataChanged == 1 && last_error_map != bitmap_error))){
		if(newLineForEachError == 0)
			PRINTF_CYCLIC_DEBUG_MSGS("[ERROR]:");

		if(bitmap_error & ERR_SSW_INTERRUPT){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_INTERRUPT");
		}
		if(bitmap_error & ERR_SSW_VBAT_UNDERVOLTAGE){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_VBAT_UNDERVOLTAGE");
		}
		if(bitmap_error & ERR_SSW_VBAT_OVERVOLTAGE){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_VBAT_OVERVOLTAGE");
		}
		if(bitmap_error & ERR_SSW_VDD_UNDERVOLTAGE){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_VDD_UNDERVOLTAGE");
		}
		if(bitmap_error & ERR_SSW_CHIP_OVERTEMPERATURE){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_CHIP_OVERTEMPERATURE");
		}
		if(bitmap_error & ERR_SSW_VDS_OVERVOLTAGE_A){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_VDS_OVERVOLTAGE_A");
		}
		if(bitmap_error & ERR_SSW_VGS_UNDERVOLTAGE_A){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_VGS_UNDERVOLTAGE_A");
		}
		if(bitmap_error & ERR_SSW_VDS_OVERVOLTAGE_B){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_VDS_OVERVOLTAGE_B");
		}
		if(bitmap_error & ERR_SSW_VGS_UNDERVOLTAGE_B){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_VGS_UNDERVOLTAGE_B");
		}
		if(bitmap_error & ERR_SSW_OVERCURRENT){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_OVERCURRENT");
		}
		if(bitmap_error & ERR_SSW_CHARGEPUMP_UNDERVOLTAGE){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_CHARGEPUMP_UNDERVOLTAGE");
		}
		if(bitmap_error & ERR_SSW_SAVESTATE_ENABLED){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_SAVESTATE_ENABLED");
		}
		if(bitmap_error & ERR_CAF_TLE9012_INTERRUPT){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_TLE9012_INTERRUPT");
		}
		if(bitmap_error & ERR_CAF_OL_ERR){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_OL_ERR");
		}
		if(bitmap_error & ERR_CAF_INT_IC_ERR){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_INT_IC_ERR");
		}
		if(bitmap_error & ERR_CAF_REG_CRC_ERR){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_REG_CRC_ERR");
		}
		if(bitmap_error & ERR_CAF_EXT_T_ERR){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_EXT_T_ERR");
		}
		if(bitmap_error & ERR_CAF_INT_OT){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_INT_OT");
		}
		if(bitmap_error & ERR_CAF_CELL_UV){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_CELL_UV");
		}
		if(bitmap_error & ERR_CAF_CELL_OV){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_CELL_OV");
		}
		if(bitmap_error & ERR_CAF_BAL_UC){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_BAL_UC");
		}
		if(bitmap_error & ERR_CAF_BAL_OC){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_BAL_OC");
		}
		if(bitmap_error & ERR_CAF_PS_SLEEP){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_PS_SLEEP");
		}
		if(bitmap_error & ERR_CAF_ADC_ERR){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" CAF_ADC_ERR");
		}
		if(bitmap_error & ERR_INT_TLE9012_QUEUE_FAILED){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" INT_TLE9012_QUEUE_FAILED");
		}
		if(bitmap_error & ERR_INT_OVERCURRENT){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" INT_OVERCURRENT");
		}
		if(bitmap_error & ERR_INT_SWITCH_ON_VETO_RECEIVED){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" INT_SWITCH_ON_VETO_RECEIVED");
		}
		if(bitmap_error & ERR_INT_ADC_COM_TIMEOUT){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" INT_ADC_COM_TIMEOUT");
		}
		if(bitmap_error & ERR_INT_VBAT_OV){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" INT_VBAT_OV");
		}
		if(bitmap_error & ERR_INT_VBAT_UV){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" INT_VBAT_UV");
		}
		if(bitmap_error & ERR_INT_TLE9012_COM_ERROR){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[FAILURE]: ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" INT_TLE9012_COM_ERROR");
		}

		// Store last state of bitmap
		last_error_map = bitmap_error;
	}

	if(bitmap_warning != 0 && (onlyPrintIfDataChanged == 0 ||
			(onlyPrintIfDataChanged == 1 && last_warning_map != bitmap_warning))){
		if(newLineForEachError == 0)
			PRINTF_CYCLIC_DEBUG_MSGS(" [WARN]: ");

		if(bitmap_warning & MSG_SSW_WARN_LOSSOFGROUND_CP){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_LOSSOFGROUND_CP");
		}
		if(bitmap_warning & MSG_SSW_WARN_LOSSOFGROUND_D){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_LOSSOFGROUND_D");
		}
		if(bitmap_warning & MSG_SSW_WARN_LOSSOFGROUND_A){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_LOSSOFGROUND_A");
		}
		if(bitmap_warning & MSG_SSW_WARN_OVERTEMP){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_OVERTEMP");
		}
		if(bitmap_warning & MSG_SSW_WARN_MEMFAIL){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_MEMFAIL");
		}
		if(bitmap_warning & MSG_SSW_INFO_SPI_ADDRESS_NOT_AVAIL){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_SPI_ADDR_NOT_AVAIL");
		}
		if(bitmap_warning & MSG_SSW_INFO_SOURCE_OVERVOLTAGE_A){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_SOURCE_OVERVOLT_A");
		}
		if(bitmap_warning & MSG_SSW_INFO_SOURCE_OVERVOLTAGE_B){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_SOURCE_OVERVOLT_B");
		}
		if(bitmap_warning & MSG_SSW_INFO_CHARGEPUMPR_READY){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" SSW_CHARGEPUMP_READY");
		}
		if(bitmap_warning & MSG_CAF_BAL_ACTIVE){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" BAL_ACTIVE");
		}
		if(bitmap_warning & MSG_INT_BATTERY_LOW){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" BAT_LOW");
		}
		if(bitmap_warning & MSG_INT_EXT_LOAD_V_ON){
			if(newLineForEachError){PRINTF_CYCLIC_DEBUG_MSGS("\r\n[WARN]:  ");}
			PRINTF_CYCLIC_DEBUG_MSGS(" LOAD_V_ON");
		}

		// Store last state of bitmap
		last_warning_map = bitmap_warning;
	}

	if(newLineForEachError)
		PRINTF_CYCLIC_DEBUG_MSGS("\r\n");
}
