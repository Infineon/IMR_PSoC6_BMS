/*
 * safety_switch.c
 * 		Abstraction off all safety switch related functions
 * 		that need to be accessed from main.
 * 		This way the driver library does not need to be implemented hard
 * 		into main file. Note that diagnostics are however used directly
 * 		(not through this file).
 *
 *  Created on: 21 Nov 2022
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

#include <stdio.h>

#include "cycfg.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "BMS_2ED4820EM/Driver_2ED4820.h"
#include "ram_public.h"
#include "global_management.h"

/*******************************************************************************
* DEFINITIONS
*******************************************************************************/
#define PRINTF_SAFETY_SWITCH(p_frm, ...)		printf(p_frm, ##__VA_ARGS__)

/*******************************************************************************
* FUNCTIONS
*******************************************************************************/

//****************************************************************************
// initSwitchDriver - startup and setup of the MOSFET driver
//****************************************************************************
void initSwitchDriver(){
	// Init
	PRINTF_SAFETY_SWITCH("Init MOSFET Driver:\r\n");
	driverInit();

	// Settings
	setCurrentSenseShuntConfig(CSSC_HIGHSIDE);
	setCrossControl(CC_OFF);
	setBlankTime(BT_100US, BT_100US);
	setFilterTime(FT_5US, FT_5US);
	setCurrentSenseAmpGain(CSAG_10VV);
	setCurrentSenseLoadAdjust(CSLA_GT100PF);
	setCurrentSenseOverCurrentThres(CSOCT_0_3);
	setDrainSourceOvervoltageThres(DSOT_600MV, DSOT_600MV); //(DSOT_200MV, DSOT_200MV);
	setDrainSourceOvervoltageDeactivation(DSOD_DISABLED, DSOD_DISABLED); //(DSOD_ENABLED, DSOD_ENABLED);
	//setVbatOvervoltageRestartTime(VORT_10US);
	//setVbatUndervoltageRestartTime(VURT_1MS);

	// Print configuration to UART
	printConfig(USE_REG_RENEW);
}


//****************************************************************************
// checkSwitchErrWarnState - Check for errors and problem indicators concerning
// Switch and Power state. If an error is detected (error interrupt or internal
// error detected) try to reset.
//****************************************************************************
void checkSwitchErrWarnState(uint8_t driverErrorPin){
	if(driverErrorPin || isProblemDetected()){
		// Print currently active errors
		//PRINTF_SAFETY_SWITCH("\tReset safety switch errors \r\n");
		//printStatusFlags(USE_REG_RENEW);

		// Reset errors
		clearProblemDetected();
		clearStatusFlags();

		// If error was not cleared write message
		//if(isProblemDetected())
			//PRINTF_SAFETY_SWITCH("\tSafety switch error prevails after reset\r\n");
	}
}

//****************************************************************************
// getSwitchState
// Check if both channels are ON.
// Returns 0 if switch is OFF (non conducting)
// and 1 if switch is ON (conducting)
//****************************************************************************
uint8_t getSwitchState(){
	return switchGetState(USE_REG_RENEW);
}

//****************************************************************************
// getSwitchState_precharge - Check if channel B is ON.
// Returns 0 if switch is OFF (non conducting) and
// 1 if switch is ON (conducting). Anything else is considered error
// useCachedReg:
// If set to 0 the last information about the switch state is used,
// if set to 1 the information is requested anew
//****************************************************************************
uint8_t getSwitchState_precharge(uint8_t useCachedReg){
	if(useCachedReg == 0){
		return getStateChannelB(USE_REG_CACHED);
	}
	else if(useCachedReg == 1){
		return getStateChannelB(USE_REG_RENEW);
	}
	return 255;
}
//****************************************************************************
// getSwitchState_mainpath - Check if channel A is ON.
// Returns 0 if switch is OFF (non conducting) and
// 1 if switch is ON (conducting). Anything else is considered error
// useCachedReg:
// If set to 0 the last information about the switch state is used,
// if set to 1 the information is requested anew
//****************************************************************************
uint8_t getSwitchState_mainpath(uint8_t useCachedReg){
	if(useCachedReg == 0){
		return getStateChannelA(USE_REG_CACHED);
	}
	else if(useCachedReg == 1){
		return getStateChannelA(USE_REG_RENEW);
	}
	return 255;
}

//****************************************************************************
// setSwitchState_precharge
// Sets the precharge channel B ON if state is 1 or off if state is 0
//****************************************************************************
uint8_t setSwitchState_precharge(uint8_t state){
	// Clear errors
	clearProblemDetected();

	if(state == 1)
		switchCH_B_On(USE_REG_RENEW);
	else if(state == 0)
		switchCH_B_Off(USE_REG_RENEW);

	// Return if a error happened
	return isProblemDetected();
}
//****************************************************************************
// setSwitchState_mainpath
// Sets the main channel A ON if state is 1 or off if state is 0
//****************************************************************************
uint8_t setSwitchState_mainpath(uint8_t state){
	// Clear errors
	clearProblemDetected();

	if(state == 1)
		switchCH_A_On(USE_REG_RENEW);
	else if(state == 0)
		switchCH_A_Off(USE_REG_RENEW);

	// Return if a error happened
	return isProblemDetected();
}

//****************************************************************************
// setSwitchIsolating
// Stop current thru the switches. Returns 1 if error and 0 if ok
//****************************************************************************
uint8_t setSwitchIsolating(){
	// Store last state
	uint8_t enStateLast = getSwitchState();

	// Clear errors
	clearProblemDetected();

	// Switch off
	switchOff(USE_REG_RENEW);
	//cyhal_system_delay_ms(1);

	// Write message if mosfet is switched off
	if(getSwitchState() != enStateLast){
		/*PRINTF_SAFETY_SWITCH("Set Safety Switch non-conducting (Error=%d)\r\n",
				isProblemDetected());*/
	}
	else if(isProblemDetected() != 0){
		PRINTF_SAFETY_SWITCH("Problem with setting Safety Switch non-conducting!\r\n");
	}

	// Return if a error happened
	return isProblemDetected();
}

//****************************************************************************
// Set save-state pin to 0 = Save state enabled, 1 = Save state disabled
//****************************************************************************
void setSafestate(uint8_t disableSS){
	setSafestateEnable(disableSS);
}

void printSwitchStatusFlags(){
	printStatusFlags(USE_REG_RENEW);
}

/* [] END OF FILE */
