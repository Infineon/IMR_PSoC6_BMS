/*
 * Driver_2ED4820.c
 *
 *	Library for the EiceDRIVER APD 2ED4820-EM 48 V smart high-side MOSFET gate driver.
 *	Created by R. Santeler @ MCI EAL\Infineon 2022
 *	(based on sample code from schwarzg created on 4 Mar 2020)
 *
 *  VERSION 1.00 (23.02.2022)
 *
 *  USAGE (see "example.c_" for details):
 * 			- Implement target specific functions in target.h/.c
 * 			- Implement ISR for Interrupt and its behavior in main code
 * 				(see examples.c_ for hint on how to implement)
 * 			- Init Driver with driverInit() command and
 * 				use set...() commands to configure it
 * 			- Use switchON() and switchOFF() to control driver simple or
 * 				use the implemented function for advanced control
 * 				(see comment of functions)
 *
 * 	NOTE: 	isProblemDetected() can be used to check if
 * 	a action (e.g. switchON()) was successful.
 * 	After the check clearProblemDetected() should be used
 * 	to reset the flag(also clearStatusFlags() resets this).
 *
 * -----------------------------------------------
 * Abbreviation Description
 * ------------------------------------------------
 * uvrt, vurt   vbat undervoltage restart time
 * ovrt, vort   vbat overvoltage restart time
 * dsov, dsot   vds overvoltage threshold ch. A/B
 * sst, sso     safe state ch. A/B
 * blk, bt      blanking time ch. A/B
 * flt, ft      filter time ch. A/B
 * pt           precharge time in ms for 'on' command
 * xctr, cc     channel cross control
 * cshs, cssc   current sense on high side
 * csld, csla   current sense output load (1: >100pF)
 * csag         current sense amplifier gain
 * octh, csoct  overcurrent threshold
 * -----------------------------------------------
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

#include "Driver_2ED4820.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h> // includes strtol

#include "registers.h"
#include "target.h"

// Used to check if an action was successful (e.g. switchON()) and
// if failure flags are set
uint8_t problemDetected = 0;

//****************************************************************************
// setEnable
//
// sets the ENABLE pin 1=high or 0=low
//****************************************************************************
void setEnable(uint8_t state)
{
	if (state == 0) {
		enable_setLOW();
	}
	else { // enable driver
		enable_setHIGH();
	}
}

//****************************************************************************
// setSafestateEnable
//
// sets the SAF pin 1=disabled or 0=enabled
//****************************************************************************
void setSafestateEnable(uint8_t state)
{
	if (state == 0) {
		saveStateEn_setLOW();
	}
	else {
		saveStateEn_setHIGH();
	}
}

//****************************************************************************
// setChannelA
//
// set state of channel A on or off, and
// check if it really switched (you may avoid check if high speed is needed!)
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setChannelA(uint8_t state, uint8_t check, uint8_t useCachedReg)
{
	if(useCachedReg == 0)
		readAllRegistersToCache(); //readRegister(MOS_CHS_CTRL);

	// Set Channel
	if (state == 1) {
		setBits(MOS_CHS_CTRL, MOSONCH_A, true); // switch on channel A
	}
	else {
		clearBits(MOS_CHS_CTRL, MOSONCH_A, true); // switch off channel A
	}

	// Check if channel really was switched on
	if(check) {
		readAllRegisters();
		if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_A) != state){
			DEBUGPRINTF("Couldn't switch channel A to %d\r\n", state);
			problemDetected = 1;
		}
	}
}

//****************************************************************************
// setChannelB
//
// set state of channel B on or off, and
// check if it really switched (you may avoid check if high speed is needed!)
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setChannelB(uint8_t state, uint8_t check, uint8_t useCachedReg)
{
	if(useCachedReg == 0)
		readAllRegistersToCache(); //readRegister(MOS_CHS_CTRL);

	// Set Channel
	if (state == 1) {
		setBits(MOS_CHS_CTRL, MOSONCH_B, true); // switch on channel B
	}
	else {
		clearBits(MOS_CHS_CTRL, MOSONCH_B, true); // switch off channel B
	}

	// Check if channel really was switched on
	if(check) {
		readAllRegisters();
		if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_B) != state){
			DEBUGPRINTF("Couldn't switch channel B to %d\r\n", state);
			problemDetected = 1;
		}
	}
}

//****************************************************************************
// burstChannelA
//
// generate number of pulses as fast as possible on channel A
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void burstChannelA(uint32_t numPulses, uint8_t useCachedReg)
{
	static uint32_t i = 0;

	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	if (isEnabled())
	{
		if (numPulses > 0 && numPulses <= 10000)
		{
			for (i = 0; i < numPulses; i++)
			{
				setBits(MOS_CHS_CTRL, MOSONCH_A, true);	// switch on channel A
				clearBits(MOS_CHS_CTRL, MOSONCH_A, true); // switch off channel A
			}
		}
		else {
			problemDetected = 1;
			DEBUGPRINTF("\t[ERROR]: Pulse count out of range\r\n");
		}
	}
	else {
		problemDetected = 1;
		DEBUGPRINTF("\t[ERROR]: Driver is disabled\r\n");
	}
}

//****************************************************************************
// burstChannelB
//
// generate number of pulses as fast as possible on channel B
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void burstChannelB(uint32_t numPulses, uint8_t useCachedReg)
{
	static uint32_t i = 0;

	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	if (isEnabled())
	{
		if (numPulses > 0 && numPulses <= 10000)
		{
			for (i = 0; i < numPulses; i++)
			{
				setBits(MOS_CHS_CTRL, MOSONCH_B, true);	// switch on channel B
				clearBits(MOS_CHS_CTRL, MOSONCH_B, true); // switch off channel B
			}
		}
		else {
			problemDetected = 1;
			DEBUGPRINTF("\t[ERROR]: Pulse count out of range\r\n");
		}
	}
	else {
		problemDetected = 1;
		DEBUGPRINTF("\t[ERROR]: Driver is disabled\r\n");
	}
}

//****************************************************************************
// burstChannels
//
// generate number of pulses as fast as possible on channel B
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void burstChannels(uint32_t numPulses, uint8_t useCachedReg)
{
	static uint32_t i = 0;

	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	if (isEnabled())
	{
		if (numPulses > 0 && numPulses <= 1000000000000) //10000
		{
			for (i = 0; i < numPulses; i++)
			{
				setBits(MOS_CHS_CTRL, MOSONCH_A, true);	// switch on channel A
				setBits(MOS_CHS_CTRL, MOSONCH_B, true);	// switch on channel B
				clearBits(MOS_CHS_CTRL, MOSONCH_B, true); // switch off channel B
				clearStatusFlags();
			}
		}
		else {
			problemDetected = 1;
			DEBUGPRINTF("\t[ERROR]: Pulse count out of range\r\n");
		}
	}
	else {
		problemDetected = 1;
		DEBUGPRINTF("\t[ERROR]: Driver is disabled\r\n");
	}
}

//****************************************************************************
// burstChannelsFast
//
// generate number of pulses as fast as possible on channel B
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void burstChannelsFast(uint32_t numPulses, uint8_t useCachedReg)
{
	static uint32_t i = 0;

	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	if (isEnabled())
	{
		if (numPulses > 0 && numPulses <= 10000)
		{
			uint8_t data;
			data = 255; //g_reg_data[MOS_CHS_CTRL]; // get current register data

			for (i = 0; i < numPulses; i++)
			{
				data = 255; //g_reg_data[MOS_CHS_CTRL]; // get current register data
			    data |= MOSONCH_A | MOSONCH_B;   // set bits
			    spiWrite(MOS_CHS_CTRL, data);    // write new register data

			    //data = 255; //g_reg_data[MOS_CHS_CTRL]; // get current register data
				//data &= ~(MOSONCH_A | MOSONCH_B);// clear bits
				//spiWrite(MOS_CHS_CTRL, data);    // write new register data

				// Fast brute force approach:
			    // Set all bits that have to do with failure clearing
				spiWrite(FAILURECLEAN, 0xFF);
				/*spiWrite(MOS_CHS_CTRL, (getRegisterCacheData(MOS_CHS_CTRL) |
						MOS_CTRL_FAILURE_CLEAR));*/
				//problemDetected = 0;
			}
		}
		else {
			problemDetected = 1;
			DEBUGPRINTF("\t[ERROR]: Pulse count out of range\r\n");
		}
	}
	else {
		problemDetected = 1;
		DEBUGPRINTF("\t[ERROR]: Driver is disabled\r\n");
	}
}

//****************************************************************************
// switchCH_A_On
//
// enable, initialize and switch on both channels
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void switchCH_A_On(uint8_t useCachedReg)
{
	if (isEnabled()){
		// Get register data if needed or use cached
		if(useCachedReg == 0)
			readRegister(MOS_CHS_CTRL);

		// Switch on chan A
		setBits(MOS_CHS_CTRL, MOSONCH_A, true);

		// Check if channel really was switched on
		readRegister(MOS_CHS_CTRL);
		if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_A) == 0){ // channel A is off
			problemDetected = 1;
			DEBUGPRINTF("Couldn't switch on channel A\r\n");
		}
	}
	else {
		problemDetected = 1;
		DEBUGPRINTF("\t[ERROR]: Driver is disabled\r\n");
	}
}

//****************************************************************************
// switchCH_B_On
//
// enable, initialize and switch on both channels
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void switchCH_B_On(uint8_t useCachedReg)
{
	if (isEnabled()){
		// Get register data if needed or use cached
		if(useCachedReg == 0)
			readRegister(MOS_CHS_CTRL);

		// Switch on chan B
		setBits(MOS_CHS_CTRL, MOSONCH_B, true);

		// Check if channel really was switched on
		readRegister(MOS_CHS_CTRL);
		if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_B) == 0){ // channel B is off
			problemDetected = 1;
			DEBUGPRINTF("Couldn't switch on channel B\r\n");
		}
	}
	else {
		problemDetected = 1;
		DEBUGPRINTF("\t[ERROR]: Driver is disabled\r\n");
	}
}

//****************************************************************************
// switchCH_A_Off
//
// switch off both channels and disable driver
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void switchCH_A_Off(uint8_t useCachedReg)
{
	// Get register data if needed or use cached
	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	// Switch off chan A
	clearBits(MOS_CHS_CTRL, MOSONCH_A, true);

	// Check if channel really was switched off
	readRegister(MOS_CHS_CTRL);
	if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_A) != 0){ // channel A is on
		problemDetected = 1;
		DEBUGPRINTF("Couldn't switch off channel A\r\n");
	}
}

//****************************************************************************
// switchCH_B_Off
//
// switch off both channels and disable driver
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void switchCH_B_Off(uint8_t useCachedReg)
{
	// Get register data if needed or use cached
	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	// Switch off chan B
	clearBits(MOS_CHS_CTRL, MOSONCH_B, true);

	// Check if channel really was switched off
	readRegister(MOS_CHS_CTRL);
	if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_B) != 0){ // channel B is on
		problemDetected = 1;
		DEBUGPRINTF("Couldn't switch off channel B\r\n");
	}
}

//****************************************************************************
// switchOn
//
// enable, initialize and switch on both channels
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void switchOn(uint8_t useCachedReg)
{
	if (isEnabled()){
		// Get register data if needed or use cached
		if(useCachedReg == 0)
			readRegister(MOS_CHS_CTRL);

		// Switch on chan A & B
		setBits(MOS_CHS_CTRL, MOSONCH_A | MOSONCH_B, true);

		// Check if channel really was switched on
		readRegister(MOS_CHS_CTRL);
		if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_B) == 0){ // channel B is off
			problemDetected = 1;
			DEBUGPRINTF("Couldn't switch on channel B\r\n");
		}
		if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_A) == 0){ // channel A is off
			problemDetected = 1;
			DEBUGPRINTF("Couldn't switch on channel A\r\n");
		}
	}
	else {
		problemDetected = 1;
		DEBUGPRINTF("\t[ERROR]: Driver is disabled\r\n");
	}
}

//****************************************************************************
// switchOff
//
// switch off both channels and disable driver
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void switchOff(uint8_t useCachedReg)
{
	// Get register data if needed or use cached
	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	// Switch off chan A & B
	clearBits(MOS_CHS_CTRL, MOSONCH_A | MOSONCH_B, true); // switch on chan A & B

	// check if channel really was switched on
	readRegister(MOS_CHS_CTRL);
	if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_B) != 0){ // channel B is on
		problemDetected = 1;
		DEBUGPRINTF("Couldn't switch off channel B\r\n");
	}
	if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_A) != 0){ // channel A is on
		problemDetected = 1;
		DEBUGPRINTF("Couldn't switch off channel A\r\n");
	}
}

//****************************************************************************
// switchGetState
//
// check if both channels are ON. Returns 0 if switch is OFF (non conducting)
// and 1 if switch is ON (conducting)
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
uint8_t switchGetState(uint8_t useCachedReg)
{
	// Get register data if needed or use cached
	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	// check if channel is switched on
	readRegister(MOS_CHS_CTRL);
	if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_B) != 0 &&
		(getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_A) != 0){ // channel A & B is on
		return 1;
	}

	// If its not ON it is OFF
	return 0;
}

//****************************************************************************
// getStateChannelA
//
// check if channel A is ON. Returns 0 if switch is OFF (non conducting)
// and 1 if switch is ON (conducting)
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
uint8_t getStateChannelA(uint8_t useCachedReg)
{
	// Get register data if needed or use cached
	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	// check if channel is switched on
	readRegister(MOS_CHS_CTRL);
	if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_A) != 0){ // channel A is on
		return 1;
	}

	// If its not ON it is OFF
	return 0;
}

//****************************************************************************
// getStateChannelB
//
// check if channel B is ON. Returns 0 if switch is OFF (non conducting)
// and 1 if switch is ON (conducting)
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
uint8_t getStateChannelB(uint8_t useCachedReg)
{
	// Get register data if needed or use cached
	if(useCachedReg == 0)
		readRegister(MOS_CHS_CTRL);

	// check if channel is switched on
	readRegister(MOS_CHS_CTRL);
	if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_B) != 0){ // channel B is on
		return 1;
	}

	// If its not ON it is OFF
	return 0;
}

//****************************************************************************
// clearStatusFlags - Try to reset errors and check if they return
// (check problemDetected for result)
//****************************************************************************
void clearStatusFlags(void)
{
	static uint8_t reg_data;

	if (isEnabled())
	{
		// brute force approach:
		// set all bits that have to do with failure clearing
		spiWrite(FAILURECLEAN, 0xFF);
		spiWrite(MOS_CHS_CTRL, (getRegisterCacheData(MOS_CHS_CTRL) |
				MOS_CTRL_FAILURE_CLEAR));
		problemDetected = 0;

		// check if there are still failures present
		readAllRegisters();
		reg_data = getRegisterCacheData(STDIAG);

		// if error flags not 0
		if (reg_data & FAILURE) // still failures present
		{
			//DEBUGPRINTF("Couldn't clear all failures: \r\n");
			problemDetected = 1; // trigger reporting of remaining failures
		}
	}
	else {
		problemDetected = 1;
		DEBUGPRINTF("\t[ERROR]: Driver is disabled\r\n");
	}
}


//****************************************************************************
// setConfig
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//**********************************************************************
void setConfig(uint8_t setting, uint8_t reg_num, uint8_t bitmask,
		uint8_t bitpos, uint8_t useCachedReg)
{
	static uint8_t reg_data;
	static uint8_t max_setting;

	max_setting = (bitmask >> bitpos);

	if (isEnabled()){
		// Get register data if needed or use cached
		if(useCachedReg == 0)
			readRegister(reg_num);

		// Check setting range and write it
		if (setting >= 0 && setting <= max_setting)
		{
			reg_data = getRegisterCacheData(reg_num); // get current register data
			reg_data &= ~bitmask;				 // clear configuration bits
			reg_data |= (setting << bitpos);	 // set selected configuration

			spiWrite(reg_num, reg_data); // write new register data
			setRegisterCacheData(reg_num, reg_data); // update shadow register
		}
		else
		{
			problemDetected = 1;
			DEBUGPRINTF("\t[ERROR]: Setting out of range\r\n");
		}
	}
	else {
		problemDetected = 1;
		DEBUGPRINTF("\t[ERROR]: Driver is disabled\r\n");
	}
}

//****************************************************************************
// setConfigBit
// only use this for latched bits - otherwise this will save the reset byte in cache
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setConfigBit(uint8_t state, uint8_t reg_num, uint8_t bit, uint8_t useCachedReg)
{
	if (isEnabled()){
		// Get register data if needed or use cached
		if(useCachedReg == 0)
			readRegister(reg_num);

		// Set state
		if (state == 0)
			clearBits(reg_num, bit, true);
		else if (state == 1)
			setBits(reg_num, bit, true);
		else{
			problemDetected = 1;
			DEBUGPRINTF("\t[ERROR]: SetConfigBit - Invalid state\r\n");
		}
	}
}


//****************************************************************************
// setCSAG (differential gain of the current sense amplifier)
// 0=10V/V, 1=15V/V, 20V/V, 25V/V, 31.5V/V, 35V/V, 40V/V(default), 7=47.7V/V
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setCurrentSenseAmpGain(CurrentSenseAmpGain_state CSAG)
{
	setConfig((uint8_t)CSAG, CSAG_OCTH, CSAG_MASK, CSAG_POS, false);
}

//****************************************************************************
// setOCTH (CSA Overcurrent detection threshold)
// 0=+-0.1*VDD, 1=+-0.2*VDD, 2=+-0.25*VDD, 3=+-0.3*VDD
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setCurrentSenseOverCurrentThres(CurrentSenseOverCurrentThres_state CSOCT)
{
	setConfig((uint8_t)CSOCT, CSAG_OCTH, OCTH_MASK, OCTH_POS, false);
}

//****************************************************************************
// setUVRT (VBAT undervoltage auto-restart time)
// 0=1ms (default), 1=5ms, 2=20ms, 3=50ms
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setVbatUndervoltageRestartTime(VbatUndervoltageRestartTime_state VURT)
{
	setConfig((uint8_t)VURT, VBATOVUVRST, VBATUVARST_MASK, VBATUVARST_POS, false);
}

//****************************************************************************
// setOVRT (VBAT overvoltage auto-restart time)
// 0=10us (default), 1=50us, 2=200us, 3=1ms
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setVbatOvervoltageRestartTime(VbatOvervoltageRestartTime_state VORT)
{
	setConfig((uint8_t)VORT, VBATOVUVRST, VBATOVARST_MASK, VBATOVARST_POS, false);
}

//****************************************************************************
// setDSOV (Drain-source overvoltage threshold +/-)
//  0=100mV, 1=150mV, 200mV(default), 250mV, 300mV, 400mV, 500mV, 7=600mV
// -1=leave channel as is
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setDrainSourceOvervoltageThres(DrainSourceOvervoltageThres_state DSOT_ChA,
		DrainSourceOvervoltageThres_state DSOT_ChB)
{
	if (DSOT_ChA != DSOT_IGNORE) {// Channel A
		setConfig((uint8_t)DSOT_ChA, VDSTHA_B, VDSTH_A_MASK, VDSTH_A_POS, false);
	}

	if (DSOT_ChB != DSOT_IGNORE) {// Channel B
		setConfig((uint8_t)DSOT_ChB, VDSTHA_B, VDSTH_B_MASK, VDSTH_B_POS, false);
	}
}

//****************************************************************************
// setVDS_SS (Drain-source channel de-activation condition)
//  0 = Ch is de-activated in case of drain-source overvolt. (default),
//  1 = Ch is not de-activated in case of drain-source overvoltage
// -1 = leave channel as is
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setDrainSourceOvervoltageDeactivation
(DrainSourceOvervoltageDeactivation_state DSOD_A,
		DrainSourceOvervoltageDeactivation_state DSOD_B)
{
	// VDS Channel A safe state: OFF when 0, ON when 1 (default)
	if (DSOD_A != DSOD_IGNORE) {
		setConfigBit((uint8_t)DSOD_A, VDSTHA_B, VDSA_SS, false);
	}

	// VDS Channel B safe state: OFF when 0, ON when 1 (default)
	if (DSOD_B != DSOD_IGNORE) {
		setConfigBit((uint8_t)DSOD_B, VDSTHA_B, VDSB_SS, false);
	}
}

//****************************************************************************
// setBLK (MOSFET voltage blank time for failure detections)
// 0=10us(default), 1=20us, 2=50us, 3=100us
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setBlankTime(BlankTime_state BT_ChA, BlankTime_state BT_ChB)
{
	if (BT_ChA != BT_IGNORE) {
		setConfig((uint8_t)BT_ChA, MOSFLTBLKA_B, MOSBLK_A_MASK, MOSBLK_A_POS, false);
	}

	if ((uint8_t)BT_ChB != BT_IGNORE) {
		setConfig((uint8_t)BT_ChB, MOSFLTBLKA_B, MOSBLK_B_MASK, MOSBLK_B_POS, false);
	}
}

//****************************************************************************
// setFLT (MOSFET voltage filter time)
// 0=0.5us, 1=1us(default), 2=2us, 3=5us
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setFilterTime(FilterTime_state FT_ChA, FilterTime_state FT_ChB)
{
	if (FT_ChA != FT_IGNORE) {
		setConfig((uint8_t)FT_ChA, MOSFLTBLKA_B, MOSFLT_A_MASK, MOSFLT_A_POS, false);
	}

	if (FT_ChB != FT_IGNORE) {
		setConfig((uint8_t)FT_ChB, MOSFLTBLKA_B, MOSFLT_B_MASK, MOSFLT_B_POS, false);
	}
}

//****************************************************************************
// setXCTR (Failures related to one channel affect the other channel)
// 0 = OFF, 1 = ON (default)
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setCrossControl(CrossControl_state CC)
{
	setConfigBit((uint8_t)CC, MOS_CHS_CTRL, CHCRCTRL, false);
}

//****************************************************************************
// setCSHS (Control signal to select the shunt configuration Low or HighSide)
// 0 = Low (default), 1 = High
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setCurrentSenseShuntConfig(CurrentSenseShuntConfig_state CSSC)
{
	setConfigBit((uint8_t)CSSC, CSAG_OCTH, CSA_HSS, false);
}

//****************************************************************************
// setCSLD (Configures current sense stage depending on the output capacitor)
// 0 = <100pF (default), 1 = >100pF
//
// useCachedReg -> 0 to read needed register anew
//              -> 1 to use result of last readAllRegistersToCache() method
//****************************************************************************
void setCurrentSenseLoadAdjust(CurrentSenseLoadAdjust_state CSLA)
{
	setConfigBit((uint8_t)CSLA, CSAG_OCTH, CSA_COUTSEL, false);
}

//****************************************************************************
// driverInit
//
// initialize the driver. Do not forget to write necessary settings after init!
//****************************************************************************
void driverInit(void)
{
	//// Save current configuration for initialization
	if (!isEnabled()){
		saveStateEn_setHIGH();

		// Disable interrupt to suppress vdd undervoltage message
		disable_interruptPin();

		// Set enable pin high
		enable_setHIGH();

		//uint8_t en = isEnabled();
		//DEBUGPRINTF("Enable - %d", en);

		//setRegisterDefaultValues();

		// Wait for the driver to power up
		// (original code waited with an while loop for 5000 cycles)
		DELAY_MS(10);

		// Clear VDD_UV failure after reset
		clearStatusFlags();

		// First read of registers to cache
		readAllRegisters();

		//// Apply init configuration
		//spiWrite(VDSTHA_B, getInitRegisterData(VDSTHA_B));
		//spiWrite(MOSFLTBLKA_B, getInitRegisterData(MOSFLTBLKA_B));
		//spiWrite(CSAG_OCTH, getInitRegisterData(CSAG_OCTH));
		//spiWrite(VBATOVUVRST, getInitRegisterData(VBATOVUVRST));

		//// XCTRL setting is just one bit in register MOS_CHS_CTRL
		/*spiWrite(MOS_CHS_CTRL, ((getRegisterCacheData(MOS_CHS_CTRL) & ~CHCRCTRL) |
				getInitRegisterData(MOS_CHS_CTRL)));*/
		//setBits(MOS_CHS_CTRL, CHCRCTRL, true);

		// Current sense amplifier on high side,
		// over current protection least strict,
		// amplifier gain standard 35V/V
		//setBits(CSAG_OCTH, CSA_HSS, true);
		//spiWrite(MOSFLTBLKA_B, 0xff); // blank and filter time at highest setting

		// Current sense amplifier on high side,
		// over current protection least strict,
		// amplifier gain standard 35V/V
		//spiWrite(CSAG_OCTH, 0x3d);

		// Enable interrupt again
		enable_interruptPin();
	}
}

//****************************************************************************
// readAllRegistersToCache
//
// If multiple registers shall be read/written use this beforehand and execute
// commands with "useCachedReg" parameter to avoid unnecessary read operations
//****************************************************************************
void readAllRegistersToCache(void){
	readAllRegisters();
}

//****************************************************************************
// printConfig
//
// print configuration
//****************************************************************************
void printConfig(uint8_t useCachedReg)
{
	static uint8_t data;
	static uint8_t csag_setting;
	static uint8_t octh_setting;

	// VBAT overvoltage restart time
	const char OVRT[4][12] = {
		"0 (10 us)",
		"1 (50 us)",
		"2 (200 us)",
		"3 (1 ms)"};

	// VBAT undervoltage restart time
	const char UVRT[4][12] = {
		"0 (1 ms)",
		"1 (5 ms)",
		"2 (20 ms)",
		"3 (50 ms)"};

	// Mosfet voltage blanking time
	const char BLK[4][12] = {
		"0 (10 us)",
		"1 (20 us)",
		"2 (50 us)",
		"3 (100 us)"};

	// Mosfet voltage filter time
	const char FLT[4][12] = {
		"0 (0.5 us)",
		"1 (1 us)",
		"2 (2 us)",
		"3 (5 us)"};

	// Drain source overvoltage thresholds
	const char VDSOV[8][12] = {
		"0 (100 mV)",
		"1 (150 mV)",
		"2 (200 mV)",
		"3 (250 mV)",
		"4 (300 mV)",
		"5 (400 mV)",
		"6 (500 mV)",
		"7 (600 mV)"};

	// Current sense amplifier gain
	const char CSAG[8][14] = {
		"0 (10 V/V)",
		"1 (15 V/V)",
		"2 (20 V/V)",
		"3 (25 V/V)",
		"4 (31.5 V/V)",
		"5 (35 V/V)",
		"6 (40 V/V)",
		"7 (47.7 V/V)"};

	// Overcurrent detection threshold
	const char OCTH[4][22] = {
		"0 (VDD/2 +- 0.1 VDD)",
		"1 (VDD/2 +- 0.2 VDD)",
		"2 (VDD/2 +- 0.25 VDD)",
		"3 (VDD/2 +- 0.3 VDD)"};

	// Overcurrent thresholds for 50 uOhm shunt in [A] by OCTH and CSAG setting
	// This must be altered for given use case
	//const char OC[4][8][8] = {
	//	{"660 A",   "440 A", "330 A", "264 A", "210 A", "189 A", "165 A", "138 A"},
	//	{"1320 A",  "880 A", "660 A", "528 A", "419 A", "377 A", "330 A", "277 A"},
	//	{"1650 A", "1100 A", "825 A", "660 A", "524 A", "471 A", "413 A", "346 A"},
	//	{"1980 A", "1320 A", "990 A", "792 A", "629 A", "566 A", "495 A", "415 A"}};

	if(useCachedReg == 0)
		readAllRegistersToCache();

	DEBUGPRINTF("\tMOSFET DRIVER 2ED4820 SETUP: \r\n"); // new line
	DEBUGPRINTF("\t---------------------------  VBAT ------------------------------\r\n");

	// print VBAT undervoltage restart time UVRT
	data = getRegisterCacheData(VBATOVUVRST); // get current register data
	data &= VBATUVARST_MASK;			 // clear non UVRT bits
	data = (data >> VBATUVARST_POS);	 // shift bits to LSB position
	DEBUGPRINTF("\tVBAT undervoltage restart time....(uvrt): %s\r\n", UVRT[data]);

	// print VBAT overvoltage restart time OVRT
	data = getRegisterCacheData(VBATOVUVRST); // get current register data
	data &= VBATOVARST_MASK;			 // clear non OVRT bits
	data = (data >> VBATOVARST_POS);	 // shift bits to LSB position
	DEBUGPRINTF("\tVBAT overvoltage restart time.....(ovrt): %s\r\n", OVRT[data]);

	DEBUGPRINTF("\t-------------------------  Channel A  --------------------------\r\n");

	// print VDS overvoltage threshold, channel A
	data = getRegisterCacheData(VDSTHA_B); // get current register data
	data &= VDSTH_A_MASK;			  // clear non FLT_A bits
	data = (data >> VDSTH_A_POS);	  // shift bits to LSB position
	DEBUGPRINTF("\tVDS overvoltage threshold A.....(dsov a): %s\r\n", VDSOV[data]);

	// print VDS overvoltage safe state, channel A
	data = getRegisterCacheData(VDSTHA_B); // get current register data
	if (data & VDSA_SS)				  // bit = 1
	{
		DEBUGPRINTF("\tVDS overvoltage safe state A.....(sst a): 1 (stay activated)\r\n");
	}
	else // bit = 0
	{
		DEBUGPRINTF("\tVDS overvoltage safe state A.....(sst a): 0 (deactivated)\r\n");
	}

	// print Mosfet voltage blanking time BLK, channel A
	data = getRegisterCacheData(MOSFLTBLKA_B); // get current register data
	data &= MOSBLK_A_MASK;				  // clear non BLK_A bits
	data = (data >> MOSBLK_A_POS);		  // shift bits to LSB position
	DEBUGPRINTF("\tMosfet voltage blanking time A...(blk a): %s\r\n", BLK[data]);

	// print Mosfet voltage filter time FLT, channel A
	data = getRegisterCacheData(MOSFLTBLKA_B); // get current register data
	data &= MOSFLT_A_MASK;				  // clear non FLT_A bits
	data = (data >> MOSFLT_A_POS);		  // shift bits to LSB position
	DEBUGPRINTF("\tMosfet voltage filter time A.....(flt a): %s\r\n", FLT[data]);

	DEBUGPRINTF("\t-------------------------  Channel B  --------------------------\r\n");

	// print drain source overvoltage threshold, channel B
	data = getRegisterCacheData(VDSTHA_B); // get current register data
	data &= VDSTH_B_MASK;			  // clear non FLT_A bits
	data = (data >> VDSTH_B_POS);	  // shift bits to LSB position
	DEBUGPRINTF("\tVDS overvoltage threshold B.....(dsov b): %s\r\n", VDSOV[data]);

	// print VDS overvoltage safe state, channel B
	data = getRegisterCacheData(VDSTHA_B); // get current register data
	if (data & VDSB_SS)				  // safe state channel B = 1 (ON)
	{
		DEBUGPRINTF("\tVDS overvoltage safe state B.....(sst b): 1 (stay activated)\r\n");
	}
	else // safe state channel B = 0 (OFF)
	{
		DEBUGPRINTF("\tVDS overvoltage safe state B.....(sst b): 0 (deactivated)\r\n");
	}

	// print Mosfet voltage blanking time BLK, channel B
	data = getRegisterCacheData(MOSFLTBLKA_B); // get current register data
	data &= MOSBLK_B_MASK;				  // clear non BLK_A bits
	data = (data >> MOSBLK_B_POS);		  // shift bits to LSB position
	DEBUGPRINTF("\tMosfet voltage blanking time B...(blk b): %s\r\n", BLK[data]);

	// print Mosfet voltage filter time FLT, channel B
	data = getRegisterCacheData(MOSFLTBLKA_B); // get current register data
	data &= MOSFLT_B_MASK;				  // clear non FLT_A bits
	data = (data >> MOSFLT_B_POS);		  // shift bits to LSB position
	DEBUGPRINTF("\tMosfet voltage filter time B.....(flt b): %s\r\n", FLT[data]);

	//// print precharge time
	/*DEBUGPRINTF("\tPrecharge time.....................(prt): %d ms\r\n",
			getPrechargeTime());*/

	DEBUGPRINTF("\t----------------------------------------------------------------\r\n");

	// print channel cross control
	data = getRegisterCacheData(MOS_CHS_CTRL); // get current register data
	if (data & CHCRCTRL)				  // bit = 1
	{
		DEBUGPRINTF("\tChannel cross control.............(xctr): 1 (activated)\r\n");
	}
	else // bit = 0
	{
		DEBUGPRINTF("\tChannel cross control.............(xctr): 0 (deactivated)\r\n");
	}

	DEBUGPRINTF("\t-----------------------  Current Sense  ------------------------\r\n");

	// print current sense location
	data = getRegisterCacheData(CSAG_OCTH); // get current register data
	if (data & CSA_HSS)				   // bit = 1
	{
		DEBUGPRINTF("\tCurrent sense shunt location......(cshs): 1 (high side)\r\n");
	}
	else // bit = 0
	{
		DEBUGPRINTF("\tCurrent sense shunt location......(cshs): 0 (low side)\r\n");
	}

	// print current sense load
	data = getRegisterCacheData(CSAG_OCTH); // get current register data
	if (data & CSA_COUTSEL)			   // bit = 1
	{
		DEBUGPRINTF("\tCurrent sense output load.........(csld): 1 (>100 pF)\r\n");
	}
	else // bit = 0
	{
		DEBUGPRINTF("\tCurrent sense output load.........(csld): 0 (<100pF)\r\n");
	}

	// print current sense amplifier gain CSAG
	data = getRegisterCacheData(CSAG_OCTH); // get current register data
	data &= CSAG_MASK;				   // clear non CSAG bits
	csag_setting = (data >> CSAG_POS); // shift bits to LSB position
	DEBUGPRINTF("\tCurrent sense amplifier gain......(csag): %s\r\n", CSAG[csag_setting]);

	// print overcurrent detection threshold OCTH
	data = getRegisterCacheData(CSAG_OCTH); // get current register data
	data &= OCTH_MASK;				   // clear non OCTH bits
	octh_setting = (data >> OCTH_POS); // shift bits to LSB position
	DEBUGPRINTF("\tOvercurrent detection threshold...(octh): %s\r\n", OCTH[octh_setting]);

	// print overcurrent threshold value in Ampere
	/*DEBUGPRINTF("\tOvercurrent threshold at 50uOhm [A].....: %s\r\n",
			OC[octh_setting][csag_setting]); // This must be altered for given use case */

	DEBUGPRINTF("\t----------------------------------------------------------------\r\n");
}

//****************************************************************************
// printAllRegisters
//****************************************************************************
void printAllRegisters(void)
{
	for(int i = 0; i < 10; i++){
		printRegister(i);
	}
	DEBUGPRINTF("\r\n");
}


//****************************************************************************
// printStatusFlags
//
// check failure bits in diagnosis registers and report failures tu debug
//****************************************************************************
uint8_t last_STDIAG;
uint8_t last_CHDIAG;
uint8_t last_DIAG;
void printStatusFlags(uint8_t useCachedReg)
{
	if(useCachedReg == 0)
		readAllRegistersToCache();

    //DEBUGPRINTF(""); // delete prompt

	// STDIAG
	if(last_STDIAG != getRegisterCacheData(STDIAG)){
		if((getRegisterCacheData(STDIAG)) & VBAT_UV)
			DEBUGPRINTF("\t[FAILURE]: Vbat undervoltage\r\n");
		if((getRegisterCacheData(STDIAG)) & VBAT_OV)
			DEBUGPRINTF("\t[FAILURE]: Vbat overvoltage\r\n");
		if((getRegisterCacheData(STDIAG)) & VDD_UV)
			DEBUGPRINTF("\t[FAILURE]: Vdd undervoltage\r\n");
		if((getRegisterCacheData(STDIAG)) & TSD)
			DEBUGPRINTF("\t[FAILURE]: Chip overtemperature\r\n");
		if((getRegisterCacheData(STDIAG)) & OT_WARNING)
			DEBUGPRINTF("\t[WARNING]: Chip overtemperature\r\n");
		if((getRegisterCacheData(STDIAG)) & MEM_FAIL)
			DEBUGPRINTF("\t[WARNING]: Memory Error\r\n");
	}

	// CHDIAG
	if(last_CHDIAG != getRegisterCacheData(CHDIAG)){
		if((getRegisterCacheData(CHDIAG)) & VSOURCE_A)
			DEBUGPRINTF("\t[MONITOR]: Source overvoltage channel A\r\n");
		if((getRegisterCacheData(CHDIAG)) & VDSTRIP_A)
			DEBUGPRINTF("\t[FAILURE]: VDS overvoltage channel A\r\n");
		if((getRegisterCacheData(CHDIAG)) & VGSTH_A)
			DEBUGPRINTF("\t[FAILURE]: VGS undervoltage channel A\r\n");
		if((getRegisterCacheData(CHDIAG)) & VSOURCE_B)
			DEBUGPRINTF("\t[MONITOR]: Source overvoltage channel B\r\n");
		if((getRegisterCacheData(CHDIAG)) & VDSTRIP_B)
			DEBUGPRINTF("\t[FAILURE]: VDS overvoltage channel B\r\n");
		if((getRegisterCacheData(CHDIAG)) & VGSTH_B)
			DEBUGPRINTF("\t[FAILURE]: VGS undervoltage channel B\r\n");
		if((getRegisterCacheData(CHDIAG)) & ITRIP)
			DEBUGPRINTF("\t[FAILURE]: Overcurrent\r\n");
		if((getRegisterCacheData(CHDIAG)) & VCP_UV)
			DEBUGPRINTF("\t[FAILURE]: Charge pump undervoltage\r\n");
	}

    // DIAG
	if(last_DIAG != getRegisterCacheData(DIAG)){
		if((getRegisterCacheData(DIAG)) & SAFESTATEN)
			DEBUGPRINTF("\t[FAILURE]: SAFESTATE activated\r\n");
		if((getRegisterCacheData(DIAG)) & ADD_NOT_AVAIL)
			DEBUGPRINTF("\t[MONITOR]: SPI address not available\r\n");
		if((getRegisterCacheData(DIAG)) & LOG_CP)
			DEBUGPRINTF("\t[WARNING]: Loss of charge pump ground\r\n");
		if((getRegisterCacheData(DIAG)) & LOG_D)
			DEBUGPRINTF("\t[WARNING]: Loss of digital ground\r\n");
		if((getRegisterCacheData(DIAG)) & LOG_A)
			DEBUGPRINTF("\t[WARNING]: Loss of analog ground\r\n");
	}

	// Store last state of registers
	last_STDIAG = getRegisterCacheData(STDIAG);
	last_CHDIAG = getRegisterCacheData(CHDIAG);
	last_DIAG   = getRegisterCacheData(DIAG);
}

//****************************************************************************
// isProblemDetected
//
// returns the current state of the problem marker, used to check if an action
// was successful (e.g. switchON()) and if failure flags are set
//****************************************************************************
uint8_t isProblemDetected(){
	return problemDetected;
}
//****************************************************************************
// clearProblemDetected
//
// resets the state of the problem marker, used to check if an action
// was successful (e.g. switchON()) and if failure flags are set
//****************************************************************************
void clearProblemDetected(){
	problemDetected = 0;
}


//****************************************************************************
// getFailureBitmap
//
// get a uint32 with all set failure bits
//****************************************************************************
uint32_t getFailureBitmap(uint8_t useCachedReg){
	if(useCachedReg == 0)
		readAllRegistersToCache();

	uint32_t bitmap = 0;

	if((getRegisterCacheData(STDIAG)) & VBAT_UV)
		bitmap |= Fail_VBAT_UNDERVOLTAGE;

	if((getRegisterCacheData(STDIAG)) & VBAT_OV)
		bitmap |= Fail_VBAT_OVERVOLTAGE;

	if((getRegisterCacheData(STDIAG)) & VDD_UV)
		bitmap |= Fail_VDD_UNDERVOLTAGE;

	if((getRegisterCacheData(STDIAG)) & TSD)
		bitmap |= Fail_CHIP_OVERTEMPERATURE;

	if((getRegisterCacheData(CHDIAG)) & VDSTRIP_A)
		bitmap |= Fail_VDS_OVERVOLTAGE_A;

	if((getRegisterCacheData(CHDIAG)) & VGSTH_A)
		bitmap |= Fail_VGS_UNDERVOLTAGE_A;

	if((getRegisterCacheData(CHDIAG)) & VDSTRIP_B)
		bitmap |= Fail_VDS_OVERVOLTAGE_B;

	if((getRegisterCacheData(CHDIAG)) & VGSTH_B)
		bitmap |= Fail_VGS_UNDERVOLTAGE_B;

	if((getRegisterCacheData(CHDIAG)) & ITRIP)
		bitmap |= Fail_OVERCURRENT;

	if((getRegisterCacheData(CHDIAG)) & VCP_UV)
		bitmap |= Fail_CHARGEPUMP_UNDERVOLTAGE;

	if((getRegisterCacheData(DIAG)) & SAFESTATEN)
		bitmap |= Fail_SAVESTATE_ENABLED;


	return bitmap;
}

//****************************************************************************
// isFailureFlagActive
//
// check specific failure bits in diagnosis registers and return state
//****************************************************************************
uint8_t isFailureFlagActive(FailureFlags Fail, uint8_t useCachedReg){
	if(useCachedReg == 0)
		readAllRegistersToCache();

	switch(Fail){
		case Fail_VBAT_UNDERVOLTAGE:
			if((getRegisterCacheData(STDIAG)) & VBAT_UV)
				return 1;
			break;
		case Fail_VBAT_OVERVOLTAGE:
			if((getRegisterCacheData(STDIAG)) & VBAT_OV)
				return 1;
			break;
		case Fail_VDD_UNDERVOLTAGE:
			if((getRegisterCacheData(STDIAG)) & VDD_UV)
				return 1;
			break;
		case Fail_CHIP_OVERTEMPERATURE:
			if((getRegisterCacheData(STDIAG)) & TSD)
				return 1;
			break;
		case Fail_VDS_OVERVOLTAGE_A:
			if((getRegisterCacheData(CHDIAG)) & VDSTRIP_A)
				return 1;
			break;
		case Fail_VGS_UNDERVOLTAGE_A:
			if((getRegisterCacheData(CHDIAG)) & VGSTH_A)
				return 1;
			break;
		case Fail_VDS_OVERVOLTAGE_B:
			if((getRegisterCacheData(CHDIAG)) & VDSTRIP_B)
				return 1;
			break;
		case Fail_VGS_UNDERVOLTAGE_B:
			if((getRegisterCacheData(CHDIAG)) & VGSTH_B)
				return 1;
			break;
		case Fail_OVERCURRENT:
			if((getRegisterCacheData(CHDIAG)) & ITRIP)
				return 1;
			break;
		case Fail_CHARGEPUMP_UNDERVOLTAGE:
			if((getRegisterCacheData(CHDIAG)) & VCP_UV)
				return 1;
			break;
		case Fail_SAVESTATE_ENABLED:
			if((getRegisterCacheData(DIAG)) & SAFESTATEN)
				return 1;
			break;
	}

	return 0;
}

//****************************************************************************
// getWarningBitmap
//
// get a uint32 with all set warning bits
//****************************************************************************
uint32_t getWarningBitmap(uint8_t useCachedReg){
	if(useCachedReg == 0)
		readAllRegistersToCache();

	uint32_t bitmap = 0;

	if((getRegisterCacheData(DIAG)) & LOG_CP)
		bitmap |= Warn_LOSSOFGROUND_CP;

	if((getRegisterCacheData(DIAG)) & LOG_D)
		bitmap |= Warn_LOSSOFGROUND_D;

	if((getRegisterCacheData(DIAG)) & LOG_A)
		bitmap |= Warn_LOSSOFGROUND_A;

	if((getRegisterCacheData(STDIAG)) & OT_WARNING)
		bitmap |= Warn_OVERTEMP;

	if((getRegisterCacheData(STDIAG)) & MEM_FAIL)
		bitmap |= Warn_MEMFAIL;

	return bitmap;
}

//****************************************************************************
// isWarningFlagActive
//
// check specific warning bits in diagnosis registers and return state
//****************************************************************************
uint8_t isWarningFlagActive(WarningFlags Warn, uint8_t useCachedReg){
	if(useCachedReg == 0)
		readAllRegistersToCache();

	switch(Warn){
		case Warn_LOSSOFGROUND_CP:
			if((getRegisterCacheData(DIAG)) & LOG_CP)
				return 1;
			break;
		case Warn_LOSSOFGROUND_D:
			if((getRegisterCacheData(DIAG)) & LOG_D)
				return 1;
			break;
		case Warn_LOSSOFGROUND_A:
			if((getRegisterCacheData(DIAG)) & LOG_A)
				return 1;
			break;
		case Warn_OVERTEMP:
			if((getRegisterCacheData(STDIAG)) & OT_WARNING)
				return 1;
			break;
		case Warn_MEMFAIL:
			if((getRegisterCacheData(STDIAG)) & MEM_FAIL)
				return 1;
			break;
	}

	return 0;
}


//****************************************************************************
// getInfoBitmap
//
// get a uint32 with all set info bits
//****************************************************************************
uint32_t getInfoBitmap(uint8_t useCachedReg){
	if(useCachedReg == 0)
		readAllRegistersToCache();

	uint32_t bitmap = 0;

	if((getRegisterCacheData(DIAG)) & ADD_NOT_AVAIL)
		bitmap |= Monitor_SPI_ADDRESS_NOT_AVAIL;

	if((getRegisterCacheData(CHDIAG)) & VSOURCE_A)
		bitmap |= Monitor_SOURCE_OVERVOLTAGE_A;

	if((getRegisterCacheData(CHDIAG)) & VSOURCE_B)
		bitmap |= Monitor_SOURCE_OVERVOLTAGE_B;

	if((getRegisterCacheData(STDIAG)) & VCP_READY)
		bitmap |= Monitor_CHARGEPUMPR_READY;

	return bitmap;
}

//****************************************************************************
// isMonitoringFlagActive
//
// check specific monitoring bits in diagnosis registers and return state
//****************************************************************************
uint8_t isMonitoringFlagActive(MonitoringFlags Monitor, uint8_t useCachedReg){
	if(useCachedReg == 0)
		readAllRegistersToCache();

	switch(Monitor){
	case Monitor_SPI_ADDRESS_NOT_AVAIL:
		if((getRegisterCacheData(DIAG)) & ADD_NOT_AVAIL)
			return 1;
		break;
	case Monitor_SOURCE_OVERVOLTAGE_A:
		if((getRegisterCacheData(CHDIAG)) & VSOURCE_A)
			return 1;
		break;
	case Monitor_SOURCE_OVERVOLTAGE_B:
		if((getRegisterCacheData(CHDIAG)) & VSOURCE_B)
			return 1;
		break;
	case Monitor_CHARGEPUMPR_READY:
		if((getRegisterCacheData(STDIAG)) & VCP_READY)
			return 1;
		break;
	}

	return 0;
}











////
////
//// -------- Functions to control pre-charge path [NOT IMPLEMENTED] -----------
////
////

////****************************************************************************
//// setPrechargeTime
////
//// set precharge time in ms for 'on' command
//// the value is stored in the EEPROM Ram buffer
//// therefore the 16 bit value is split into two bytes
////****************************************************************************
//void setPrechargeTime(char *arg)
//{
//	static uint16_t precharge_time = 0;
//	static uint8_t upper; // upper 8 bit
//	static uint8_t lower; // lower 8 bit
//
//	precharge_time = atoi(arg); // convert string to integer
//
//	if (precharge_time < 0 || precharge_time >= 3000)
//	{
//		DEBUGPRINTF("\r\n");
//		DEBUGPRINTF("\t[ERROR]: Precharge time out of range");
//	}
//	else
//	{
//		upper = (precharge_time & 0xFF00) >> 8;
//		lower = precharge_time & 0xFF;
//
//		E_EEPROM_XMC4_WriteByte(EE_PRECH_TIME_UPPER, upper);
//		E_EEPROM_XMC4_WriteByte(EE_PRECH_TIME_LOWER, lower);
//	}
//}

////****************************************************************************
//// getPrechargeTime
////
//// returns precharge time (stored in EEPROM Ram buffer)
////****************************************************************************
//uint16_t getPrechargeTime()
//{
//	static uint16_t precharge_time;
//	static uint8_t upper; // upper 8 bit
//	static uint8_t lower; // lower 8 bit
//
//	E_EEPROM_XMC4_ReadByte(EE_PRECH_TIME_UPPER, &upper);
//	E_EEPROM_XMC4_ReadByte(EE_PRECH_TIME_LOWER, &lower);
//
//	precharge_time = 0;
//	precharge_time |= upper << 8; // set upper 8 bits
//	precharge_time |= lower;	  // set lower 8 bits
//
//	return precharge_time;
//}

//****************************************************************************
//
// Interrupt Service Routines
//
//****************************************************************************

////****************************************************************************
//// ISR_timerPrecharge
////
//// gets called when TIMER_PRECHARGE is expired
////****************************************************************************
//void ISR_timerPrecharge(void)
//{
//	// stop the timer
//	TIMER_Stop(&TIMER_PRECHARGE);
//	// clear interrupt event so next event is considered as new
//	TIMER_ClearEvent(&TIMER_PRECHARGE);
//	// force update of shadow registers
//	readAllRegisters();
//	reportChanges();
//	// switch on channel A
//	setBits(MOS_CHS_CTRL, MOSONCH_A, true);
//	// wait (Original code waited 100000 NOP cycles with an while loop ~ 1ms)
//	delay_ms(1);
//	// switch off channel B
//	// force update of shadow registers
//	readAllRegisters();
//	reportChanges();
//	clearBits(MOS_CHS_CTRL, MOSONCH_B, true);
//}

////****************************************************************************
//// switchOnWithPrecharge
////
//// enable, initialise, precharge and switch on main channel
////****************************************************************************
//void switchOnWithPrecharge(void)
//{
//	//static uint32_t pulse_time; // precharge pulse time in ms
//
//	if (isEnabled() == 0) // driver not enabled yet
//	{
//		driverInit();	 // initialize driver with last configuration
//		reportChanges(); // make sure the setting gets reported
//	}
//
//	pulse_time = (uint32_t)getPrechargeTime();
//
//	if (pulse_time > 0 && pulse_time <= 3000)
//	{
//		// pulse time is in ms, timer expects time in 0.01 us
//		// (there is a scaling factor of 100
//		//  e.g. 10.56 us --> value = 1056)
//		pulse_time *= 100000;
//
//		TIMER_Clear(&TIMER_PRECHARGE);
//		TIMER_SetTimeInterval(&TIMER_PRECHARGE, pulse_time);
//		setBits(MOS_CHS_CTRL, MOSONCH_B, true); // switch on chan B (precharge)
//		TIMER_Start(&TIMER_PRECHARGE);	  // start timer
//
//		reportChanges(); // make sure the setting gets reported
//
//		// when the timer is expired it will trigger an interrupt
//		// the interrupt will call ISR_timerChannelA()
//		// channel A is switched of in this function
//	}
//	else
//	{
//		DEBUGPRINTF("\r\n"); // new line
//		DEBUGPRINTF("\t[ERROR]: Precharge time out of range");
//	}
//}
