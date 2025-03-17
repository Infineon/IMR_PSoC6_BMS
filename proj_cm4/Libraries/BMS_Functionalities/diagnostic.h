/*
 * diagnostic.h
 * Abstraction of the diagnostic system, see doSystemDiagnostic for details
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

#ifndef DIAGNOSTIC_H_
#define DIAGNOSTIC_H_

#include <stdint.h>

// Errors Warnings and Messages. THESE MUST NEVER BE BIGHER THAN 32 BIT!
//NOTE: !!!  If any error codes are changed here
// they must also be updated in BMS_TLE9012.c/.h and Driver_2ED4820.c/.h
// SSW - Safety Switch, CAF - Cell/AnalogFrontend, INT - Internal errors
typedef enum {
	ERR_SSW_INTERRUPT = 1U,
	ERR_SSW_VBAT_UNDERVOLTAGE = 2U,
	ERR_SSW_VBAT_OVERVOLTAGE = 4U,
	ERR_SSW_VDD_UNDERVOLTAGE = 8U,
	ERR_SSW_CHIP_OVERTEMPERATURE = 16U,
	ERR_SSW_VDS_OVERVOLTAGE_A = 32U,
	ERR_SSW_VGS_UNDERVOLTAGE_A = 64U,
	ERR_SSW_VDS_OVERVOLTAGE_B = 128U,
	ERR_SSW_VGS_UNDERVOLTAGE_B = 256U,
	ERR_SSW_OVERCURRENT = 512U,
	ERR_SSW_CHARGEPUMP_UNDERVOLTAGE = 1024U,
	ERR_SSW_SAVESTATE_ENABLED = 2048U,

	ERR_CAF_TLE9012_INTERRUPT = 4096U,
	ERR_CAF_OL_ERR = 8192U,
	ERR_CAF_INT_IC_ERR = 16384U,
	ERR_CAF_REG_CRC_ERR = 32768U,
	ERR_CAF_EXT_T_ERR = 65536U,
	ERR_CAF_INT_OT = 131072U,
	ERR_CAF_CELL_UV = 262144U,
	ERR_CAF_CELL_OV = 524288U,
	ERR_CAF_BAL_UC = 1048576U,
	ERR_CAF_BAL_OC = 2097152U,
	ERR_CAF_PS_SLEEP = 4194304U,
	ERR_CAF_ADC_ERR = 8388608U,

	ERR_INT_TLE9012_QUEUE_FAILED = 16777216U,
	ERR_INT_OVERCURRENT = 33554432U,
	ERR_INT_SWITCH_ON_VETO_RECEIVED = 67108864U,
	ERR_INT_ADC_COM_TIMEOUT = 134217728U,
	ERR_INT_VBAT_OV = 268435456U,
	ERR_INT_VBAT_UV = 536870912U,
	ERR_INT_TLE9012_COM_ERROR = 1073741824U
} errors_bitmask;

typedef enum {
	MSG_SSW_WARN_LOSSOFGROUND_CP = 1U,
	MSG_SSW_WARN_LOSSOFGROUND_D = 2U,
	MSG_SSW_WARN_LOSSOFGROUND_A = 4U,
	MSG_SSW_WARN_OVERTEMP = 8U,
	MSG_SSW_WARN_MEMFAIL = 16U,
	MSG_SSW_INFO_SPI_ADDRESS_NOT_AVAIL = 32U,
	MSG_SSW_INFO_SOURCE_OVERVOLTAGE_A = 64U,
	MSG_SSW_INFO_SOURCE_OVERVOLTAGE_B = 128U,
	MSG_SSW_INFO_CHARGEPUMPR_READY = 256U,

	MSG_CAF_BAL_ACTIVE = 512U,

	MSG_INT_BATTERY_LOW = 1024U,
	MSG_INT_EXT_LOAD_V_ON = 2048U
} messages_bitmask;

typedef enum {
	DIAGNOSTIC_EVAL_CLEAR,
	DIAGNOSTIC_READ
} diagnostic_states;

extern diagnostic_states diagnostic_state;

// A binary mask on where in the error_map the 2ED4820 safety switch driver errors
// can be found (next error after driver errors minus 1)
#define ERRORS_SSW_MASK	 (uint32_t)((uint32_t)ERR_CAF_TLE9012_INTERRUPT - (uint32_t)1U)
// A binary mask on where in the error_map the TLE9012 analog front-end errors
// can be found (next error after TLE9012 errors minus 1 and minus the driver errors)
#define ERRORS_CAF_MASK ((uint32_t)((uint32_t)ERR_INT_TLE9012_QUEUE_FAILED - \
		(uint32_t)1U) - ERRORS_SSW_MASK)
// A binary mask on where in the error_map the Internal errors
// (not from TLE9012 or 2ED4820) can be found
#define ERRORS_INT_MASK ((uint32_t)((uint32_t)ERR_INT_TLE9012_QUEUE_FAILED | \
		(uint32_t)ERR_INT_OVERCURRENT | (uint32_t)ERR_INT_SWITCH_ON_VETO_RECEIVED | \
		(uint32_t)ERR_INT_ADC_COM_TIMEOUT | (uint32_t)ERR_INT_VBAT_OV | \
		(uint32_t)ERR_INT_VBAT_UV | (uint32_t)ERR_INT_TLE9012_COM_ERROR))

// A binary mask on where in the message_map the 2ED4820 safety switch driver
// warnings and messages can be found
#define MSG_SSW_MASK	 (uint32_t)((uint32_t)MSG_CAF_BAL_ACTIVE - (uint32_t)1U)
// A binary mask on where in the message_map the TLE9012 analog front-end
// warnings and messages can be found
#define MSG_CAF_MASK 	((uint32_t)((uint32_t)MSG_INT_BATTERY_LOW - \
		(uint32_t)1U) - MSG_SSW_MASK)
// A binary mask on where in the message_map the Internal messages and warnings
// (not from TLE9012 or 2ED4820) can be found
#define MSG_INT_MASK 	((uint32_t)((uint32_t)MSG_INT_BATTERY_LOW | \
		(uint32_t)MSG_INT_EXT_LOAD_V_ON ))

// The cell voltage and diagnostic share the 16 bit in PUB8_CELL_V.
// Diagnostic uses the first 3, the rest describes the voltage in mV
#define CELL_DIAGNOSTIC_STATES_MAP	((uint16_t)0b1110000000000000)
#define CELL_DIAGNOSTIC_MAX_mV		((uint16_t)~CELL_DIAGNOSTIC_STATES_MAP)
#define CELL_VOLTAGE_MAP			((uint16_t)~CELL_DIAGNOSTIC_STATES_MAP)

typedef enum {
	CELL_DIAG_OK 			   = 0,
	CELL_DIAG_OPEN_LOOP        = 0b0010000000000000,
	CELL_DIAG_UNDERVOLTAGE     = 0b0100000000000000,
	CELL_DIAG_OVERVOLTAGE      = 0b0110000000000000,
	CELL_DIAG_BAL_UNDERVOLTAGE = 0b1000000000000000,
	CELL_DIAG_BAL_OVERVOLTAGE  = 0b1010000000000000,
	CELL_DIAG_IS_BALANCING     = 0b1100000000000000
} cellDiagnosticStates;

// Errors
uint32_t getErrorBitmap();
uint8_t getErrorBit(errors_bitmask errBit);
void setErrorBit(errors_bitmask errBit);
void clearErrorBit(errors_bitmask errBit);

// Messages and Warnings
uint32_t getMsgBitmap();
uint8_t getMsgBit(messages_bitmask msgBit);
void setMsgBit(messages_bitmask msgBit);
void clearMsgBit(messages_bitmask msgBit);

// Main diagnostic function
void doSystemDiagnostic();
void diagnostic_printStatusFlags(uint32_t bitmap_error, uint32_t bitmap_warning,
		uint8_t newLineForEachError, uint8_t onlyPrintIfDataChanged);

#endif /* DIAGNOSTIC_H_ */
