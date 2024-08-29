/*
 * BMS_TLE9012.h
 *
 *	Library for TLE9012 Li-Ion battery monitoring and balancing system IC. See main file Lib_TLE9012.c for how to use guide
 *	Created by R. Santeler @ Infineon 2024
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef BMS_TLE9012_H_
#define BMS_TLE9012_H_

#include <stdint.h>
#include "tle9012_registers.h"
#include "tle9012_interpreter.h"
#include "tle9012_queue.h"

//****************************************************************************
// Definitions
//****************************************************************************
#define USE_REG_CACHED 1
#define USE_REG_RENEW  0

#define ENABLE_ALL_ADC 1
#define IS_FINAL_NODE 1
#define NOT_SET 0

// Argument enumerations for "is...FlagActive()" functions. Representation of an Flag in register
typedef enum {Fail_SYS_OL_ERR = 8192U, Fail_SYS_INT_IC_ERR = 16384U, Fail_SYS_REG_CRC_ERR = 32768U, Fail_SYS_EXT_T_ERR = 65536U, Fail_SYS_INT_OT = 131072U, Fail_SYS_CELL_UV = 262144U, Fail_SYS_CELL_OV = 524288U, Fail_SYS_BAL_ERR_UC = 1048576U, Fail_SYS_BAL_ERR_OC = 2097152U, Fail_SYS_PS_ERR_SLEEP = 4194304U, Fail_SYS_ADC_ERR = 8388608U,} FailureFlags_tle9012;
//typedef enum {Warn_1 = 1} WarningFlags_tle9012;
//typedef enum {Monitor_1 = 2} MonitoringFlags_tle9012;

// All Request return one of the following results. All error codes greater or equal to REQUEST_ERROR can be considered failed
typedef enum {REQUEST_OK = 0U, REQUEST_ADDED = 1U, REQUEST_REGISTER_NOT_READY = 2U, REQUEST_REGISTER_DATA_INVALID_OR_OLD = 3U, REQUEST_REGISTER_ALREADY_QUEUED = 4U, REQUEST_ERROR = 5U, REQUEST_ADD_FAILED = 6U, REQUEST_TIMEOUT = 7U, REQUEST_DONE_BUT_REGISTER_NOT_READY = 8U } requestResult;

//****************************************************************************
// Objects
//****************************************************************************
extern uint8_t generalDiagnosticProblemDetected;

//****************************************************************************
// Function prototypes
//****************************************************************************
// Device state control
uint8_t tle9012_initUART(void);
void tle9012_deviceInit(uint8_t reinitializeConfig);
void tle9012_sendWakeup(uint8_t nodeID);
void tle9012_assignID(uint8_t nodeID, uint8_t isFinalNode, uint8_t enableAllADC);

// Direct register access
requestResult tle9012_writeRegisterByIndex(uint8_t nodeID, uint8_t reg_idx, uint16_t data, uint8 isBlocking, uint8_t isBroadcast);
requestResult tle9012_readRegisterByIndex(uint8_t nodeID, uint8_t reg_idx, uint8 isBlocking, uint8_t isBroadcast);
requestResult tle9012_writeMultiReadConfig(uint8_t nodeID, uint16_t pcvm_sel, uint16_t bvm_sel, uint16_t ext_temp_sel, uint16_t ext_temp_r_sel, uint16_t int_temp_sel, uint16_t scvm_sel, uint16_t stress_pcvm_sel, uint8 isBlocking);
requestResult tle9012_readAllRegistersToCache(uint8_t nodeID, uint8_t isBlocking, uint8_t isBroadcast);

// General settings
requestResult tle9012_setConfig  (uint8_t nodeID, subreg_item subreg, uint16_t setting, uint8_t isBlocking, uint8_t isBroadcast);
uint8_t       tle9012_checkConfig(uint8_t nodeID, subreg_item subreg, uint16_t setting, uint8_t isBlocking, uint8_t isBroadcast);

// Diagnostic
requestResult tle9012_clearStatusFlags(uint8_t nodeID, uint8_t isBlocking, uint8_t isBroadcast);

// Special settings and reads
requestResult tle9012_convertPrimaryCellVoltages(uint8_t nodeID, uint16_t ignoreCellMap, uint8_t toCellIdx, uint8_t checkIfQueued);
requestResult tle9012_convertPrimaryCellVoltageByIndex(uint8_t nodeID, uint8_t cellsIdx, uint8_t checkIfQueued);
requestResult tle9012_convertBlockVoltage(uint8_t nodeID, uint8_t checkIfQueued);
requestResult tle9012_convertTemperatures(uint8_t nodeID, uint8_t toSensIdx, uint8_t checkIfQueued);
requestResult tle9012_convertTemperatureByIndex(uint8_t nodeID, uint8_t channelIdx, uint8_t checkIfQueued);
requestResult tle9012_convertInternalTemperatureByIndex(uint8_t nodeID, uint8_t channelIdx, uint8_t checkIfQueued);
uint16_t  	  tle9012_convertExternalTemperatureThreshold(double th_degree_C, uint8_t ext_temp_z_INTC);

// Debug functions
void tle9012_printRegister(uint8_t nodeID, uint8_t reg_index);
void tle9012_printAllRegisters(uint8_t nodeID);
void tle9012_printStatusFlags(uint8_t nodeID, uint8_t newLineForEachError, uint8_t onlyPrintIfDataChanged);

// Used to check if an action was successful (e.g. switchON()) and if failure flags are set
uint8_t tle9012_isProblemDetected();
void    tle9012_clearProblemDetected();

// Use these to check if Failures, Warnings or Monitoring Flags are active (e.g. during interrupt routine)
uint32_t tle9012_getFailureBitmap(uint8_t nodeID);
uint8_t tle9012_isFailureFlagActive(uint8_t nodeID, FailureFlags_tle9012 Fail);

#endif /* BMS_TLE9012_H_ */
