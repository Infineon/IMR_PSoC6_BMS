/*
 * tle9012_registers.c
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

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <stdlib.h> // includes strtol

#include "tle9012_target.h"
#include "tle9012_registers.h"

//****************************************************************************
// Definitions and Objects for the queuing system
//****************************************************************************
// Debug definitions - comment "DEBUGPRINTF(p_frm, ##__VA_ARGS__)" out to remove messages from build
#define DEBUGPRINTF_BITMAP_CHANGE(p_frm, ...) //DEBUGPRINTF(p_frm, ##__VA_ARGS__)
#define DEBUGPRINTF_WATCHDOG(p_frm, ...)      //DEBUGPRINTF(p_frm, ##__VA_ARGS__)

tle9012_node tle9012_nodes[TLE9012_NODES_SIZE] = {0};
uint8_t multi_read_cfg_reg_idx_list[23] = {0};
uint8_t multi_read_cfg_reg_idx_list_num = 0;

uint32_t timeLastWatchdogRefresh = 0;

//****************************************************************************
// Global variables for register and bit names (Note that index and address differs)
//****************************************************************************
// Register index to address and label
reg_item  reg_items[NUM_REGISTERS] = {
    {0x0001, "PART_CONFIG"},
    {0x0002, "OL_OV_THR"},
    {0x0003, "OL_UV_THR"},
    {0x0004, "TEMP_CONF"},
    {0x0005, "INT_OT_WARN_CONF"},
    {0x0008, "RR_ERR_CNT"},
    {0x0009, "RR_CONFIG"},
    {0x000A, "FAULT_MASK"},
    {0x000B, "GEN_DIAG"},
    {0x000C, "CELL_UV"},
    {0x000D, "CELL_OV"},
    {0x000E, "EXT_TEMP_DIAG"},
    {0x0010, "DIAG_OL"},
    {0x0011, "REG_CRC_ERR"},
    {0x0012, "CELL_UV_DAC_COMP"},
    {0x0013, "CELL_OV_DAC_COMP"},
    {0x0014, "OP_MODE"},
    {0x0015, "BAL_CURR_THR"},
    {0x0016, "BAL_SETTINGS"},
    {0x0017, "AVM_CONFIG"},
    {0x0018, "MEAS_CTRL"},
	{0x0019, "PCVM_0"},
	{0x001A, "PCVM_1"},
	{0x001B, "PCVM_2"},
	{0x001C, "PCVM_3"},
	{0x001D, "PCVM_4"},
	{0x001E, "PCVM_5"},
	{0x001F, "PCVM_6"},
	{0x0020, "PCVM_7"},
	{0x0021, "PCVM_8"},
	{0x0022, "PCVM_9"},
	{0x0023, "PCVM_10"},
	{0x0024, "PCVM_11"},
    {0x0025, "SCVM_HIGH"},
    {0x0026, "SCVM_LOW"},
    {0x0027, "STRESS_PCVM"},
    {0x0028, "BVM"},
    {0x0029, "EXT_TEMP_0"},
    {0x002A, "EXT_TEMP_1"},
    {0x002B, "EXT_TEMP_2"},
    {0x002C, "EXT_TEMP_3"},
    {0x002D, "EXT_TEMP_4"},
    {0x002F, "EXT_TEMP_R_DIAG"},
    {0x0030, "INT_TEMP"},
    {0x0031, "MULTI_READ"},
    {0x0032, "MULTI_READ_CFG"},
    {0x0033, "BAL_DIAG_OC"},
    {0x0034, "BAL_DIAG_UC"},
    {0x0035, "INT_TEMP_2"},
    {0x0036, "CONFIG"},
    {0x0037, "GPIO"},
    {0x0038, "GPIO_PWM"},
    {0x0039, "ICVID"},
    {0x003A, "MAILBOX"},
    {0x003B, "CUSTOMER_ID_0"},
    {0x003C, "CUSTOMER_ID_1"},
    {0x003D, "WDOG_CNT"},
    {0x003E, "SCVM_CONFIG"},
    {0x003F, "STRESS_AUX"},
    {0x005B, "BAL_PWM"},
    {0x005C, "BAL_CNT_0"},
    {0x005D, "BAL_CNT_1"},
    {0x005E, "BAL_CNT_2"},
    {0x005F, "BAL_CNT_3"},
};

// Items for every subregister/setting containing: main register index, bitmask of specific setting and label
const subreg_item  PART_CONFIG__EN_CELLi = 				{0 ,0b0000011111111111, "EN_CELLi"};			 // RW - Enable cell monitoring for cell i
const subreg_item  PART_CONFIG__EN_CELL11 = 			{0 ,0b0000100000000000, "EN_CELL11"};			 // RW - Enable cell monitoring for cell 11
const subreg_item  OL_OV_THR__OV_THR = 					{1 ,0b0000001111111111, "OV_THR"};				 // RW - Overvoltage fault threshold 10-bit overvoltage fault threshold. Battery input voltages (U0 to U11)are tested for overvoltage with given value. OV error is detected andindicated in GEN_DIAG register if cell voltage is higher than OV faultthreshold.
const subreg_item  OL_OV_THR__OL_THR_MAX = 				{1 ,0b1111110000000000, "OL_THR_MAX"};			 // RW - Open load maximum voltage drop threshold (LSB8) 6-bit (LSB8) to define the maximum threshold for the voltage dropwhile OL-diagnostics (I OL_DIAG * R F). If voltage drop > OL_THR_MAX, theOLx bit of channel x is set.
const subreg_item  OL_UV_THR__UV_THR = 					{2 ,0b0000001111111111, "UV_THR"};				 // RW - Undervoltage fault threshold 10-bit undervoltage fault threshold. Battery input voltages (U0 to U11)are tested for undervoltage with given value. UV error is detected andindicated in GEN_DIAG register if cell voltage is lower than UV faultthreshold.
const subreg_item  OL_UV_THR__OL_THR_MIN = 				{2 ,0b1111110000000000, "OL_THR_MIN"};			 // RW - Open load minimum voltage drop threshold (LSB8) 6-bit (LSB8) to define the minimum threshold for the voltage drop whileOL-diagnostics (I OL_DIAG * R F). If voltage drop < OL_THR_MIN, the OLxbit of channel x is set.
const subreg_item  TEMP_CONF__EXT_OT_THR = 				{3 ,0b0000001111111111, "EXT_OT_THR"};			 // RW - External overtemperature threshold 10-bit overtemperature fault threshold. Overtemperature error isdetected and indicated in GEN_DIAG register if the externaltemperature result is lower than the fault threshold. Balancing isdeactivated.
const subreg_item  TEMP_CONF__I_NTC = 					{3 ,0b0000110000000000, "I_NTC"};				 // RW - Current source used for OT fault
const subreg_item  TEMP_CONF__NR_TEMP_SENSE = 			{3 ,0b0111000000000000, "NR_TEMP_SENSE"};		 // RW - Number of external temperature sensors
const subreg_item  INT_OT_WARN_CONF__INT_OT_THR = 		{4 ,0b0000001111111111, "INT_OT_THR"};			 // RW - Internal overtemperature threshold 10-bit overtemperature fault threshold. Overtemperature error isdetected and indicated in GEN_DIAG register if the internal temperatureresult is LOWER than the fault threshold. Balancing is deactivated.
const subreg_item  RR_ERR_CNT__NR_ERR = 				{5 ,0b0000000000000111, "NR_ERR"};				 // RW - Number of errors Number of consecutive detected errors before error is valid and setin GEN_DIAG and individual fault registers. Only used for faults wherecounter NR_ERR is active (this can be set in register NR_ERR_MASK).Please note: The register CRC errors as well as the internal IC errors donot have an error counter.
const subreg_item  RR_ERR_CNT__NR_EXT_TEMP_START = 		{5 ,0b0000000000111000, "NR_EXT_TEMP_START"};	 // RW - External temperature triggering in round robin
const subreg_item  RR_ERR_CNT__RR_SLEEP_CNT = 			{5 ,0b1111111111000000, "RR_SLEEP_CNT"};		 // RW - Round robin timing in sleep mode
const subreg_item  RR_CONFIG__RR_CNT = 					{6 ,0b0000000001111111, "RR_CNT"};				 // RW - Round robin counter
const subreg_item  RR_CONFIG__RR_SYNC = 				{6 ,0b0000000010000000, "RR_SYNC"};				 // RW - Round robin synchronization
const subreg_item  RR_CONFIG__M_NR_ERR_ADC_ERR = 		{6 ,0b0000000100000000, "M_NR_ERR_ADC_ERR"};	 // RW - Mask NR_ERR counter for ADC error
const subreg_item  RR_CONFIG__M_NR_ERR_OL_ERR = 		{6 ,0b0000001000000000, "M_NR_ERR_OL_ERR"};		 // RW - Mask NR_ERR counter for open load error
const subreg_item  RR_CONFIG__M_NR_ERR_EXT_T_ERR = 		{6 ,0b0000010000000000, "M_NR_ERR_EXT_T_ERR"};	 // RW - Mask NR_ERR counter for external temperature error
const subreg_item  RR_CONFIG__M_NR_ERR_INT_OT = 		{6 ,0b0000100000000000, "M_NR_ERR_INT_OT"};		 // RW - Mask NR_ERR counter for internal temperature error
const subreg_item  RR_CONFIG__M_NR_ERR_CELL_UV = 		{6 ,0b0001000000000000, "M_NR_ERR_CELL_UV"};	 // RW - Mask NR_ERR counter for undervoltage error
const subreg_item  RR_CONFIG__M_NR_ERR_CELL_OV = 		{6 ,0b0010000000000000, "M_NR_ERR_CELL_OV"};	 // RW - Mask NR_ERR counter for overvoltage error
const subreg_item  RR_CONFIG__M_NR_ERR_BAL_UC = 		{6 ,0b0100000000000000, "M_NR_ERR_BAL_UC"};		 // RW - Mmask NR_ERR counter for balancing error undercurrent
const subreg_item  RR_CONFIG__M_NR_ERR_BAL_OC = 		{6 ,0b1000000000000000, "M_NR_ERR_BAL_OC"};		 // RW - Mask NR_ERR counter for balancing error overcurrent
const subreg_item  FAULT_MASK__ERR_PIN = 				{7 ,0b0000000000100000, "ERR_PIN"};				 // RW - Enable Error PIN functionality
const subreg_item  FAULT_MASK__M_ADC_ERR = 				{7 ,0b0000000001000000, "M_ADC_ERR"};			 // RW - EMM/ERR mask for ADC error
const subreg_item  FAULT_MASK__M_OL_ERR = 				{7 ,0b0000000010000000, "M_OL_ERR"};			 // RW - EMM/ERR mask for open load error
const subreg_item  FAULT_MASK__M_INT_IC_ERR = 			{7 ,0b0000000100000000, "M_INT_IC_ERR"};		 // RW - EMM/ERR mask for internal IC error
const subreg_item  FAULT_MASK__M_REG_CRC_ERR = 			{7 ,0b0000001000000000, "M_REG_CRC_ERR"};		 // RW - EMM/ERR mask for register CRC error
const subreg_item  FAULT_MASK__M_EXT_T_ERR = 			{7 ,0b0000010000000000, "M_EXT_T_ERR"};			 // RW - EMM/ERR mask for external temperature error
const subreg_item  FAULT_MASK__M_INT_OT = 				{7 ,0b0000100000000000, "M_INT_OT"};			 // RW - EMM/ERR mask for internal temperature error
const subreg_item  FAULT_MASK__M_CELL_UV = 				{7 ,0b0001000000000000, "M_CELL_UV"};			 // RW - EMM/ERR mask for cell undervoltage error
const subreg_item  FAULT_MASK__M_CELL_OV = 				{7 ,0b0010000000000000, "M_CELL_OV"};			 // RW - EMM/ERR mask for cell overvoltage error
const subreg_item  FAULT_MASK__M_BAL_ERR_UC = 			{7 ,0b0100000000000000, "M_BAL_ERR_UC"};		 // RW - EMM/ERR mask for balancing error undercurrent
const subreg_item  FAULT_MASK__M_BAL_ERR_OC = 			{7 ,0b1000000000000000, "M_BAL_ERR_OC"};		 // RW - EMM/ERR mask for balancing error overcurrent
const subreg_item  GEN_DIAG__UART_WAKEUP = 				{8 ,0b0000000000000001, "UART_WAKEUP"};			 // RH - Wake-up via UART
const subreg_item  GEN_DIAG__MOT_MOB_N = 				{8 ,0b0000000000000010, "MOT_MOB_N"};			 // RH - Primary on Top/Bottom configuration
const subreg_item  GEN_DIAG__BAL_ACTIVE = 				{8 ,0b0000000000000100, "BAL_ACTIVE"};			 // RH - Balancing active
const subreg_item  GEN_DIAG__LOCK_MEAS = 				{8 ,0b0000000000001000, "LOCK_MEAS"};			 // RH - Lock measurement This bit indicates an ongoing PCVM, SCVM, BVM or AVM measurement ora delayed RR.
const subreg_item  GEN_DIAG__RR_ACTIVE = 				{8 ,0b0000000000010000, "RR_ACTIVE"};			 // RH - Round robin active This bit indicates if the round robin was active during read.
const subreg_item  GEN_DIAG__PS_ERR_SLEEP = 			{8 ,0b0000000000100000, "PS_ERR_SLEEP"};		 // ROCW - Power supply error induced sleep
const subreg_item  GEN_DIAG__ADC_ERR = 					{8 ,0b0000000001000000, "ADC_ERR"};				 // ROCW - ADC Error
const subreg_item  GEN_DIAG__OL_ERR = 					{8 ,0b0000000010000000, "OL_ERR"};				 // ROCWL - Open load error Please note: Resetting the error here resets also the respective detailederror register.
const subreg_item  GEN_DIAG__INT_IC_ERR = 				{8 ,0b0000000100000000, "INT_IC_ERR"};			 // ROCW - Internal IC error
const subreg_item  GEN_DIAG__REG_CRC_ERR = 				{8 ,0b0000001000000000, "REG_CRC_ERR"};			 // ROCWL - Register CRC error Please note: Resetting the error here resets also the respective detailederror register.
const subreg_item  GEN_DIAG__EXT_T_ERR = 				{8 ,0b0000010000000000, "EXT_T_ERR"};			 // ROCWL - External temperature error Please note: Resetting the error here resets also the respective detailederror register.
const subreg_item  GEN_DIAG__INT_OT = 					{8 ,0b0000100000000000, "INT_OT"};				 // ROCW - Internal temperature (OT) error
const subreg_item  GEN_DIAG__CELL_UV = 					{8 ,0b0001000000000000, "CELL_UV"};				 // ROCWL - Cell undervoltage (UV) error Please note: Resetting the error here resets also the respective detailederror register.
const subreg_item  GEN_DIAG__CELL_OV = 					{8 ,0b0010000000000000, "CELL_OV"};				 // ROCWL - Cell overvoltage (OV) error Please note: Resetting the error here resets also the respective detailederror register.
const subreg_item  GEN_DIAG__BAL_ERR_UC = 				{8 ,0b0100000000000000, "BAL_ERR_UC"};			 // ROCWL - Balancing error undercurrent (will be reset in sleep mode) Please note: Resetting the error here resets also the respective detailederror register.
const subreg_item  GEN_DIAG__BAL_ERR_OC = 				{8 ,0b1000000000000000, "BAL_ERR_OC"};			 // ROCWL - Balancing error overcurrent (will be reset in sleep mode) Please Note: Resetting the error here resets also the respective detailederror register.
const subreg_item  CELL_UV__UV_i = 						{9 ,0b0000111111111111, "UV_i"};				 // ROCW - Undervoltage in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  CELL_OV__OV_i = 						{10 ,0b0000111111111111, "OV_i"};				 // ROCW - Overvoltage in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OPEN0 = 				{11 ,0b0000000000000001, "OPEN0"};				 // ROCW - Open load in external temp 0 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__SHORT0 = 				{11 ,0b0000000000000010, "SHORT0"};				 // ROCW - Short in external temp 0 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OT0 = 				{11 ,0b0000000000000100, "OT0"};				 // ROCW - Overtemperature in external temp 0 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OPEN1 = 				{11 ,0b0000000000001000, "OPEN1"};				 // ROCW - Open load in external temp 1 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__SHORT1 = 				{11 ,0b0000000000010000, "SHORT1"};				 // ROCW - Short in external temp 1 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OT1 = 				{11 ,0b0000000000100000, "OT1"};				 // ROCW - Overtemperature in external temp 1 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OPEN2 = 				{11 ,0b0000000001000000, "OPEN2"};				 // ROCW - Open load in external temp 2 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__SHORT2 = 				{11 ,0b0000000010000000, "SHORT2"};				 // ROCW - Short in external temp 2 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OT2 = 				{11 ,0b0000000100000000, "OT2"};				 // ROCW - Overtemperature in external temp 2 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OPEN3 = 				{11 ,0b0000001000000000, "OPEN3"};				 // ROCW - Open load in external temp 3 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__SHORT3 = 				{11 ,0b0000010000000000, "SHORT3"};				 // ROCW - Short in external temp 3 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OT3 = 				{11 ,0b0000100000000000, "OT3"};				 // ROCW - Overtemperature in external temp 3 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OPEN4 = 				{11 ,0b0001000000000000, "OPEN4"};				 // ROCW - Open load in external temp 4 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__SHORT4 = 				{11 ,0b0010000000000000, "SHORT4"};				 // ROCW - Short in external temp 4 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  EXT_TEMP_DIAG__OT4 = 				{11 ,0b0100000000000000, "OT4"};				 // ROCW - Overtemperature in external temp 4 Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  DIAG_OL__OL_i = 						{12 ,0b0000111111111111, "OL_i"};				 // ROCW - Open load channel i Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  REG_CRC_ERR__ADDR = 					{13 ,0b0000000001111111, "ADDR"};				 // ROCW - Register address of register where CRC check failed Register address of CRC check fail, can also be cleared on write '0' toconnected GEN_DIAG register bit.
const subreg_item  CELL_UV_DAC_COMP__UV_i = 			{14 ,0b0000111111111111, "UV_i"};				 // ROCW - Undervoltage in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  CELL_OV_DAC_COMP__OV_i = 			{15 ,0b0000111111111111, "OV_i"};				 // ROCW - Overvoltage in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  OP_MODE__PD = 						{16 ,0b0000000000000001, "PD"};					 // RW - Activate sleep mode
const subreg_item  OP_MODE__EXT_WD = 					{16 ,0b0000000000000010, "EXT_WD"};				 // RW - Extended watchdog
const subreg_item  OP_MODE__SLEEP_REG_RESET = 			{16 ,0b0000000000000100, "SLEEP_REG_RESET"};	 // RWH - Sleep mode registers reset
const subreg_item  OP_MODE__I_DIAG_EN = 				{16 ,0b0000000000001000, "I_DIAG_EN"};			 // RW - Force OL diagnostics currents
const subreg_item  OP_MODE__LR_TIME = 					{16 ,0b1111110000000000, "LR_TIME"};			 // RW - Long-running mode measurement restart time
const subreg_item  BAL_CURR_THR__OC_THR = 				{17 ,0b0000000011111111, "OC_THR"};				 // RW - Overcurrent fault threshold 8-bit to define the maximum voltage drop during balancingdiagnostics. If the voltage drop (I Bal*R F) > OC_THR the overcurrentis detected.
const subreg_item  BAL_CURR_THR__UC_THR = 				{17 ,0b1111111100000000, "UC_THR"};				 // RW - Undercurrent fault threshold 8-bit to define the minimum voltage drop during balancing diagnostics.If the voltage drop (I Bal*R F) < UC_THR the undercurrent is detected.
const subreg_item  AVM_CONFIG__TEMP_MUX_DIAG_SEL = 		{19 ,0b0000000000000111, "TEMP_MUX_DIAG_SEL"};	 // RW - Selector for external temp diagnose
const subreg_item  AVM_CONFIG__AVM_TMP0_MASK = 			{19 ,0b0000000000001000, "AVM_TMP0_MASK"};		 // RW - Activate auxiliary measurement via deactive TMP0 as part of AVM
const subreg_item  AVM_CONFIG__AVM_TMP1_MASK = 			{19 ,0b0000000000010000, "AVM_TMP1_MASK"};		 // RW - Activate auxiliary measurement via deactive TMP1 as part of AVM
const subreg_item  AVM_CONFIG__AVM_TMP2_MASK = 			{19 ,0b0000000000100000, "AVM_TMP2_MASK"};		 // RW - Activate auxiliary measurement via deactive TMP2 as part of AVM
const subreg_item  AVM_CONFIG__AVM_TMP3_MASK = 			{19 ,0b0000000001000000, "AVM_TMP3_MASK"};		 // RW - Activate auxiliary measurement via deactive TMP3 as part of AVM
const subreg_item  AVM_CONFIG__AVM_TMP4_MASK = 			{19 ,0b0000000010000000, "AVM_TMP4_MASK"};		 // RW - Activate auxiliary measurement via deactive TMP4 as part of AVM
const subreg_item  AVM_CONFIG__AUX_BIPOLAR = 			{19 ,0b0000000100000000, "AUX_BIPOLAR"};		 // RWH - Bipolar AUX measurement instead of BVM
const subreg_item  AVM_CONFIG__R_DIAG = 				{19 ,0b0000001000000000, "R_DIAG"};				 // RW - Masking diagnostics resistor as part of AVM
const subreg_item  AVM_CONFIG__R_DIAG_SEL_0 = 			{19 ,0b0001100000000000, "R_DIAG_SEL_0"};		 // RW - R_DIAG current source 0
const subreg_item  AVM_CONFIG__R_DIAG_SEL_1 = 			{19 ,0b0110000000000000, "R_DIAG_SEL_1"};		 // RW - R_DIAG current source 1
const subreg_item  AVM_CONFIG__R_DIAG_CUR_SRC = 		{19 ,0b1000000000000000, "R_DIAG_CUR_SRC"};		 // RW - R_DIAG current source selection
const subreg_item  MEAS_CTRL__CVM_DEL = 				{20 ,0b0000000000011111, "CVM_DEL"};			 // RW - Wait time before CVM and/or BVM is started when PBOFF=1
const subreg_item  MEAS_CTRL__PBOFF = 					{20 ,0b0000000000100000, "PBOFF"};				 // RW - Enable PBOFF
const subreg_item  MEAS_CTRL__SCVM_START = 				{20 ,0b0000000001000000, "SCVM_START"};			 // RWH - Start secondary cell voltage measurement Bit cleared if conversion done
const subreg_item  MEAS_CTRL__AVM_START = 				{20 ,0b0000000010000000, "AVM_START"};			 // RWH - Start auxillary voltage measurement Bit cleared if conversion done
const subreg_item  MEAS_CTRL__BVM_MODE = 				{20 ,0b0000011100000000, "BVM_MODE"};			 // RW - Block voltage measurement mode
const subreg_item  MEAS_CTRL__BVM_START = 				{20 ,0b0000100000000000, "BVM_START"};			 // RWH - Start block voltage measurement Bit cleared if conversion done
const subreg_item  MEAS_CTRL__CVM_MODE = 				{20 ,0b0111000000000000, "CVM_MODE"};			 // RW - Cell voltage measurement mode
const subreg_item  MEAS_CTRL__PCVM_START = 				{20 ,0b1000000000000000, "PCVM_START"};			 // RWH - Start primary cell voltage measurement Bit cleared if conversion done
const subreg_item  PCVM_0__RESULT = 					{21 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_1__RESULT = 					{22 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_2__RESULT = 					{23 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_3__RESULT = 					{24 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_4__RESULT = 					{25 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_5__RESULT = 					{26 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_6__RESULT = 					{27 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_7__RESULT = 					{28 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_8__RESULT = 					{29 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_9__RESULT = 					{30 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_10__RESULT = 					{31 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  PCVM_11__RESULT = 					{32 ,0b1111111111111111, "RESULT"};				 // RH - Result of cell voltage measurement
const subreg_item  SCVM_HIGH__UPD_CNT = 				{33 ,0b0000000000000011, "UPD_CNT"};			 // RH - Update counter
const subreg_item  SCVM_HIGH__RESULT = 					{33 ,0b1111111111100000, "RESULT"};				 // RH - Result of SCVM measurement (highest cell voltage)
const subreg_item  SCVM_LOW__UPD_CNT = 					{34 ,0b0000000000000011, "UPD_CNT"};			 // RH - Update counter
const subreg_item  SCVM_LOW__RESULT = 					{34 ,0b1111111111100000, "RESULT"};				 // RH - Result of SCVM measurement (lowest cell voltage)
const subreg_item  STRESS_PCVM__STRESS_CORRECTION = 	{35 ,0b1111111111111111, "STRESS_CORRECTION"};	 // RH - Stress correction value PCVM
const subreg_item  BVM__RESULT = 						{36 ,0b1111111111111111, "RESULT"};				 // RH - Result of block voltage measurement
const subreg_item  EXT_TEMP_0__RESULT = 				{37 ,0b0000001111111111, "RESULT"};				 // RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
const subreg_item  EXT_TEMP_0__INTC = 					{37 ,0b0000110000000000, "INTC"};				 // RH - Indicates which current source was used Number of current source that was active for latest measurement.
const subreg_item  EXT_TEMP_0__PULLDOWN = 				{37 ,0b0001000000000000, "PULLDOWN"};			 // RH - Indicating pull-down switch state
const subreg_item  EXT_TEMP_0__VALID = 					{37 ,0b0010000000000000, "VALID"};				 // ROCR - Indicating a valid result
const subreg_item  EXT_TEMP_0__PD_ERR = 				{37 ,0b0100000000000000, "PD_ERR"};				 // ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
const subreg_item  EXT_TEMP_1__RESULT = 				{38 ,0b0000001111111111, "RESULT"};				 // RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
const subreg_item  EXT_TEMP_1__INTC = 					{38 ,0b0000110000000000, "INTC"};				 // RH - Indicates which current source was used Number of current source that was active for latest measurement.
const subreg_item  EXT_TEMP_1__PULLDOWN = 				{38 ,0b0001000000000000, "PULLDOWN"};			 // RH - Indicating pull-down switch state
const subreg_item  EXT_TEMP_1__VALID = 					{38 ,0b0010000000000000, "VALID"};				 // ROCR - Indicating a valid result
const subreg_item  EXT_TEMP_1__PD_ERR = 				{38 ,0b0100000000000000, "PD_ERR"};				 // ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
const subreg_item  EXT_TEMP_2__RESULT = 				{39 ,0b0000001111111111, "RESULT"};				 // RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
const subreg_item  EXT_TEMP_2__INTC = 					{39 ,0b0000110000000000, "INTC"};				 // RH - Indicates which current source was used Number of current source that was active for latest measurement.
const subreg_item  EXT_TEMP_2__PULLDOWN = 				{39 ,0b0001000000000000, "PULLDOWN"};			 // RH - Indicating pull-down switch state
const subreg_item  EXT_TEMP_2__VALID = 					{39 ,0b0010000000000000, "VALID"};				 // ROCR - Indicating a valid result
const subreg_item  EXT_TEMP_2__PD_ERR = 				{39 ,0b0100000000000000, "PD_ERR"};				 // ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
const subreg_item  EXT_TEMP_3__RESULT = 				{40 ,0b0000001111111111, "RESULT"};				 // RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
const subreg_item  EXT_TEMP_3__INTC = 					{40 ,0b0000110000000000, "INTC"};				 // RH - Indicates which current source was used Number of current source that was active for latest measurement.
const subreg_item  EXT_TEMP_3__PULLDOWN = 				{40 ,0b0001000000000000, "PULLDOWN"};			 // RH - Indicating pull-down switch state
const subreg_item  EXT_TEMP_3__VALID = 					{40 ,0b0010000000000000, "VALID"};				 // ROCR - Indicating a valid result
const subreg_item  EXT_TEMP_3__PD_ERR = 				{40 ,0b0100000000000000, "PD_ERR"};				 // ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
const subreg_item  EXT_TEMP_4__RESULT = 				{41 ,0b0000001111111111, "RESULT"};				 // RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
const subreg_item  EXT_TEMP_4__INTC = 					{41 ,0b0000110000000000, "INTC"};				 // RH - Indicates which current source was used Number of current source that was active for latest measurement.
const subreg_item  EXT_TEMP_4__PULLDOWN = 				{41 ,0b0001000000000000, "PULLDOWN"};			 // RH - Indicating pull-down switch state
const subreg_item  EXT_TEMP_4__VALID = 					{41 ,0b0010000000000000, "VALID"};				 // ROCR - Indicating a valid result
const subreg_item  EXT_TEMP_4__PD_ERR = 				{41 ,0b0100000000000000, "PD_ERR"};				 // ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
const subreg_item  EXT_TEMP_R_DIAG__RESULT = 			{42 ,0b0000001111111111, "RESULT"};				 // RH - Result of diagnostics resistor measurement SD-ADC result of diagnostics resistor measurement.
const subreg_item  EXT_TEMP_R_DIAG__INTC = 				{42 ,0b0000110000000000, "INTC"};				 // RH - Indicates which current source was used Number of current source that was active for latest measurement.
const subreg_item  EXT_TEMP_R_DIAG__CUR_SRC = 			{42 ,0b0001000000000000, "CUR_SRC"};			 // RH - Indicates which current source was used
const subreg_item  EXT_TEMP_R_DIAG__VALID = 			{42 ,0b0010000000000000, "VALID"};				 // ROCR - Indicating a valid result
const subreg_item  INT_TEMP__RESULT = 					{43 ,0b0000001111111111, "RESULT"};				 // RH - Result of internal temperatur measurement SD-ADC result of internal temperature measurement.
const subreg_item  INT_TEMP__VALID = 					{43 ,0b0010000000000000, "VALID"};				 // ROCR - Indicating a valid result
const subreg_item  MULTI_READ__RES = 					{44 ,0b1111111111111111, "RES"};				 // RH - Used in combination with MULTI_READ_CFG Reading this register by the host starts the multiple register readroutine which got define in the MULTI_READ_CFG register.
const subreg_item  MULTI_READ_CFG__PCVM_SEL = 			{45 ,0b0000000000001111, "PCVM_SEL"};			 // RW - Selects which PCVM results are part of the multiread No PCVM result for 1101B...1111B
const subreg_item  MULTI_READ_CFG__BVM_SEL = 			{45 ,0b0000000000010000, "BVM_SEL"};			 // RW - Selects if BVM result is part of multiread
const subreg_item  MULTI_READ_CFG__EXT_TEMP_SEL = 		{45 ,0b0000000011100000, "EXT_TEMP_SEL"};		 // RW - Selects which TEMP result is part of the multiread
const subreg_item  MULTI_READ_CFG__EXT_TEMP_R_SEL = 	{45 ,0b0000000100000000, "EXT_TEMP_R_SEL"};		 // RW - Selects if R_DIAG result is part of the multiread
const subreg_item  MULTI_READ_CFG__INT_TEMP_SEL = 		{45 ,0b0000001000000000, "INT_TEMP_SEL"};		 // RW - 0B NO_INT_TMP_RES: No INT_TEMP result (default)
const subreg_item  MULTI_READ_CFG__SCVM_SEL = 			{45 ,0b0000010000000000, "SCVM_SEL"};			 // RW - Selects if SCVM results are part of the multiread
const subreg_item  MULTI_READ_CFG__STRESS_PCVM_SEL = 	{45 ,0b0000100000000000, "STRESS_PCVM_SEL"};	 // RW - Selects if PCVM stress correction value is part of the multiread
const subreg_item  BAL_DIAG_OC__OC_i = 					{46 ,0b0000111111111111, "OC_i"};				 // ROCW - Balancing overcurrent in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  BAL_DIAG_UC__UC_i = 					{47 ,0b0000111111111111, "UC_i"};				 // ROCW - Balancing undercurrent in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
const subreg_item  INT_TEMP_2__RESULT = 				{48 ,0b0000001111111111, "RESULT"};				 // RH - Result of internal temperatur 2 measurement SD-ADC result of internal temperature 2 measurement.
const subreg_item  INT_TEMP_2__VALID = 					{48 ,0b0010000000000000, "VALID"};				 // ROCR - Indicating a valid result
const subreg_item  CONFIG__NODE_ID = 					{49 ,0b0000000000111111, "NODE_ID"};			 // RWO - Address (ID) of the node, distributed during enumeration NODE_ID = 0 --> iso UART signals are not forwardedNODE_ID = 63 --> reserved for broadcast commands
const subreg_item  CONFIG__EN_ALL_ADC = 				{49 ,0b0000001000000000, "EN_ALL_ADC"};			 // RW - Enable all ADCs If this bit is set, PCVM is done for each channel, independent ofPART_CONFIG setup.
const subreg_item  CONFIG__FN = 						{49 ,0b0000100000000000, "FN"};					 // RWO - Final Node The final node in stack must have this bit set. If final node does nothave FN set, no reply frame on broadcast will be sent.
const subreg_item  GPIO__IN_GPIO0 = 					{50 ,0b0000000000000001, "IN_GPIO0"};			 // RH - GPIO 0 input state (ignored if communication over GPIO pins)
const subreg_item  GPIO__OUT_GPIO0 = 					{50 ,0b0000000000000010, "OUT_GPIO0"};			 // RW - GPIO 0 output setting (ignored if communication over GPIO pins)
const subreg_item  GPIO__DIR_GPIO0 = 					{50 ,0b0000000000000100, "DIR_GPIO0"};			 // RW - GPIO 0 direction (ignored if communication over GPIO pins)
const subreg_item  GPIO__IN_GPIO1 = 					{50 ,0b0000000000001000, "IN_GPIO1"};			 // RH - GPIO 1 input state (ignored if communication over GPIO pins)
const subreg_item  GPIO__OUT_GPIO1 = 					{50 ,0b0000000000010000, "OUT_GPIO1"};			 // RW - GPIO 1 output setting (ignored if communication over GPIO pins)
const subreg_item  GPIO__DIR_GPIO1 = 					{50 ,0b0000000000100000, "DIR_GPIO1"};			 // RW - GPIO 1 direction (ignored if communication over GPIO pins)
const subreg_item  GPIO__IN_PWM1 = 						{50 ,0b0000000001000000, "IN_PWM1"};			 // RH - PWM 1 input state
const subreg_item  GPIO__PWM_PWM1 = 					{50 ,0b0000000010000000, "PWM_PWM1"};			 // RW - PWM 1 enable PWM function
const subreg_item  GPIO__OUT_PWM1 = 					{50 ,0b0000000100000000, "OUT_PWM1"};			 // RW - PWM 1 output setting
const subreg_item  GPIO__DIR_PWM1 = 					{50 ,0b0000001000000000, "DIR_PWM1"};			 // RW - PWM 1 direction
const subreg_item  GPIO__IN_PWM0 = 						{50 ,0b0000010000000000, "IN_PWM0"};			 // RH - PWM 0 input state
const subreg_item  GPIO__PWM_PWM0 = 					{50 ,0b0000100000000000, "PWM_PWM0"};			 // RW - PWM 0 enable PWM function
const subreg_item  GPIO__OUT_PWM0 = 					{50 ,0b0001000000000000, "OUT_PWM0"};			 // RW - PWM 0 Output Setting
const subreg_item  GPIO__DIR_PWM0 = 					{50 ,0b0010000000000000, "DIR_PWM0"};			 // RW - PWM 0 direction
const subreg_item  GPIO__VIO_UV = 						{50 ,0b1000000000000000, "VIO_UV"};				 // ROCW - VIO undervoltage error
const subreg_item  GPIO_PWM__PWM_DUTY_CYCLE = 			{51 ,0b0000000000011111, "PWM_DUTY_CYCLE"};		 // RW - PWM duty cycle 0 - 100%
const subreg_item  GPIO_PWM__PWM_PERIOD = 				{51 ,0b0001111100000000, "PWM_PERIOD"};			 // RW - PWM period time setting 2us - 62us
const subreg_item  ICVID__VERSION_ID = 					{52 ,0b0000000011111111, "VERSION_ID"};			 // RH - Version ID Read only version ID.
const subreg_item  ICVID__MANUFACTURER_ID = 			{52 ,0b1111111100000000, "MANUFACTURER_ID"};	 // RH - Manufacturer ID Read only manufacture ID.
const subreg_item  MAILBOX__DATA = 						{53 ,0b1111111111111111, "DATA"};				 // RW - Data storage register 2 data byte data storage
const subreg_item  CUSTOMER_ID_0__DATA = 				{54 ,0b1111111111111111, "DATA"};				 // RH - Unique ID part 1
const subreg_item  CUSTOMER_ID_1__DATA = 				{55 ,0b1111111111111111, "DATA"};				 // RH - Unique ID part 2
const subreg_item  WDOG_CNT__WD_CNT = 					{56 ,0b0000000001111111, "WD_CNT"};				 // RWH - Watchdog counter
const subreg_item  WDOG_CNT__MAIN_CNT = 				{56 ,0b1111111110000000, "MAIN_CNT"};			 // RH - Main counter Used to enable host controller to measure the main oscillatorfrequency. LSB = t Count_LSB. Reset by write access to WD_CNT ifRR_SYNC=1.
const subreg_item  SCVM_CONFIG__EN_SCVMi = 				{57 ,0b0000111111111111, "EN_SCVMi"};			 // RW - Enable SCVM for cell i
const subreg_item  STRESS_AUX__STRESS_CORRECTION = 		{58 ,0b1111111111111111, "STRESS_CORRECTION"};	 // RH - Stress correction value PCVM
const subreg_item  BAL_PWM__BAL_PWM = 					{59 ,0b0000000000000111, "BAL_PWM"};			 // RW - PWM balancing, starts with off-phase
const subreg_item  BAL_CNT_0__BAL_CNT_0 = 				{60 ,0b0000000000011111, "BAL_CNT_0"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_0__BAL_CNT_1 = 				{60 ,0b0000001111100000, "BAL_CNT_1"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_0__BAL_CNT_2 = 				{60 ,0b0111110000000000, "BAL_CNT_2"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_1__BAL_CNT_3 = 				{61 ,0b0000000000011111, "BAL_CNT_3"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_1__BAL_CNT_4 = 				{61 ,0b0000001111100000, "BAL_CNT_4"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_1__BAL_CNT_5 = 				{61 ,0b0111110000000000, "BAL_CNT_5"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_2__BAL_CNT_6 = 				{62 ,0b0000000000011111, "BAL_CNT_6"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_2__BAL_CNT_7 = 				{62 ,0b0000001111100000, "BAL_CNT_7"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_2__BAL_CNT_8 = 				{62 ,0b0111110000000000, "BAL_CNT_8"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_3__BAL_CNT_9 = 				{63 ,0b0000000000011111, "BAL_CNT_9"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_3__BAL_CNT_10 = 				{63 ,0b0000001111100000, "BAL_CNT_10"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
const subreg_item  BAL_CNT_3__BAL_CNT_11 = 				{63 ,0b0111110000000000, "BAL_CNT_11"};			 // RW - Balancing counter to switch off balancing after a certain time, counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.

// Concatenation of all subregister items used for iteration
const subreg_item* subreg_items[NUM_SUBREGISTERS] = {
	&PART_CONFIG__EN_CELLi, &PART_CONFIG__EN_CELL11, &OL_OV_THR__OV_THR, &OL_OV_THR__OL_THR_MAX, &OL_UV_THR__UV_THR, &OL_UV_THR__OL_THR_MIN, &TEMP_CONF__EXT_OT_THR, &TEMP_CONF__I_NTC, &TEMP_CONF__NR_TEMP_SENSE, &INT_OT_WARN_CONF__INT_OT_THR, &RR_ERR_CNT__NR_ERR, &RR_ERR_CNT__NR_EXT_TEMP_START, &RR_ERR_CNT__RR_SLEEP_CNT, &RR_CONFIG__RR_CNT, &RR_CONFIG__RR_SYNC, &RR_CONFIG__M_NR_ERR_ADC_ERR, &RR_CONFIG__M_NR_ERR_OL_ERR, &RR_CONFIG__M_NR_ERR_EXT_T_ERR, &RR_CONFIG__M_NR_ERR_INT_OT, &RR_CONFIG__M_NR_ERR_CELL_UV, &RR_CONFIG__M_NR_ERR_CELL_OV, &RR_CONFIG__M_NR_ERR_BAL_UC, &RR_CONFIG__M_NR_ERR_BAL_OC, &FAULT_MASK__ERR_PIN, &FAULT_MASK__M_ADC_ERR, &FAULT_MASK__M_OL_ERR, &FAULT_MASK__M_INT_IC_ERR, &FAULT_MASK__M_REG_CRC_ERR, &FAULT_MASK__M_EXT_T_ERR, &FAULT_MASK__M_INT_OT, &FAULT_MASK__M_CELL_UV, &FAULT_MASK__M_CELL_OV, &FAULT_MASK__M_BAL_ERR_UC, &FAULT_MASK__M_BAL_ERR_OC, &GEN_DIAG__UART_WAKEUP, &GEN_DIAG__MOT_MOB_N, &GEN_DIAG__BAL_ACTIVE, &GEN_DIAG__LOCK_MEAS, &GEN_DIAG__RR_ACTIVE, &GEN_DIAG__PS_ERR_SLEEP, &GEN_DIAG__ADC_ERR, &GEN_DIAG__OL_ERR, &GEN_DIAG__INT_IC_ERR, &GEN_DIAG__REG_CRC_ERR, &GEN_DIAG__EXT_T_ERR, &GEN_DIAG__INT_OT, &GEN_DIAG__CELL_UV, &GEN_DIAG__CELL_OV, &GEN_DIAG__BAL_ERR_UC, &GEN_DIAG__BAL_ERR_OC, &CELL_UV__UV_i, &CELL_OV__OV_i, &EXT_TEMP_DIAG__OPEN0, &EXT_TEMP_DIAG__SHORT0, &EXT_TEMP_DIAG__OT0, &EXT_TEMP_DIAG__OPEN1, &EXT_TEMP_DIAG__SHORT1, &EXT_TEMP_DIAG__OT1, &EXT_TEMP_DIAG__OPEN2, &EXT_TEMP_DIAG__SHORT2, &EXT_TEMP_DIAG__OT2, &EXT_TEMP_DIAG__OPEN3, &EXT_TEMP_DIAG__SHORT3, &EXT_TEMP_DIAG__OT3, &EXT_TEMP_DIAG__OPEN4, &EXT_TEMP_DIAG__SHORT4, &EXT_TEMP_DIAG__OT4, &DIAG_OL__OL_i, &REG_CRC_ERR__ADDR, &CELL_UV_DAC_COMP__UV_i, &CELL_OV_DAC_COMP__OV_i, &OP_MODE__PD, &OP_MODE__EXT_WD, &OP_MODE__SLEEP_REG_RESET, &OP_MODE__I_DIAG_EN, &OP_MODE__LR_TIME, &BAL_CURR_THR__OC_THR, &BAL_CURR_THR__UC_THR, &AVM_CONFIG__TEMP_MUX_DIAG_SEL, &AVM_CONFIG__AVM_TMP0_MASK, &AVM_CONFIG__AVM_TMP1_MASK, &AVM_CONFIG__AVM_TMP2_MASK, &AVM_CONFIG__AVM_TMP3_MASK, &AVM_CONFIG__AVM_TMP4_MASK, &AVM_CONFIG__AUX_BIPOLAR, &AVM_CONFIG__R_DIAG, &AVM_CONFIG__R_DIAG_SEL_0, &AVM_CONFIG__R_DIAG_SEL_1, &AVM_CONFIG__R_DIAG_CUR_SRC, &MEAS_CTRL__CVM_DEL, &MEAS_CTRL__PBOFF, &MEAS_CTRL__SCVM_START, &MEAS_CTRL__AVM_START, &MEAS_CTRL__BVM_MODE, &MEAS_CTRL__BVM_START, &MEAS_CTRL__CVM_MODE, &MEAS_CTRL__PCVM_START, &PCVM_0__RESULT, &PCVM_1__RESULT, &PCVM_2__RESULT, &PCVM_3__RESULT, &PCVM_4__RESULT, &PCVM_5__RESULT, &PCVM_6__RESULT, &PCVM_7__RESULT, &PCVM_8__RESULT, &PCVM_9__RESULT, &PCVM_10__RESULT, &PCVM_11__RESULT, &SCVM_HIGH__UPD_CNT, &SCVM_HIGH__RESULT, &SCVM_LOW__UPD_CNT, &SCVM_LOW__RESULT, &STRESS_PCVM__STRESS_CORRECTION, &BVM__RESULT, &EXT_TEMP_0__RESULT, &EXT_TEMP_0__INTC, &EXT_TEMP_0__PULLDOWN, &EXT_TEMP_0__VALID, &EXT_TEMP_0__PD_ERR, &EXT_TEMP_1__RESULT, &EXT_TEMP_1__INTC, &EXT_TEMP_1__PULLDOWN, &EXT_TEMP_1__VALID, &EXT_TEMP_1__PD_ERR, &EXT_TEMP_2__RESULT, &EXT_TEMP_2__INTC, &EXT_TEMP_2__PULLDOWN, &EXT_TEMP_2__VALID, &EXT_TEMP_2__PD_ERR, &EXT_TEMP_3__RESULT, &EXT_TEMP_3__INTC, &EXT_TEMP_3__PULLDOWN, &EXT_TEMP_3__VALID, &EXT_TEMP_3__PD_ERR, &EXT_TEMP_4__RESULT, &EXT_TEMP_4__INTC, &EXT_TEMP_4__PULLDOWN, &EXT_TEMP_4__VALID, &EXT_TEMP_4__PD_ERR, &EXT_TEMP_R_DIAG__RESULT, &EXT_TEMP_R_DIAG__INTC, &EXT_TEMP_R_DIAG__CUR_SRC, &EXT_TEMP_R_DIAG__VALID, &INT_TEMP__RESULT, &INT_TEMP__VALID, &MULTI_READ__RES, &MULTI_READ_CFG__PCVM_SEL, &MULTI_READ_CFG__BVM_SEL, &MULTI_READ_CFG__EXT_TEMP_SEL, &MULTI_READ_CFG__EXT_TEMP_R_SEL, &MULTI_READ_CFG__INT_TEMP_SEL, &MULTI_READ_CFG__SCVM_SEL, &MULTI_READ_CFG__STRESS_PCVM_SEL, &BAL_DIAG_OC__OC_i, &BAL_DIAG_UC__UC_i, &INT_TEMP_2__RESULT, &INT_TEMP_2__VALID, &CONFIG__NODE_ID, &CONFIG__EN_ALL_ADC, &CONFIG__FN, &GPIO__IN_GPIO0, &GPIO__OUT_GPIO0, &GPIO__DIR_GPIO0, &GPIO__IN_GPIO1, &GPIO__OUT_GPIO1, &GPIO__DIR_GPIO1, &GPIO__IN_PWM1, &GPIO__PWM_PWM1, &GPIO__OUT_PWM1, &GPIO__DIR_PWM1, &GPIO__IN_PWM0, &GPIO__PWM_PWM0, &GPIO__OUT_PWM0, &GPIO__DIR_PWM0, &GPIO__VIO_UV, &GPIO_PWM__PWM_DUTY_CYCLE, &GPIO_PWM__PWM_PERIOD, &ICVID__VERSION_ID, &ICVID__MANUFACTURER_ID, &MAILBOX__DATA, &CUSTOMER_ID_0__DATA, &CUSTOMER_ID_1__DATA, &WDOG_CNT__WD_CNT, &WDOG_CNT__MAIN_CNT, &SCVM_CONFIG__EN_SCVMi, &STRESS_AUX__STRESS_CORRECTION, &BAL_PWM__BAL_PWM, &BAL_CNT_0__BAL_CNT_0, &BAL_CNT_0__BAL_CNT_1, &BAL_CNT_0__BAL_CNT_2, &BAL_CNT_1__BAL_CNT_3, &BAL_CNT_1__BAL_CNT_4, &BAL_CNT_1__BAL_CNT_5, &BAL_CNT_2__BAL_CNT_6, &BAL_CNT_2__BAL_CNT_7, &BAL_CNT_2__BAL_CNT_8, &BAL_CNT_3__BAL_CNT_9, &BAL_CNT_3__BAL_CNT_10, &BAL_CNT_3__BAL_CNT_11,
};

//****************************************************************************
// Register ready/error/queued bitmap functions - Every bit of these maps represents if the related register index is ready, without error or queued
//		When a command is queued    	  		  - 'queued' bit is set to 1 (prevents queuing of one register multiple times)
// 	 	When queue item is send 		  		  - 'ready'  bit is set to 0 (prevents conversion or usage of non-ready data)
//		When result received or timeout 		  - 'ready'  bit is set to 1 and 'queued' is set to 0 (regardless if error or OK)
//		When command/response fail, CRC fail, etc - 'error'  bit is set (check this before using the register data)
//
// Note: MULTI_READ will be treated differently but can be used the same way as all others.
//       This will check/set/reset all registers mentioned in multi_read_cfg_reg_idx_list that are written by tle9012_writeMultiReadConfig())
//       Using MULTI_READ without proper prior use of tle9012_writeMultiReadConfig() will result in catastrophic bugs (e.g. Simultaneous UART TX and RX, overflows, etc)
//
// nodeID		-> The ID of the node that shall be effected. Range 1 to TLE9012_NODES_SIZE
// reg_idx		-> The index of the register that shall be used, see register.h definitions.  E.g. PART_CONFIG etc
// isBroadcast  -> If set to 1 this will effect all nodes (will check/set/reset all bits of one register in all nodes)
//****************************************************************************
// READY FUNCTIONS
// Check if the ready bit for the register index is set
uint8_t checkRegisterReady_tle9012(uint8_t nodeID, uint8_t reg_index, uint8_t isBroadcast) {
	if(reg_index == MULTI_READ){
		uint8_t result = 1;
		for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++){
			result &= checkRegisterReady_tle9012(nodeID, multi_read_cfg_reg_idx_list[i], isBroadcast);
		}
		//DEBUGPRINTF_BITMAP_CHANGE("MULTI check result %d\r\n", result);
		return result;
	}
	else if(isBroadcast == 0)
		return (tle9012_nodes[nodeID-1].reg_items_ready_bitmap & ((uint64_t)1 << ((uint64_t)reg_index))) != 0;
	else{
		uint64_t result = UINT64_MAX;
		uint8_t curNodeIdx = 0;
		do{
			result &= (tle9012_nodes[curNodeIdx].reg_items_ready_bitmap & ((uint64_t)1 << ((uint64_t)reg_index)));
			//DEBUGPRINTF_BITMAP_CHANGE("\t\t\tCheck ready register index %2d of Node index %2d: %d\r\n", reg_index, curNodeIdx, result != 0);
			curNodeIdx++;
		}while(curNodeIdx < TLE9012_NODES_SIZE);
		return result != 0;
	}
}
// Set ready bit for the the register index
void setRegisterReady_tle9012(uint8_t nodeID, uint8_t reg_index, uint8_t isBroadcast) {
	if(reg_index == MULTI_READ){
		for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++){
			setRegisterReady_tle9012(nodeID, multi_read_cfg_reg_idx_list[i], isBroadcast);
		}
	}
	else if(isBroadcast == 0){
		DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of NodeID %2d: Ready\r\n", reg_index, nodeID);
		tle9012_nodes[nodeID-1].reg_items_ready_bitmap |= ((uint64_t)1 << ((uint64_t)reg_index));
	}
	else{
		uint8_t curNodeIdx = 0;
		do{
			DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node index %2d: Ready\r\n", reg_index, curNodeIdx);
			tle9012_nodes[curNodeIdx].reg_items_ready_bitmap |= ((uint64_t)1 << ((uint64_t)reg_index));
			curNodeIdx++;
		}while(curNodeIdx < TLE9012_NODES_SIZE);
	}
}
// Reset ready bit for the the register index
void resetRegisterReady_tle9012(uint8_t nodeID, uint8_t reg_index, uint8_t isBroadcast) {
	if(reg_index == MULTI_READ){
		for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++){
			resetRegisterReady_tle9012(nodeID, multi_read_cfg_reg_idx_list[i], isBroadcast);
		}
	}
	else if(isBroadcast == 0){
		DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of NodeID %2d: NOT Ready\r\n", reg_index, nodeID);
		tle9012_nodes[nodeID-1].reg_items_ready_bitmap &= ~((uint64_t)1 << ((uint64_t)reg_index));
	}
	else{
		uint8_t curNodeIdx = 0;
		do{
			DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node index %2d: NOT Ready\r\n", reg_index, curNodeIdx);
			tle9012_nodes[curNodeIdx].reg_items_ready_bitmap &= ~((uint64_t)1 << ((uint64_t)reg_index));
			curNodeIdx++;
		}while(curNodeIdx < TLE9012_NODES_SIZE);
	}
}
// ERROR FUNCTIONS
// Check if the error bit for the register index is set
uint8_t checkRegisterError_tle9012(uint8_t nodeID, uint8_t reg_index, uint8_t isBroadcast) {
	if(reg_index == MULTI_READ){
		uint8_t result = 0;
		for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++){
			result |= checkRegisterError_tle9012(nodeID, multi_read_cfg_reg_idx_list[i], isBroadcast);
		}
		return result;
	}
	else if(isBroadcast == 0)
		return (tle9012_nodes[nodeID-1].reg_items_error_bitmap & ((uint64_t)1 << ((uint64_t)reg_index))) != 0;
	else{
		uint64_t result = 0;
		uint8_t curNodeIdx = 0;
		do{
			result |= (tle9012_nodes[curNodeIdx].reg_items_error_bitmap & ((uint64_t)1 << ((uint64_t)reg_index)));
			DEBUGPRINTF_BITMAP_CHANGE("\t\t\tCheck error register index %2d of Node index %2d: %d\r\n", reg_index, curNodeIdx, result != 0);
			curNodeIdx++;
		}while(curNodeIdx < TLE9012_NODES_SIZE);
		return result != 0;
	}
}
// Set error bit for the the register index
void setRegisterError_tle9012(uint8_t nodeID, uint8_t reg_index, uint8_t isBroadcast) {
	if(reg_index == MULTI_READ){
		for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++){
			setRegisterError_tle9012(nodeID, multi_read_cfg_reg_idx_list[i], isBroadcast);
		}
	}
	else if(isBroadcast == 0){
		DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node ID %2d: Error\r\n", reg_index, nodeID);
		tle9012_nodes[nodeID-1].reg_items_error_bitmap |= ((uint64_t)1 << ((uint64_t)reg_index));
	}
	else{
		uint8_t curNodeIdx = 0;
		do{
			DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node index %2d: Error\r\n", reg_index, curNodeIdx);
			tle9012_nodes[curNodeIdx].reg_items_error_bitmap |= ((uint64_t)1 << ((uint64_t)reg_index));
			curNodeIdx++;
		}while(curNodeIdx < TLE9012_NODES_SIZE);
	}
}
// Reset error bit for the the register index
void resetRegisterError_tle9012(uint8_t nodeID, uint8_t reg_index, uint8_t isBroadcast) {
	if(reg_index == MULTI_READ){
		for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++){
			resetRegisterError_tle9012(nodeID, multi_read_cfg_reg_idx_list[i], isBroadcast);
		}
	}
	else if(isBroadcast == 0){
		DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node ID %2d: NO Error\r\n", reg_index, nodeID);
		tle9012_nodes[nodeID-1].reg_items_error_bitmap &= ~((uint64_t)1 << ((uint64_t)reg_index));
	}
	else{
		uint8_t curNodeIdx = 0;
		do{
			DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node index %2d: NO Error\r\n", reg_index, curNodeIdx);
			tle9012_nodes[curNodeIdx].reg_items_error_bitmap &= ~((uint64_t)1 << ((uint64_t)reg_index));
			curNodeIdx++;
		}while(curNodeIdx < TLE9012_NODES_SIZE);
	}
}
// QUEUED FUNCTIONS
// Check if the queued bit for the register index is set
uint8_t checkRegisterQueued_tle9012(uint8_t nodeID, uint8_t reg_index, uint8_t isBroadcast) {
	if(reg_index == MULTI_READ){
		uint8_t result = 1;
		for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++){
			result &= checkRegisterQueued_tle9012(nodeID, multi_read_cfg_reg_idx_list[i], isBroadcast);
		}
		return result;
	}
	else if(isBroadcast == 0)
		return (tle9012_nodes[nodeID-1].reg_items_queued_bitmap & ((uint64_t)1 << ((uint64_t)reg_index))) != 0;
	else{
		uint64_t result = UINT64_MAX;
		uint8_t curNodeIdx = 0;
		do{
			result &= (tle9012_nodes[curNodeIdx].reg_items_queued_bitmap & ((uint64_t)1 << ((uint64_t)reg_index)));
			//DEBUGPRINTF_BITMAP_CHANGE("\t\t\tCheck Queued register index %2d of Node index %2d: %d\r\n", reg_index, curNodeIdx, result != 0);
			curNodeIdx++;
		}while(curNodeIdx < TLE9012_NODES_SIZE);
		return result != 0;
	}
}
// Set queued bit for the the register index
void setRegisterQueued_tle9012(uint8_t nodeID, uint8_t reg_index, uint8_t isBroadcast) {
	if(reg_index == MULTI_READ){
		for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++){
			setRegisterQueued_tle9012(nodeID, multi_read_cfg_reg_idx_list[i], isBroadcast);
		}
	}
	else if(isBroadcast == 0){
		DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node ID %2d: Queued\r\n", reg_index, nodeID);
		tle9012_nodes[nodeID-1].reg_items_queued_bitmap |= ((uint64_t)1 << ((uint64_t)reg_index));
	}
	else{
		uint8_t curNodeIdx = 0;
		do{
			DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node index %2d: Queued\r\n", reg_index, curNodeIdx);
			tle9012_nodes[curNodeIdx].reg_items_queued_bitmap |= ((uint64_t)1 << ((uint64_t)reg_index));
			curNodeIdx++;
		}while(curNodeIdx < TLE9012_NODES_SIZE);
	}
}
// Reset queued bit for the the register index
void resetRegisterQueued_tle9012(uint8_t nodeID, uint8_t reg_index, uint8_t isBroadcast) {
	if(reg_index == MULTI_READ){
		for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++){
			resetRegisterQueued_tle9012(nodeID, multi_read_cfg_reg_idx_list[i], isBroadcast);
		}
	}
	else if(isBroadcast == 0){
		DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node index %2d: NOT Queued\r\n", reg_index, nodeID);
		tle9012_nodes[nodeID-1].reg_items_queued_bitmap &= ~((uint64_t)1 << ((uint64_t)reg_index));
	}
	else{
		uint8_t curNodeIdx = 0;
		do{
			DEBUGPRINTF_BITMAP_CHANGE("\t\t\tRegister index %2d of Node index %2d: NOT Queued\r\n", reg_index, curNodeIdx);
			tle9012_nodes[curNodeIdx].reg_items_queued_bitmap &= ~((uint64_t)1 << ((uint64_t)reg_index));
			curNodeIdx++;
		}while(curNodeIdx < TLE9012_NODES_SIZE);
	}
}


//****************************************************************************
// setRegisterCacheDataByIndex
//****************************************************************************
 inline void setRegisterCacheDataByIndex_tle9012(uint8_t nodeID, uint8_t reg_index, uint16_t reg_data)
{
	 tle9012_nodes[nodeID-1].reg_data[reg_index] = reg_data;
}

//****************************************************************************
// getRegisterCacheDataByIndex
//****************************************************************************
inline uint16_t getRegisterCacheDataByIndex_tle9012(uint8_t nodeID, uint8_t reg_index)
{
    return tle9012_nodes[nodeID-1].reg_data[reg_index];
}

//****************************************************************************
// sendWrite - Writes all 2 bytes to a given register to the ICs (ID specific based on node)
// NOTE: This works around the queuing system and can only be used if it is
// certain that no data is receive at that time.
//
// node         -> Pointer to the tle9012 node object representing one IC
// reg_address  -> Actual address of the register to be changed (not the index!)
// data         -> The 2 byte data that will be written
// isBroadcast  -> If 1 the write will be send as broadcast (all IDs)
//**********************************************************************
void sendWrite(tle9012_node* node, uint8_t reg_address, uint16_t data, uint8_t isBroadcast)
{
    if (isEnabled()) {
    	// Determine ID frame (send to ID or as broadcast?)
    	uint8_t ID;
    	if(isBroadcast)
    		ID = FRAME_ID_WRITE | FRAME_ID_BROADCAST;
    	else
    		ID = FRAME_ID_WRITE | node->ID;

    	// Compose write frame and send data
		tle9012_tx_buffer[0] = FRAME_SYNC;
		tle9012_tx_buffer[1] = ID;
		tle9012_tx_buffer[2] = reg_address;
		tle9012_tx_buffer[3] = (data >> 8);
		tle9012_tx_buffer[4] = data & 0xff;
		tle9012_tx_buffer[5] = crc8(tle9012_tx_buffer, 5);
		tle9012_tx_buffer_size = 6;
		uartWrite(tle9012_tx_buffer_size, 0);

		node->timeLastCommandSent = TIMER_COUNTER_1MS;
    }
    else {
        tle9012_problemDetected = 1;
        DEBUGPRINTF("[ERROR]: sendWrite - Device is disabled");
    }
}

//****************************************************************************
// sendRead - Reads all 2 bytes of a given register from the ICs (ID specific based on node)
// NOTE: This works around the queuing system and can only be used if it is
// certain that no data is receive at that time.
//
// node         -> Pointer to the tle9012 node object representing one IC
// reg_address  -> Actual address of the register to be read (not the index!)
// isBroadcast  -> If 1 the read will be send as broadcast (all IDs)
//**********************************************************************
void sendRead(tle9012_node* node, uint8_t reg_address, uint8_t isBroadcast)
{
    if (isEnabled()) {
    	// Determine ID frame (send to ID or as broadcast?)
    	uint8_t ID;
    	if(isBroadcast)
    		ID = FRAME_ID_READ | FRAME_ID_BROADCAST;
    	else
    		ID = FRAME_ID_READ | node->ID;

    	// Compose read frame and send data
		tle9012_tx_buffer[0] = FRAME_SYNC;
		tle9012_tx_buffer[1] = ID;
		tle9012_tx_buffer[2] = reg_address;
		tle9012_tx_buffer[3] = crc8(tle9012_tx_buffer, 3);
		tle9012_tx_buffer_size = 4;
		uartWrite(tle9012_tx_buffer_size, 0);

		node->timeLastCommandSent = TIMER_COUNTER_1MS;
    }
    else {
        tle9012_problemDetected = 1;
        DEBUGPRINTF("[ERROR]: sendRead - Device is disabled");
    }
}


//****************************************************************************
// sendWatchdogUpdate - Resets the watchdog counter to the highest value and
//                      therefore prevents device from going to sleep
// NOTE: This works around the queuing system and can only be used if it is
// certain that no data is receive at that time (e.g in manageQueue() EXECUTE_TX state or after an blocking command)
//
// nodeID       -> Pointer to the tle9012 node object representing one IC
// isBroadcast  -> If 1 the write will be send as broadcast (all IDs)
//**********************************************************************
void sendWatchdogUpdate(uint8_t nodeID, uint8_t isBroadcast)
{
    if (isEnabled()) {
    	// Determine ID frame (send to ID or as broadcast?)
    	uint8_t ID;
    	if(isBroadcast)
    		ID = FRAME_ID_WRITE | FRAME_ID_BROADCAST;
    	else
    		ID = FRAME_ID_WRITE | nodeID;

    	// Set register not ready
    	resetRegisterReady_tle9012(nodeID, WDOG_CNT, isBroadcast);

    	// Compose write frame and send data
		tle9012_tx_buffer[0] = FRAME_SYNC;
		tle9012_tx_buffer[1] = ID;
		tle9012_tx_buffer[2] = reg_items[WDOG_CNT].address;
		tle9012_tx_buffer[3] = (WD_CNT__127 >> 8);
		tle9012_tx_buffer[4] = WD_CNT__127 & 0xff;
		tle9012_tx_buffer[5] = crc8(tle9012_tx_buffer, 5);
		tle9012_tx_buffer_size = 6;
		uartWrite(tle9012_tx_buffer_size, 1);

		// Refresh interpreter until register is ready, timeout occurred or the loop was executed to often (last resort)
		uint32_t startTime = TIMER_COUNTER_1MS;
		for(uint32_t i = 0; i<UINT32_MAX; i++){
			// Execute management of interpreter
			tle9012_manageInterpreter(tle9012_nodes, TLE9012_NODES_SIZE);

			// If blocking takes to long stop loop
			if(TIMER_COUNTER_1MS > (startTime + TIMEOUT_COMMAND+50)){
				DEBUGPRINTF("[ERROR]: sendWatchdogUpdate reply timeout after %ld ms (%ld loops) on Node %d \r\n", (uint32_t)(TIMER_COUNTER_1MS - startTime), i, nodeID);
				break;
			}

			// Stop loop if register ready
			if(checkRegisterReady_tle9012(nodeID, WDOG_CNT, isBroadcast)){
				DEBUGPRINTF_WATCHDOG("sendWatchdogUpdate reply received after %ld ms (took %ld ms and %ld loops) on Node %d \r\n", TIMER_COUNTER_1MS-timeLastWatchdogRefresh, TIMER_COUNTER_1MS - startTime, i, nodeID);
				timeLastWatchdogRefresh = TIMER_COUNTER_1MS;
				break;
			}
		}

    }
    else {
        tle9012_problemDetected = 1;
        DEBUGPRINTF_WATCHDOG("[ERROR]: sendWrite - Device is disabled");
    }
}

//****************************************************************************
// setConfigBit TODO untested
//
// only use this for latched bits - otherwise this will save the reset byte in cache
// The used register must be read to cache before this!
//
//****************************************************************************
void setConfigBit_tle9012(uint8_t nodeID, uint8_t state, uint8_t reg_index, uint16_t bit, uint8_t blocking, uint8_t isBroadcast)
{
	// Set state
	if (state == 0)
		clearBits_tle9012(nodeID, reg_index, bit, blocking, isBroadcast);
	else if (state == 1)
		setBits_tle9012(nodeID, reg_index, bit, blocking, isBroadcast);
	else {
		tle9012_problemDetected = 1;
		DEBUGPRINTF("[ERROR]: SetConfigBit - Invalid state\r\n");
	}
}

//****************************************************************************
// setBits TODO untested
//
// sets the bits in given register and writes register
// The used register must be read to cache before this!
//
//***************************************************************************
void setBits_tle9012(uint8_t nodeID, uint8_t reg_index, uint16_t bitmask, uint8_t blocking, uint8_t isBroadcast)
{
	if (isEnabled()) {
		// Get current register data
		uint16_t data = getRegisterCacheDataByIndex_tle9012(nodeID, reg_index);

		// Set bits
		data |= bitmask;

		// Write change to node
		tle9012_writeRegisterByIndex(nodeID, reg_index, data, blocking, isBroadcast);
	}
	else {
		DEBUGPRINTF("[ERROR]: Device is disabled");
	}
}

//****************************************************************************
// clearBits TODO untested
//
// clears the bits in given register and writes register
// (the used register must be read to cache before this!)
//
//***************************************************************************
void clearBits_tle9012(uint8_t nodeID, uint8_t reg_index, uint16_t bitmask, uint8_t blocking, uint8_t isBroadcast)
{
    if(isEnabled()){
    	// Get current register data
    	uint16_t data = getRegisterCacheDataByIndex_tle9012(nodeID, reg_index);

    	// Clear bits
    	data &= ~bitmask;

		// Write change to node
		tle9012_writeRegisterByIndex(nodeID, reg_index, data, blocking, isBroadcast);
    }
	else {
		DEBUGPRINTF("[ERROR]: Device is disabled");
	}
}



