/*
 * tle9012_registers.h
 *
 *	Library for TLE9012 Li-Ion battery monitoring and balancing system IC.
 *	See main file Lib_TLE9012.c for how to use guide
 *	Created by R. Santeler @ Infineon 2024
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

#ifndef TLE9012_REGISTERS_H__
#define TLE9012_REGISTERS_H__

//#pragma once

#include <stdint.h>


//****************************************************************************
// Settings
//****************************************************************************
// Number of nodes that shall be initialized.
// This must match exactly the number of TLE9012 Nodes that are to be addressed
#define TLE9012_NODES_SIZE 				1
// Defines the minimum time in ms that must pass between watchdog refreshes,
// used to keep the ICs alive.
// Note: Per default the ICs can go to sleep when they did not receive
// a update after 1.8 seconds.
// However the here mentioned time must take the maximum time
// between tle9012_manageQueue() executions into account (time between main loops).
#define TLE9012_WATCHDOG_REFRESH_TIME 	1300
#define TLE9012_WATCHDOG_MAXIMUM_TIME 	1755

// Conversion Settings
#define BVM_FSR             60.0			// V. Full scale range
#define BVM_BIT_DIVIDER  65536.0 			// 2^result_bits - If result precision is changed this must also be changed. see datasheet/manual
#define PCVM_FSR             5.0			// V. Full scale range
#define PCVM_BIT_DIVIDER 65536.0 			// 2^result_bits - If result precision is changed this must also be changed. see datasheet/manual
#define CELL_OVUV_LSB		 0.0048828125	// =(TEMP_FSR / 2^10). LSB used to set cell over and undervoltage thresholds.
											// E.g. OL_OV_THR = 4.6/CELL_OVUV_LSB
#define TEMP_INT_LSB		 0.6624			// TMP internal temperature resolution in Kelvin
#define TEMP_FSR   		  	 2.0 			// V. Full scale range
#define TEMP_R   		   100.0 			// Ohm. Base resistance of e.g.: PT100. see datasheet/manual
#define TEMP_INTC            1.0
#define TEMP_DENOMINATOR     0.32768		// =(2^10 * 320*10^-6). see datasheet/manual
#define SCVM_FSR             5.0			// V. Full scale range
#define COMP_FSR             5.0			// V. Full scale range
#define BAVM_FSR             2.0			// V. Full scale range
#define AVM_FSR              2.0			// V. Full scale range

//****************************************************************************
// Global flags
//****************************************************************************
// Used to check if an action was successful (e.g. switchON())
// and if failure flags are set
extern uint8_t tle9012_problemDetected;
extern uint8_t generalDiagnosticProblemDetected;

//****************************************************************************
// Definitions
//****************************************************************************
#define NUM_REGISTERS 64 		// Number of registers
#define NUM_SUBREGISTERS 200	// Number of sub-registers in all registers

// Representation item of each TLE9012 IC management, state,
// register data and converted values
typedef struct {
	//****************************************************************************
	// The node ID of the TLE9012 node - must be in order inside array starting with 1
	//****************************************************************************
	uint8_t ID;
	//****************************************************************************
	// Time in us when the last command was send to this node
	//****************************************************************************
	uint32_t timeLastCommandSent;
	//****************************************************************************
	// Data buffer for each register - shadow registers
	//****************************************************************************
	uint16_t reg_data[NUM_REGISTERS];
	//****************************************************************************
	// Register ready/error bitmap - Every bit of these maps represents
	// if the related register
	// index is ready and without error
	//		if command is queued - ready is set to 0 until result has been
	// 		received or timeout occurred
	//		if command failed or CRC was not correct - the error bit is set
	//****************************************************************************
	uint64_t reg_items_ready_bitmap;
	uint64_t reg_items_error_bitmap;
	uint64_t reg_items_queued_bitmap;
	//****************************************************************************
	// converted results from reg_data
	//****************************************************************************
	//uint32_t multi_read_cfg_bitmap;
	//****************************************************************************
	// converted results from reg_data
	//****************************************************************************
	uint16_t cell_voltages[12]; 	// mV
	double block_voltage;			//  V
	double temperatures[5];			//  C
	double temperaturesInternal[2]; //  C
} tle9012_node;

typedef struct {
    uint16_t address;
    char* name;
} reg_item;

typedef struct {
    uint8_t  reg_index;
    uint16_t mask;
    char* name;
} subreg_item;

//****************************************************************************
// Objects
//****************************************************************************
// Combined objects of all register and subregister objects
// Array of register addresses and names
extern reg_item  reg_items[NUM_REGISTERS];
// Array of subregister addresses, masks, and names
// (combines all predefined subreg_items to make the iterable)
extern const subreg_item* subreg_items[NUM_SUBREGISTERS];
// Array of all nodes, holden all data of a node
// (shadow registers, bitmaps, conversion results, etc)
extern tle9012_node tle9012_nodes[TLE9012_NODES_SIZE];
extern uint8_t multi_read_cfg_reg_idx_list[23];
extern uint8_t multi_read_cfg_reg_idx_list_num;
// Time (ms) of the last refresh of the watchdog counter.
// If the time difference exceeds WD maximum interval the device will go to sleep
extern uint32_t timeLastWatchdogRefresh;

//****************************************************************************
// Predefined Communication Frames
//****************************************************************************
#define FRAME_WAKE	 		0xAA
#define FRAME_SYNC 			0x1E
#define FRAME_ID_READ 		0b00000000	// Template for a ID frame to read a register.
										// Actual device ID must be added e.g. via or command
#define FRAME_ID_WRITE 		0b10000000	// Template for a ID frame to write a register.
										// Actual device ID must be added e.g. via or command
#define FRAME_ID_BROADCAST 	0b00111111	// Template for a ID frame to execute a broadcast.
										// Read/write bit must be added e.g. via or command


//****************************************************************************
// Macro definition of register names to indexes - These shall be used whenever a reg_index parameter is given
//****************************************************************************
#define  PART_CONFIG 0 					  // 0001H Partitioning config [supplied in sleep)
#define  OL_OV_THR 1 					  // 0002H Cell voltage thresholds [supplied in sleep)
#define  OL_UV_THR 2 					  // 0003H Cell voltage thresholds [supplied in sleep)
#define  TEMP_CONF 3 					  // 0004H Temperature measurement configuration [supplied in sleep)
#define  INT_OT_WARN_CONF 4 			  // 0005H Internal temperature measurement configuration [supplied in sleep)
#define  RR_ERR_CNT 5 					  // 0008H Round robin ERR counters [supplied in sleep)
#define  RR_CONFIG 6 					  // 0009H Round robin configuration [supplied in sleep)
#define  FAULT_MASK 7 					  // 000AH ERR pin / EMM mask [supplied in sleep)
#define  GEN_DIAG 8 					  // 000BH General diagnostics [supplied in sleep)
#define  CELL_UV 9 						  // 000CH Cell voltage supervision warning flags UV [supplied in sleep)
#define  CELL_OV 10 					  // 000DH Cell voltage supervision warning flags OV [supplied in sleep)
#define  EXT_TEMP_DIAG 11 				  // 000EH External overtemperature warning flags [supplied in sleep)
#define  DIAG_OL 12 					  // 0010H Diagnostics open load [supplied in sleep)
#define  REG_CRC_ERR 13 				  // 0011H REG_CRC_ERR [supplied in sleep)
#define  CELL_UV_DAC_COMP 14 			  // 0012H Cell voltage supervision warning flags UV [supplied in sleep)
#define  CELL_OV_DAC_COMP 15 			  // 0013H Cell voltage supervision warning flags OV [supplied in sleep)
#define  OP_MODE 16 					  // 0014H Operation mode
#define  BAL_CURR_THR 17 				  // 0015H Balancing current thresholds
#define  BAL_SETTINGS 18 				  // 0016H Balance settingsONi (i=0-11) i rwh Switching state of balancing switch i
#define  AVM_CONFIG 19 					  // 0017H Auxiliary voltage measurement configuration
#define  MEAS_CTRL 20 					  // 0018H Measurement control
#define  PCVM_0 21 						  // 0019H Primary cell voltage measurement i
#define  PCVM_1 22 						  // 0020H Primary cell voltage measurement i
#define  PCVM_2 23 						  // 0021H Primary cell voltage measurement i
#define  PCVM_3 24 						  // 0022H Primary cell voltage measurement i
#define  PCVM_4 25 						  // 0023H Primary cell voltage measurement i
#define  PCVM_5 26 						  // 0024H Primary cell voltage measurement i
#define  PCVM_6 27 						  // 0025H Primary cell voltage measurement i
#define  PCVM_7 28 						  // 0026H Primary cell voltage measurement i
#define  PCVM_8 29 						  // 0027H Primary cell voltage measurement i
#define  PCVM_9 30 						  // 0028H Primary cell voltage measurement i
#define  PCVM_10 31 					  // 0029H Primary cell voltage measurement i
#define  PCVM_11 32 					  // 0030H Primary cell voltage measurement i
#define  SCVM_HIGH 33 					  // 0025H SCVM highest cell voltage
#define  SCVM_LOW 34 					  // 0026H SCVM lowest cell voltage
#define  STRESS_PCVM 35 				  // 0027H Stress correction PCVM
#define  BVM 36 						  // 0028H Block voltage measurement
#define  EXT_TEMP_0 37 					  // 0029H Temp result 0
#define  EXT_TEMP_1 38 					  // 002AH Temp result 1
#define  EXT_TEMP_2 39 					  // 002BH Temp result 2
#define  EXT_TEMP_3 40 					  // 002CH Temp result 3
#define  EXT_TEMP_4 41 					  // 002DH Temp result 4
#define  EXT_TEMP_R_DIAG 42 			  // 002FH Temp result R diagnose
#define  INT_TEMP 43 					  // 0030H Chip temperature
#define  MULTI_READ 44 					  // 0031H Multiread command
#define  MULTI_READ_CFG 45 				  // 0032H Multiread configuration
										  // (This register must be written with a broadcast write command)
#define  BAL_DIAG_OC 46 				  // 0033H Passive balancing diagnostics OVERCURRENT
#define  BAL_DIAG_UC 47 				  // 0034H Passive balancing diagnostics UNDERCURRENT
#define  INT_TEMP_2 48 					  // 0035H Chip temperature 2
#define  CONFIG_MAIN 49 				  // 0036H Configuration  (added _MAIN because of name conflicts)
#define  GPIO_MAIN 50 					  // 0037H General purpose input / output (added _MAIN because of name conflicts)
#define  GPIO_PWM 51 					  // 0038H PWM settings
#define  ICVID 52 						  // 0039H IC version and manufacturing ID
#define  MAILBOX 53 					  // 003AH Mailbox register
#define  CUSTOMER_ID_0 54 				  // 003BH Customer ID 0
#define  CUSTOMER_ID_1 55 				  // 003CH Customer ID 1
#define  WDOG_CNT 56 					  // 003DH Watchdog counter
#define  SCVM_CONFIG 57 				  // 003EH SCVM configuration
#define  STRESS_AUX 58 					  // 003FH Stress correction AUX
#define  BAL_PWM 59 					  // 005BH Balancing PWM
#define  BAL_CNT_0 60 					  // 005CH Balancing counter register 0
#define  BAL_CNT_1 61 					  // 005DH Balancing counter register 1
#define  BAL_CNT_2 62 					  // 005EH Balancing counter register 2
#define  BAL_CNT_3 63 					  // 005FH Balancing counter register 3



//****************************************************************************
// Macro definition of subregister items and settings
// These shall be used whenever a subreg or setting parameter is given
//****************************************************************************
// PART_CONFIG - 0001H Partitioning config [supplied in sleep)
extern const subreg_item  PART_CONFIG__EN_CELLi;				// RW - Enable cell monitoring for cell i
    #define EN_CELLi__NO_CELL_ATTACHED		 0b0000000000000000	// No cell attached (default) 
    #define EN_CELLi__CELL_ATTACHED			 0b0000010000000000	// Cell attached 
extern const subreg_item  PART_CONFIG__EN_CELL11;				// RW - Enable cell monitoring for cell 11
    #define EN_CELL11__NO_CELL_ATTACHED		 0b0000000000000000	// No cell attached 
    #define EN_CELL11__CELL_ATTACHED		 0b0000100000000000	// Cell attached (default) 

// OL_OV_THR - 0002H Cell voltage thresholds [supplied in sleep)
extern const subreg_item  OL_OV_THR__OV_THR;					// RW - Overvoltage fault threshold 10-bit overvoltage fault threshold.
																// Battery input voltages (U0 to U11)are tested for overvoltage with given value.
																// OV error is detected and indicated in GEN_DIAG register
																// if cell voltage is higher than OV faultthreshold.
    #define OV_THR__THRESHOLD				 0b0000001111111111	// Threshold (default) 
extern const subreg_item  OL_OV_THR__OL_THR_MAX; 				// RW - Open load maximum voltage drop threshold (LSB8) 6-bit (LSB8)
																// to define the maximum threshold for the voltage drop
																// while OL-diagnostics (I OL_DIAG * R F).
																// If voltage drop > OL_THR_MAX, theOLx bit of channel x is set.
    #define OL_THR_MAX__THRESHOLD			 0b1111110000000000	// Threshold (default) 

// OL_UV_THR - 0003H Cell voltage thresholds [supplied in sleep)
extern const subreg_item  OL_UV_THR__UV_THR; 					// RW - Undervoltage fault threshold 10-bit undervoltage fault threshold.
																// Battery input voltages (U0 to U11)are tested for undervoltage with given value.
																// UV error is detected andindicated in GEN_DIAG register if cell voltage
																// is lower than UV faultthreshold.
    #define UV_THR__THRESHOLD				 0b0000000000000000	// Threshold (default) 
extern const subreg_item  OL_UV_THR__OL_THR_MIN; 				// RW - Open load minimum voltage drop threshold (LSB8) 6-bit (LSB8)
																// to define the minimum threshold for the voltage drop
																// while OL-diagnostics (I OL_DIAG * R F).
																// If voltage drop < OL_THR_MIN, the OLxbit of channel x is set.
    #define OL_THR_MIN__THRESHOLD			 0b0000000000000000	// Threshold (default) 

// TEMP_CONF - 0004H Temperature measurement configuration [supplied in sleep)
extern const subreg_item  TEMP_CONF__EXT_OT_THR; 				// RW - External overtemperature threshold 10-bit overtemperature fault threshold.
																// Overtemperature error is detected and indicated in GEN_DIAG register
																// if the external temperature result is lower than the fault threshold.
																// Balancing is deactivated.
    #define EXT_OT_THR__THRESHOLD			 0b0000000000000000	// Threshold (default) 
extern const subreg_item  TEMP_CONF__I_NTC; 					// RW - Current source used for OT fault
    #define I_NTC__I_0						 0b0000000000000000	// ITMPz_0 used (default) 
    #define I_NTC__I_1						 0b0000010000000000	// ITMPz_1 used 
    #define I_NTC__I_2						 0b0000100000000000	// ITMPz_2 used 
    #define I_NTC__I_3						 0b0000110000000000	// ITMPz_3 used 
extern const subreg_item  TEMP_CONF__NR_TEMP_SENSE; 			// RW - Number of external temperature sensors
    #define NR_TEMP_SENSE__NO_EXT_SENSOR	 0b0000000000000000	// No external TMP sensor (default) 
    #define NR_TEMP_SENSE__TMP0				 0b0001000000000000	// TMP0 active 
    #define NR_TEMP_SENSE__TMP0_1			 0b0010000000000000	// TMP0 + TMP1 active 
    #define NR_TEMP_SENSE__TMP0_2			 0b0011000000000000	// TMP0 + TMP1 + TMP2 active 
    #define NR_TEMP_SENSE__TMP0_3			 0b0100000000000000	// TMP0 + TMP1 + TMP2 + TMP3 active 
    #define NR_TEMP_SENSE__TMP0_4			 0b0101000000000000	// TMP0 + TMP1 +TMP2 + TMP3 + TMP4 active 

// INT_OT_WARN_CONF - 0005H Internal temperature measurement configuration [supplied in sleep)
extern const subreg_item  INT_OT_WARN_CONF__INT_OT_THR; 		// RW - Internal overtemperature threshold 10-bit overtemperature fault threshold.
																// Overtemperature error is detected and indicated in GEN_DIAG register
																// if the internal temperature result is LOWER than the fault threshold.
																// Balancing is deactivated.
    #define INT_OT_THR__THRESHOLD			 0b0000000000000000	// Threshold (default) 

// RR_ERR_CNT - 0008H Round robin ERR counters [supplied in sleep)
extern const subreg_item  RR_ERR_CNT__NR_ERR; 					// RW - Number of errors Number of consecutive detected errors before error is valid
																// and set in GEN_DIAG and individual fault registers.
																// Only used for faults where counter NR_ERR is active (this can be set in register NR_ERR_MASK).
																// Please note: The register CRC errors as well as the internal IC errors do not have an error counter.
    #define NR_ERR__ERR_0					 0b0000000000000000	// 0 
    #define NR_ERR__ERR_1					 0b0000000000000001	// 1 
    #define NR_ERR__ERR_2					 0b0000000000000010	// 2 (default) 
    #define NR_ERR__ERR_3					 0b0000000000000011	// 3 
    #define NR_ERR__ERR_4					 0b0000000000000100	// 4 
    #define NR_ERR__ERR_5					 0b0000000000000101	// 5 
    #define NR_ERR__ERR_6					 0b0000000000000110	// 6 
    #define NR_ERR__ERR_7					 0b0000000000000111	// 7 
extern const subreg_item  RR_ERR_CNT__NR_EXT_TEMP_START; 		// RW - External temperature triggering in round robin
    #define NR_EXT_TEMP_START__EVERY_RR		 0b0000000000000000	// Every RR (default)
    #define NR_EXT_TEMP_START__1RR_1RR_NO_MES 0b0000000000001000// 1 RR measurement, 1 RR no measurement
    #define NR_EXT_TEMP_START__1RR_2RR_NO_MES 0b0000000000010000// 1 RR measurement, 2 RR no measurement
    #define NR_EXT_TEMP_START__1RR_3RR_NO_MES 0b0000000000011000// 1 RR measurement, 3 RR no measurement
    #define NR_EXT_TEMP_START__1RR_4RR_NO_MES 0b0000000000100000// 1 RR measurement, 4 RR no measurement
    #define NR_EXT_TEMP_START__1RR_5RR_NO_MES 0b0000000000101000// 1 RR measurement, 5 RR no measurement
    #define NR_EXT_TEMP_START__1RR_6RR_NO_MES 0b0000000000110000// 1 RR measurement, 6 RR no measurement
    #define NR_EXT_TEMP_START__1RR_7RR_NO_MES 0b0000000000111000// 1 RR measurement, 7 RR no measurement
extern const subreg_item  RR_ERR_CNT__RR_SLEEP_CNT; 			// RW - Round robin timing in sleep mode
    #define RR_SLEEP_CNT__DEACT				 0b0000000000000000	// RR in sleep mode is deactivated (default). 
    #define RR_SLEEP_CNT__RR_SLEEP_1		 0b0000000001000000	// tRR_sleep_LSB 
    #define RR_SLEEP_CNT__RR_SLEEP_1023		 0b1111111111000000	// tRR_sleep_LSB*1023 

// RR_CONFIG - 0009H Round robin configuration [supplied in sleep)
extern const subreg_item  RR_CONFIG__RR_CNT; 					// RW - Round robin counter
    #define RR_CNT__RR_0					 0b0000000000000000	// Round robin starts every tRR_min. 
    #define RR_CNT__RR_36					 0b0000000000100100	// Round robin starts every tRR_min + 36 * tRR_LSB (default).
    #define RR_CNT__RR_127					 0b0000000001111111	// Round robin starts every tRR_max. 
extern const subreg_item  RR_CONFIG__RR_SYNC; 					// RW - Round robin synchronization
    #define RR_SYNC__NO_SYNC				 0b0000000000000000	// No synch with WD_CNT write access (default). 
    #define RR_SYNC__SYNC					 0b0000000010000000	// Synch with WD_CNT write access (RR will be started by write access,
																// if RR is already running the RR will be restarted from beginning again).
extern const subreg_item  RR_CONFIG__M_NR_ERR_ADC_ERR; 			// RW - Mask NR_ERR counter for ADC error
    #define M_NR_ERR_ADC_ERR__DISABLE		 0b0000000000000000	// No masking of NR_ERR. Counter is active (default). 
    #define M_NR_ERR_ADC_ERR__ENABLE		 0b0000000100000000	// NR_ERR counter masked. Fault valid after first detection. 
extern const subreg_item  RR_CONFIG__M_NR_ERR_OL_ERR; 			// RW - Mask NR_ERR counter for open load error
    #define M_NR_ERR_OL_ERR__DISABLE		 0b0000000000000000	// No masking of NR_ERR. Counter is active (default). 
    #define M_NR_ERR_OL_ERR__ENABLE			 0b0000001000000000	// NR_ERR counter masked. Fault valid after first detection. 
extern const subreg_item  RR_CONFIG__M_NR_ERR_EXT_T_ERR; 		// RW - Mask NR_ERR counter for external temperature error
    #define M_NR_ERR_EXT_T_ERR__DISABLE		 0b0000000000000000	// No masking of NR_ERR. Counter is active (default). 
    #define M_NR_ERR_EXT_T_ERR__ENABLE		 0b0000010000000000	// NR_ERR counter masked. Fault valid after first detection. 
extern const subreg_item  RR_CONFIG__M_NR_ERR_INT_OT; 			// RW - Mask NR_ERR counter for internal temperature error
    #define M_NR_ERR_INT_OT__DISABLE		 0b0000000000000000	// No masking of NR_ERR. Counter is active (default). 
    #define M_NR_ERR_INT_OT__ENABLE			 0b0000100000000000	// NR_ERR counter masked. Fault valid after first detection. 
extern const subreg_item  RR_CONFIG__M_NR_ERR_CELL_UV; 			// RW - Mask NR_ERR counter for undervoltage error
    #define M_NR_ERR_CELL_UV__DISABLE		 0b0000000000000000	// No masking of NR_ERR. Counter is active (default). 
    #define M_NR_ERR_CELL_UV__ENABLE		 0b0001000000000000	// NR_ERR counter masked. Fault valid after first detection. 
extern const subreg_item  RR_CONFIG__M_NR_ERR_CELL_OV; 			// RW - Mask NR_ERR counter for overvoltage error
    #define M_NR_ERR_CELL_OV__DISABLE		 0b0000000000000000	// No masking of NR_ERR. Counter is active (default). 
    #define M_NR_ERR_CELL_OV__ENABLE		 0b0010000000000000	// NR_ERR counter masked. Fault valid after first detection. 
extern const subreg_item  RR_CONFIG__M_NR_ERR_BAL_UC; 			// RW - Mmask NR_ERR counter for balancing error undercurrent
    #define M_NR_ERR_BAL_UC__DISABLE		 0b0000000000000000	// No masking of NR_ERR. Counter is active (default). 
    #define M_NR_ERR_BAL_UC__ENABLE			 0b0100000000000000	// NR_ERR counter masked. Fault valid after first detection. 
extern const subreg_item  RR_CONFIG__M_NR_ERR_BAL_OC; 			// RW - Mask NR_ERR counter for balancing error overcurrent
    #define M_NR_ERR_BAL_OC__DISABLE		 0b0000000000000000	// No masking of NR_ERR. Counter is active. 
    #define M_NR_ERR_BAL_OC__ENABLE			 0b1000000000000000	// NR_ERR counter masked. Fault valid after first detection (default).

// FAULT_MASK - 000AH ERR pin / EMM mask [supplied in sleep)
extern const subreg_item  FAULT_MASK__ERR_PIN; 					// RW - Enable Error PIN functionality
    #define ERR_PIN__DISABLE				 0b0000000000000000	// ERR pin deactivated, EMM signal active.
																// Device goes back to the mode as it was before the EMM (default).
    #define ERR_PIN__ENABLE					 0b0000000000100000	// ERR Pin function enabled. Fault indication only via ERR Pin.
																// EMM signal deactivated.
																// If ERR PIN triggered, pin stays high(device is then in normal mode)
																// until watchdog runs out or pin is cleared.
extern const subreg_item  FAULT_MASK__M_ADC_ERR; 				// RW - EMM/ERR mask for ADC error
    #define M_ADC_ERR__DISABLE				 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_ADC_ERR__ENABLE				 0b0000000001000000	// ERR/EMM will be set for this type of error. 
extern const subreg_item  FAULT_MASK__M_OL_ERR; 				// RW - EMM/ERR mask for open load error
    #define M_OL_ERR__DISABLE				 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_OL_ERR__ENABLE				 0b0000000010000000	// ERR/EMM will be set for this type of error. 
extern const subreg_item  FAULT_MASK__M_INT_IC_ERR; 			// RW - EMM/ERR mask for internal IC error
    #define M_INT_IC_ERR__DISABLE			 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_INT_IC_ERR__ENABLE			 0b0000000100000000	// ERR/EMM will be set for this type of error. 
extern const subreg_item  FAULT_MASK__M_REG_CRC_ERR; 			// RW - EMM/ERR mask for register CRC error
    #define M_REG_CRC_ERR__DISABLE			 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_REG_CRC_ERR__ENABLE			 0b0000001000000000	// ERR/EMM will be set for this type of error. 
extern const subreg_item  FAULT_MASK__M_EXT_T_ERR; 				// RW - EMM/ERR mask for external temperature error
    #define M_EXT_T_ERR__DISABLE			 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_EXT_T_ERR__ENABLE				 0b0000010000000000	// ERR/EMM will be set for this type of error. 
extern const subreg_item  FAULT_MASK__M_INT_OT; 				// RW - EMM/ERR mask for internal temperature error
    #define M_INT_OT__DISABLE				 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_INT_OT__ENABLE				 0b0000100000000000	// ERR/EMM will be set for this type of error. 
extern const subreg_item  FAULT_MASK__M_CELL_UV; 				// RW - EMM/ERR mask for cell undervoltage error
    #define M_CELL_UV__DISABLE				 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_CELL_UV__ENABLE				 0b0001000000000000	// ERR/EMM will be set for this type of error. 
extern const subreg_item  FAULT_MASK__M_CELL_OV; 				// RW - EMM/ERR mask for cell overvoltage error
    #define M_CELL_OV__DISABLE				 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_CELL_OV__ENABLE				 0b0010000000000000	// ERR/EMM will be set for this type of error. 
extern const subreg_item  FAULT_MASK__M_BAL_ERR_UC; 			// RW - EMM/ERR mask for balancing error undercurrent
    #define M_BAL_ERR_UC__DISABLE			 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_BAL_ERR_UC__ENABLE			 0b0100000000000000	// ERR/EMM will be set for this type of error. 
extern const subreg_item  FAULT_MASK__M_BAL_ERR_OC; 			// RW - EMM/ERR mask for balancing error overcurrent
    #define M_BAL_ERR_OC__DISABLE			 0b0000000000000000	// ERR/EMM will NOT be set if this type of error occurs (default).
    #define M_BAL_ERR_OC__ENABLE			 0b1000000000000000	// ERR/EMM will be set for this type of error. 

// GEN_DIAG - 000BH General diagnostics [supplied in sleep)
extern const subreg_item  GEN_DIAG__UART_WAKEUP; 				// RH - Wake-up via UART
    #define UART_WAKEUP__ISOUART			 0b0000000000000000	// Wake-up via iso UART (default) 
    #define UART_WAKEUP__UART				 0b0000000000000001	// Wake-up via UART (GPIO) 
extern const subreg_item  GEN_DIAG__MOT_MOB_N; 					// RH - Primary on Top/Bottom configuration
    #define MOT_MOB_N__BOTTOM				 0b0000000000000000	// Configured as primary on bottom (default) 
    #define MOT_MOB_N__TOP					 0b0000000000000010	// Configured as primary on top 
extern const subreg_item  GEN_DIAG__BAL_ACTIVE; 				// RH - Balancing active
    #define BAL_ACTIVE__OFF					 0b0000000000000000	// No balancing ongoing (default) 
    #define BAL_ACTIVE__ON					 0b0000000000000100	// Balancing ongoing, at least one channel is on 
extern const subreg_item  GEN_DIAG__LOCK_MEAS; 					// RH - Lock measurement
																// This bit indicates an ongoing PCVM, SCVM, BVM or AVM measurement
																// or a delayed RR.
    #define LOCK_MEAS__MEAS_OFF				 0b0000000000000000	// No measurement ongoing (default) 
    #define LOCK_MEAS__MEAS_ON				 0b0000000000001000	// PCVM, SCVM, BVM or AVM measurement ongoing 
extern const subreg_item  GEN_DIAG__RR_ACTIVE; 					// RH - Round robin active
																// This bit indicates if the round robin was active during read.
    #define RR_ACTIVE__OFF					 0b0000000000000000	// No round robin active (default) 
    #define RR_ACTIVE__ON					 0b0000000000010000	// Round robin active 
extern const subreg_item  GEN_DIAG__PS_ERR_SLEEP; 				// ROCW - Power supply error induced sleep
    #define PS_ERR_SLEEP__NO_ERR			 0b0000000000000000	// No power supply error induced sleep (default) 
    #define PS_ERR_SLEEP__ERR_OCCURED		 0b0000000000100000	// Power supply error induced sleep occured 
extern const subreg_item  GEN_DIAG__ADC_ERR; 					// ROCW - ADC Error
    #define ADC_ERR__NO_ERR					 0b0000000000000000	// No ADC mismatch between sum of PCVM and BVM (default)
    #define ADC_ERR__ERR_OCCURED			 0b0000000001000000	// ERROR of ADC-result comparison. Balancing deactivated.
extern const subreg_item  GEN_DIAG__OL_ERR; 					// ROCWL - Open load error
																// Please note: Resetting the error here resets also the respective detailed error register.
    #define OL_ERR__NO_ERR					 0b0000000000000000	// No open load error (default) 
    #define OL_ERR__ERR_OCCURED				 0b0000000010000000	// Open load error occurred.
																// Detailed information in the respective error register. Balancing deactivated.
extern const subreg_item  GEN_DIAG__INT_IC_ERR; 				// ROCW - Internal IC error
    #define INT_IC_ERR__NO_ERR				 0b0000000000000000	// No internal error (default) 
    #define INT_IC_ERR__ERR_OCCURED			 0b0000000100000000	// Internal IC check error occurred. Balancing is deactivated.
extern const subreg_item  GEN_DIAG__REG_CRC_ERR; 				// ROCWL - Register CRC error
																// Please note: Resetting the error here resets also the respective detailed error register.
    #define REG_CRC_ERR__NO_ERR				 0b0000000000000000	// No CRC check error (default) 
    #define REG_CRC_ERR__ERR_OCCURED		 0b0000001000000000	// CRC check error occurred.
																// Detailed information in the respective error register. Balancing deactivated.
extern const subreg_item  GEN_DIAG__EXT_T_ERR; 					// ROCWL - External temperature error
																// Please note: Resetting the error here resets also the respective detailed error register.
    #define EXT_T_ERR__NO_ERR				 0b0000000000000000	// No external temperature error (default) 
    #define EXT_T_ERR__ERR_OCCURED			 0b0000010000000000	// External temperature error occurred.
																// Detailed information in the respective error register. Balancing is deactivated.
extern const subreg_item  GEN_DIAG__INT_OT; 					// ROCW - Internal temperature (OT) error
    #define INT_OT__NO_ERR					 0b0000000000000000	// No internal OT error (default) 
    #define INT_OT__ERR_OCCURED				 0b0000100000000000	// Internal OT error occured. Balancing is deactivated.
extern const subreg_item  GEN_DIAG__CELL_UV; 					// ROCWL - Cell undervoltage (UV) error
																// Please note: Resetting the error here resets also the respective detailed error register.
    #define CELL_UV__NO_ERR					 0b0000000000000000	// No UV error (default) 
    #define CELL_UV__ERR_OCCURED			 0b0001000000000000	// UV error occured. Detailed information in the respective error register.
extern const subreg_item  GEN_DIAG__CELL_OV; 					// ROCWL - Cell overvoltage (OV) error
																// Please note: Resetting the error here resets also the respective detailed error register.
    #define CELL_OV__NO_ERR					 0b0000000000000000	// No OV error (default) 
    #define CELL_OV__ERR_OCCURED			 0b0010000000000000	// OV error occured. Detailed information in the respective error register.
extern const subreg_item  GEN_DIAG__BAL_ERR_UC; 				// ROCWL - Balancing error undercurrent (will be reset in sleep mode)
																// Please note: Resetting the error here resets also the respective detailed error register.
    #define BAL_ERR_UC__NO_ERR				 0b0000000000000000	// No balancing error (default) 
    #define BAL_ERR_UC__ERR_OCCURED			 0b0100000000000000	// Balancing error occured. Detailed information in the respective error register.
extern const subreg_item  GEN_DIAG__BAL_ERR_OC; 				// ROCWL - Balancing error overcurrent (will be reset in sleep mode)
																// Please Note: Resetting the error here resets also the respective detailed error register.
    #define BAL_ERR_OC__NO_ERR				 0b0000000000000000	// No balancing error (default) 
    #define BAL_ERR_OC__ERR_OCCURED			 0b1000000000000000	// Balancing error occured. Detailed information in the respective error register.

// CELL_UV - 000CH Cell voltage supervision warning flags UV [supplied in sleep)
extern const subreg_item  CELL_UV__UV_i; 						// ROCW - Undervoltage in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define UV_i__NO_UV						 0b0000000000000000	// No undervoltage detected in respective cell (default) 
    #define UV_i__UV						 0b0000100000000000	// Undervoltage detected in respective cell. Balancing is deactivated.

// CELL_OV - 000DH Cell voltage supervision warning flags OV [supplied in sleep)
extern const subreg_item  CELL_OV__OV_i; 						// ROCW - Overvoltage in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OV_i__NO_OV						 0b0000000000000000	// No overvoltage detected in respective cell (default) 
    #define OV_i__OV						 0b0000100000000000	// Overvoltage detected in respective cell. Balancing is deactivated.

// EXT_TEMP_DIAG - 000EH External overtemperature warning flags [supplied in sleep)
extern const subreg_item  EXT_TEMP_DIAG__OPEN0; 				// ROCW - Open load in external temp 0 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OPEN0__NO_OL					 0b0000000000000000	// No open load in external temp 0 (default) 
    #define OPEN0__OL_OCCURED				 0b0000000000000001	// Open load in external temp 0 
extern const subreg_item  EXT_TEMP_DIAG__SHORT0; 				// ROCW - Short in external temp 0 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define SHORT0__NO_SHORT				 0b0000000000000000	// No short in external temp 0 (default) 
    #define SHORT0__SHORT_OCCURED			 0b0000000000000010	// Short in external temp 0 
extern const subreg_item  EXT_TEMP_DIAG__OT0; 					// ROCW - Overtemperature in external temp 0 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OT0__NO_OT						 0b0000000000000000	// No overtemperature in external temp 0 (default) 
    #define OT0__OT_OCCURED					 0b0000000000000100	// ADC conversion of external temp 0 measurement < overtemperature threshold EXT_OT_THR
extern const subreg_item  EXT_TEMP_DIAG__OPEN1; 				// ROCW - Open load in external temp 1 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OPEN1__NO_OL					 0b0000000000000000	// No open load in external temp 1 (default) 
    #define OPEN1__OL_OCCURED				 0b0000000000001000	// Open load in external temp 1 
extern const subreg_item  EXT_TEMP_DIAG__SHORT1; 				// ROCW - Short in external temp 1 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define SHORT1__NO_SHORT				 0b0000000000000000	// No short in external temp 1 (default) 
    #define SHORT1__SHORT_OCCURED			 0b0000000000010000	// Short in external temp 1 
extern const subreg_item  EXT_TEMP_DIAG__OT1; 					// ROCW - Overtemperature in external temp 1 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OT1__NO_OT						 0b0000000000000000	// No overtemperature in external temp 1 (default) 
    #define OT1__OT_OCCURED					 0b0000000000100000	// ADC conversion of external temp 1 measurement < overtemperature threshold EXT_OT_THR
extern const subreg_item  EXT_TEMP_DIAG__OPEN2; 				// ROCW - Open load in external temp 2 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OPEN2__NO_OL					 0b0000000000000000	// No open load in external temp 2 (default) 
    #define OPEN2__OL_OCCURED				 0b0000000001000000	// Open load in external temp 2 
extern const subreg_item  EXT_TEMP_DIAG__SHORT2; 				// ROCW - Short in external temp 2 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define SHORT2__NO_SHORT				 0b0000000000000000	// No short in external temp 2 (default) 
    #define SHORT2__SHORT_OCCURED			 0b0000000010000000	// Short in external temp 2 
extern const subreg_item  EXT_TEMP_DIAG__OT2; 					// ROCW - Overtemperature in external temp 2 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OT2__NO_OT						 0b0000000000000000	// No overtemperature in external temp 2 (default) 
    #define OT2__OT_OCCURED					 0b0000000100000000	// ADC conversion of external temp 2 measurement < overtemperature threshold EXT_OT_THR
extern const subreg_item  EXT_TEMP_DIAG__OPEN3; 				// ROCW - Open load in external temp 3 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OPEN3__NO_OL					 0b0000000000000000	// No open load in external temp 3 (default) 
    #define OPEN3__OL_OCCURED				 0b0000001000000000	// Open load in external temp 3 
extern const subreg_item  EXT_TEMP_DIAG__SHORT3; 				// ROCW - Short in external temp 3 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define SHORT3__NO_SHORT				 0b0000000000000000	// No short in external temp 3 (default) 
    #define SHORT3__SHORT_OCCURED			 0b0000010000000000	// Short in external temp 3 
extern const subreg_item  EXT_TEMP_DIAG__OT3; 					// ROCW - Overtemperature in external temp 3 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OT3__NO_OT						 0b0000000000000000	// No overtemperature in external temp 3 (default) 
    #define OT3__OT_OCCURED					 0b0000100000000000	// ADC conversion of external temp 3 measurement < overtemperature threshold EXT_OT_THR
extern const subreg_item  EXT_TEMP_DIAG__OPEN4; 				// ROCW - Open load in external temp 4 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OPEN4__NO_OL					 0b0000000000000000	// No open load in external temp 4 (default) 
    #define OPEN4__OL_OCCURED				 0b0001000000000000	// Open load in external temp 4 
extern const subreg_item  EXT_TEMP_DIAG__SHORT4; 				// ROCW - Short in external temp 4 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define SHORT4__NO_SHORT				 0b0000000000000000	// No short in external temp 4 (default) 
    #define SHORT4__SHORT_OCCURED			 0b0010000000000000	// Short in external temp 4 
extern const subreg_item  EXT_TEMP_DIAG__OT4; 					// ROCW - Overtemperature in external temp 4 Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OT4__NO_OT						 0b0000000000000000	// No overtemperature in external temp 4 (default) 
    #define OT4__OT_OCCURED					 0b0100000000000000	// ADC conversion of external temp 4 measurement < overtemperature threshold EXT_OT_THR

// DIAG_OL - 0010H Diagnostics open load [supplied in sleep)
extern const subreg_item  DIAG_OL__OL_i; 						// ROCW - Open load channel i Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OL_i__NO_OL						 0b0000000000000000	// No open load detected for respective channel (default) 
    #define OL_i__OL						 0b0000100000000000	// Open load detected for respective channel 

// REG_CRC_ERR - 0011H REG_CRC_ERR [supplied in sleep)
extern const subreg_item  REG_CRC_ERR__ADDR; 					// ROCW - Register address of register where CRC check failed Register address of CRC check fail,
																// can also be cleared on write '0' toconnected GEN_DIAG register bit.
    #define ADDR__DEFAULT					 0b0000000000000000	// Register address 

// CELL_UV_DAC_COMP - 0012H Cell voltage supervision warning flags UV [supplied in sleep)
extern const subreg_item  CELL_UV_DAC_COMP__UV_i; 				// ROCW - Undervoltage in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define UV_i__NO_UV						 0b0000000000000000	// No undervoltage detected in respective cell (default) 
    #define UV_i__UV						 0b0000100000000000	// Uundervoltage detected in respective cell 

// CELL_OV_DAC_COMP - 0013H Cell voltage supervision warning flags OV [supplied in sleep)
extern const subreg_item  CELL_OV_DAC_COMP__OV_i; 				// ROCW - Overvoltage in cell i Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OV_i__NO_OV						 0b0000000000000000	// No overvoltage detected in respective cell (default) 
    #define OV_i__OV						 0b0000100000000000	// Overvoltage detected in respective cell 

// OP_MODE - 0014H Operation mode
extern const subreg_item  OP_MODE__PD; 							// RW - Activate sleep mode
    #define PD__NORMAL_OP					 0b0000000000000000	// Chip operating in normal mode (default) 
    #define PD__PWD_OP						 0b0000000000000001	// Chip is set to sleep mode. No further communication possible after bit is set to '1'.
extern const subreg_item  OP_MODE__EXT_WD; 						// RW - Extended watchdog
    #define EXT_WD__NOT_ACTIVE				 0b0000000000000000	// No extended watchdog (default) 
    #define EXT_WD__ACTIVE					 0b0000000000000010	// Extended watchdog active 
extern const subreg_item  OP_MODE__SLEEP_REG_RESET; 			// RWH - Sleep mode registers reset
    #define SLEEP_REG_RESET__NORMAL_OP		 0b0000000000000000	// Chip operating in normal mode (default) 
    #define SLEEP_REG_RESET__SLEEP_REG_RESET_OP 0b0000000000000100// Resets all registers that are supplied in sleep mode.
																  // No further communication possible after bit is set to '1'.
extern const subreg_item  OP_MODE__I_DIAG_EN; 					// RW - Force OL diagnostics currents
    #define I_DIAG_EN__OFF					 0b0000000000000000	// Diagnostics currents switched via round robin (default) 
    #define I_DIAG_EN__ON					 0b0000000000001000	// Diagnostics currents enabled for all channels 
extern const subreg_item  OP_MODE__LR_TIME; 					// RW - Long-running mode measurement restart time
    #define LR_TIME__MIN					 0b0000000000000000	// Minimum (1.17ms) 
    #define LR_TIME__DEFAULT				 0b1100010000000000	// 6.25 ms 

// BAL_CURR_THR - 0015H Balancing current thresholds
extern const subreg_item  BAL_CURR_THR__OC_THR; 				// RW - Overcurrent fault threshold 8-bit
																//to define the maximum voltage drop during balancing diagnostics.
																// If the voltage drop (I Bal*R F) > OC_THR the overcurrentis detected.
    #define OC_THR__DEFAULT					 0b0000000010101100	// Default 
extern const subreg_item  BAL_CURR_THR__UC_THR; 				// RW - Undercurrent fault threshold 8-bit
																// to define the minimum voltage drop during balancing diagnostics.
																// If the voltage drop (I Bal*R F) < UC_THR the undercurrent is detected.
    #define UC_THR__DEFAULT					 0b0000000000000000	// Default 

// BAL_SETTINGS - 0016H Balance settingsONi (i=0-11) i rwh Switching state of balancing switch i
    #define BAL_SETTINGS__OFF				 0b0000000000000000	// Respective balancing switch off (default)

// AVM_CONFIG - 0017H Auxiliary voltage measurement configuration
extern const subreg_item  AVM_CONFIG__TEMP_MUX_DIAG_SEL; 		// RW - Selector for external temp diagnose
    #define TEMP_MUX_DIAG_SEL__PD_EXT_TEMP_0 0b0000000000000000	// Pulldown for external temp 0 measurement is active.
    #define TEMP_MUX_DIAG_SEL__PD_EXT_TEMP_1 0b0000000000000001	// Pulldown for external temp 1 measurement is active.
    #define TEMP_MUX_DIAG_SEL__PD_EXT_TEMP_2 0b0000000000000010	// Pulldown for external temp 2 measurement is active.
    #define TEMP_MUX_DIAG_SEL__PD_EXT_TEMP_3 0b0000000000000011	// Pulldown for external temp 3 measurement is active.
    #define TEMP_MUX_DIAG_SEL__PD_EXT_TEMP_4 0b0000000000000100	// Pulldown for external temp 4 measurement is active.
    #define TEMP_MUX_DIAG_SEL__NO_PD		 0b0000000000000111	// No pulldown is active (default), 
extern const subreg_item  AVM_CONFIG__AVM_TMP0_MASK; 			// RW - Activate auxiliary measurement via deactive TMP0 as part of AVM
    #define AVM_TMP0_MASK__MASKED			 0b0000000000000000	// Manual AVM TMP0 measurement masked out when AVM_START bit triggered (default).
    #define AVM_TMP0_MASK__PERFORMED		 0b0000000000001000	// Manual AVM TMP0 measurement performed when AVM_START bit triggered.
extern const subreg_item  AVM_CONFIG__AVM_TMP1_MASK; 			// RW - Activate auxiliary measurement via deactive TMP1 as part of AVM
    #define AVM_TMP1_MASK__MASKED			 0b0000000000000000	// Manual AVM TMP1 measurement masked out when AVM_START bit triggered (default).
    #define AVM_TMP1_MASK__PERFORMED		 0b0000000000010000	// Manual AVM TMP1 measurement performed when AVM_START bit triggered.
extern const subreg_item  AVM_CONFIG__AVM_TMP2_MASK; 			// RW - Activate auxiliary measurement via deactive TMP2 as part of AVM
    #define AVM_TMP2_MASK__MASKED			 0b0000000000000000	// Manual AVM TMP2 measurement masked out when AVM_START bit triggered (default).
    #define AVM_TMP2_MASK__PERFORMED		 0b0000000000100000	// Manual AVM TMP2 measurement performed when AVM_START bit triggered.
extern const subreg_item  AVM_CONFIG__AVM_TMP3_MASK; 			// RW - Activate auxiliary measurement via deactive TMP3 as part of AVM
    #define AVM_TMP3_MASK__MASKED			 0b0000000000000000	// Manual AVM TMP3 measurement masked out when AVM_START bit triggered (default).
    #define AVM_TMP3_MASK__PERFORMED		 0b0000000001000000	// Manual AVM TMP3 measurement performed when AVM_START bit triggered.
extern const subreg_item  AVM_CONFIG__AVM_TMP4_MASK; 			// RW - Activate auxiliary measurement via deactive TMP4 as part of AVM
    #define AVM_TMP4_MASK__MASKED			 0b0000000000000000	// Manual AVM TMP4 measurement masked out when AVM_START bit triggered (default).
    #define AVM_TMP4_MASK__PERFORMED		 0b0000000010000000	// Manual AVM TMP4 measurement performed when AVM_START bit triggered.
extern const subreg_item  AVM_CONFIG__AUX_BIPOLAR; 				// RWH - Bipolar AUX measurement instead of BVM
    #define AUX_BIPOLAR__BVM				 0b0000000000000000	// Normal BVM measurement 
    #define AUX_BIPOLAR__AUX				 0b0000000100000000	// Bipolar AUX measurement instead of BVM
																// (Bit is reset when NR_TEMP_SENSE > 3, this can cause a REG_CRC_ERR).
extern const subreg_item  AVM_CONFIG__R_DIAG; 					// RW - Masking diagnostics resistor as part of AVM
    #define R_DIAG__MASKED					 0b0000000000000000	// Manual AVM diagnostics resistor measurement masked out when AVM_START bit triggered (default)
    #define R_DIAG__PERFORMED				 0b0000001000000000	// Manual AVM diagnostics resistor measurement performed when AVM_START bit triggered
extern const subreg_item  AVM_CONFIG__R_DIAG_SEL_0; 			// RW - R_DIAG current source 0
    #define R_DIAG_SEL_0__I_0				 0b0000000000000000	// Source ITMP0_0: 320uA (default) 
    #define R_DIAG_SEL_0__I_1				 0b0000100000000000	// Source ITMP0_1: 80uA 
    #define R_DIAG_SEL_0__I_2				 0b0001000000000000	// Source ITMP0_2: 20uA 
    #define R_DIAG_SEL_0__I_3				 0b0001100000000000	// Source ITMP0_3: 5uA 
extern const subreg_item  AVM_CONFIG__R_DIAG_SEL_1; 			// RW - R_DIAG current source 1
    #define R_DIAG_SEL_1__I_0				 0b0000000000000000	// Source ITMP1_0: 320uA (default) 
    #define R_DIAG_SEL_1__I_1				 0b0010000000000000	// Source ITMP1_1: 80uA 
    #define R_DIAG_SEL_1__I_2				 0b0100000000000000	// Source ITMP1_2: 20uA 
    #define R_DIAG_SEL_1__I_3				 0b0110000000000000	// Source ITMP1_3: 5uA 
extern const subreg_item  AVM_CONFIG__R_DIAG_CUR_SRC; 			// RW - R_DIAG current source selection
    #define R_DIAG_CUR_SRC__SRC_0			 0b0000000000000000	// Current source 0 (default) 
    #define R_DIAG_CUR_SRC__SRC_1			 0b1000000000000000	// Current source 1 

// MEAS_CTRL - 0018H Measurement control
extern const subreg_item  MEAS_CTRL__CVM_DEL; 					// RW - Wait time before CVM and/or BVM is started when PBOFF=1
    #define CVM_DEL__NO_SETTLING_TIME		 0b0000000000000000	// No settling time 
    #define CVM_DEL__1						 0b0000000000000001	// tVM_del_LSB (default) 
    #define CVM_DEL__31						 0b0000000000011111	// 31 x tVM_del_LSB 
extern const subreg_item  MEAS_CTRL__PBOFF; 					// RW - Enable PBOFF
    #define PBOFF__BAL_CONTINUE				 0b0000000000000000	// Keep balancing state for PCVM/SCVM/BVM, no CVM_DEL. 
    #define PBOFF__BAL_INTERRUPT			 0b0000000000100000	// Switch off balancing before conversion starts (default).
extern const subreg_item  MEAS_CTRL__SCVM_START; 				// RWH - Start secondary cell voltage measurement Bit cleared if conversion done
    #define SCVM_START__NO_MEAS				 0b0000000000000000	// No measurement ongoing (default) 
    #define SCVM_START__TRIG_MEAS			 0b0000000001000000	// Trigger measurement 
extern const subreg_item  MEAS_CTRL__AVM_START; 				// RWH - Start auxiliary voltage measurement Bit cleared if conversion done
    #define AVM_START__NO_MEAS				 0b0000000000000000	// No measurement ongoing (default) 
    #define AVM_START__TRIG_MEAS			 0b0000000010000000	// Trigger measurement if BVM_START=0 
extern const subreg_item  MEAS_CTRL__BVM_MODE; 					// RW - Block voltage measurement mode
    #define BVM_MODE__10_BIT				 0b0000000000000000	// 10 bit (default) 
    #define BVM_MODE__11_BIT				 0b0000000100000000	// 11 bit 
    #define BVM_MODE__16_BIT				 0b0000011000000000	// 16 bit 
    #define BVM_MODE__LONG					 0b0000011100000000	// Long-running mode 
extern const subreg_item  MEAS_CTRL__BVM_START; 				// RWH - Start block voltage measurement Bit cleared if conversion done
    #define BVM_START__NO_MEAS				 0b0000000000000000	// No measurement ongoing (default) 
    #define BVM_START__TRIG_MEAS			 0b0000100000000000	// Trigger measurement 
extern const subreg_item  MEAS_CTRL__CVM_MODE; 					// RW - Cell voltage measurement mode
    #define CVM_MODE__10_BIT				 0b0000000000000000	// 10 bit (default) 
    #define CVM_MODE__11_BIT				 0b0001000000000000	// 11 bit 
    #define CVM_MODE__16_BIT				 0b0110000000000000	// 16 bit 
    #define CVM_MODE__LONG					 0b0111000000000000	// Long-running mode 
extern const subreg_item  MEAS_CTRL__PCVM_START; 				// RWH - Start primary cell voltage measurement Bit cleared if conversion done
    #define PCVM_START__NO_MEAS				 0b0000000000000000	// No measurement ongoing (default) 
    #define PCVM_START__TRIG_MEAS			 0b1000000000000000	// Trigger measurement 

// PCVM_0 - 0019H Primary cell voltage measurement i
extern const subreg_item  PCVM_0__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_1 - 0020H Primary cell voltage measurement i
extern const subreg_item  PCVM_1__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_2 - 0021H Primary cell voltage measurement i
extern const subreg_item  PCVM_2__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_3 - 0022H Primary cell voltage measurement i
extern const subreg_item  PCVM_3__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_4 - 0023H Primary cell voltage measurement i
extern const subreg_item  PCVM_4__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_5 - 0024H Primary cell voltage measurement i
extern const subreg_item  PCVM_5__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_6 - 0025H Primary cell voltage measurement i
extern const subreg_item  PCVM_6__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_7 - 0026H Primary cell voltage measurement i
extern const subreg_item  PCVM_7__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_8 - 0027H Primary cell voltage measurement i
extern const subreg_item  PCVM_8__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_9 - 0028H Primary cell voltage measurement i
extern const subreg_item  PCVM_9__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_10 - 0029H Primary cell voltage measurement i
extern const subreg_item  PCVM_10__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// PCVM_11 - 0030H Primary cell voltage measurement i
extern const subreg_item  PCVM_11__RESULT; 						// RH - Result of cell voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// SCVM_HIGH - 0025H SCVM highest cell voltage
extern const subreg_item  SCVM_HIGH__UPD_CNT; 					// RH - Update counter
extern const subreg_item  SCVM_HIGH__RESULT; 					// RH - Result of SCVM measurement (highest cell voltage)

// SCVM_LOW - 0026H SCVM lowest cell voltage
extern const subreg_item  SCVM_LOW__UPD_CNT; 					// RH - Update counter
extern const subreg_item  SCVM_LOW__RESULT; 					// RH - Result of SCVM measurement (lowest cell voltage)

// STRESS_PCVM - 0027H Stress correction PCVM
extern const subreg_item  STRESS_PCVM__STRESS_CORRECTION; 		// RH - Stress correction value PCVM
    #define STRESS_CORRECTION__DEFAULT		 0b0000000000000000	// Default 

// BVM - 0028H Block voltage measurement
extern const subreg_item  BVM__RESULT; 							// RH - Result of block voltage measurement
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 

// EXT_TEMP_0 - 0029H Temp result 0
extern const subreg_item  EXT_TEMP_0__RESULT; 					// RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 
extern const subreg_item  EXT_TEMP_0__INTC; 					// RH - Indicates which current source was used Number of current source that was active for latest measurement.
    #define INTC__I_0						 0b0000000000000000	// Source ITMPz_0 used (default) 
    #define INTC__I_1						 0b0000010000000000	// Source ITMPz_1 used 
    #define INTC__I_2						 0b0000100000000000	// Source ITMPz_2 used 
    #define INTC__I_3						 0b0000110000000000	// Source ITMPz_3 used 
extern const subreg_item  EXT_TEMP_0__PULLDOWN; 				// RH - Indicating pull-down switch state
    #define PULLDOWN__NORMAL_MEAS			 0b0000000000000000	// Normal measurement done (default) 
    #define PULLDOWN__PULL_DOWN				 0b0001000000000000	// Pull-down for Mux-test was active during conversion.
extern const subreg_item  EXT_TEMP_0__VALID; 					// ROCR - Indicating a valid result
    #define VALID__NO_NEW_RESULT			 0b0000000000000000	// No new result available (default) 
    #define VALID__NEW_RESULT_STORED		 0b0010000000000000	// A new result is stored in the register, cleared automatically after readout of the result register.
extern const subreg_item  EXT_TEMP_0__PD_ERR; 					// ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
    #define PD_ERR__NO_ERR					 0b0000000000000000	// No pulldown error 
    #define PD_ERR__ERR						 0b0100000000000000	// Pulldown error 

// EXT_TEMP_1 - 002AH Temp result 1
extern const subreg_item  EXT_TEMP_1__RESULT; 					// RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 
extern const subreg_item  EXT_TEMP_1__INTC; 					// RH - Indicates which current source was used Number of current source that was active for latest measurement.
    #define INTC__I_0						 0b0000000000000000	// Source ITMPz_0 used (default) 
    #define INTC__I_1						 0b0000010000000000	// Source ITMPz_1 used 
    #define INTC__I_2						 0b0000100000000000	// Source ITMPz_2 used 
    #define INTC__I_3						 0b0000110000000000	// Source ITMPz_3 used 
extern const subreg_item  EXT_TEMP_1__PULLDOWN; 				// RH - Indicating pull-down switch state
    #define PULLDOWN__NORMAL_MEAS			 0b0000000000000000	// Normal measurement done (default) 
    #define PULLDOWN__PULL_DOWN				 0b0001000000000000	// Pull-down for Mux-test was active during conversion.
extern const subreg_item  EXT_TEMP_1__VALID; 					// ROCR - Indicating a valid result
    #define VALID__NO_NEW_RESULT			 0b0000000000000000	// No new result available (default) 
    #define VALID__NEW_RESULT_STORED		 0b0010000000000000	// A new result is stored in the register, cleared automatically after readout of the result register.
extern const subreg_item  EXT_TEMP_1__PD_ERR; 					// ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
    #define PD_ERR__NO_ERR					 0b0000000000000000	// No pulldown error 
    #define PD_ERR__ERR						 0b0100000000000000	// Pulldown error 

// EXT_TEMP_2 - 002BH Temp result 2
extern const subreg_item  EXT_TEMP_2__RESULT; 					// RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 
extern const subreg_item  EXT_TEMP_2__INTC; 					// RH - Indicates which current source was used Number of current source that was active for latest measurement.
    #define INTC__I_0						 0b0000000000000000	// Source ITMPz_0 used (default) 
    #define INTC__I_1						 0b0000010000000000	// Source ITMPz_1 used 
    #define INTC__I_2						 0b0000100000000000	// Source ITMPz_2 used 
    #define INTC__I_3						 0b0000110000000000	// Source ITMPz_3 used 
extern const subreg_item  EXT_TEMP_2__PULLDOWN; 				// RH - Indicating pull-down switch state
    #define PULLDOWN__NORMAL_MEAS			 0b0000000000000000	// Normal measurement done (default) 
    #define PULLDOWN__PULL_DOWN				 0b0001000000000000	// Pull-down for Mux-test was active during conversion.
extern const subreg_item  EXT_TEMP_2__VALID; 					// ROCR - Indicating a valid result
    #define VALID__NO_NEW_RESULT			 0b0000000000000000	// No new result available (default) 
    #define VALID__NEW_RESULT_STORED		 0b0010000000000000	// A new result is stored in the register, cleared automatically after readout of the result register.
extern const subreg_item  EXT_TEMP_2__PD_ERR; 					// ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
    #define PD_ERR__NO_ERR					 0b0000000000000000	// No pulldown error 
    #define PD_ERR__ERR						 0b0100000000000000	// Pulldown error 

// EXT_TEMP_3 - 002CH Temp result 3
extern const subreg_item  EXT_TEMP_3__RESULT; 					// RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 
extern const subreg_item  EXT_TEMP_3__INTC; 					// RH - Indicates which current source was used
																// Number of current source that was active for latest measurement.
    #define INTC__I_0						 0b0000000000000000	// Source ITMPz_0 used (default) 
    #define INTC__I_1						 0b0000010000000000	// Source ITMPz_1 used 
    #define INTC__I_2						 0b0000100000000000	// Source ITMPz_2 used 
    #define INTC__I_3						 0b0000110000000000	// Source ITMPz_3 used 
extern const subreg_item  EXT_TEMP_3__PULLDOWN; 				// RH - Indicating pull-down switch state
    #define PULLDOWN__NORMAL_MEAS			 0b0000000000000000	// Normal measurement done (default) 
    #define PULLDOWN__PULL_DOWN				 0b0001000000000000	// Pull-down for Mux-test was active during conversion.
extern const subreg_item  EXT_TEMP_3__VALID; 					// ROCR - Indicating a valid result
    #define VALID__NO_NEW_RESULT			 0b0000000000000000	// No new result available (default) 
    #define VALID__NEW_RESULT_STORED		 0b0010000000000000	// A new result is stored in the register, cleared automatically after readout of the result register.
extern const subreg_item  EXT_TEMP_3__PD_ERR; 					// ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
    #define PD_ERR__NO_ERR					 0b0000000000000000	// No pulldown error 
    #define PD_ERR__ERR						 0b0100000000000000	// Pulldown error 

// EXT_TEMP_4 - 002DH Temp result 4
extern const subreg_item  EXT_TEMP_4__RESULT; 					// RH - Result of external temp measurement SD-ADC result of resistance or voltage measurement.
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 
extern const subreg_item  EXT_TEMP_4__INTC; 					// RH - Indicates which current source was used
																// Number of current source that was active for latest measurement.
    #define INTC__I_0						 0b0000000000000000	// Source ITMPz_0 used (default) 
    #define INTC__I_1						 0b0000010000000000	// Source ITMPz_1 used 
    #define INTC__I_2						 0b0000100000000000	// Source ITMPz_2 used 
    #define INTC__I_3						 0b0000110000000000	// Source ITMPz_3 used 
extern const subreg_item  EXT_TEMP_4__PULLDOWN; 				// RH - Indicating pull-down switch state
    #define PULLDOWN__NORMAL_MEAS			 0b0000000000000000	// Normal measurement done (default) 
    #define PULLDOWN__PULL_DOWN				 0b0001000000000000	// Pull-down for Mux-test was active during conversion.
extern const subreg_item  EXT_TEMP_4__VALID; 					// ROCR - Indicating a valid result
    #define VALID__NO_NEW_RESULT			 0b0000000000000000	// No new result available (default) 
    #define VALID__NEW_RESULT_STORED		 0b0010000000000000	// A new result is stored in the register, cleared automatically after readout of the result register2.
extern const subreg_item  EXT_TEMP_4__PD_ERR; 					// ROCW - Pull-down error Can also be cleared on write '0' to GEN_DIAG.EXT_T_ERR register bit.
    #define PD_ERR__NO_ERR					 0b0000000000000000	// No pulldown error 
    #define PD_ERR__ERR						 0b0100000000000000	// Pulldown error 

// EXT_TEMP_R_DIAG - 002FH Temp result R diagnose
extern const subreg_item  EXT_TEMP_R_DIAG__RESULT; 				// RH - Result of diagnostics resistor measurement SD-ADC
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 
extern const subreg_item  EXT_TEMP_R_DIAG__INTC; 				// RH - Indicates which current source was used
																// Number of current source that was active for latest measurement.
    #define INTC__I_0						 0b0000000000000000	// Source ITMPz_0 used (default) 
    #define INTC__I_1						 0b0000010000000000	// Source ITMPz_1 used 
    #define INTC__I_2						 0b0000100000000000	// Source ITMPz_2 used 
    #define INTC__I_3						 0b0000110000000000	// Source ITMPz_3 used 
extern const subreg_item  EXT_TEMP_R_DIAG__CUR_SRC; 			// RH - Indicates which current source was used
    #define CUR_SRC__SRC_0					 0b0000000000000000	// Source 0 (default) 
    #define CUR_SRC__SRC_1					 0b0001000000000000	// Source 1 
extern const subreg_item  EXT_TEMP_R_DIAG__VALID; 				// ROCR - Indicating a valid result
    #define VALID__NO_NEW_RESULT			 0b0000000000000000	// No new result available (default) 
    #define VALID__NEW_RESULT_STORED		 0b0010000000000000	// A new result is stored in the register,
																// cleared automatically after readout of the result register.

// INT_TEMP - 0030H Chip temperature
extern const subreg_item  INT_TEMP__RESULT; 					// RH - Result of internal temperature measurement SD-ADC
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 
extern const subreg_item  INT_TEMP__VALID; 						// ROCR - Indicating a valid result
    #define VALID__NO_NEW_RESULT			 0b0000000000000000	// No new result available (default) 
    #define VALID__NEW_RESULT_STORED		 0b0010000000000000	// A new result is stored in the register,
																// cleared automatically after readout of the result register.

// MULTI_READ - 0031H Multiread command
extern const subreg_item  MULTI_READ__RES; 						// RH - Used in combination with MULTI_READ_CFG Reading this register
																// by the host starts the multiple register readroutine which got define
																// in the MULTI_READ_CFG register.
    #define RES__DEFAULT					 0b0000000000000000	// Not defined (default) 

// MULTI_READ_CFG - 0032H Multiread configuration (This register must be written with a broadcast write command)
extern const subreg_item  MULTI_READ_CFG__PCVM_SEL; 			// RW - Selects which PCVM results are part of the multiread No PCVM result for 1101B...1111B
    #define PCVM_SEL__NO_PCVM				 0b0000000000000000	// No PCVM result (default) 
    #define PCVM_SEL__RES_CELL_11			 0b0000000000000001	// Only result for cell 11 
    #define PCVM_SEL__RES_CELL_11_10		 0b0000000000000010	// Result of Cell 11-10 
    #define PCVM_SEL__RES_CELL_11_9			 0b0000000000000011	// Result of Cell 11-9 
    #define PCVM_SEL__RES_CELL_11_0			 0b0000000000001100	// Result of Cell 11-0 
extern const subreg_item  MULTI_READ_CFG__BVM_SEL; 				// RW - Selects if BVM result is part of multiread
    #define BVM_SEL__NO_RESULT				 0b0000000000000000	// No BVM result (default) 
    #define BVM_SEL__RESULT					 0b0000000000010000	// Result of BVM 
extern const subreg_item  MULTI_READ_CFG__EXT_TEMP_SEL; 		// RW - Selects which TEMP result is part of the multiread
    #define EXT_TEMP_SEL__NO_TEMP			 0b0000000000000000	// No TEMP result (default) 
    #define EXT_TEMP_SEL__RES_TMP0			 0b0000000000100000	// Result of TEMP_0 
    #define EXT_TEMP_SEL__RES_TMP0_1		 0b0000000001000000	// Result of TEMP_0 & TEMP_1 
    #define EXT_TEMP_SEL__RES_TMP0_2		 0b0000000001100000	// Result of TEMP_0 & TEMP_1 & TEMP_2 
    #define EXT_TEMP_SEL__RES_TMP0_3		 0b0000000010000000	// Result of TEMP_0 & TEMP_1 & TEMP_2 & TEMP_3 
    #define EXT_TEMP_SEL__RES_TMP0_4		 0b0000000010100000	// Result of TEMP_0 & TEMP_1 & TEMP_2 & TEMP_3 & TEMP_4 
extern const subreg_item  MULTI_READ_CFG__EXT_TEMP_R_SEL; 		// RW - Selects if R_DIAG result is part of the multiread
    #define EXT_TEMP_R_SEL__NO_R_DIAG_RES	 0b0000000000000000	// No R_DIAG result (default) 
    #define EXT_TEMP_R_SEL__R_DIAG_RES		 0b0000000100000000	// Result of R_DIAG
extern const subreg_item  MULTI_READ_CFG__INT_TEMP_SEL; 		// RW - 0B NO_INT_TMP_RES: No INT_TEMP result (default)
    #define INT_TEMP_SEL__NO_INT_TMP_RES	 0b0000000000000000	// Result of INT_TEMP
	#define INT_TEMP_SEL__INT_TMP_RES		 0b0000001000000000	// Result of INT_TEMP
extern const subreg_item  MULTI_READ_CFG__SCVM_SEL; 			// RW - Selects if SCVM results are part of the multiread
    #define SCVM_SEL__NO_RESULT				 0b0000000000000000	// No SCVM result (default) 
    #define SCVM_SEL__RESULT				 0b0000010000000000	// Result of SCVM_HIGH and SCVM_LOW 
extern const subreg_item  MULTI_READ_CFG__STRESS_PCVM_SEL; 		// RW - Selects if PCVM stress correction value is part of the multiread
    #define STRESS_PCVM_SEL__NO_RESULT		 0b0000000000000000	// No STRESS_PCVM result (default) 
    #define STRESS_PCVM_SEL__RESULT			 0b0000100000000000	// Result of STRESS_PCVM 

// BAL_DIAG_OC - 0033H Passive balancing diagnostics OVERCURRENT
extern const subreg_item  BAL_DIAG_OC__OC_i; 					// ROCW - Balancing overcurrent in cell i
																// Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define OC_i__NO_OC						 0b0000000000000000	// No balancing overcurrent detected in respective cell (default) 
    #define OC_i__OC						 0b0000100000000000	// Balancing overcurrent detected in respective cell.
																// Balancing is deactivated for this cell.
// BAL_DIAG_UC - 0034H Passive balancing diagnostics UNDERCURRENT
extern const subreg_item  BAL_DIAG_UC__UC_i; 					// ROCW - Balancing undercurrent in cell i
																// Can also be cleared on write '0' to connected GEN_DIAG register bit.
    #define UC_i__NO_UC						 0b0000000000000000	// No balancing undercurrent detected in respective cell (default)
    #define UC_i__UC						 0b0000100000000000	// Balancing undercurrent detected in respective cell.
																// Balancing is deactivated for this cell.

// INT_TEMP_2 - 0035H Chip temperature 2
extern const subreg_item  INT_TEMP_2__RESULT; 					// RH - Result of internal temperature 2 measurement SD-ADC
    #define RESULT__DEFAULT					 0b0000000000000000	// Default 
extern const subreg_item  INT_TEMP_2__VALID; 					// ROCR - Indicating a valid result
    #define VALID__NO_NEW_RESULT			 0b0000000000000000	// No new result available (default) 
    #define VALID__NEW_RESULT_STORED		 0b0010000000000000	// A new result is stored in the register,
																// cleared automatically after readout of the result register.

// CONFIG_MAIN (CONFIG) - 0036H Configuration // Renamed to CONFIG_MAIN because CONFIG is to generic and used otherwise
extern const subreg_item  CONFIG__NODE_ID; 						// RWO - Address (ID) of the node, distributed during enumeration NODE_ID = 0 -->
																// iso UART signals are not forwardedNODE_ID = 63 --> reserved for broadcast commands
extern const subreg_item  CONFIG__EN_ALL_ADC; 					// RW - Enable all ADCs
																// If this bit is set, PCVM is done for each channel, independent ofPART_CONFIG setup.
    #define EN_ALL_ADC__SEL_ADC				 0b0000000000000000	// Only ADCs enabled which are defined in PART_CONFIG as active cell
    #define EN_ALL_ADC__ALL_ADC				 0b0000001000000000	// All ADCs enabled 
extern const subreg_item  CONFIG__FN; 							// RWO - Final Node The final node in stack must have this bit set.
																// If final node does nothave FN set, no reply frame on broadcast will be sent.
    #define FN__NOT_FN						 0b0000000000000000	// Not the final node (default) 
    #define FN__FN							 0b0000100000000000	// Final node 

// GPIO - 0037H General purpose input / output
extern const subreg_item  GPIO__IN_GPIO0; 						// RH - GPIO 0 input state (ignored if communication over GPIO pins)
    #define IN_GPIO0__LOW					 0b0000000000000000	// Pin reads L (default) Also active for DIR_GPIO0 =1 (reading back driven value).
    #define IN_GPIO0__HIGH					 0b0000000000000001	// Pin reads H 
extern const subreg_item  GPIO__OUT_GPIO0; 						// RW - GPIO 0 output setting (ignored if communication over GPIO pins)
    #define OUT_GPIO0__LOW					 0b0000000000000000	// Drive L (default) 
    #define OUT_GPIO0__HIGH					 0b0000000000000010	// Drive H 
extern const subreg_item  GPIO__DIR_GPIO0; 						// RW - GPIO 0 direction (ignored if communication over GPIO pins)
    #define DIR_GPIO0__INPUT				 0b0000000000000000	// Input (output stage = HiZ) (default) 
    #define DIR_GPIO0__OUTPUT				 0b0000000000000100	// Output (output stage enabled) 
extern const subreg_item  GPIO__IN_GPIO1; 						// RH - GPIO 1 input state (ignored if communication over GPIO pins)
    #define IN_GPIO1__LOW					 0b0000000000000000	// Pin reads L (default) Also active for DIR_GPIO1 =1 (reading back driven value).
    #define IN_GPIO1__HIGH					 0b0000000000001000	// Pin reads H 
extern const subreg_item  GPIO__OUT_GPIO1; 						// RW - GPIO 1 output setting (ignored if communication over GPIO pins)
    #define OUT_GPIO1__LOW					 0b0000000000000000	// Drive L (default) 
    #define OUT_GPIO1__HIGH					 0b0000000000010000	// Drive H 
extern const subreg_item  GPIO__DIR_GPIO1; 						// RW - GPIO 1 direction (ignored if communication over GPIO pins)
    #define DIR_GPIO1__INPUT				 0b0000000000000000	// Input (output stage = HiZ) (default) 
    #define DIR_GPIO1__OUTPUT				 0b0000000000100000	// Output (output stage enabled) 
extern const subreg_item  GPIO__IN_PWM1; 						// RH - PWM 1 input state
    #define IN_PWM1__LOW					 0b0000000000000000	// Pin reads L (default) Also active for DIR_PWM1 =1 (reading back driven value).
    #define IN_PWM1__HIGH					 0b0000000001000000	// Pin reads H 
extern const subreg_item  GPIO__PWM_PWM1; 						// RW - PWM 1 enable PWM function
    #define PWM_PWM1__DISABLE				 0b0000000000000000	// No PWM function (default) 
    #define PWM_PWM1__ENABLE				 0b0000000010000000	// If DIR_PWM1 = 1, then PWM output regarding PWM_GPIO register and OUT_PWM1 is ignored.
																// If DIR_PWM1 = 0, GPIO inputand no PWM function.
extern const subreg_item  GPIO__OUT_PWM1; 						// RW - PWM 1 output setting
    #define OUT_PWM1__LOW					 0b0000000000000000	// Drive L (default) 
    #define OUT_PWM1__HIGH					 0b0000000100000000	// Drive H 
extern const subreg_item  GPIO__DIR_PWM1; 						// RW - PWM 1 direction
    #define DIR_PWM1__INPUT					 0b0000000000000000	// Input (output stage = HiZ) (default) 
    #define DIR_PWM1__OUTPUT				 0b0000001000000000	// Output (output stage enabled) 
extern const subreg_item  GPIO__IN_PWM0; 						// RH - PWM 0 input state
    #define IN_PWM0__LOW					 0b0000000000000000	// Pin reads L (default) Also active for DIR_PWM0 =1 (reading back driven value).
    #define IN_PWM0__HIGH					 0b0000010000000000	// Pin reads H 
extern const subreg_item  GPIO__PWM_PWM0; 						// RW - PWM 0 enable PWM function
    #define PWM_PWM0__DISABLE				 0b0000000000000000	// No PWM function (default) 
    #define PWM_PWM0__ENABLE				 0b0000100000000000	// If DIR_PWM0= 1, then PWM output regarding PWM_GPIO register and OUT_PWM0 is ignored.
																// If DIR_PWM0= 0, GPIO inputand no PWM function.
extern const subreg_item  GPIO__OUT_PWM0; 						// RW - PWM 0 Output Setting
    #define OUT_PWM0__LOW					 0b0000000000000000	// Drive L (default) 
    #define OUT_PWM0__HIGH					 0b0001000000000000	// Drive H 
extern const subreg_item  GPIO__DIR_PWM0; 						// RW - PWM 0 direction
    #define DIR_PWM0__INPUT					 0b0000000000000000	// Input (output stage = HiZ) (default) 
    #define DIR_PWM0__OUTPUT				 0b0010000000000000	// Output (output stage enabled) 
extern const subreg_item  GPIO__VIO_UV; 						// ROCW - VIO undervoltage error
    #define VIO_UV__NO_ERR					 0b0000000000000000	// No VIO undervoltage error (default) 
    #define VIO_UV__ERR						 0b1000000000000000	// VIO undervoltage error occurred 

// GPIO_PWM - 0038H PWM settings
extern const subreg_item  GPIO_PWM__PWM_DUTY_CYCLE; 			// RW - PWM duty cycle 0 - 100%
    #define PWM_DUTY_CYCLE__OFF				 0b0000000000000000	// No PWM (default) 
    #define PWM_DUTY_CYCLE__3_5				 0b0000000000000001	// 3.57% 
    #define PWM_DUTY_CYCLE__7_1				 0b0000000000000010	// 7.14% 
    #define PWM_DUTY_CYCLE__100				 0b0000000000011100	// 100% 
    #define PWM_DUTY_CYCLE__100_			 0b0000000000011111	// 100% 
extern const subreg_item  GPIO_PWM__PWM_PERIOD; 				// RW - PWM period time setting 2µs - 62µs
    #define PWM_PERIOD__OFF					 0b0000000000000000	// No PWM (default) 
    #define PWM_PERIOD__2us					 0b0000000100000000	// 2µs 
    #define PWM_PERIOD__4us					 0b0000001000000000	// 4 µs 
    #define PWM_PERIOD__62us				 0b0001111100000000	// 62µs 

// ICVID - 0039H IC version and manufacturing ID
extern const subreg_item  ICVID__VERSION_ID; 					// RH - Version ID Read only version ID.
    #define VERSION_ID__DEFAULT				 0b0000000001000000	// Default 
extern const subreg_item  ICVID__MANUFACTURER_ID; 				// RH - Manufacturer ID Read only manufacture ID.
    #define MANUFACTURER_ID__DEFAULT		 0b1100000100000000	// Default 

// MAILBOX - 003AH Mailbox register
extern const subreg_item  MAILBOX__DATA; 						// RW - Data storage register 2 data byte data storage

// CUSTOMER_ID_0 - 003BH Customer ID 0
extern const subreg_item  CUSTOMER_ID_0__DATA; 					// RH - Unique ID part 1

// CUSTOMER_ID_1 - 003CH Customer ID 1
extern const subreg_item  CUSTOMER_ID_1__DATA; 					// RH - Unique ID part 2

// WDOG_CNT - 003DH Watchdog counter
extern const subreg_item  WDOG_CNT__WD_CNT; 					// RWH - Watchdog counter
    #define WD_CNT__SLEEP					 0b0000000000000000	// Device goes to sleep 
    #define WD_CNT__1						 0b0000000000000001	// tWD_LSB (EXT_WD = 0) / tWD_EXT_LSB (EXT_WD = 1 ) 
    #define WD_CNT__127						 0b0000000001111111	// tWD_LSB * 127 (EXT_WD = 0) / tWD_EXT_LSB * 127 (EXT_WD = 1) (default) 
extern const subreg_item  WDOG_CNT__MAIN_CNT; 					// RH - Main counter Used to enable host controller
																// to measure the main oscillator frequency. LSB = t Count_LSB.
																// Reset by write access to WD_CNT ifRR_SYNC=1.
    #define MAIN_CNT__DEFAULT				 0b0000000000000000	// Default 

// SCVM_CONFIG - 003EH SCVM configuration
extern const subreg_item  SCVM_CONFIG__EN_SCVMi; 				// RW - Enable SCVM for cell i
    #define EN_SCVMi__DIS					 0b0000000000000000	// SCVM disabled 
    #define EN_SCVMi__EN					 0b0000100000000000	// SCVM enabled (default) 

// STRESS_AUX - 003FH Stress correction AUX
extern const subreg_item  STRESS_AUX__STRESS_CORRECTION; 		// RH - Stress correction value PCVM
    #define STRESS_CORRECTION__DEFAULT		 0b0000000000000000	// Default 

// BAL_PWM - 005BH Balancing PWM
extern const subreg_item  BAL_PWM__BAL_PWM; 					// RW - PWM balancing, starts with off-phase
    #define BAL_PWM__VAL_0					 0b0000000000000000	// 100% duty cycle (function disabled) 
    #define BAL_PWM__VAL_1					 0b0000000000000001	// 87,5% duty cycle 
    #define BAL_PWM__VAL_7					 0b0000000000000111	// 12.5% duty cycle 

// BAL_CNT_0 - 005CH Balancing counter register 0
extern const subreg_item  BAL_CNT_0__BAL_CNT_0; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_0__VAL_1				 0b0000000000000001	// 7.5 min 
    #define BAL_CNT_0__VAL_31				 0b0000000000011111	// 3.87 h 
extern const subreg_item  BAL_CNT_0__BAL_CNT_1; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_1__VAL_1				 0b0000000000100000	// 7.5 min 
    #define BAL_CNT_1__VAL_31				 0b0000001111100000	// 3.87 h 
extern const subreg_item  BAL_CNT_0__BAL_CNT_2; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_2__VAL_1				 0b0000010000000000	// 7.5 min 
    #define BAL_CNT_2__VAL_31				 0b0111110000000000	// 3.87 h 

// BAL_CNT_1 - 005DH Balancing counter register 1
extern const subreg_item  BAL_CNT_1__BAL_CNT_3; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_3__VAL_1				 0b0000000000000001	// 7.5 min 
    #define BAL_CNT_3__VAL_31				 0b0000000000011111	// 3.87 h 
extern const subreg_item  BAL_CNT_1__BAL_CNT_4; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_4__VAL_1				 0b0000000000100000	// 7.5 min 
    #define BAL_CNT_4__VAL_31				 0b0000001111100000	// 3.87 h 
extern const subreg_item  BAL_CNT_1__BAL_CNT_5; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_5__VAL_1				 0b0000010000000000	// 7.5 min 
    #define BAL_CNT_5__VAL_31				 0b0111110000000000	// 3.87 h 

// BAL_CNT_2 - 005EH Balancing counter register 2
extern const subreg_item  BAL_CNT_2__BAL_CNT_6; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_6__VAL_1				 0b0000000000000001	// 7.5 min 
    #define BAL_CNT_6__VAL_31				 0b0000000000011111	// 3.87 h 
extern const subreg_item  BAL_CNT_2__BAL_CNT_7; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_7__VAL_1				 0b0000000000100000	// 7.5 min 
    #define BAL_CNT_7__VAL_31				 0b0000001111100000	// 3.87 h 
extern const subreg_item  BAL_CNT_2__BAL_CNT_8; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_8__VAL_1				 0b0000010000000000	// 7.5 min 
    #define BAL_CNT_8__VAL_31				 0b0111110000000000	// 3.87 h 

// BAL_CNT_3 - 005FH Balancing counter register 3
extern const subreg_item  BAL_CNT_3__BAL_CNT_9; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_9__VAL_1				 0b0000000000000001	// 7.5 min 
    #define BAL_CNT_9__VAL_31				 0b0000000000011111	// 3.87 h 
extern const subreg_item  BAL_CNT_3__BAL_CNT_10; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_10__VAL_1				 0b0000000000100000	// 7.5 min 
    #define BAL_CNT_10__VAL_31				 0b0000001111100000	// 3.87 h 
extern const subreg_item  BAL_CNT_3__BAL_CNT_11; 				// RW - Balancing counter to switch off balancing after a certain time,
																// counter is (re-)started by a write access to WDOG_CNT when EXT_WD=1.
    #define BAL_CNT_11__VAL_1				 0b0000010000000000	// 7.5 min 
    #define BAL_CNT_11__VAL_31				 0b0111110000000000	// 3.87 h 


//****************************************************************************
// Special combinations of settings
//****************************************************************************
// Mask of all GEN_DIAG flags that are read only
#define GEN_DIAG_READ_ONLY_FLAGS (uint16_t)(UART_WAKEUP__UART | \
		MOT_MOB_N__TOP | BAL_ACTIVE__ON | LOCK_MEAS__MEAS_ON | RR_ACTIVE__ON)
// Mask of all GEN_DIAG flags that are can be reset by writing 0
#define GEN_DIAG_RESETABLE_FLAGS (uint16_t)(~((uint16)GEN_DIAG_READ_ONLY_FLAGS))

//****************************************************************************
// Function prototypes
//****************************************************************************
void setRegisterCacheDataByIndex_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint16_t reg_data);
uint16_t getRegisterCacheDataByIndex_tle9012(uint8_t nodeID, uint8_t reg_num);

void sendWrite(tle9012_node* node, uint8_t reg_address, uint16_t data,
		uint8_t isBroadcast);
void sendRead(tle9012_node* node, uint8_t reg_address, uint8_t isBroadcast);

void sendWatchdogUpdate(uint8_t nodeID, uint8_t isBroadcast);

void setConfigBit_tle9012(uint8_t nodeID, uint8_t state, uint8_t reg_index,
		uint16_t bit, uint8_t blocking, uint8_t isBroadcast);
void setBits_tle9012(uint8_t nodeID, uint8_t reg_index, uint16_t bitmask,
		uint8_t blocking, uint8_t isBroadcast);
void clearBits_tle9012(uint8_t nodeID, uint8_t reg_index, uint16_t bitmask,
		uint8_t blocking, uint8_t isBroadcast);


//****************************************************************************
// Register ready/error bitmap - Every bit of these maps represents if the
// related register index is ready and without error
//		if command is queued - ready is set to 0 until result has been
//		received or timeout occurred
//		if command failed or CRC was not correct - the error bit is set
//****************************************************************************
uint8_t checkRegisterReady_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint8_t isBroadcast);
void setRegisterReady_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint8_t isBroadcast);
void resetRegisterReady_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint8_t isBroadcast);
uint8_t checkRegisterError_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint8_t isBroadcast);
void setRegisterError_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint8_t isBroadcast);
void resetRegisterError_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint8_t isBroadcast);
uint8_t checkRegisterQueued_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint8_t isBroadcast);
void setRegisterQueued_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint8_t isBroadcast);
void resetRegisterQueued_tle9012(uint8_t nodeID, uint8_t reg_index,
		uint8_t isBroadcast);


//****************************************************************************
// getRegisterIndexFromAddress
//
// Converts a register address to the correlating register index.
// WARNING: Usage should be kept at an minimum because this transition takes
// much more resources than direct index access. Needed primarily when receiving
// packages where only address is given.
//
// Returns the index if it is found or 255 if not (error)
//****************************************************************************
static inline uint8_t getRegisterIndexFromAddress(uint8_t reg_address) {
	uint8_t i = 0;
	// Go through each register index and check if address is found
	do{
		// If address is found return the index
		if(reg_items[i].address == reg_address) return i;
		i++;
	}while(i < NUM_REGISTERS);

	// The given address was not found return 255 to indicate error
	return 255;

	// Use this simulation for unit testing - comment all above
	//return reg_address;
}

#endif /* TLE9012_REGISTERS_H__ */
