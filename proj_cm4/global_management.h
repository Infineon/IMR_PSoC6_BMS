/*
 * global_management.h - Abstraction off all settings and objects that can be accessed from every part of the system.
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

#ifndef GLOBAL_MANAGEMENT_H_
#define GLOBAL_MANAGEMENT_H_

#include "cycfg.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "ram_public.h"
#include "safety_switch.h"
#include "BMS_TLE9012/BMS_TLE9012.h"
#include "battery_state.h"


/*******************************************************************************
* Debug message definitions - Un/comment the printf commands to show or hide certain messages
*******************************************************************************/
#define PRINTF_CYCLIC_DEBUG_MSGS(p_frm, ...)			printf(p_frm, ##__VA_ARGS__)

//****************************************************************************
//           SETTING DEFINITIONS
//****************************************************************************
// System constants
#define NUM_CELLS 12

// LED control
#define BLINK_TIME_LED_ORANGE 			300				 // Time in ms between each blink of the orange LED during precharge
#define BLINK_TIME_LED_RED 				750				 // Time in ms between each blink of the red LED when system charge state is below warning threshold
#define BLINK_TIME_LED_RED_ON_TIME		50				 // Time in ms that the red LED stays on while blinking on BLINK_TIME_LED_RED interval
#define BLINK_TIME_LED_GREEN_SHUTDOWN	200				 // Time in ms between each blink of the green LED during shutdown

// Coding resister - slot detection
#define CODING_RES_MAINBOARD_2_BELOW_mV	2578			 // The voltage threshold when the system is inside the IMR mainboard 2 with the  13KOhm resistor installed (part of a voltage divider measured by internal ADC)
#define CODING_RES_MAINBOARD_1_BELOW_mV	1800			 // The voltage threshold when the system is inside the IMR mainboard 1 with the 7.5KOhm resistor installed (part of a voltage divider measured by internal ADC)
#define CODING_RES_CHARGER_BELOW_mV		705				 // The voltage threshold when the system is in the charger with 0Ohm installed as coding resistor
#define SLOT_PLUG_DEBOUNCE_TIME			1000			 // Time in ms that must pass before slot state changes to a state where switch on is possible are accepted. Prevents switch on during plug in etc
#define DETECT_SLOT_BASED_ON_CURRENT    0 				 // EXPERIMENTAL: Determine slot based on current and coding resistor (1, experimental/untested) or only based on coding resistor (0, default). Can be used for lab cycling

// Main loop task intervals
#define CAN_MINIMAL_MESSAGE_TIME		5				 // Minimal time between CAN messages to limit bus usage. Also defines how fast recorded real time (RT) data can be transfered via CAN command BMS_RT_Data_GetLines (e.g. 3275 lines with 2 messages per line and 5ms between messages -> 32.75s)
#define CAN_DATA_REFRESH_TIME			300				 // Time between each send of status information via CAN (voltage, current, SoC, ...)
#define CAN_SWICH_ON_VETO_REFRESH_TIME	100				 // Time between each send of the switch on veto via CAN. This is needed to signal any other BMS that this BMS is supplying the load and that it should not switch on
#define EXTERNAL_COMMUNICATION_TIME    	1000  			 // Time in ms that must pass between cyclic debug message sending
#define BALANCING_REFRESH_TIME			5000  			 // Time in ms that must pass between execution of the balancing algorithm (includes calculation of the cell imbalance, creation of balancing map, and debug message to show cell levels. Does not include send of balancing map to analog frontend)
#define TLE9012_MEASURE_TIME			100				 // Time in ms that must pass between 2 main measurement triggers of the TLE9012 (analog frontend measuring block/cell voltage, etc). In the same cycle as this the current measurement will be triggered. Do not set this value lower than two times the minimal ADC refresh time, which is mostly determined by the over-sampling value: MCO3x6x_ADC.h -> CONFIG1_CFG -> OSR_24576 (this setting takes around 30ms). Two times because the ADc has 2 channels
#define TLE9012_MEASURE_CHARGE_TIME		250				 // Time in ms that must pass between 2 main measurement triggers of the TLE9012 while the system is charging. NOTE that the balancing will be disabled at the start of the measurement and will be activated again afterwards (see processMeasurements(), takes around 7ms). Therefore this limits how effective the balancing is. In addition the RoundRobin cycle of the TLE9012 will temporarily deactivate balancing for around ~1ms every 50ms.
#define UPDATE_PUBLIC_DISPLAY_DATA_TIME 10				 // Time in ms that must pass between the refresh of the data for the CM0+ to be shown on the display. Note that some values might not be transmitted every cycle.
#define LAST_UPDATE_DISPLAY_NUM			500				 // Number of times the system waits LAST_UPDATE_DISPLAY_DELAY ms for the last display refresh at shutdown. This times the DELAY must be long enough to update the display at least 2 times (mainly limited by display refresh time and CM0_REFRESH_PERIOD_MIN) e.g. 500*9ms = 4500ms
#define LAST_UPDATE_DISPLAY_DELAY		9				 // Delay in ms that the system waits between last display refresh tries, after shutdown button is pressed. This times the NUM   must be long enough to update the display at least 2 times (mainly limited by display refresh time and CM0_REFRESH_PERIOD_MIN) e.g. 500*9ms = 4500ms

// Safety switch control
// Note that the setup of the switch must be adjusted in initSwitchDriver() of safety_switch.c
#define PRECHARGE_MIN_TIME				200				 // Minimum time in ms that must pass between turn on of precharge and turn on of main current switch in hotswap MAIN mode (initial switch on)
#define PRECHARGE_MAX_TIME				2000			 // Maximum time in ms that can  pass between turn on of precharge and turn on of main current switch in hotswap MAIN mode (initial switch on). At the latest after this time the main path is switched on, if no error occurs
#define PRECHARGE_TARGET_V_CHANGE_RATE	0.01			 // V/ms. If the change rate of the output voltage is below this value and the PRECHARGE_MIN_TIME has passed the main path is switched on. To deactivate this make set value to -1 (only max time will control precharge time)
#define MOSFET_ON_RETRY_TIME        	4000  			 // Time in ms that must pass between 2 actual tries to set the MOSFET conducting
#define SAFETY_SWITCH_RETRY_COUNT		1				 // Limit number of switch on tries // TODO Untested to set this higher than 1. Retrying it by execution of reset_safetySwitchTurnOn() works anyway

// Memory
// Note that the setup (size) of the memory can be adjusted in FRAM_DIRECT_SPI.h
#define MEMORY_SYSTEM_DATA_REFRESH_TIME (1*6*1000)		 // Time in ms. Time between checks if system data like settings and status (stored SoC and SoH) must be refreshed
#define MEMORY_RT_DATA_C_DIF_MULTIPLIER (1000.0)		 // Value multiplied with the capacity difference dC for float to integer conversion (e.g. 1000.0 preserves 3 floating point digits)
#define MEMORY_RT_DATA_DEF_REFRESH_TIME (5000)		 	 // Time in ms. Default time between last and next real time data line must be stored. Used when MEMORY_RT_DATA_FIX_REFRESH_T = 1
#define MEMORY_RT_DATA_MAX_REFRESH_TIME (65400)		 	 // Time in ms. Maximal time between last and next real time data line must be stored. Note that the time difference is stored as unsigned 2byte - when dynamic sampling time is used (MEMORY_RT_DATA_FIX_REFRESH_T = 0) this must never be higher than its range (~65sec)! On Fixed time this can be set much higher.
#define MEMORY_RT_DATA_MIN_REFRESH_TIME (250)		 	 // Time in ms. Minimal time between last and next real time data line must be stored. If this is set lower than TLE9012_MEASURE_TIME or TLE9012_MEASURE_CHARGE_TIME it will be limited by it because at least one measurement must be between recording lines.
#define MEMORY_RT_DATA_FIX_REFRESH_T	(0)		 	 	 // Boolean. If set to 0 the time between recording of RT data lines will be calculated based on remaining runtime and remaining size in memory. This makes it possible to record a full charge/discharge cycle even on differing loads, but limits the interval to the size of the time field (~65s). The minimal interval is always limited by measurement speed.
														 //          If set to 1 the time between recording or RT data lines will be fixed to MEMORY_RT_DATA_DEF_REFRESH_TIME. If this is used the stored time field only stores the time offset from the set time. This way the interval can be much higher. The minimal interval is always limited by measurement speed.


// Hotswap, shutdown and watchdog
#define ENABLE_HARDWARE_WATCHDOG		1				 // Disable or enable hardware watchdog with cyhal_wdt_init(). Note that this feature is deactivated in Hardware when a debug probe is attached to the board (see PSoC documentation)
#define ENABLE_SOFTWARE_WATCHDOG		1				 // Disable or enable software watchdog. Use this for easier debugging.
#define HARDWARE_WATCHDOG_TIMEOUT		6000			 // Time in ms between watchdog checks if controller hanged up. If a main loop cycle takes longer than this or controller enters fault state, the system will be reset hard (no soft reset). Note this must be lower than result of cyhal_wdt_get_max_timeout_ms()
#define SOFTWARE_WATCHDOG_TIMEOUT		500				 // Time in ms between watchdog checks if main loop hanged up.  If a main loop cycle takes longer than this, the system will be shut down controlled.
#define MAXIMUM_SHUTDOWN_TIME			6000			 // Maximum time that can pass between shutdown button press and hard shutdown. Also determines how long the button must be pressed continuously to trigger hard shutdown. Note that this code also runs on CM0 to make sure a shutdown is possible even if one core stops working. The value for the CM0 core must be set in proc_cm0p/main.c
#define HOTSWAP_INIT_SWITCHON_VETO_TIME 1000			 // Time in ms at the beginning where the systems send switch on veto messages in order to detect if another one is present and not switch on simultaneously
#define HOTSWAP_CAN_MSG_DELAY_TIME		10				 // Time in ms that limits the CAN message sending of switch on veto during init of hotswap
#define HOTSWAP_WAIT_REDUCE_POWER_TIME	1000			 // Time in ms that must pass before hot swap goes on after reduce power command is send
#define HOTSWAP_TIMEOUT_TIME			1500			 // Time in ms that must pass before a timeout during the hot swap is recognized and the system is shut down
#define HOTSWAP_TAKER_SHUTDWN_VEXT_TIME 10000			 // Time in ms that must pass between system startup and the first check if system is in hotswap TAKER role and external voltage is gone (this will lead to shutdown because system can never take over in this condition). This must be high enough so that the potential giver BMS can enable its output when both systems are started at the same time. To deactivate feature set value to UINT32_MAX.

// Diagnostic and Error thresholds
#define ACTIVATE_CELL_ERROR_IGNORE		0				 // IF THIS IS ACTIVE (1) ALL ERRORS CONCEARNING TLE9012_INTERRUPT, ADC_ERR, OL_ERR, CELL_OV, CELL_UV, VBAT_UV ARE IGNORED IN ORDER TO USE A EXTERNAL POWER SUPPLY WITHOUT CELLS AS SOURCE. NEVER ACTIVATE THIS IN PRODUCTION - ONLY FOR DEBUGGING. WHEN ACTIVE THR READ LED ON THE CONTROLER BOARD IS PERMANENTLY ON
#define LIMIT_SYS_UNDER_VOLTAGE_mV		(2805*NUM_CELLS) // mV. Undervoltage limit for the whole battery. If the pack voltage goes under this the safety switch will be deactivated (MOSFETs blocking)
#define LIMIT_SYS_OVER_VOLTAGE_mV		(4255*NUM_CELLS) // mV. Overvoltage  limit for the whole battery. If the pack voltage goes over this the safety switch will be deactivated (MOSFETs blocking)
#define LIMIT_CELL_UNDER_VOLTAGE_mV		2800			 // mV. Undervoltage limit for each cell. If any cell goes under this the safety switch will be deactivated (MOSFETs blocking)
#define LIMIT_CELL_OVER_VOLTAGE_mV		4250			 // mV. Overvoltage  limit for each cell. If any cell goes over this the safety switch will be deactivated (MOSFETs blocking)
#define OVERCURRENT_THRESHOLD_POS_mA	6000.0			 // mA. Discharge current threshold over which an error is triggered
#define OVERCURRENT_THRESHOLD_NEG_mA	-2500.0 		 // mA. Charge current threshold under which an error is triggered
#define TLE9012_INTERNAL_OVERTEMP_C		100.0 			 // Celsius. Internal overtemperature threshold of TLE9012
#define TLE9012_EXTERNAL_OVERTEMP_C		60.0 			 // Celsius. Internal overtemperature threshold of TLE9012
#define DEEP_DISCHARGE_PROTECTION_V		(2.75f*NUM_CELLS)// V.  If the system voltage drops below this for DEEP_DISCHARGE_PROTECTION_TIME ms the system is shut down to prevent deep discharge
#define DEEP_DISCHARGE_PROTECTION_TIME	5000			 // ms. If the system voltage drops below DEEP_DISCHARGE_PROTECTION_V for the given amount of time the system is shut down to prevent deep discharge
#define CHARGE_FROM_UV_EXT_OFFSET_V		0.2				 // V. A undervoltage condition is ignored if the external load voltage + this offset is higher than the system voltage. This is the case before output is ON if a charger is connected or after the output is ON because V load = V system. Note that this must me at least high enough to compensate errors between the two voltage measurements (e.g. when ON V_load=43.8 and V_system=43.9 due to calibration error)
#define CHARGE_FROM_UV_EXT_MIN_V		(3.0f*NUM_CELLS) // V. The minimal external load voltage that must be present so that undervoltage errors are ignored to charge battery from undervoltage condition
#define OPEN_LOOP_MIN_V_THRESHOLD		0.0				 // V. OL_THR_MIN - Minimum voltage drop threshold at which a open load error will be detected for every cell (user manual p54 suggests with RF=10Ohm default = 0.06V  -> 0x3 or 0 to turn off)
#define OPEN_LOOP_MAX_V_THRESHOLD		0.223			 // V. OL_THR_MAX - Maximum voltage drop threshold at which a open load error will be detected for every cell (user manual p54 suggests with RF=10Ohm default = 0.223V -> 0xB)
#define EXT_LOAD_VOLTAGE_THRESHOLD_V	17.000			 // V. Threshold above which it is assumed that there is already a voltage supplying the output. Note that at the current configuration of the external ADC the lowest value it can read is around 16V!
#define TLE9012_MAX_COM_ERRORS			15				 // Number of common errors (e.g. during conversion or other commands) can happen before the error ERR_EXT_TLE9012_COM_ERROR triggers
#define TLE9012_INTERVAL_COM_ERRORS		1000			 // ms. Every TLE9012_INTERVAL_COM_ERRORS ms the numComErrors_TLE9012 counter will be reduced by 1. This way errors cannot accumulate over time but are time limited
#define DIRECT_SAVESTATE_ON_INTERRUPT	0				 // Enables (0) or disables (1) the direct activation of the save state pin if error interrupts are received. This will speed up the shutdown from cycle time (~1ms) to instant but may disrupt reset feature
// Error latching for analog frontend (Based on round robin time. This limits how fast the system tries restart after error. Can be counteracted by setting number of consecutive detected errors before error is valid in RR_ERR_CNT)
#define TLE9012_OVERTMP_ERR_LATCH_TIME	4000			 // Time in ms that must pass before int/ext overtemperature bits from TLE9012 inside the error_map will be cleared (see diagnostic.c).
#define TLE9012_COMMON_ERR_LATCH_TIME	1000			 // Time in ms that must pass before all other error bits from TLE9012 inside the error_map will be cleared (see diagnostic.c).

// ADC conversion external from MCP3465 (current and external load voltage measurement)
// Note that the actual setup of the ADC is done by changing the CONFIGx_CFG macros in MCO3x6x_ADC.h
#define ADC_V_MIN_PLAUSIBLE				17.000			 // V. The minimum voltage of the external load voltage measurement that can be considered usable (depends e.g. on ADC settings)
#define ADC_V_RAW_CONVERSION			16.166667f*0.050354f // Conversion from raw data to unit based on datasheet
#define ADC_V_EXT_CONST					868.4f			 // Calibration constant for external voltage
#define ADC_V_EXT_P1					0.9721f			 // Calibration coefficient for external voltage
#define ADC_I_RAW_CONVERSION			0.05035f		 // Conversion from raw data to unit based on datasheet
#define ADC_I_CONST						58.89f			 // Calibration constant for current
#define ADC_I_P1						18.28f			 // Calibration coefficient for current
#define ADC_EXT_ZEROING_NUM				16				 // Number of times the external ADC value is read (call of manage_external_ADC_MCP3465) during init, used for filter startup and zeroing of current. ADC_EXT_ZEROING_DELAY must be set so that conversion can be finished in this delay. Only every second call will read channel 0 (current). E.g.: 16 will read current 8 times -> filter interval SYSTEM_FILTER_INTERVAL_I must not be higher!
#define ADC_EXT_ZEROING_DELAY			10				 // Delay in ms between each read of external ADC during init (see ADC_EXT_ZEROING_NUM description). Blocking
#define ADC_EXT_ZEROING_TIMEOUT_TIME	3000			 // Time in ms that must pass before a timeout during the zeroing of external ADC is recognized
#define SYSTEM_DEFAULT_CURRENT_mA		25.0			 // mA. Current that the default system draws after startup with isolating safety switch. Used to determine setupBMS.MeasCalib_Offset_I
#define ADC_EXT_READ_TIMEOUT			3000			 // Time in ms that must pass before the communication with the ADC is considered broken and a error flag is set (diagnostic.c)
#define ADC_EXT_V_CHANGE_RATE_MAX		100.0			 // V/ms. Values below this threshold can be recognized as plausible. The value is set to this if change rate can not be calculated (first/ error read value)

// ADC conversion internal (rough current measurement with 2ED4820 CSO)
#define ADC_I_DRIVER_CONV_P0 			-276820.05032114195637
#define ADC_I_DRIVER_CONV_P1 			281.68833898924356
#define ADC_I_DRIVER_CONV_P2 			-0.06922250940858

// Filter
#define SYSTEM_FILTER_INTERVAL_IMBALANCE 3				 // Number of values over which the cell imbalance is averaged. The filter time is depending on the frequency of measure_movAvgFilter_fast() execution. At 5ms, a value of 10 means the value is filtered over 50ms
#define SYSTEM_FILTER_INTERVAL_I 		 5				 // Number of raw values over which the system current is averaged. The filter time is depending on the measurement frequency. At 5ms, a value of 10 means the value is filtered over 50ms. ADC_EXT_ZEROING_NUM must not be lower than this
#define SYSTEM_FILTER_INTERVAL_V 		 1				 // Number of raw values over which the system voltage is averaged. The filter time is depending on the measurement frequency. At 5ms, a value of 10 means the value is filtered over 50ms
#define SYSTEM_FILTER_INTERVAL_TEMPCELL	 15				 // Number of raw values over which the system average cell temperature is averaged. The filter time is depending on the measurement frequency. At 5ms, a value of 10 means the value is filtered over 50ms

// Battery state
#define BATTERY_LOW_WARNING_SOC			20				 // SoC in percent below which the red LED starts flashing to signalizes recommended battery change
#define BATTERY_LOW_WARNING_DELAY_TIME	1000			 // Time in ms during which the SOC must be below the SOC Threshold before system signalizes the user to change battery
#define ETA_C 							0.97f 			 // Percent/100. SoC/SoH battery charge efficiency
#define ETA_D  							0.998f			 // Percent/100. SoC/SoH battery discharge efficiency
#define BATTERY_STATE_MINMAX_TOL        0.2f			 // in V. Tolerance of how far below/above the cell is still recognized as completely empty/full 0%/100% (see battery_state.c)
#define BATTERY_RATED_CAPACITY			(3.45*1.0*3600.0)//	Rated Capacity in As, of each series cell => 2.6Ah per cell * 1 cells parallel * 3600 to get As
#define REMAINING_RUNTIME_SOC_PERCENT	0.10			 // in % 0.00 to 1.00. The SOC where the battery is expected to be empty (cannot supply enough current for expected load). The remaining battery runtime is calculated for the point where the SOC is expected to go below this value.
#define SOC_UPPER_TH_USE_LAST_VALUE		4.10			 // in V. Cell voltage above which the last stored SoC value is used (not the value from the OCV curve). To deactivate this feature set the value above maximal cell voltage
#define SOC_LOWER_TH_USE_LAST_VALUE		3.00			 // in V. Cell voltage below which the last stored SoC value is used (not the value from the OCV curve). To deactivate this feature set the value below minimal cell voltage
#define SOC_LAST_TO_OCV_MAX_DIFF		7.00			 // in %. At initialization of the SoC, this is the maximum difference between the stored and OCV evaluated SoC at which the stored value can still be used (useful e.g. if BMS was switched off with full/empty battery and than subjected to cell changes)
#define SOH_INIT 						100  			 // in %. Initial SoH set when it was never stored
#define BATTERY_SOH_CAPACITY_RANGE		4900			 // in As. Amount of capacity that should change between SOH_UPPER_THRESHOLD and SOH_LOWER_THRESHOLD voltage to get 100% SoH (If zero it will be estimated from OCV curve. Write correct value after first estimation with expected current here)
#define SOH_LOWER_THRESHOLD	   			20				 // in %. The SoC threshold at which the SoH calculation is started (charge counting between lower and upper threshold).   Determined by conversion of the threshold percent to V with OCV curve and compared to average cell voltage
#define SOH_UPPER_THRESHOLD	   			90				 // in %. The SoC threshold at which the SoH calculation is finalized (charge counting between lower and upper threshold). Determined by conversion of the threshold percent to V with OCV curve and compared to average cell voltage
#define SOH_INITIAL_CURRENT	   			1.5				 // in A. The minimal current that must be present for SoH estimation cycle to start
#define SOH_MAX_TEMPERATURE	   			50.5			 // in C. The maximal temperature that is allowed for the SoH estimation to be usable (BATTERY_SOH_CAPACITY_RANGE changes with temperature). If temperature is to high the SoH will be calculated but not used
#define SOH_MIN_TEMPERATURE	   			20.5			 // in C. The minimal temperature that is allowed for the SoH estimation to be usable (BATTERY_SOH_CAPACITY_RANGE changes with temperature). If temperature is to low the SoH will be calculated but not used

// Balancing
#define BALANCING_MIN_MAX_THRESHOLD 	30 	 			 // mV. If the cell with the minimum cell voltage and that with the maximum cell voltage is further than this vale apart balancing is (generally) activated
#define BALANCING_CELL_TH_LIMIT			10   			 // mV. The balancing algorithm calculates and sends the level of the lowest cell to each cell as "BalVolt" (balancing threshold). If the cell recognizes its own voltage to be more than BALANCING_CELL_TH_LIMIT over the "BalVolt" it starts balancing.
#define BALANCING_DEACTIVATED_VALUE 	10000			 // Fixed value to which the balancing threshold will be set in order to signalizes that balancing is deactivated



// Debug
#define ENABLE_DEBUG
#define SOC_FAKE_MAP_0_PERCENT_mV		(2800.0*NUM_CELLS)
#define SOC_FAKE_MAP_100_PERCENT_mV		(4200.0*NUM_CELLS)
// Unused
//#define BATTERY_CRITICAL_TAKEOVER_SOC	15				 // SoC in percent below which the system tries to give over to a second BMS
//#define CURRENT_CSO_DISBLED_THRESHOLD	10    			 // Raw ADC values below which the CSO is determined disabled (the driver deactivates the CSO on some errors and at that point it does not give out 0A in terms of raw ~2048 (half of rev. voltage) but drops to 0 which would be interpreted as max charge current)
//#define IDLE_CURRENT_THRESHOLD		0.150 			 // mA. Current around 0A that is considered idle (not considered charge or discharge)
//#define HOLD_ZERO_CURRENT_THRESHOLD 	0.02  			 // mA. Current around 0A that is considered zero (prevents flickering in GUI)
//#define PUSH_PC_DATA_REFRESH_TIME   	500   			 // Time in ms between each refresh of high priority data to the PC (Dashboard etc)
//#define PUSH_PC_SETTINGS_REFRESH_TIME	6000  			 // Time in ms between each refresh of low priority data to the PC

// Simple function to report a failed TLE9012 function execution via debug UART: Example -TLE9012_DETECT_COM_ERROR(tle9012_convertBlockVoltage(1, 1), convertBlockVoltage_error);
//   FUNCTION_WITH_PARAMETERS - The function to be executed with parameters
//   STRING                   - The string that shall be added to debug message (there might be no spaces allowed)
extern requestResult tle9012_print_if_error_result;
#define SUCCESS_FUNCTION_NONE (void)0;
#define TLE9012_DETECT_COM_ERROR(FUNCTION_WITH_PARAMETERS, STRING, SUCCESS_COMMAND) { \
		tle9012_print_if_error_result = FUNCTION_WITH_PARAMETERS;                   \
	if(tle9012_print_if_error_result >= REQUEST_ERROR){                             \
		DEBUGPRINTF("[ERROR]: " #STRING " returned %d\r\n", tle9012_print_if_error_result); \
		numComErrors_TLE9012++; \
	}                       	\
	else{                   	\
		SUCCESS_COMMAND;    	\
	}                       	\
}


//****************************************************************************
//           ENUMS AND OBJECT DEFINITION
//****************************************************************************
typedef enum {ACTION_OK, ACTION_FAIL, ACTION_WAITING} action_status;

/// STATE MACHINES
typedef enum {BMS_CHARGING = 5, BMS_DISCHARGING = 6, BMS_IDLE = 7, BMS_ERROR = 10} bms_states; // Represents the current state of the system it self. State BMS_CHARGING and BMS_DISCHARGING are considered normal operation.
// State of measurement
typedef enum {MEASUREMENT_WAIT_RESULT = 0U, MEASUREMENT_READ_RESULT = 1U, MEASUREMENT_TRIGGER_NEXT = 2U} measurementStates;
// State of the safety switch
typedef enum {SWITCH_IDLE, SWITCH_PRECHARGE} switchStates;
// HotSwap roles determined by circumstances
typedef enum {HOTSWAP_MAIN, HOTSWAP_TAKER, HOTSWAP_GIVER, HOTSWAP_ERROR} hotSwapRoles;
// HotSwap state machines of different roles
typedef enum {TAKER_WAIT_FOR_HANDSHAKE, TAKER_WAIT_SWAP, TAKER_WAIT_FINISH} takerHotSwapStates;
typedef enum {GIVER_SEND_HANDSHAKE, GIVER_WAIT_FOR_HANDSHAKE, GIVER_WAIT_REDUCE_POWER, GIVER_WAIT_SWAP} giverHotSwapStates;
// BMS status data send via CAN states
typedef enum {CAN_DATA_SEND_SOC, CAN_DATA_SEND_VOLTAGE, CAN_DATA_SEND_CURRENT} canDataSendStates;
// External ADC active channel
typedef enum {ACTIVE_EXT_ADC_CH0_CURRENT, ACTIVE_EXT_ADC_CH1_LOAD_VOLTAGE, ACTIVE_EXT_ADC_WAIT_FOR_CURRENT_TRIGGER} activeChannelExternelADCStates;



// Unused
//typedef enum {TAKEOVER_NONE, TAKEOVER_SHUTDOWN_PRESSED, TAKEOVER_SOC_CRITICAL} takeoverStates;


/// MAIN STRUCT
typedef struct {
	uint8_t returnValue;
	bms_states bms_state;
	volatile float current;
	volatile float voltage;
	float avgTemp;			// Average cell temperature
	float SoC;
	float SoH;
	uint32_t error_map;		// Current error map (for latched version see latched_error_map)
	uint32_t message_map;	// Current warning and message map (for latched version see latched_message_map)
	uint16_t ol_map;		// Open load map (latched till external communication)
	uint16_t uv_map;		// Under voltage map (latched till external communication)
	uint16_t ov_map;		// Over voltage map (latched till external communication)
	uint16_t bal_uc_map;	// Balancing under current map (latched till external communication)
	uint16_t bal_oc_map;	// Balancing over current map (latched till external communication)
	uint16_t imbalance;		// Difference between the highest and lowest cell voltage (refreshed only during balancing)
	uint16_t avgCellVolt_mV;
	uint16_t bal_cell_map;
	measurementStates measurementState;
	switchStates switchState;
	outputStates outputState;
	slotStates slotState;
	float extLoadVoltage;
	hotSwapRoles hotSwapRole;
	giverHotSwapStates giverHotSwapState;
	takerHotSwapStates takerHotSwapState;
	float voltageChangeRate_ms;
} statusBMS_struct;
extern volatile statusBMS_struct statusBMS;

/// Latched bitmaps
extern uint32_t latched_error_map_switch_on;	// Used to latch errors between switch ON of the safety switch. Will be reset if switchState is SWITCH_IDLE and switch on conditions are met (if an error happens which causes the switch to shut of it might be gone afterwards. This retains the information until the system can switch on again)
extern uint32_t latched_error_map_ext_com;		// Used to latch errors between PC/CAN transmissions. Will be reset at every call of manageExternalCommunicationBMS (if an error happens only once during loops where there is no CAN/PC transmission it would otherwise not be detected)
extern uint32_t latched_message_map;			// Used to latch warnings and messages between PC/CAN transmissions (if an error happens only once during loops where there is no CAN/PC transmission it would otherwise not be detected)

/// SoC and SoH management
extern battery_state_struct batteryState;

// Setup
typedef struct{
	uint16_t NumCells;
	uint8_t  CellPackID[4];
	int16_t  MeasCalib_Offset_I;					// Offset of used for current calculation
	uint16_t LimitUnderVoltage;
	uint16_t LimitOverVoltage;
	uint16_t LimitUnderVoltageCell;
	uint16_t LimitOverVoltageCell;
	uint16_t BalVolt;
	uint16_t BalLimit;
	uint16_t BatteryRatedCapacity;
} setupBMS_struct;
extern volatile setupBMS_struct setupBMS;


//****************************************************************************
//           VARIABLES
//****************************************************************************
// Interrupt variables
extern volatile uint8_t errorInterruptDetected_2ED4820; // Safety switch driver error interrupt detection
extern volatile uint8_t errorInterruptDetected_TLE9012; // BMS analog front end error interrupt detection

/// Timestamps for time interval control
// Slot detection
extern uint32_t slotPlugDebounceTimestamp;
// Memory
extern uint32_t memorySystemDataRefreshTimestamp;
extern uint32_t memoryRtDataRefreshTimestamp;
extern uint32_t memoryRtDataRefreshTime;
extern uint8_t  memoryRtDataRefresh_isTimeFixed;
// Safety switch
extern uint32_t switchPrechargeStartTimestamp;
extern uint32_t lastSwitchOnTimestamp;					// Time stamp when the switch where last tried to switch conducting
extern uint8_t safety_switch_retry_counter;
// Main loop intervals
extern uint32_t lastExternalCommunicationTimestamp;		// Time stamp when the communication from and to the BMS was last managed
extern uint32_t lastBatterySateCalcTimestamp;			// Time stamp when the battery state was last calculated
extern uint32_t canLastMessageTimestamp;
extern uint32_t canSwitchOnVetoRefreshTimestamp;
extern uint32_t canDataRefreshTimestamp;
extern uint32_t shutdownTriggeredTimestamp;
extern uint32_t batteryLowWarningTimestamp;
extern uint32_t updatePublicDisplayDataTimestamp;
// Analog frontend TLE9012
extern uint32_t lastTLE9012MeasurementTimestamp;		     // Time stamp when the measurement on the TLE9012 was last triggered
extern uint32_t lastTLE9012IntervalComErrorReduceTimestamp;  // Time stamp when numComErrors_TLE9012 was last reduced by 1 // See description of TLE9012_MAX_COM_ERRORS
// LED blink
extern uint16_t blinkTimeLedOrange;
extern uint32_t blinkTimeLedRedTimestamp;
// Hotswap
extern uint32_t giverHotSwapTimestamp;
extern uint32_t takerHotSwapTimestamp;
extern uint32_t switchOnVetoTimestamp;
// ADC
extern uint32_t adcZeroingTimestamp;
extern uint32_t adcExtLastReadSuccessTimestamp;
extern uint32_t adcLastVoltageChangeRateTimestamp;
// Deep discharge protection
extern uint32_t deepDischargeTimestamp;

// Flags
extern uint8_t isCurrentOffsetCalibrating;
extern uint8_t isShutdownTriggered;
extern uint8_t isTLE9012ResultReceivedThisCycle;
extern uint8_t TriggerCurrentMeasurement;

/// TLE9012 errors
extern uint16_t numComErrors_TLE9012;
extern QueueResult lastQueueResult_tle9012;

/// Watchdog
extern uint32_t mainLoopCounter;
extern uint32_t mainLoopCounter_lastWatchdog;
extern uint8_t isWatchdogActive;
/// ADC
// External ADC
extern activeChannelExternelADCStates activeChannelExternalADC;    // Currently checked channel of external ADC MCP3465 (default off)
extern float lastExtVoltageMeasurement;
// Internal ADC (coding resistor and low resolution current measurement)
extern int32_t adc_coding_res_mV;
extern int32_t adc_safety_switch_current_mV;
extern int32_t adc_safety_switch_current;

// RT data recording
extern uint32_t rt_data_time_since_last_record;		// THIS IS ONLY UPDATED AFTER EACH CURRENT MEASUREMENT (TRIGGERED BY ANALOG FRONTEND MEASUREMENT)! If rt data is recorded faster than measurement this might be 0
extern float rt_data_charge_since_last_record;	// THIS IS ONLY UPDATED AFTER EACH CURRENT MEASUREMENT (TRIGGERED BY ANALOG FRONTEND MEASUREMENT)! If rt data is recorded faster than measurement this might be 0
extern float rt_Data_dC_lostDecimals;

/// FILTER
typedef struct {
    uint16_t  bufIdx;           // Index of current value in buffers
    int32_t  lastIdx;           // Index of the oldest value in filter interval (will be erased)
    uint16_t  bufMaxIdx;        // Maximum index of all buffers
    int32_t* bufRaw;           // The raw value buffer
    float     filterResult;         // The filtered value
    float     avgFilterSum;         // Sum of all values in filter interval (moving)
    uint16_t  avgFilterInterval;    // Size of the filter interval
    int32_t   initFilterCompesation;// Variable used at the beginning so that the divider is ramped up as values are added to the sum
} avgFilter;
extern volatile avgFilter avgFilter_I;
extern volatile avgFilter avgFilter_V;
extern volatile avgFilter avgFilter_Imbalance;
extern volatile avgFilter avgFilter_tempCell;


//****************************************************************************
//           FUNCTION PROTOTYPES
//****************************************************************************
void measure_movAvgFilter_fast(volatile avgFilter* filter, int32_t newValue);
void measure_movAvgFilter_clean(volatile avgFilter* filter, int32_t newValue);


float mapRangeToScale(float n, float lRange, float hRange, float scale);
void printBinary(uint32_t data, uint32_t mask, uint8 sizeBits);
float polyEvalLinear(float acd_reading);

#endif /* GLOBAL_MANAGEMENT_H_ */
