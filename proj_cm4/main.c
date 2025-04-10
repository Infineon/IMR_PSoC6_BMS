/******************************************************************************
* File Name:   main.c (CM4)
* Created on: 1 Jun 2024
* Author: r. santeler (based on MTB example)
* Last change: 2025-01-20, S. Detzel

* Description:
* 		This is the source code for the CM4 core in PSoC6_BatteryManagement.
* 		Here all main features of the system are implemented.
* 		Code hosted on this processor must never use blocking layouts
* 		during main loop, due to safety critical functions.
* 		Code run once during initialization can be blocking but must always
* 		be timeout terminated to make sure the system enters main loop.
*
* 		The main functions are described in this file. Diagnostic, CAN,
* 		battery state management and safety switch functions are out-sourced
* 		to different files. Functions, definitions and objects that must be
* 		accessed from all components are found in global_management.
*
* 		All settings and parameters can be manipulated centrally in the
* 		"SETTING DEFINITION" section of global_management.h
*
* 		Note that commonly needed variables are grouped into statusBMS,
* 		setupBMS and batteryState.
*
* 		Note that "ram_public.h" and "ipc_def.h" is used for
* 		communication between processors and the public RAM area.
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

#include <stdint.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

#include "cycfg.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

// Cross processor communication and common elements
#include "ipc_def.h"
#include "ram_public.h"

// BMS functionality abstraction
#include "global_management.h"	// Common elements between all components
								// on CM4 core
#include "safety_switch.h"		// Safety switch related functions
								// (init, get/set switch state, etc)
#include "memory.h"				// Memory related functions
#include "balancing.h"			// Battery balancing related functions
#include "diagnostic.h"			// Diagnostic subsystem

// Peripheral component library's - The analog front-end tle9012,
// the external ADC and the memory
#include "BMS_TLE9012.h"
#include "MCP3x6x_ADC.h"
#include "MCP3x6x_SPI.h"
#include "MCP3x6x_Type.h"
#include "FRAM_DIRECT_SPI.h"

// CAN communication with other subsystems
#include "IMR_CAN.h"

/*******************************************************************************
* Debug message definitions:
* Un/comment the printf commands to show or hide certain messages
*******************************************************************************/
#define PRINTF_MAIN_DEBUG(p_frm, ...)				printf(p_frm, ##__VA_ARGS__)
#define PRINTF_HOT_SWAP(p_frm, ...)					printf(p_frm, ##__VA_ARGS__)
#define PRINTF_MAIN_RT_CAN_REQUEST(p_frm, ...)		printf(p_frm, ##__VA_ARGS__)
#define TIMER_COUNTER_1MS \
	Cy_TCPWM_Counter_GetCounter(TIMER_1MS_HW, TIMER_1MS_NUM)
#define PRINTF_MEASUREMENT(p_frm, ...) \
	//printf(p_frm, ##__VA_ARGS__)
#define PRINTF_MAIN_RT_GETLINE_DEBUG(p_frm, ...) \
	//printf(p_frm, ##__VA_ARGS__)

/******************************************************************************
* Common variables
******************************************************************************/
// SPI object and mode indicator
cyhal_spi_t mSPI;				// SPI used for safety switch 2ED4820,
								// ADC MCP3465, and Memory CY15B256Q-SXA.
								// Use "extern" if needed in other files
cyhal_spi_mode_t mSPI_curMode;	// Marks currently set SPI mode.
								// Needed because SPI devices require
								// different SPI modes and must reinitialize
								// SPI if mode is not correct for them.
								// Use "extern" if needed in other files

/* Shadow buffer for public variables*/
uint32_t  	 sh_pub_error_map = 0;
uint16_t 	 sh_pub_soc = 0;
uint16_t 	 sh_pub_soh = 0;
uint16_t 	 sh_pub_volt = 0;
int16_t 	 sh_pub_current = 0;
outputStates sh_pub_outputState = OUTPUT_OFF;
slotStates	 sh_pub_slotState = SLOT_NONE;
uint16_t 	 sh_pub_cells[NUM_CELLS];
public_core_commands 	 sh_pub_command = PUBLIC_COMMAND_IGNORE;

// Timer
cy_stc_rtc_config_t dateTime;
cyhal_wdt_t wdt_obj; // Watchdog object

// Init
uint8_t initTLE9012Error = 0;
uint8_t initExtADCError = 0;

// Marker for direct save state set on error interrupt.
// If this is 0 any error interrupt will activate the save state
// for a short time to switch system off without main loop.
uint8_t SafetySwitchInterrupt_SaveStateSet = DIRECT_SAVESTATE_ON_INTERRUPT;
uint8_t AnalogFrontendInterrupt_SaveStateSet = DIRECT_SAVESTATE_ON_INTERRUPT;

uint8_t* git_commit_hash = (uint8_t*)GIT_COMMIT_HASH;
#if (EXT_DATALOG_ENABLED)
uint32_t datalog_refreshTimestamp = 0;
#endif

/******************************************************************************
* Function Prototypes
******************************************************************************/
// Initialization functions (run once at start) - blocking code allowed
void init_UART_debug();
void init_UART_tle9012();
void init_CAN();
void init_Timer_RTC();
void init_SPI_peripheral();
void init_interrupt_2ED4820();
void init_interrupt_TLE9012();
void init_internal_ADC();
void init_external_ADC_MCP3465();
void init_hotSwap();
requestResult init_analogFrontendTLE9012();
void init_batteryState();

// Loop function (run every cycle) - no blocking code allowed
void processMeasurements();
void manage_SystemAndSwitchState();
void manage_externalCommunicationBMS();
void manage_analogFrontendTLE9012();
uint8_t manage_external_ADC_MCP3465(uint8_t waitForCurrentMeasTrigger);
void manage_internal_ADC();
void manage_timer();
void manage_slotState(uint8_t ignoreDebounce);
void manage_shutdown();

// Special functions
void keepHardwareWatchdogAlive();
void shutdownSystem();
void reset_safetySwitchTurnOn();
void reset_latchedBitmaps(uint8_t reset_error_map_switch_on,
		uint8_t reset_error_map_ext_com);
void calc_voltageChangeRate(float loadVoltage, float minLoadVoltage);
// Update public variables from shadow buffer
// if resource is not used at the moment (non blocking)
uint8_t updatePublicDisplayData(public_core_commands command);

// Interrupt routines
void timer_watchdog_interrupt_handler();
// Error pin interrupt from safety switch
static void IRQ_ERR_2ED4820(void* handler_arg, cyhal_gpio_event_t event);
// Error pin interrupt from analog front-end
static void IRQ_ERR_TLE9012(void* handler_arg, cyhal_gpio_event_t event);

// Test functions
void debugButtonCheck();	// Function extraction of debug code executed
							// when debug button is pressed
void debugRecordCycleTime();

/*****************************************************************************/

int main(void)
{
    cy_rslt_t result;

    // ---- INIT CM4 ---- blocking code allowed
    // (but must finished in reasonable time)
    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS){
    	cyhal_gpio_write(BMS_LED3_RD, 1);
    	cyhal_system_delay_ms(500);
    	cyhal_gpio_write(VCC_HOLD, 0);
    	CY_ASSERT(0);
    }

	// Init millisecond counter timer and Real Time Clock
	init_Timer_RTC();

    // Initialize Hardware Watchdog
	#if ENABLE_HARDWARE_WATCHDOG == 1
		result = cyhal_wdt_init(&wdt_obj, HARDWARE_WATCHDOG_TIMEOUT);
		if (result != CY_RSLT_SUCCESS){
			cyhal_gpio_write(BMS_LED4_BL, 1);
			cyhal_gpio_write(BMS_LED3_RD, 1);
			cyhal_system_delay_ms(1000);
			cyhal_gpio_write(VCC_HOLD, 0);
			CY_ASSERT(0);
		}
		keepHardwareWatchdogAlive();
	#endif /* #if defined (ENABLE_HARDWARE_WATCHDOG) */

    // Init debug message sending via printf
	init_UART_debug();

	#if (ACTIVATE_CELL_ERROR_IGNORE == 1 || \
			ENABLE_HARDWARE_WATCHDOG == 0 || \
			ENABLE_SOFTWARE_WATCHDOG == 0)
		PRINTF_MAIN_DEBUG
		("\r\n\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   "
		 "WATCHDOGS OR CELL ERRORS ARE BYPASSED FOR DEBUGGING, "
		 "SAFETY FEATURES ARE COMPROMISED IN THIS SYSTEM. "
		 "DO NOT DISTRIBUTE, DO NOT USE OUTSIDE OF LAB   "
		 "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
		cyhal_gpio_write(LED_Red, 0);
	#endif
	PRINTF_MAIN_DEBUG("\r\n\r\n"
			"----------------------------------------------------\r\n");
	PRINTF_MAIN_DEBUG("START CM4 - Version 10.04.2025 Release 1.7\r\n");
	PRINTF_MAIN_DEBUG("\t Last shutdown reason %u (on hard resets, "
			"system might restart multiple times!)\r\n",
			(uint16_t)cyhal_system_get_reset_reason());

	//CYHAL_SYSTEM_RESET_NONE            = 0,   /**< No cause */
	//CYHAL_SYSTEM_RESET_WDT             = 1,   /**< A watchdog timer (WDT) reset has occurred */
	//CYHAL_SYSTEM_RESET_ACTIVE_FAULT    = 2,   /**< The fault logging system requested a reset from its Active logic. */
	//CYHAL_SYSTEM_RESET_DEEPSLEEP_FAULT = 4,   /**< The fault logging system requested a reset from its Deep-Sleep logic. */
	//CYHAL_SYSTEM_RESET_SOFT            = 8,   /**< The CPU requested a system reset through it's SYSRESETREQ. */
	//CYHAL_SYSTEM_RESET_HIB_WAKEUP      = 16,  /**< A reset has occurred due to a a wakeup from hibernate power mode. */
	//CYHAL_SYSTEM_RESET_WCO_ERR         = 32,  /**< A reset has occurred due to a watch-crystal clock error */
	//CYHAL_SYSTEM_RESET_SYS_CLK_ERR     = 64,  /**< A reset has occurred due to a system clock error */
	//CYHAL_SYSTEM_RESET_PROTECTION      = 128, /**< A reset has occurred due to a protection violation */
	//CYHAL_SYSTEM_RESET_WARMBOOT        = 256, /**< A reset has occurred due wake up from DSRAM, which is a Warm Boot */

	// Keep hardware watchdog alive
	keepHardwareWatchdogAlive();

	// Init communication interface with tle9012 analog front-end
	init_UART_tle9012();

	// Init SPI for 2ED4820, Memory and external ADC
	init_SPI_peripheral();

	// Init interrupts
	init_interrupt_2ED4820();
	init_interrupt_TLE9012();

	// Initialize analog resources
	init_internal_ADC();

	// Keep hardware watchdog alive
	keepHardwareWatchdogAlive();

	// Read slot state for the first time
	manage_slotState(1);

	// From this point keep the power alive.
	// The green MCU ready LED is turned on later when pin is full and
	// that system does not shut down after it is released.
	// Doing it here prevents the system from starting up
	// after reset/shutdown etc
	cyhal_gpio_write(VCC_HOLD, 1);
	cyhal_system_delay_ms(10);

	/* Enable global interrupts */
	__enable_irq();

	// Init and setup MOSFET driver (and write Setup to debug)
	initSwitchDriver();
	setSwitchIsolating();
	cyhal_system_delay_ms(50);

	// Keep hardware watchdog alive
	keepHardwareWatchdogAlive();

	// Turn on green LED
	cyhal_gpio_write(BMS_LED1_GR, 1);

	// Init Memory
	// Enter a testing terminal for the memory.
	// Need to be activated in FRAM_DIRECT_SPI.h
	//FRAM_testTerminal();
	init_memory();

	// Initialize MCP3465 external ADC and
	// wait till enough measurements are done
	// to ensure correct filtering
	isCurrentOffsetCalibrating = 1;
	init_external_ADC_MCP3465();

	// Initialize TLE9012 analog frontend
	if(init_analogFrontendTLE9012() != REQUEST_OK){
		PRINTF_MAIN_DEBUG("TLE9012 init not successful! "
				"-------------------- \r\n");
		// If the init fails show both TLE9012 errors
		initTLE9012Error = 1;
		numComErrors_TLE9012 = 99;
	}

	// Keep hardware watchdog alive
	keepHardwareWatchdogAlive();

    // Initialize SoC and SoH of the system
    init_batteryState();

    // Initialize CAN communication
	init_CAN();

	// Send out commit hash of currently flashed software
	CAN_TX_Request(COMMIT_HASH, (uint8_t*)git_commit_hash, 7);

	// Determine initial role of BMS
	init_hotSwap();

    /* Unlock all semaphores and wake-up the CM0+ */
    Cy_IPC_Sema_Clear(SEMA_BTN,         false);
	Cy_IPC_Sema_Clear(SEMA_DISPLAY_VAR, false);
	Cy_IPC_Sema_Clear(SEMA_DEBUG_MSG, 	false);
	Cy_IPC_Sema_Clear(SEMA_START, 		false);
	__SEV();

	// Deactivate offset calibration
	isCurrentOffsetCalibrating = 0;

	// First batteryState estimation
	manage_battery_state();

	// Enable software watchdog
	mainLoopCounter_lastWatchdog = UINT32_MAX;
	isWatchdogActive = ENABLE_SOFTWARE_WATCHDOG;
	// Keep hardware watchdog alive
	keepHardwareWatchdogAlive();

	// Reset all control CAN message counter
	resetCanMessageCounters();

	PRINTF_MAIN_DEBUG("\r\n\r\n---------------------------------------\r\n");
	PRINTF_MAIN_DEBUG("MAIN LOOP\r\n");

	// ---- MAIN LOOP CM4 ---- No blocking code below this line
    for (;;)
    {
	    // Read voltages, currents, temperatures etc
	    processMeasurements();

	    // Check if system is in slot
	    manage_slotState(0);

		// Check if balancing conditions are met,
	    // execute balancing algorithm and
	    // determine the mapping of which cell needs balancing
	    // (actual write performed in processMeasurements())
		manage_balancing();

	    // Receive (when possible) errors and warnings from safety switch,
		// analog front-end TLE9012 and rest of system.
	    // Set bms_state to BMS_ERROR if needed or
		// to BMS_IDLE if error cleared.
		// Controls red LED.
	    doSystemDiagnostic();

		// Control switch state based on bms_state.
	    // When no error present determine bms_state charge direction,
	    // pre-charge and minimum restart timer.
		manage_SystemAndSwitchState();

		// Manage data from and to the BMS (CAN, UART debug, etc).
		// Resets latched bitmaps
		manage_externalCommunicationBMS();

    	// Processing of TLE9012 queue, interpreter and data
    	manage_analogFrontendTLE9012();

    	// Estimate SoC and SoH
    	manage_battery_state();

    	// Manage Timer overflow
    	manage_timer();

    	// If shutdown button is pressed kill system by disabling shutdown pin
		manage_shutdown();

    	// Update public variables from shadow buffer if
		// resource is not used at the moment (non blocking)
    	updatePublicDisplayData(PUBLIC_COMMAND_IGNORE);

    	// Update memory if needed
    	manage_memory();

		// Debug functions if button was pressed
		debugButtonCheck();

    	// Debug: Record cycle time for some time and print it
    	//debugRecordCycleTime();

    	// Keep software watchdog alive
		// Increment main loop counter to keep watchdog handler inactive
    	mainLoopCounter++;

    	// Keep hardware watchdog alive
    	keepHardwareWatchdogAlive();
    }
}

/****************************************************************************
 * timer_watchdog_interrupt_handler:
 * Handler function for the watchdog timer (check if main loop died)
 ***************************************************************************/
void timer_watchdog_interrupt_handler()
{
    // If main loop counter did non change since last time shut down system
    if(isWatchdogActive == 1 &&
    		mainLoopCounter == mainLoopCounter_lastWatchdog){
    	PRINTF_MAIN_DEBUG("System watchdog detected no activity for %d ms "
    			"and will deactivate system (%ld == %ld)\r\n",
    			SOFTWARE_WATCHDOG_TIMEOUT, mainLoopCounter,
				mainLoopCounter_lastWatchdog);

    	// Disable MOSFETs/current flow by activating save state
    	setSafestate(SET_SAFE_STATE_ON);

		// Disable shutdown pin and wait till voltage source breaks down
		cyhal_gpio_write(BMS_LED1_GR, 0);
		cyhal_gpio_write(BMS_LED2_OR, 1);
		cyhal_gpio_write(BMS_LED3_RD, 1);
		cyhal_gpio_write(BMS_LED4_BL, 1);
		for(int i = 0; i < 12; i++){
			cyhal_system_delay_ms(50);
			cyhal_gpio_toggle(BMS_LED1_GR);
			cyhal_gpio_toggle(BMS_LED2_OR);
			cyhal_gpio_toggle(BMS_LED3_RD);
			cyhal_gpio_toggle(BMS_LED4_BL);
			cyhal_gpio_write(VCC_HOLD, 0);
		}
    }

    // Store last state of main loop counter
    mainLoopCounter_lastWatchdog = mainLoopCounter;

    // Kill system if shutdown button is continuously pressed
    // for multiple seconds
    // (this code also runs on CM0 to make sure a shutdown is possible
    // even if one core stops working)
	static uint32_t shutdownButtonNotPressedTimestamp;
	if(cyhal_gpio_read(BUTTON_SHUTDOWN) == 0)
		shutdownButtonNotPressedTimestamp = TIMER_COUNTER_1MS;
	if(TIMER_COUNTER_1MS >
				(shutdownButtonNotPressedTimestamp + MAXIMUM_SHUTDOWN_TIME)){
		PRINTF_MAIN_DEBUG("HARD SHUTDOWN TRIGGERED - BUTTON PRESSED FOR "
				"LONGER THAN MAXIMUM_SHUTDOWN_TIME\r\n");
		cyhal_gpio_write(VCC_HOLD, 0);
		shutdownSystem();
	}

    // Kill system if refresh of display with resulting shutdown took to long
	if((isShutdownTriggered == 1) && (TIMER_COUNTER_1MS >
									 (shutdownTriggeredTimestamp +
											 MAXIMUM_SHUTDOWN_TIME))){
		PRINTF_MAIN_DEBUG("HARD SHUTDOWN TRIGGERED - "
				"SHUTDOWN TOOK LONGER THAN MAXIMUM_SHUTDOWN_TIME\r\n");
		shutdownSystem();
	}

    // Clear the terminal count interrupt
    Cy_TCPWM_ClearInterrupt(TIMER_WATCHDOG_HW, TIMER_WATCHDOG_NUM,
    		CY_TCPWM_INT_ON_TC );
}

//****************************************************************************
// init_UART_debug - Initialize debug message sending via printf
//****************************************************************************
void init_UART_debug()
{
	cy_rslt_t result;
	/* Free the hardware instance object if initialized by other core
	 * before initializing the same hardware instance object in this core. */
	cyhal_hwmgr_free(&CYBSP_UART_obj);
	cyhal_hwmgr_free(&CYBSP_DEBUG_UART_RX_obj);
	cyhal_hwmgr_free(&CYBSP_DEBUG_UART_TX_obj);

	/* Initialize retarget-io to use the debug UART port. */
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
			CY_RETARGET_IO_BAUDRATE);
	if (result != CY_RSLT_SUCCESS){ CY_ASSERT(0); }
}

//****************************************************************************
// init_UART_tle9012 - Initialize communication with TLE9012
//****************************************************************************
void init_UART_tle9012()
{
	// Initialize analog frontend TLE9012 UART
	cy_rslt_t result_uart_tle = tle9012_initUART();
	if (result_uart_tle == CY_RSLT_SUCCESS){
		PRINTF_MAIN_DEBUG("Init UART TLE9012: %d\r\n", (int)result_uart_tle);
	}
	else{
		PRINTF_MAIN_DEBUG("Init UART TLE9012 failed: %d\r\n",
				(int)result_uart_tle);
	}
}

// Initialize the interrupt vector table with
// the timer interrupt handler address and assign priority.
cy_stc_sysint_t timer_watchdog_interrupt_config =
{
    /* Interrupt source is Timer interrupt */
    .intrSrc = TIMER_WATCHDOG_IRQ,

    /* Interrupt priority is 1 */
    .intrPriority = 1UL
};

//****************************************************************************
// init_CAN - Initialize CAN communication
//****************************************************************************
void init_CAN()
{
	/* Init CANFD */
	cy_en_canfd_status_t status_can = init_canfd();
	if (status_can != CY_CANFD_SUCCESS){ CY_ASSERT(0); }
}

//****************************************************************************
// init_Timer_RTC - Init millisecond counter timer and Real Time Clock
//****************************************************************************
void init_Timer_RTC()
{
	// Init RTC
	Cy_RTC_Init(&srss_0_rtc_0_config);

	// Enable 1msec & 1sec timers
	cy_en_tcpwm_status_t result_timer =
			Cy_TCPWM_Counter_Init(TIMER_1MS_HW, TIMER_1MS_NUM,
					&TIMER_1MS_config);
	if (CY_TCPWM_SUCCESS != result_timer){
		PRINTF_MAIN_DEBUG("Init TIMER_1MS failed: %d\r\n", (int)result_timer);
	}
	// Enable the initialized counter
	Cy_TCPWM_Counter_Enable(TIMER_1MS_HW, TIMER_1MS_NUM);
	// Then start the counter
	Cy_TCPWM_TriggerStart_Single(TIMER_1MS_HW, TIMER_1MS_NUM);

	// Init watchdog interrupt
	cy_rslt_t result =
			Cy_SysInt_Init(&timer_watchdog_interrupt_config,
					timer_watchdog_interrupt_handler);
	if(result != CY_SYSINT_SUCCESS){
		PRINTF_MAIN_DEBUG("Interrupt init TIMER_WATCHDOG failed: %d\r\n",
				(int)result);
	}
	/* Enable Interrupt */
	NVIC_EnableIRQ(timer_watchdog_interrupt_config.intrSrc);

	// Enable watchdog timer
	cy_en_tcpwm_status_t result_timer_wd =
			Cy_TCPWM_Counter_Init(TIMER_WATCHDOG_HW, TIMER_WATCHDOG_NUM,
					&TIMER_WATCHDOG_config);
	if (CY_TCPWM_SUCCESS != result_timer_wd){
		PRINTF_MAIN_DEBUG("Init TIMER_WATCHDOG failed: %d\r\n",
				(int)result_timer_wd);
	}
	// Enable the initialized counter
	Cy_TCPWM_Counter_Enable(TIMER_WATCHDOG_HW, TIMER_WATCHDOG_NUM);

	// Check if the desired interrupt is enabled prior to triggering
	if (0UL != (CY_TCPWM_INT_ON_TC &
			Cy_TCPWM_GetInterruptMask(TIMER_WATCHDOG_HW,
					TIMER_WATCHDOG_NUM))) {
		Cy_TCPWM_SetInterrupt(TIMER_WATCHDOG_HW,
				TIMER_WATCHDOG_NUM, CY_TCPWM_INT_ON_TC);
	}

    // Set the timer period in milliseconds.
	// To count N cycles, period should be set to N-1.
    Cy_TCPWM_Counter_SetPeriod(TIMER_WATCHDOG_HW, TIMER_WATCHDOG_NUM,
    		SOFTWARE_WATCHDOG_TIMEOUT-1 );

	// Then start the counter
	Cy_TCPWM_TriggerStart_Single(TIMER_WATCHDOG_HW, TIMER_WATCHDOG_NUM);
}

//****************************************************************************
// init_SPI_peripheral
// Startup and setup SPI used for all peripherals except display
//****************************************************************************
void init_SPI_peripheral()
{
	cy_rslt_t result;
	// Init hal SPI for 2ED4820
	// (does not matter for which device it is configured,
	// will reinitialized when needed for each device)
	result = cyhal_spi_init(&mSPI,PERI_SPI_MOSI,
							PERI_SPI_MISO,
							PERI_SPI_CLK,
							NC,NULL,8,CYHAL_SPI_MODE_01_MSB,false);

	// Set the SPI baud rate
	if (CY_RSLT_SUCCESS == result) {
		PRINTF_MAIN_DEBUG("Init SPI first %d\r\n", (int)(uint8_t)result);
		result = cyhal_spi_set_frequency(&mSPI, 5000000);
		if (CY_RSLT_SUCCESS != result) {
			PRINTF_MAIN_DEBUG("Failed to set SPI Frequency first %d\r\n",
					(int)(uint8_t)result);
		}
	}
	else{
		PRINTF_MAIN_DEBUG("Failed to init SPI first %d\r\n",
				(int)(uint8_t)result);
	}
	mSPI_curMode = CYHAL_SPI_MODE_01_MSB;

	// Init SPI CS 2ED4820
    result = cyhal_gpio_init(PERI_SPI_CS_2ED4820,
    		CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
	if (CY_RSLT_SUCCESS == result) {
		PRINTF_MAIN_DEBUG("\tInit CS pin 2ED4820: %d\r\n",
				(int)(uint8_t)result);
	}
	else{
		PRINTF_MAIN_DEBUG("\tFailed to init CS pin 2ED4820: %d\r\n",
				(int)(uint8_t)result);
	}

	// Init SPI CS MCP3465R
    result = cyhal_gpio_init(PERI_SPI_CS_MCP3465,
    		CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
	if (CY_RSLT_SUCCESS == result) {
		PRINTF_MAIN_DEBUG("\tInit CS pin ADC MCP3465: %d\r\n",
				(int)(uint8_t)result);
	}
	else{
		PRINTF_MAIN_DEBUG("\tFailed to init CS pin ADC MCP3465: %d\r\n",
				(int)(uint8_t)result);
	}

	// Init SPI CS CY15B256
    result = cyhal_gpio_init(PERI_SPI_CS_CY15B,
    		CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
	if (CY_RSLT_SUCCESS == result) {
		PRINTF_MAIN_DEBUG("\tInit CS pin MEM CY15B256: %d\r\n",
				(int)(uint8_t)result);
	}
	else{
		PRINTF_MAIN_DEBUG("\tFailed to init CS pin MEM CY15B256: %d\r\n",
				(int)(uint8_t)result);
	}

	// Init enable pin
    result = cyhal_gpio_init(D_2ED4820_ENABLE,
    		CYHAL_GPIO_DIR_BIDIRECTIONAL, CYHAL_GPIO_DRIVE_STRONG, 0);
	if (CY_RSLT_SUCCESS == result) {
		PRINTF_MAIN_DEBUG("\tInit Enable pin 2ED4820: %d\r\n",
				(int)(uint8_t)result);
	}
	else{
		PRINTF_MAIN_DEBUG("\tFailed to init Enable pin 2ED4820: %d\r\n",
				(int)(uint8_t)result);
	}

	// Init safestate pin
	result = cyhal_gpio_init(D_2ED4820_SAVE_STATE_EN,
			CYHAL_GPIO_DIR_BIDIRECTIONAL, CYHAL_GPIO_DRIVE_PULLDOWN, 0);
	if (CY_RSLT_SUCCESS == result) {
		PRINTF_MAIN_DEBUG("\tInit SaveState pin 2ED4820: %d\r\n",
				(int)(uint8_t)result);
	}
	else{
		PRINTF_MAIN_DEBUG("\tFailed to init SaveState pin 2ED4820: %d\r\n",
				(int)(uint8_t)result);
	}
}

//****************************************************************************
// 2ED4820 Interrupt function, object and init
//****************************************************************************
// Interrupt handler callback function
static void IRQ_ERR_2ED4820(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
	CY_UNUSED_PARAMETER(event);

	// Directly set the safe state (no main loop interaction required)
	if(SafetySwitchInterrupt_SaveStateSet == 0){
		setSafestate(SET_SAFE_STATE_ON);
		cyhal_system_delay_us(100);
		setSafestate(SET_SAFE_STATE_OFF);
		SafetySwitchInterrupt_SaveStateSet = 1;
	}

	// Mark error
	errorInterruptDetected_2ED4820 = 1;
}
// Interrupt object
cyhal_gpio_callback_data_t gpio_2ED4820_callback_data = {
	.callback     = IRQ_ERR_2ED4820
};

// Interrupt initialization function
void init_interrupt_2ED4820()
{
	cy_rslt_t rslt;
	rslt = cyhal_gpio_init(D_2ED4820_INT, CYHAL_GPIO_DIR_INPUT,
			CYHAL_GPIO_DRIVE_PULLDOWN, 0);
	if(CY_RSLT_SUCCESS != rslt){
		PRINTF_MAIN_DEBUG("[ERROR]: init_interrupt_2ED4820 error %ld\r\n",
				rslt);
	}
	else{
		PRINTF_MAIN_DEBUG("init_interrupt_2ED4820 done\r\n");
	}
	// Register callback function
	// gpio_interrupt_handler and pass the value global_count
	cyhal_gpio_register_callback(D_2ED4820_INT, &gpio_2ED4820_callback_data);

	// Enable rising edge interrupt event with interrupt priority set to 3
    cyhal_gpio_enable_event(D_2ED4820_INT, CYHAL_GPIO_IRQ_RISE,
    		CYHAL_ISR_PRIORITY_DEFAULT, 1);
}

//****************************************************************************
// TLE9012 Interrupt function, object and init
//****************************************************************************
// Interrupt handler callback function
static void IRQ_ERR_TLE9012(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
	CY_UNUSED_PARAMETER(event);

	// Directly set the safe state (no main loop interaction required)
	if(AnalogFrontendInterrupt_SaveStateSet == 0){
		setSafestate(SET_SAFE_STATE_ON);
		cyhal_system_delay_us(100);
		setSafestate(SET_SAFE_STATE_OFF);
		AnalogFrontendInterrupt_SaveStateSet = 1;
	}

	// Mark error
	errorInterruptDetected_TLE9012 = 1;
}

// Interrupt object
cyhal_gpio_callback_data_t gpio_TLE9012_callback_data = {
	.callback     = IRQ_ERR_TLE9012
};

// Interrupt initialization function
void init_interrupt_TLE9012()
{
	cy_rslt_t rslt;
	rslt = cyhal_gpio_init(TLE9012_ERR, CYHAL_GPIO_DIR_INPUT,
			CYHAL_GPIO_DRIVE_PULLDOWN, 0);
	if(CY_RSLT_SUCCESS != rslt){
		PRINTF_MAIN_DEBUG("[ERROR]: init_interrupt_TLE9012 error %ld\r\n",
				rslt);
	}
	else{
		PRINTF_MAIN_DEBUG("init_interrupt_TLE9012 done\r\n");
	}
	// Register callback function
	// gpio_interrupt_handler and pass the value global_count
	cyhal_gpio_register_callback(TLE9012_ERR, &gpio_TLE9012_callback_data);

	// Enable falling edge interrupt event with interrupt priority set to 3
    cyhal_gpio_enable_event(TLE9012_ERR, CYHAL_GPIO_IRQ_RISE,
    		CYHAL_ISR_PRIORITY_DEFAULT, true);
}

//****************************************************************************
// init_internal_ADC - This function initializes analog components - SAR ADC
//****************************************************************************
void init_internal_ADC()
{
    /* Variable to capture return value of functions */
    cy_rslt_t result;

    /* Initialize AREF */
    result = Cy_SysAnalog_Init(&pass_0_aref_0_config);
    if (CY_SYSANALOG_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Enable AREF */
    Cy_SysAnalog_Enable();

    /* Initialize common resources for SAR ADCs. */
    /* Common resources include simultaneous trigger parameters, scan count
       and power up delay. This is configured in the device configurator. */
	//    result = Cy_SAR_CommonInit(PASS, &pass_0_saradc_0_config);
	//    if (CY_SAR_SUCCESS != result)
	//    {
	//        CY_ASSERT(0);
	//    }

    /* Initialize SAR_HW and SAR1 */
    result = Cy_SAR_Init(SAR_HW, &SAR_config );
    if (CY_SAR_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Enable SAR block */
    Cy_SAR_Enable(SAR_HW);

    //Cy_SAR_SetInterruptMask(SAR_HW, CY_SAR_INTR);

    //(void)Cy_SysInt_Init(&SAR_HW_IRQ_cfg, SAR_HW_interrupt);

    /* Enable the SAR interrupts */
    //NVIC_EnableIRQ(SAR_HW_IRQ_cfg.intrSrc);

    //Cy_SAR_StartConvert(SAR_HW, CY_SAR_START_CONVERT_CONTINUOUS);

	// Read slot state for the first time
    Cy_SAR_StartConvert(SAR_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
    cyhal_system_delay_ms(50);
    manage_internal_ADC();
	Cy_SAR_StartConvert(SAR_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
	cyhal_system_delay_ms(10);
}

//****************************************************************************
// init_external_ADC_MCP3465 - Initialize external ADC chip
//****************************************************************************
void init_external_ADC_MCP3465()
{
	// Write all configuration registers
    PRINTF_MAIN_DEBUG("Init SPI ADC MCP3465:\r\n");
    PRINTF_MAIN_DEBUG("\tADC Write  _CONFIG0_: %X\r\n", CONFIG0_CFG);
    SPI_WRT(_CONFIG0_, CONFIG0_CFG);
    cyhal_system_delay_ms(10);
    PRINTF_MAIN_DEBUG("\tADC Write  _CONFIG1_: %X\r\n", CONFIG1_CFG);
    SPI_WRT(_CONFIG1_, CONFIG1_CFG);
    cyhal_system_delay_ms(10);
    PRINTF_MAIN_DEBUG("\tADC Write  _CONFIG2_: %X\r\n", CONFIG2_CFG);
    SPI_WRT(_CONFIG2_, CONFIG2_CFG);
    cyhal_system_delay_ms(10);
    PRINTF_MAIN_DEBUG("\tADC Write  _CONFIG3_: %X\r\n", CONFIG3_CFG);
    SPI_WRT(_CONFIG3_, CONFIG3_CFG);
    cyhal_system_delay_ms(10);

    // Initialize first measurement on channel 1
    activeChannelExternalADC = ACTIVE_EXT_ADC_CH1_LOAD_VOLTAGE;
    CONV_START(MUX_CFG_CH1_SINGLE_END);

	// Initialize MCP3465 external ADC and
    // wait till enough measurements are done
    // to ensure correct filtering
	isCurrentOffsetCalibrating = 1;
	uint16_t readSuccessfulCount = 0;
	adcZeroingTimestamp = TIMER_COUNTER_1MS; // Store timestamp for timeout

	// Try to read results until wanted number of reads are read,
	// timeout occurred (used for loop to avoid while(true))
	for(uint16_t i = 0; i < UINT16_MAX; i++){
		// Trigger potential read
		uint8_t readChannelresult = manage_external_ADC_MCP3465(0);

		// Log how often a valid result was received
		if(readChannelresult != 0){
			readSuccessfulCount++;
		}

		// If amount of needed results is reached stop load
		if(readSuccessfulCount >= ADC_EXT_ZEROING_NUM){
			PRINTF_MAIN_DEBUG("\t\tExternal ADC read %d times for zeroing. "
					"Read successful after %ld ms\r\n",
					readSuccessfulCount,
					TIMER_COUNTER_1MS-adcZeroingTimestamp);
			return;
		}

		// Stop loop if takes to long
		if(TIMER_COUNTER_1MS >
			(adcZeroingTimestamp + ADC_EXT_ZEROING_TIMEOUT_TIME)){
			PRINTF_MAIN_DEBUG("[ERROR]: External ADC read %d times "
					"for zeroing. Read took too long (Timeout)\r\n",
					readSuccessfulCount);
			// Mark permanent error (if this flag is set
			// manage_external_ADC_MCP3465 will keep the timeout and
			// therefore the error triggered even if it disappears)
			initExtADCError = 1;
			return;
		}

		// Limit speed of result request
		cyhal_system_delay_ms(ADC_EXT_ZEROING_DELAY);
	}

	// Mark permanent error
	initExtADCError = 1;
}

//****************************************************************************
// init_hotSwap - Determine if the BMS shall start normally
// (alone in system, in charger or not in slot)
// in HotSwap MAIN mode or in HotSwap TAKER role
//****************************************************************************
void init_hotSwap()
{
	// Print debug messages
	PRINTF_MAIN_DEBUG("HotSwap Role: \r\n");

	// Init hot swap state check variables
	uint8_t  wasSlotStateConstantOnASystem = 1;
	uint8_t  wasExternalVoltageAboveThreshold = 0;
	uint8_t  wasSwitchOnVetoReceived = 0;
	uint16_t switchOnVetoReceivedSocOtherBMS = 0;
	uint8_t  switchOnVetoReceivedActiveStateOtherBMS = 0;
	uint16_t switchOnVetoSendSuccessCounter = 0;

	cy_en_canfd_status_t can_status;

	// Get initial slot state
	manage_slotState(1);
	slotStates hotSwap_initialSlotState = statusBMS.slotState;

	// Store SoC in CAN data, to be send with switchOnVeto
	uint8_t targetData[6];
	targetData[0] = (uint8_t)statusBMS.SoC;
	targetData[1] = 0; // Set active state off

	// Hotswap state acquisition
	uint32_t hotSwapInitStartTimestamp = TIMER_COUNTER_1MS;
	while(TIMER_COUNTER_1MS <
			(hotSwapInitStartTimestamp + HOTSWAP_INIT_SWITCHON_VETO_TIME)){
		// Check if slot state stays the same
		manage_slotState(1);
		if(statusBMS.slotState != hotSwap_initialSlotState){
			wasSlotStateConstantOnASystem = 0;
		}
		// Check if a switch on veto was received
		if(receivedCanSwitchOnVetoCount != 0){
			wasSwitchOnVetoReceived = 1;
			switchOnVetoReceivedSocOtherBMS = receivedCanSwitchOnVetoSoC;
			// Used OR operator to make variable true
			// even if it was 1 only once
			switchOnVetoReceivedActiveStateOtherBMS |=
					receivedCanSwitchOnVetoActiveState;
		}
		// Check if a external load voltage was detected
		// (other BMS is supplying)
		if(statusBMS.extLoadVoltage > EXT_LOAD_VOLTAGE_THRESHOLD_V){
			wasExternalVoltageAboveThreshold = 1;
		}

		// Send switch on veto
		if((TIMER_COUNTER_1MS > (canLastMessageTimestamp +
				CAN_MINIMAL_MESSAGE_TIME + (rand() % 4)))){
			can_status = 1;
			if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
				can_status = CAN_TX_Request(BMS_SLOT_1_SWITCH_ON_VETO,
						targetData, 2);
			}
			else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
				can_status = CAN_TX_Request(BMS_SLOT_2_SWITCH_ON_VETO,
						targetData, 2);
			}
			if(can_status == CY_CANFD_SUCCESS){
				switchOnVetoSendSuccessCounter++;
			}
		}

		// Keep analog frontend TLE9012 communication alive
		manage_analogFrontendTLE9012();

		// Limit loop time
		cyhal_system_delay_ms(25);
	}

	// Print result of acquisition
	PRINTF_MAIN_DEBUG("\tInitial slot state %d stayed constant = %d. "
			"External load voltage above threshold = %d. "
			"Switch on veto send successful = %d (own SoC %d). "
			"Switch on veto received = %d (SoC other BMS = %d)\r\n",
			hotSwap_initialSlotState, wasSlotStateConstantOnASystem,
			wasExternalVoltageAboveThreshold, switchOnVetoSendSuccessCounter,
			targetData[0], wasSwitchOnVetoReceived,
			switchOnVetoReceivedSocOtherBMS);

	// Check in which state the BMS should start.
	// The first two condition are used to detect definitive states,
	// the rest detects special states on to of them
	// Shutdown condition - slot changed during init
	if(wasSlotStateConstantOnASystem == 0){
		PRINTF_MAIN_DEBUG("\tState can not be determined - shutdown system. "
				"Slot state did not stay constant during hotswap init\r\n");
		statusBMS.hotSwapRole = HOTSWAP_ERROR;
		shutdownSystem();
	}
	// Explicit TAKER condition - Always wait for takeover if
	// external load voltage is detected
	else if(wasExternalVoltageAboveThreshold == 1 &&
			(hotSwap_initialSlotState == SLOT_IN_SYSTEM_1 ||
			hotSwap_initialSlotState == SLOT_IN_SYSTEM_2)){
		PRINTF_MAIN_DEBUG("\tState explicitly TAKER. "
				"Slot state stayed constant, but external "
				"load voltage was detected\r\n");
		statusBMS.hotSwapRole = HOTSWAP_TAKER;
	}
	// Explicit MAIN/GIVER condition - No switchOnVeto CAN message received
	// and not able to send any (there is no second BMS)
	else if((hotSwap_initialSlotState != SLOT_IN_SYSTEM_1) &&
			(hotSwap_initialSlotState != SLOT_IN_SYSTEM_2)){
		PRINTF_MAIN_DEBUG("\tState explicitly MAIN. "
				"Slot state stayed constant and no external "
				"load voltage was detected, "
				"but system is not in a system slot\r\n");
		statusBMS.hotSwapRole = HOTSWAP_MAIN;
	}
	else if(wasSwitchOnVetoReceived == 0 &&
			switchOnVetoSendSuccessCounter <= 1){
		PRINTF_MAIN_DEBUG("\tState explicitly MAIN. "
				"Slot state stayed constant and no external "
				"load voltage was detected, "
				"but no switch ON veto received and "
				"switch ON veto sending was not successful\r\n");
		statusBMS.hotSwapRole = HOTSWAP_MAIN;
	}
	// TAKER condition, second BMS startup - A second BMS is active
	// but did not enable output yet
	else if(switchOnVetoReceivedActiveStateOtherBMS == 1){
		PRINTF_MAIN_DEBUG("\tState second BMS TAKER. "
				"Slot state stayed constant, no external "
				"load voltage was detected, switch ON veto received, "
				"but other BMS sends active flag\r\n");
		statusBMS.hotSwapRole = HOTSWAP_TAKER;
	}
	// Both BMS want to be active - determine hotswap role
	else if(wasSwitchOnVetoReceived == 1 &&
			switchOnVetoSendSuccessCounter > 1){
		if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
			PRINTF_MAIN_DEBUG("\tState MAIN. Slot state stayed constant "
					"and no external load voltage was detected, "
					"switch ON veto received, "
					"switch ON veto sending successful, "
					"but BMS is in main slot 1\r\n");
			statusBMS.hotSwapRole = HOTSWAP_MAIN;
		}
		else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
			PRINTF_MAIN_DEBUG("\tState TAKER. Slot state stayed constant "
					"and no external load voltage was detected, "
					"switch ON veto received, "
					"switch ON veto sending successful, "
					"but BMS is in main slot 2\r\n");
			statusBMS.hotSwapRole = HOTSWAP_TAKER;
		}
		else{
			PRINTF_MAIN_DEBUG("\t[ERROR]: BMS not in system\r\n");
			statusBMS.hotSwapRole = HOTSWAP_ERROR;
			shutdownSystem();
		}
	}
	else{
		PRINTF_MAIN_DEBUG("\t[ERROR]: HotSwap init encountered "
				"not handled condition, "
				"make sure that there is no CAN sniffer active "
				"(would ack switch on veto messages but does not reply)\r\n");
		statusBMS.hotSwapRole = HOTSWAP_ERROR;
		shutdownSystem();
	}

	// If role allows switch on:
	// send a first switch on veto message with value set to max
	// to signal that this BMS will try to switch on
	if(statusBMS.hotSwapRole == HOTSWAP_MAIN){
		cyhal_system_delay_ms(5);
		targetData[0] = 255;
		targetData[1] = 1; // Set active state off

		// Try to send message 3 times
		can_status = 1;
		for(int i = 0; i < 3; i++){
			if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
				can_status = CAN_TX_Request(BMS_SLOT_1_SWITCH_ON_VETO,
						targetData, 2);
			}
			else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
				can_status = CAN_TX_Request(BMS_SLOT_2_SWITCH_ON_VETO,
						targetData, 2);
			}
			cyhal_system_delay_ms(CAN_MINIMAL_MESSAGE_TIME + (rand() % 4));

			//
			if(can_status == CY_CANFD_SUCCESS){
				PRINTF_MAIN_DEBUG("\tSent switch ON veto successful\r\n");
				break;
			}
		}

		// Reset CAN message counter so that system is in known state
		// (especially switch on veto counter would trigger
		// ERR_INT_SWITCH_ON_VETO_RECEIVED if not reset)
		resetCanMessageCounters();
	}
}

void init_batteryState()
{
	PRINTF_MAIN_DEBUG("Init battery state monitoring:\r\n");
	// Init battery state detection (SoC & SoH)
	statusBMS.SoC = init_battery_state(&batteryState,
			setupBMS.BatteryRatedCapacity,
			statusBMS.voltage, setupBMS.NumCells);
	// For the time being - will later be replaced with a stored SoH value
	statusBMS.SoH = batteryState.SoH_t0;
	PRINTF_MAIN_DEBUG("\tSoC_t0 %.3f, capacity_init %.3fmAh, "
			"voltage_init %.3f, SoH_lower_threshold_V %.3f, "
			"SoH_upper_threshold_V %.3f, capacity_range_SoH %.3f\r\n",
			batteryState.SoC_t0, batteryState.capacity_init/3.6,
			batteryState.voltage_init, batteryState.SoH_lower_threshold_V,
			batteryState.SoH_upper_threshold_V,
			batteryState.capacity_range_SoH);
}

/****************************************************************************
 * init_analogFrontendTLE9012
 * Initialize communication with the analog frontend TLE9012, configure it,
 * read initial data and refresh tle9012 watchdog
 * so it does not turn off before main loop starts
 ***************************************************************************/
requestResult init_analogFrontendTLE9012()
{
	PRINTF_MAIN_DEBUG("TLE9012 analog frontend:\r\n");

	// Wake devices and init ID etc
	//requestResult tle9012_result;
	tle9012_deviceInit(0);
	cyhal_system_delay_ms(10);

	/// TLE9012 initial configuration
	// PART_CONFIG: Enable monitoring for all cells
	TLE9012_DETECT_COM_ERROR(
			tle9012_writeRegisterByIndex(1, PART_CONFIG, 0x0FFF, 1, 0),
			tle9012_writeRegisterByIndex_PART_CONFIG , SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// MEAS_CTRL: Configure and trigger measurement
	// of Block and Primary Cell Voltage
	TLE9012_DETECT_COM_ERROR(
			tle9012_writeRegisterByIndex(1, MEAS_CTRL, CVM_DEL__1 |
			PBOFF__BAL_INTERRUPT | BVM_MODE__16_BIT | BVM_START__TRIG_MEAS |
			CVM_MODE__16_BIT | PCVM_START__TRIG_MEAS, 1, 0),
			tle9012_writeRegisterByIndex_MEAS_CTRL , SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// OP_MODE: Extended watchdog and Diagnostics currents
	// enabled for all channels WARNING:
	// SOME SETTINGS MIGHT RESULT IN THE SYSTEM NOT STARTING PROPERLY IF
	// THEY ARE NOT MANUALLY RESET BEFORE SHUTDOWN
	TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(1,
			OP_MODE, PD__NORMAL_OP | EXT_WD__NOT_ACTIVE |
			SLEEP_REG_RESET__NORMAL_OP | I_DIAG_EN__OFF |
			LR_TIME__DEFAULT, 1, 0),
			tle9012_writeRegisterByIndex_OP_MODE , SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// FAULT_MASK: Decide which errors trigger the ERR pin to rise and TLE9012 interrupt to trigger
	TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(1,
			FAULT_MASK, ERR_PIN__ENABLE | M_ADC_ERR__ENABLE |
			M_OL_ERR__ENABLE | M_INT_IC_ERR__ENABLE | M_REG_CRC_ERR__ENABLE |
			M_EXT_T_ERR__ENABLE | M_INT_OT__ENABLE | M_CELL_UV__ENABLE |
			M_CELL_OV__ENABLE | M_BAL_ERR_UC__ENABLE |
			M_NR_ERR_BAL_OC__ENABLE, 1, 0),
			tle9012_writeRegisterByIndex_FAULT_MASK , SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// OL_OV_THR:
	// Cell over-voltage and maximum open loop voltage drop threshold
	uint16_t ov_thr =
			(uint16_t)((((float)setupBMS.LimitOverVoltageCell)/1000.0)/
					CELL_OVUV_LSB);
	uint16_t ol_thr_max = (uint16_t)(OPEN_LOOP_MAX_V_THRESHOLD/CELL_OVUV_LSB);

	/*PRINTF_MAIN_DEBUG("Set ov_thr %d, ol_thr_max %d => %d\r\n",
			ov_thr, ol_thr_max, ov_thr | (ol_thr_max  << 10));*/
	TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(1,
			OL_OV_THR, ov_thr | (ol_thr_max  << 10), 1, 0),
			tle9012_writeRegisterByIndex_OL_OV_THR , SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// OL_UV_THR:
	// Cell under-voltage and minimum open loop voltage drop threshold
	uint16_t uv_thr =
			(uint16_t)((((float)(setupBMS.LimitUnderVoltageCell))/1000.0)/
					CELL_OVUV_LSB);
	uint16_t ol_thr_min = (uint16_t)(OPEN_LOOP_MIN_V_THRESHOLD/CELL_OVUV_LSB);

	/*PRINTF_MAIN_DEBUG("Set uv_thr %d, ol_thr_min %d => %d\r\n",
			uv_thr, ol_thr_min, uv_thr | (ol_thr_min  << 10));*/
	TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(1,
			OL_UV_THR, uv_thr | (ol_thr_min  << 10), 1, 0),
			tle9012_writeRegisterByIndex_OL_UV_THR , SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// TEMP_CONF: Enable all temperature sensors
	uint16_t e_overtmp_th =
			tle9012_convertExternalTemperatureThreshold(
					TLE9012_EXTERNAL_OVERTEMP_C, I_NTC__I_0);
	PRINTF_MAIN_DEBUG("\tSet external overtemperature threshold to %d\r\n",
			e_overtmp_th);
	//uint16_t e_overtmp_th = 0;
	TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(1,
			TEMP_CONF, e_overtmp_th | I_NTC__I_0 |
			NR_TEMP_SENSE__TMP0_4, 1, 0),
			tle9012_writeRegisterByIndex_TEMP_CONF , SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// INT_OT_WARN_CONF: Configure internal overtemperature
	TLE9012_DETECT_COM_ERROR(
			tle9012_writeRegisterByIndex(1, INT_OT_WARN_CONF,
			(-TLE9012_INTERNAL_OVERTEMP_C+547.3F)/TEMP_INT_LSB, 1, 0),
			tle9012_writeRegisterByIndex_INT_OT_WARN_CONF ,
			SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// BAL_PWM: Set the balancing PWM to 0 (off)
	TLE9012_DETECT_COM_ERROR(
			tle9012_writeRegisterByIndex(1, BAL_PWM, 0, 1, 0),
			tle9012_writeRegisterByIndex_BAL_PWM , SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// BAL_CURR_THR:
	TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(1,
			BAL_CURR_THR, OC_THR__DEFAULT | UC_THR__DEFAULT , 1, 0),
			tle9012_writeRegisterByIndex_BAL_CURR_THR ,
			SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// BAL_SETTINGS: Disable all balancing switches
	TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(
			1, BAL_SETTINGS, 0, 1, 0),
			tle9012_writeRegisterByIndex_BAL_SETTINGS ,
			SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// RR_CONFIG: Enable error counter for all errors except
	// balancing overcurrent (as per default)
	TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(1, RR_CONFIG,
			RR_CNT__RR_36 |
			RR_SYNC__NO_SYNC |
			M_NR_ERR_ADC_ERR__DISABLE |
			M_NR_ERR_OL_ERR__DISABLE |
			M_NR_ERR_EXT_T_ERR__DISABLE |
			M_NR_ERR_INT_OT__DISABLE |
			M_NR_ERR_CELL_UV__DISABLE |
			M_NR_ERR_CELL_OV__DISABLE |
			M_NR_ERR_BAL_UC__DISABLE |
			M_NR_ERR_BAL_OC__ENABLE, 1, 0),
			tle9012_writeRegisterByIndex_RR_CONFIG , SUCCESS_FUNCTION_NONE);
	if (tle9012_print_if_error_result >= REQUEST_ERROR){
		return tle9012_print_if_error_result;
	}

	// Set the multi-read configuration
	requestResult res =
			tle9012_writeMultiReadConfig(1, PCVM_SEL__RES_CELL_11_0,
			BVM_SEL__RESULT, EXT_TEMP_SEL__RES_TMP0_4,
			EXT_TEMP_R_SEL__NO_R_DIAG_RES, INT_TEMP_SEL__INT_TMP_RES,
			SCVM_SEL__RESULT, STRESS_PCVM_SEL__RESULT, 1 );
	if (res >= REQUEST_ERROR){
		return res;
	}

	// Leave time for settings to take effect
	cyhal_system_delay_ms(300); //90

	// Read all registers to cache
	TLE9012_DETECT_COM_ERROR( tle9012_readAllRegistersToCache(1, 1, 0),
			tle9012_readAllRegistersToCache , SUCCESS_FUNCTION_NONE);

	/// Print configuration registers
	tle9012_printRegister(1, OP_MODE);
	tle9012_printRegister(1, RR_CONFIG);
	tle9012_printRegister(1, BAL_CURR_THR);
	tle9012_printRegister(1, MEAS_CTRL);
	tle9012_printRegister(1, CONFIG_MAIN);
	tle9012_printRegister(1, FAULT_MASK);
	tle9012_printRegister(1, MULTI_READ_CFG);

	/// Diagnostic: Read, write and print register for first time
	// Read and print errors
	TLE9012_DETECT_COM_ERROR( tle9012_readRegisterByIndex(1, GEN_DIAG, 1, 0),
			tle9012_readRegisterByIndex_GEN_DIAG , SUCCESS_FUNCTION_NONE);
	tle9012_printRegister(1, GEN_DIAG);
	// Clear tle9012 errors for the first time
	TLE9012_DETECT_COM_ERROR(
			tle9012_writeRegisterByIndex(1, GEN_DIAG, 0x0000, 1, 0),
			tle9012_writeRegisterByIndex_GEN_DIAG , SUCCESS_FUNCTION_NONE);

	// Convert BVM, PCVM, TEMP values and print them
	TLE9012_DETECT_COM_ERROR( tle9012_convertPrimaryCellVoltages(1, 0, 11, 0),
			tle9012_convertPrimaryCellVoltages , SUCCESS_FUNCTION_NONE);
	TLE9012_DETECT_COM_ERROR( tle9012_convertBlockVoltage(1, 0),
			convertBlockVoltage , SUCCESS_FUNCTION_NONE);
	TLE9012_DETECT_COM_ERROR( tle9012_convertTemperatures(1, 4, 1),
			tle9012_convertTemperatures , SUCCESS_FUNCTION_NONE);
	TLE9012_DETECT_COM_ERROR(
			tle9012_convertInternalTemperatureByIndex(1, 0, 0),
			tle9012_convertInternalTemperatures_0 , SUCCESS_FUNCTION_NONE);
	TLE9012_DETECT_COM_ERROR(
			tle9012_convertInternalTemperatureByIndex(1, 1, 0),
			tle9012_convertInternalTemperatures_1 , SUCCESS_FUNCTION_NONE);

	// Print voltage and temperature
	PRINTF_MAIN_DEBUG("Initial TLE9012 measurements: \r\n");
	PRINTF_MAIN_DEBUG("\tBVM:    %2.3lf V \r\n",
			tle9012_nodes[0].block_voltage);
	for(uint8_t i = 0; i < NUM_CELLS; i++){
		PRINTF_MAIN_DEBUG("\tPCVM:    %2.3lf V \r\n",
				(double)(tle9012_nodes[0].cell_voltages[i])/1000.0);
		sh_pub_cells[i] = tle9012_nodes[0].cell_voltages[i];
	}
	for(int i = 0; i <= 4; i++){
		PRINTF_MAIN_DEBUG("\tTemp %d: %2.3lf C \r\n",
				i, tle9012_nodes[0].temperatures[i]);
	}
	PRINTF_MAIN_DEBUG("\tTempInt 0: %2.3lf C \r\n",
			tle9012_nodes[0].temperaturesInternal[0]);
	PRINTF_MAIN_DEBUG("\tTempInt 1: %2.3lf C \r\n",
			tle9012_nodes[0].temperaturesInternal[1]);

	// Set initial values
	sh_pub_volt = (uint16_t)(tle9012_nodes[0].block_voltage*1000.0);
	statusBMS.voltage = (float)tle9012_nodes[0].block_voltage;
	// Calculate the average temperature between cell temperature sensors
	statusBMS.avgTemp = (float)(tle9012_nodes[0].temperatures[0] +
								tle9012_nodes[0].temperatures[3])/2;
	statusBMS.avgCellVolt_mV = UINT16_MAX;

	// Send a watchdog update because the next steps till the main loop enters
	// could take longer than wake time.
	// NOTE: This is only allowed here because the last command
	// was executed blocking and it is therefore sure
	// that no data can be received during the update
	// (see comment in sendWatchdogUpdate())
	cyhal_system_delay_ms(300); // Todo: This delay might not be needed
	sendWatchdogUpdate(1, 1);

	// Success
	return REQUEST_OK;
}

/****************************************************************************
 * processMeasurements
 * Manages the measurement and conversion of analog front-end
 * (cell voltage, block voltage temperatures and everything in multiread),
 * external ADC (current and load voltage) and internal ADC.
 * Note that the current measurement is triggered in the same cycle
 * as the analog front-end measurements, as long as they are not slower
 ***************************************************************************/
void processMeasurements()
{
	// Reset flag that notices rest of code that the result are ready
	isTLE9012ResultReceivedThisCycle = 0;

	// TLE9012 analog frontend measurement processing state machine ->
	// Cell/block voltage, temperature, diagnostic, etc.
	// (see call of tle9012_writeMultiReadConfig() in init)
	requestResult tle9012_result;
	switch (statusBMS.measurementState) {
		case MEASUREMENT_WAIT_RESULT:
			// Check the MEAS_CTRL control register until the trigger bit
			// is gone (in this case the measurement is finished)

			// If MEAS_CTRL is not queued anymore read is finished
			if(checkRegisterQueued_tle9012(1, MEAS_CTRL, 0) == 0){
				// If the PCVM and BVM Start measurement bit is gone
				// trigger MULTI_READ, else trigger next read of MEAS_CTRL
				if((getRegisterCacheDataByIndex_tle9012(1, MEAS_CTRL) &
						(MEAS_CTRL__PCVM_START.mask |
								MEAS_CTRL__BVM_START.mask)) == 0){
					PRINTF_MEASUREMENT("Measurement finished - "
							"Queue next multi read\r\n");

					// BALANCING: Write balancing map here as early as possible
					// after measurement deactivated it, so it can balance
					// till next measurement is triggered
					TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(1,
							BAL_SETTINGS, statusBMS.bal_cell_map, 1, 0),
							tle9012_writeRegisterByIndex_BAL_SETTINGS ,
							SUCCESS_FUNCTION_NONE);

					// Trigger multi-read of necessary registers
					tle9012_result =
							tle9012_readRegisterByIndex(1, MULTI_READ, 0, 0);
					if(tle9012_result >= REQUEST_ERROR){
						PRINTF_MAIN_DEBUG("[ERROR]: "
								"Trigger next multi read returned %d\r\n",
								tle9012_result);
					}
					// Change State to wait for the result
					statusBMS.measurementState = MEASUREMENT_READ_RESULT;
				}
				// If the start bits are not gone yet trigger another read
				else{
					PRINTF_MEASUREMENT("Measurement not finished - "
							"Queue measurement control again\r\n");

					// Read measurement control register again till start bit is gone
					tle9012_result =
							tle9012_readRegisterByIndex(1, MEAS_CTRL, 0, 0);
					if(tle9012_result >= REQUEST_ERROR){
						PRINTF_MAIN_DEBUG("[ERROR]: "
								"Read MEAS_CTRL returned %d\r\n",
								tle9012_result);
					}
				}
			}
			return;

			break;
		case MEASUREMENT_READ_RESULT:
			// If the minimal sample time is exceeded trigger next measurement
			// MULTI READ TLE9012:  Read and evaluate voltages
			if(checkRegisterQueued_tle9012(1, MULTI_READ, 0) == 0){
				PRINTF_MEASUREMENT("Multi read done\r\n");

				// Convert latest raw data and write converted data to display
				// BVM - Voltage of pack and write the result to statusBMS
				// if successful
				TLE9012_DETECT_COM_ERROR(tle9012_convertBlockVoltage(1, 1),
						tle9012_convertBlockVoltage,
						statusBMS.voltage =
								(float)tle9012_nodes[0].block_voltage);

				// PCVM - Primary cell voltage = voltage of each cell
				TLE9012_DETECT_COM_ERROR(
						tle9012_convertPrimaryCellVoltages(1, 0, 11, 1),
						tle9012_convertPrimaryCellVoltages,
						SUCCESS_FUNCTION_NONE);

				// External Temperature
				TLE9012_DETECT_COM_ERROR(tle9012_convertTemperatures(1, 4, 1),
						tle9012_convertTemperatures, SUCCESS_FUNCTION_NONE);

				// Internal Temperature.
				// NOTE: Only the first internal temperature sensor can be read
				// with multi-read. Therefore the second one is ignored.
				TLE9012_DETECT_COM_ERROR(
						tle9012_convertInternalTemperatureByIndex(1, 0, 1),
						tle9012_convertInternalTemperatureByIndex_0 ,
						SUCCESS_FUNCTION_NONE);
				/*TLE9012_DETECT_COM_ERROR(
						tle9012_convertInternalTemperatureByIndex(1, 1, 1),
						tle9012_convertInternalTemperatureByIndex_1 ,
						SUCCESS_FUNCTION_NONE); */

				// Calculate the average temperature between
				// cell temperature sensors
				float temp = (tle9012_nodes[0].temperatures[0] +
							  tle9012_nodes[0].temperatures[3])/2;
				// Filter temperature and store it
				measure_movAvgFilter_fast(&avgFilter_tempCell, temp);
				statusBMS.avgTemp = avgFilter_tempCell.filterResult;

				// Calculate the average cell voltage
				statusBMS.avgCellVolt_mV =
						(uint16_t)((statusBMS.voltage/(float)NUM_CELLS)*1000);

				// Change State to wait for the result
				statusBMS.measurementState = MEASUREMENT_TRIGGER_NEXT;
				//return;
			}
			break;
		case MEASUREMENT_TRIGGER_NEXT:
			PRINTF_MEASUREMENT("Trigger next measurement\r\n");

			// Wait till the next measurement interval is reached.
			// Note that there is a different interval while charging
			// because the trigger is deactivating the balancing.
			// Therefore the effectiveness of the balancing is improved
			// by measuring slower while charging.
			if( (statusBMS.bms_state == BMS_CHARGING && (TIMER_COUNTER_1MS >
				(lastTLE9012MeasurementTimestamp +
						TLE9012_MEASURE_CHARGE_TIME))) ||
				(statusBMS.bms_state != BMS_CHARGING && (TIMER_COUNTER_1MS >
				(lastTLE9012MeasurementTimestamp + TLE9012_MEASURE_TIME))) ){
				/*printf("Measurement time: %ud\r\n",
						TIMER_COUNTER_1MS-lastTLE9012MeasurementTimestamp);*/
				lastTLE9012MeasurementTimestamp = TIMER_COUNTER_1MS;

				// Mark that the result of the measurement can be used
				// this cycle. This will trigger e.g. manage of balancing
				isTLE9012ResultReceivedThisCycle = 1;

				// Trigger next measurement and set
				TLE9012_DETECT_COM_ERROR( tle9012_writeRegisterByIndex(1,
						MEAS_CTRL, (CVM_DEL__1*2) | PBOFF__BAL_INTERRUPT |
						BVM_MODE__16_BIT | BVM_START__TRIG_MEAS |
						CVM_MODE__16_BIT | PCVM_START__TRIG_MEAS, 1, 0),
						MEAS_CTRL_trigger_measurement , SUCCESS_FUNCTION_NONE);

				if(TriggerCurrentMeasurement == 1)
					PRINTF_MAIN_DEBUG("[ERROR]: "
							"processMeasurements tried to trigger next AF "
							"before ADC result was ready\r\n");

				// Mark that the analog frontend measurement was triggered
				// and trigger measurement of current in this cycle
				TriggerCurrentMeasurement = 1;

				// Change State to wait for the result
				statusBMS.measurementState = MEASUREMENT_WAIT_RESULT;
			}
			//return;
			break;
		default:
			PRINTF_MAIN_DEBUG("[ERROR]: "
					"processMeasurements reached invalid state\r\n");
			statusBMS.measurementState = MEASUREMENT_TRIGGER_NEXT;
			break;
	}


	// Read external ADC and trigger next conversion ->
	// Accurate current and output/load voltage measurement
	// (current measurement will trigger in the same cycle as
	// analog front-end TLE9012)
	manage_external_ADC_MCP3465(1);

	// Read internal ADC and trigger next conversion ->
	// Rough driver current and slot coding resistor voltage measurement
	manage_internal_ADC();
}

/****************************************************************************
 * In no error condition determine BMS_STATE charge direction and
 * try to keep switch on (with pre-charge and minimum restart timer).
 * In error condition keep switch off.
 * Also controls orange LED (blink during pre-charge, on when conducting,
 * off when error) and statusBMS.outputState
 ***************************************************************************/
void manage_SystemAndSwitchState(){
	uint8_t targetData[6];
	targetData[0] = 0;

	// Get switch state
	//uint8_t isSwitch_mainpath_ON = getStateChannelA(USE_REG_RENEW);
	uint8_t isSwitch_mainpath_ON  = getSwitchState_mainpath(USE_REG_RENEW_SS);
	//uint8_t isSwitch_precharge_ON = getStateChannelB(USE_REG_CACHED);
	uint8_t isSwitch_precharge_ON = getSwitchState_precharge(USE_REG_CACHED_SS);

	/// Control orange LED directly based on output paths state
	if(isSwitch_mainpath_ON){ // Main (low resistance) path
		// Turn on orange LED
		cyhal_gpio_write(BMS_LED2_OR, 1);
		statusBMS.outputState = OUTPUT_ON;
		blinkTimeLedOrange = 0;
	}
	else if(isSwitch_precharge_ON){ // Precharge path
		// Blink Orange LED
		if(TIMER_COUNTER_1MS > (switchPrechargeStartTimestamp +
				blinkTimeLedOrange)){
			cyhal_gpio_toggle(BMS_LED2_OR);
			blinkTimeLedOrange += BLINK_TIME_LED_ORANGE;
		}
		statusBMS.outputState = OUTPUT_PRECHARGE;
	}
	else{
		// Turn off orange LED
		cyhal_gpio_write(BMS_LED2_OR, 0);
		statusBMS.outputState = OUTPUT_OFF;
		blinkTimeLedOrange = 0;
	}

	// State and Switch Control:
	// If BMS is plugged in and in a operational state,
	// evaluate BMS state and manage switch, else keep switch off
	if((statusBMS.bms_state == BMS_CHARGING ||
			statusBMS.bms_state == BMS_DISCHARGING ||
			statusBMS.bms_state == BMS_IDLE) &&
			(statusBMS.slotState != SLOT_NONE)){

		// Check if BMS is charging or discharging
		if((statusBMS.slotState == SLOT_IN_SYSTEM_1 ||
				statusBMS.slotState == SLOT_IN_SYSTEM_2) &&
				statusBMS.bms_state != BMS_DISCHARGING){
			//(statusBMS.current > IDLE_CURRENT_THRESHOLD)
			statusBMS.bms_state = BMS_DISCHARGING;
		}
		else if(statusBMS.slotState == SLOT_IN_CHARGER &&
				statusBMS.bms_state != BMS_CHARGING){
			//(statusBMS.current < -IDLE_CURRENT_THRESHOLD)
			statusBMS.bms_state = BMS_CHARGING;
		}
		// BMS_IDLE state is only used for transition to discharge or charge
		// and must therefore never be set here

		// If shutdown is triggered and conditions are given initiate HotSwap
		if(isShutdownTriggered == 1 && statusBMS.hotSwapRole == HOTSWAP_MAIN &&
				(isSwitch_mainpath_ON & isSwitch_precharge_ON) == 1){
			// Prepare bms state for takeover
			statusBMS.hotSwapRole = HOTSWAP_GIVER;
			statusBMS.giverHotSwapState = GIVER_SEND_HANDSHAKE;
			giverHotSwapTimestamp = TIMER_COUNTER_1MS;
			receivedCanHandshakeCount = 0;
			receivedCanSwapCount = 0;
			receivedCanFinishCount = 0;
			receivedCanSwitchOnVetoCount = 0;
		}
		else if(isShutdownTriggered == 1 &&
				(statusBMS.hotSwapRole == HOTSWAP_TAKER ||
				statusBMS.hotSwapRole == HOTSWAP_ERROR)){
			// Shut down normally, no hotswap needed
			PRINTF_MAIN_DEBUG("Direct shutdown - "
					"BMS is in HotSwap TAKER or ERROR\r\n");
			shutdownSystem();
		}
		else if((getMsgBit(MSG_INT_EXT_LOAD_V_ON) == 0) &&
				(statusBMS.hotSwapRole == HOTSWAP_TAKER) &&
				TIMER_COUNTER_1MS > HOTSWAP_TAKER_SHUTDWN_VEXT_TIME){
			// If BMS is hotswap TAKER and the external voltage disappears,
			// the BMS can never taker over -> shutdown
			PRINTF_MAIN_DEBUG("Direct shutdown - BMS is in HotSwap TAKER and "
					"external voltage disappeared - could never take over\r\n");
			shutdownSystem();
		}

		// State machine Switch on
		//// ---- MAIN ---- ////
		if(statusBMS.hotSwapRole == HOTSWAP_MAIN){
			switch (statusBMS.switchState){
				case SWITCH_IDLE:
					// If one of the safety switch channels are not
					// conducting and time since last turn on is
					// greater than minimal ...
					if((isSwitch_mainpath_ON & isSwitch_precharge_ON) == 0 &&
							((TIMER_COUNTER_1MS > (lastSwitchOnTimestamp +
												   MOSFET_ON_RETRY_TIME)) ||
							 lastSwitchOnTimestamp == 0) &&
							(getMsgBit(MSG_INT_EXT_LOAD_V_ON) == 0 ||
									statusBMS.slotState == SLOT_IN_CHARGER)){
						// Limit number of switch on tries
						if(safety_switch_retry_counter >=
								SAFETY_SWITCH_RETRY_COUNT){
							statusBMS.bms_state = BMS_IDLE;
							statusBMS.outputState = OUTPUT_OFF_TILL_RESTART;
							return;
						}

						PRINTF_MAIN_DEBUG("Switch on CH_B - Precharge\r\n");

						// Reset latched error and warning bitmaps
						reset_latchedBitmaps(1, 0);

						// Make sure all channels are turned off
						setSwitchIsolating();

						// Store current time as last turn on time
						lastSwitchOnTimestamp = TIMER_COUNTER_1MS;

						// Store current time as precharge start time
						switchPrechargeStartTimestamp = TIMER_COUNTER_1MS;

						// Turn precharge path on
						isCurrentOffsetCalibrating = 0;
						//switchCH_B_On(USE_REG_RENEW);
						setSwitchState_precharge(1);

						// Set state machine switch to initialize switch on
						statusBMS.switchState = SWITCH_PRECHARGE;
					}

					break;

				case SWITCH_PRECHARGE:
					// If precharge time elapsed, switch all channels on
					if(TIMER_COUNTER_1MS > (switchPrechargeStartTimestamp +
											PRECHARGE_MIN_TIME)){
						if( (TIMER_COUNTER_1MS >
						(switchPrechargeStartTimestamp + PRECHARGE_MAX_TIME))
								|| (statusBMS.voltageChangeRate_ms <
										PRECHARGE_TARGET_V_CHANGE_RATE) ){
							if(TIMER_COUNTER_1MS >
						(switchPrechargeStartTimestamp + PRECHARGE_MAX_TIME)){
								PRINTF_MAIN_DEBUG("Switch on CH_A - "
										"Main path (maximum "
										"pre-charge time exceeded "
										"after %ldms)\r\n",
										TIMER_COUNTER_1MS -
										switchPrechargeStartTimestamp);}
							if(statusBMS.voltageChangeRate_ms <
									PRECHARGE_TARGET_V_CHANGE_RATE){
								PRINTF_MAIN_DEBUG("Switch on CH_A - "
										"Main path (target voltage "
										"change rate reached after "
										"%ldms - min precharge time %dms)\r\n",
										TIMER_COUNTER_1MS-
										switchPrechargeStartTimestamp,
										PRECHARGE_MIN_TIME);}

							// Turn main path on
							isCurrentOffsetCalibrating = 0;
							//switchCH_A_On(USE_REG_RENEW);
							setSwitchState_mainpath(1);

							// Set statemachine switch to wait for switch off
							statusBMS.switchState = SWITCH_IDLE;
							// Signal to display on CM0+
							statusBMS.outputState = OUTPUT_ON;

							// Limit number of switch on tries
							safety_switch_retry_counter++;
						}
					}
					break;

				default:
					break;
			}
		}
		//// ---- GIVER ---- ////
		else if(statusBMS.hotSwapRole == HOTSWAP_GIVER){
			switch (statusBMS.giverHotSwapState) {
				case GIVER_SEND_HANDSHAKE:
					PRINTF_HOT_SWAP("GIVER_SEND_HANDSHAKE - "
							"Send BMS_HS_Handshake (A%d B%d)\r\n",
							getSwitchState_mainpath(USE_REG_RENEW_SS),
							getSwitchState_precharge(USE_REG_RENEW_SS));
					// Goto next state and reset timestamp
					statusBMS.giverHotSwapState = GIVER_WAIT_FOR_HANDSHAKE;
					giverHotSwapTimestamp = TIMER_COUNTER_1MS;
					break;
				case GIVER_WAIT_FOR_HANDSHAKE:
					// Send handshake continuously
					if((TIMER_COUNTER_1MS > (canLastMessageTimestamp +
											 CAN_MINIMAL_MESSAGE_TIME +
											 (rand() % 4)))){
						if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
							CAN_TX_Request(BMS_SLOT_1_HS_HANDSHAKE,
									targetData, 1);
						}
						else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
							CAN_TX_Request(BMS_SLOT_2_HS_HANDSHAKE,
									targetData, 1);
						}
					}
					// If handshake received Send command to reduce power
					if(receivedCanHandshakeCount >= 1){
						PRINTF_HOT_SWAP("GIVER_WAIT_FOR_HANDSHAKE - "
								"Received, Send BMS_ReducePowerForHS! "
								"(A%d B%d)\r\n",
								getSwitchState_mainpath(USE_REG_RENEW_SS),
								getSwitchState_precharge(USE_REG_RENEW_SS));

						// Goto next state and reset timestamp
						statusBMS.giverHotSwapState = GIVER_WAIT_REDUCE_POWER;
						giverHotSwapTimestamp = TIMER_COUNTER_1MS;
					}
					else if(TIMER_COUNTER_1MS > (giverHotSwapTimestamp +
												 HOTSWAP_TIMEOUT_TIME)){
						PRINTF_HOT_SWAP("GIVER_WAIT_FOR_HANDSHAKE - "
								"Timeout (A%d B%d)\r\n",
								getSwitchState_mainpath(USE_REG_RENEW_SS),
								getSwitchState_precharge(USE_REG_RENEW_SS));
						shutdownSystem();
					}
					break;
				case GIVER_WAIT_REDUCE_POWER:
					// Send reduce power continuously
					if((TIMER_COUNTER_1MS > (canLastMessageTimestamp +
											 CAN_MINIMAL_MESSAGE_TIME +
											 (rand() % 4)))){
						CAN_TX_Request(BMS_REDUCE_POWER_FOR_HS, targetData, 1);
					}
					// Give rest of system time to reduce power, then send swap command
					if(TIMER_COUNTER_1MS > (giverHotSwapTimestamp +
											HOTSWAP_WAIT_REDUCE_POWER_TIME)){
						PRINTF_HOT_SWAP("GIVER_WAIT_REDUCE_POWER - "
								"Send BMS_HS_Swap! (A%d B%d)\r\n",
								getSwitchState_mainpath(USE_REG_RENEW_SS),
								getSwitchState_precharge(USE_REG_RENEW_SS));

						// Goto next state and reset timestamp
						statusBMS.giverHotSwapState = GIVER_WAIT_SWAP;
						giverHotSwapTimestamp = TIMER_COUNTER_1MS;
					}
					break;
				case GIVER_WAIT_SWAP:
					// Send swap continuously
					if((TIMER_COUNTER_1MS > (canLastMessageTimestamp +
											 CAN_MINIMAL_MESSAGE_TIME +
											 (rand() % 4)))){
						if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
							CAN_TX_Request(BMS_SLOT_1_HS_SWAP, targetData, 1);
						}
						else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
							CAN_TX_Request(BMS_SLOT_2_HS_SWAP, targetData, 1);
						}
					}
					// If swap received turn output of and send done
					if(receivedCanSwapCount >= 1){
						PRINTF_HOT_SWAP("GIVER_WAIT_SWAP - Received, "
								"output off and send BMS_HS_Finish! "
								"(A%d B%d)\r\n",
								getSwitchState_mainpath(USE_REG_RENEW_SS),
								getSwitchState_precharge(USE_REG_RENEW_SS));

						// Turn output off
						setSwitchIsolating();

						// Send done message 5 times
						for(uint8_t i = 0; i < 5; i++){
							if((TIMER_COUNTER_1MS > (canLastMessageTimestamp +
									CAN_MINIMAL_MESSAGE_TIME + (rand() % 4)))){
								if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
									CAN_TX_Request(BMS_SLOT_1_HS_FINISH,
											targetData, 1);
								}
								else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
									CAN_TX_Request(BMS_SLOT_2_HS_FINISH,
											targetData, 1);
								}
								cyhal_system_delay_ms(HOTSWAP_CAN_MSG_DELAY_TIME);
							}
						}
						// Shutdown system
						PRINTF_HOT_SWAP("GIVER_WAIT_SWAP - Finished, "
								"shutdown! (A%d B%d)\r\n",
								getSwitchState_mainpath(USE_REG_RENEW_SS),
								getSwitchState_precharge(USE_REG_RENEW_SS));
						shutdownSystem();
					}
					else if(TIMER_COUNTER_1MS > (giverHotSwapTimestamp +
												 HOTSWAP_TIMEOUT_TIME)){
						PRINTF_HOT_SWAP("GIVER_WAIT_SWAP - "
								"Timeout (A%d B%d)\r\n",
								getSwitchState_mainpath(USE_REG_RENEW_SS),
								getSwitchState_precharge(USE_REG_RENEW_SS));
						shutdownSystem();
					}
					break;
			}
			// During give sequence the main loop is delayed
			cyhal_system_delay_ms(HOTSWAP_CAN_MSG_DELAY_TIME);
		}
		//// ---- TAKER ---- ////
		else if(statusBMS.hotSwapRole == HOTSWAP_TAKER){
			switch (statusBMS.takerHotSwapState) {
				case TAKER_WAIT_FOR_HANDSHAKE:
					// If handshake received send handshake command back
					if(receivedCanHandshakeCount >= 1){
						PRINTF_HOT_SWAP("TAKER_WAIT_FOR_HANDSHAKE - Received, "
								"Send BMS_HS_Handshake! (A%d B%d)\r\n",
								getSwitchState_mainpath(USE_REG_RENEW_SS),
								getSwitchState_precharge(USE_REG_RENEW_SS));

						// Goto next state and reset timestamp
						statusBMS.takerHotSwapState = TAKER_WAIT_SWAP;
						takerHotSwapTimestamp = TIMER_COUNTER_1MS;
					}
					break;
				case TAKER_WAIT_SWAP:
					// Send handshake continuously
					if((TIMER_COUNTER_1MS > (canLastMessageTimestamp +
											 CAN_MINIMAL_MESSAGE_TIME +
											 (rand() % 4)))){
						if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
							CAN_TX_Request(BMS_SLOT_1_HS_HANDSHAKE,
									targetData, 1);
						}
						else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
							CAN_TX_Request(BMS_SLOT_2_HS_HANDSHAKE,
									targetData, 1);
						}
					}
					// If swap received turn on precharge and
					// send command swap back
					if(receivedCanSwapCount >= 1){
						setSwitchState_precharge(1);
						PRINTF_HOT_SWAP("TAKER_WAIT_SWAP - Received, "
								"turn on pre-charge and send BMS_HS_Swap! "
								"(A%d B%d)\r\n",
								getSwitchState_mainpath(USE_REG_RENEW_SS),
								getSwitchState_precharge(USE_REG_RENEW_SS));

						// Goto next state and reset timestamp
						statusBMS.takerHotSwapState = TAKER_WAIT_FINISH;
						takerHotSwapTimestamp = TIMER_COUNTER_1MS;
					}
					else if(TIMER_COUNTER_1MS > (takerHotSwapTimestamp +
							HOTSWAP_TIMEOUT_TIME +
							HOTSWAP_WAIT_REDUCE_POWER_TIME)){
						PRINTF_HOT_SWAP("TAKER_WAIT_SWAP - Timeout "
								"%ld %ld %d\r\n",
								TIMER_COUNTER_1MS, takerHotSwapTimestamp,
								HOTSWAP_TIMEOUT_TIME);
						shutdownSystem();
					}
					break;
				case TAKER_WAIT_FINISH:
					// Send swap continuously
					if((TIMER_COUNTER_1MS > (canLastMessageTimestamp +
											 CAN_MINIMAL_MESSAGE_TIME +
											 (rand() % 4)))){
						if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
							CAN_TX_Request(BMS_SLOT_1_HS_SWAP, targetData, 1);
						}
						else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
							CAN_TX_Request(BMS_SLOT_2_HS_SWAP, targetData, 1);
						}
					}
					// If done received turn on full and go into MAIN mode
					if(receivedCanFinishCount >= 1){
						PRINTF_HOT_SWAP("TAKER_WAIT_FINISH - Received, "
								"turn on full and goto MAIN mode! "
								"(A%d B%d)\r\n",
								getSwitchState_mainpath(USE_REG_RENEW_SS),
								getSwitchState_precharge(USE_REG_RENEW_SS));
						//switchCH_A_On(USE_REG_RENEW);
						setSwitchState_mainpath(1);

						// Goto default state and reset timestamp
						statusBMS.takerHotSwapState = TAKER_WAIT_FOR_HANDSHAKE;
						takerHotSwapTimestamp = TIMER_COUNTER_1MS;

						// Clear robot to use power again
						for(uint8_t i = 0; i < 5; i++){
							if((TIMER_COUNTER_1MS > (canLastMessageTimestamp +
									CAN_MINIMAL_MESSAGE_TIME + (rand() % 4)))){
								CAN_TX_Request(BMS_ENABLE_POWER_FOR_HS,
										targetData, 1);
								cyhal_system_delay_ms(HOTSWAP_CAN_MSG_DELAY_TIME);
							}
						}
						// Reset can message counter so that
						// system is in known state
						// (especially switch on veto counter
						// would trigger ERR_INT_SWITCH_ON_VETO_RECEIVED
						// if not reset)
						resetCanMessageCounters();

						// Change to MAIN mode
						statusBMS.hotSwapRole = HOTSWAP_MAIN;

						// Set statemachine switch to wait for switch off
						statusBMS.switchState = SWITCH_IDLE;
						// Signal to display on CM0+
						statusBMS.outputState = OUTPUT_ON;

						// Limit number of switch on tries
						safety_switch_retry_counter++;
					}
					else if(TIMER_COUNTER_1MS > (takerHotSwapTimestamp +
												 HOTSWAP_TIMEOUT_TIME)){
						PRINTF_HOT_SWAP("TAKER_WAIT_FINISH - Timeout \r\n");
						//switchCH_B_Off(USE_REG_RENEW);
						setSwitchState_precharge(0);
						shutdownSystem();
					}
					break;
			}
			// During give the main loop is delayed
			cyhal_system_delay_ms(HOTSWAP_CAN_MSG_DELAY_TIME);
		}
	}
	// Output off condition
	else if(((isSwitch_mainpath_ON | isSwitch_precharge_ON) == 1) ||
			(statusBMS.bms_state != BMS_ERROR &&
					statusBMS.bms_state != BMS_IDLE)){
		/*PRINTF_MAIN_DEBUG("Switch off --- slot state %d, bms state %d \r\n",
				statusBMS.slotState, statusBMS.bms_state);*/
		PRINTF_MAIN_DEBUG("Switch off \r\n");

		// Disable MOSFETs/current flow in any other case
		setSwitchIsolating();

		// Reset state machine switch to check and
		// try switch on when error clears
		statusBMS.switchState = SWITCH_IDLE;

		// Change state to idle if needed
		if(statusBMS.slotState == SLOT_NONE &&
				statusBMS.bms_state != BMS_IDLE &&
				statusBMS.bms_state != BMS_ERROR){
			PRINTF_MAIN_DEBUG("BMS_IDLE state set "
					"(was %d)\r\n", statusBMS.bms_state);
			statusBMS.bms_state = BMS_IDLE;
		}

		// Signal to display on CM0+
		if(safety_switch_retry_counter >= SAFETY_SWITCH_RETRY_COUNT){
			statusBMS.outputState = OUTPUT_OFF_TILL_RESTART;
		}
		else{
			statusBMS.outputState = OUTPUT_READY;
		}
	}
}

/****************************************************************************
 * manage_analogFrontendTLE9012:
 * Execute queue management and interpretation of received data from
 * analog front-end. This also keeps the IC awake.
 * See comment of tle9012_manageQueue and tle9012_manageInterpreter
 ***************************************************************************/
void manage_analogFrontendTLE9012(){
	// If init was not successful always report fail
	// to signal error (and make system never switch on)
	if(initTLE9012Error == 1){
		lastQueueResult_tle9012 = QUEUE_FAIL;
	}
	// Else check TLE9012 queue and interpreter
	else{
		lastQueueResult_tle9012 = tle9012_manageQueue(1);
		tle9012_manageInterpreter();
	}
}

/****************************************************************************
 * manage_external_ADC_MCP3465:
 * Read and trigger measurement of external adc subsequently
 * for both channels
 * returns:	0 if measurement was not ready,
 * 			1 if current was read and
 * 			2 if external voltage was read
 ***************************************************************************/
uint8_t manage_external_ADC_MCP3465(uint8_t waitForCurrentMeasTrigger){
	uint8_t STATUS_DATA;
	uint16_t SGN_DATA, DATA;

	// Activate SPI and check status of ADC
    set_CS_SPI_LOW();
    STATUS_DATA = RETRIEVE_STATUS();
    set_CS_SPI_HIGH();

    // While waiting for the current measurement trigger do nothing.
    // When it is set trigger the next measurement
	if(activeChannelExternalADC == ACTIVE_EXT_ADC_WAIT_FOR_CURRENT_TRIGGER){
		if(TriggerCurrentMeasurement == 1){
			// Trigger ADC Conversion on channel 0 current
			SPI_WRT(_MUX_, MUX_CFG_CH0_SINGLE_END);
			activeChannelExternalADC = ACTIVE_EXT_ADC_CH0_CURRENT;
			/*PRINTF_MAIN_DEBUG("\tCurrent measurement "
					"triggered by TLE9012\r\n");*/

			// Reset flag
			TriggerCurrentMeasurement = 0;
		}
		return 0;
	}

    // Wait till the Data-Ready(DR) Bit of STATUS Byte is set
	if (STATUS_DATA == 0x13) {
    	// Mark time of last successful read.
		// Used to detect timeout error if communication fails
		// for to long (diagnostic.c)
		// Use this for ADC sample time measurement
    	/*PRINTF_MAIN_DEBUG("\tExternal ADC CH %d received after %lu ms\r\n",
    			activeChannelExternalADC,
				TIMER_COUNTER_1MS-adcExtLastReadSuccessTimestamp);*/
    	adcExtLastReadSuccessTimestamp = TIMER_COUNTER_1MS;

    	// Activate SPI and get result data from ADC
    	set_CS_SPI_LOW();
    	STATUS_DATA = spiTransfer((_ADCDATA_ << 2) | _RD_CTRL_);
    	// Get sign of data
        SGN_DATA = (spiTransfer(0x00) << 8);
        SGN_DATA |= (spiTransfer(0x00) & 0x00FF);
        //Get actual data and deactivate SPI
        DATA = (spiTransfer(0x00) << 8);
        DATA |= (spiTransfer(0x00) & 0x00FF);
        set_CS_SPI_HIGH();

        // Depending on which channel is currently active
        // store result for it and trigger the other one
        if (activeChannelExternalADC == ACTIVE_EXT_ADC_CH0_CURRENT) {
        	// Determine raw value based on signedness
        	int32_t system_I_raw = 0;
        	if (SGN_DATA == 0xFFFF) {
        		system_I_raw = (65536 - (float) DATA);
        	} else if (SGN_DATA == 0x0000) {
        		system_I_raw = (0 - (float) DATA);
        	}
        	// Filter result
        	measure_movAvgFilter_fast(&avgFilter_I, system_I_raw);

        	// Convert raw value to voltage
        	float DATA_LOAD_mV =
        			(float) avgFilter_I.filterResult * ADC_I_RAW_CONVERSION;

        	// Convert voltage with calibration to current
			float DATA_LOAD_CALIB_mA =
					(ADC_I_P1 * (float) DATA_LOAD_mV) - ADC_I_CONST;
        	/*PRINTF_MAIN_DEBUG("\tMUX_CFG_CH0_SINGLE_END:   "
        			"%.2fmV   ...   %X   ...   %.2fmA\r\n",
        			DATA_LOAD_mV, SGN_DATA, DATA_LOAD_CALIB_mA);*/

			// Store result and basic offset calib
        	if(isCurrentOffsetCalibrating == 0){
        		// If system is not calibrating subtract calibrated offset
        		statusBMS.current =
        				DATA_LOAD_CALIB_mA -
						(float)setupBMS.MeasCalib_Offset_I;
        	}
        	else{
        		// If system is calibrating do not use subtract
        		// but calculate calibrated offset
        		statusBMS.current = DATA_LOAD_CALIB_mA;
        		setupBMS.MeasCalib_Offset_I =
        				(int16_t)(statusBMS.current - SYSTEM_DEFAULT_CURRENT_mA);
        		// Write debug message during calibration
        		PRINTF_MAIN_DEBUG("\tUpdate MeasCalib_Offset_I to %4dmA, "
        				"with current %4.2fmA\r\n",
        				setupBMS.MeasCalib_Offset_I, statusBMS.current);
        	}

			// Use current to calculate charge for SoC and SoH
			update_batteryState_currentMeasured();

        	// Trigger ADC Conversion on channel 1 load voltage
    		SPI_WRT(_MUX_, MUX_CFG_CH1_SINGLE_END);
    		activeChannelExternalADC = ACTIVE_EXT_ADC_CH1_LOAD_VOLTAGE;

    		// Return that sensor 1 - current was read
			return 1;
    	}
        else if (activeChannelExternalADC == ACTIVE_EXT_ADC_CH1_LOAD_VOLTAGE) {
    		int32_t system_V_raw = DATA;

    		// Filter result
			measure_movAvgFilter_fast(&avgFilter_V, system_V_raw);

			// Convert raw value to voltage
    		float DATA_LOAD_mV = (float) system_V_raw * ADC_V_RAW_CONVERSION;

    		// Convert raw voltage with calibration to current
    		float DATA_LOAD_CALIB_mV =
    				(ADC_V_EXT_P1 * (float) DATA_LOAD_mV) + ADC_V_EXT_CONST;
    		/*PRINTF_MAIN_DEBUG("\tMUX_CFG_CH1_SINGLE_END:   "
    				"%X   ...   %X   ...   %.2fmV\r\n",
    				DATA, SGN_DATA, DATA_LOAD_CALIB_mV); */

    		// Store result
    		statusBMS.extLoadVoltage = DATA_LOAD_CALIB_mV/1000.0;

    		// Write debug message for voltage during calibration
        	if(isCurrentOffsetCalibrating == 1){
        		PRINTF_MAIN_DEBUG("\tExternal Voltage is %4.2fV\r\n",
        				statusBMS.extLoadVoltage);
        	}

        	// Calculate voltage change rate if possible
        	calc_voltageChangeRate(statusBMS.extLoadVoltage,
        			ADC_V_MIN_PLAUSIBLE);

        	// Wait for analog frontend to trigger current measurement
        	if(waitForCurrentMeasTrigger == 1)
        		activeChannelExternalADC =
        				ACTIVE_EXT_ADC_WAIT_FOR_CURRENT_TRIGGER;
        	else{
        		// Trigger ADC Conversion on channel 0 current
        		SPI_WRT(_MUX_, MUX_CFG_CH0_SINGLE_END);
        		activeChannelExternalADC = ACTIVE_EXT_ADC_CH0_CURRENT;
        		//PRINTF_MAIN_DEBUG("\tCurrent measurement triggered direct\r\n");
        	}

    		// Return that sensor 2 - current was read
			return 2;
    	}
    }

    // If an error during init occurred:
	// set last successful read timestamp to 0
	// in order to make error permanent
    if(initExtADCError == 1){
    	adcExtLastReadSuccessTimestamp = 0;
    }

    // Return that measurement was not ready
    return 0;
}

//****************************************************************************
// manage_internal_ADC
// Read and trigger measurement of internal adc for both channels
//****************************************************************************
void manage_internal_ADC(){
	// Check if measurements of both channels are finished
	uint32_t chanMask = 0b00000011;
	uint32_t chanResultUpdated = Cy_SAR_GetChanResultUpdated(SAR_HW);
	if (chanMask == (chanResultUpdated & chanMask))
	{
		// Check internal ADC for coding
		uint32_t adc_coding_res = Cy_SAR_GetResult32(SAR_HW, 0UL);
		adc_coding_res_mV = Cy_SAR_CountsTo_mVolts(SAR_HW, 0UL, adc_coding_res);

		// Check internal ADC for safety switch current
		adc_safety_switch_current = Cy_SAR_GetResult32(SAR_HW, 1UL);
		adc_safety_switch_current_mV =
				Cy_SAR_CountsTo_mVolts(SAR_HW, 1UL, adc_safety_switch_current);

		// Trigger next internal ADC conversion
		Cy_SAR_StartConvert(SAR_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
	}
}

//****************************************************************************
// manageCommunicationBMS - Check timer overflow and cycle time
//****************************************************************************
void manage_externalCommunicationBMS(){
	//static canDataSendStates canDataSendState;
	uint8_t targetData[CAN_MAX_DATA_LENGTH];

	/// DEBUG UART: CYCLIC STATUS SENDING
	// Debug status messages
	if(TIMER_COUNTER_1MS > (lastExternalCommunicationTimestamp +
							EXTERNAL_COMMUNICATION_TIME)){
		// Store current time as last execution time
		lastExternalCommunicationTimestamp = TIMER_COUNTER_1MS;

		/*float safety_switch_current_mA =
				polyEvalLinear((float)adc_safety_switch_current_mV);*/

		PRINTF_CYCLIC_DEBUG_MSGS("V %5.2f, I %4.0fmA, VExt %5.2f, VImb %4d, "
				"BAL_MAP %4d, TH %5d, C %.1fmAh, C_SoH_i %.0fmAh, "
				"T_RR %9.3fmin, T_RT %5lums, T_E %3.1f %3.1f %3.1f, "
				"T_C %3.1f, T_I %3.1f, RTL %4u, "
				"CAN_ERR %3d",
				statusBMS.voltage,
				statusBMS.current,
				statusBMS.extLoadVoltage,
				statusBMS.imbalance,
				statusBMS.bal_cell_map,
				setupBMS.BalVolt,
				//adc_safety_switch_current_mV,
				//safety_switch_current_mA,
				(batteryState.capacity_init - batteryState.capacity_t)/3.6,
				batteryState.init_capacity_SoH_at_thres/3.6,
				((double)batteryState.remaining_runtime_ms)/1000.0/60.0,
				memoryRtDataRefreshTime,
				//tle9012_nodes[0].temperatures[0], // Cell temp 1
				tle9012_nodes[0].temperatures[1],
				tle9012_nodes[0].temperatures[2],
				//tle9012_nodes[0].temperatures[3], // Cell temp 2
				tle9012_nodes[0].temperatures[4],
				statusBMS.avgTemp,
				tle9012_nodes[0].temperaturesInternal[0],
				nextMemoryRtLineAddressOffset/MEM_SIZE_RT_DATA,
				canSendErrorCount
		);

		/*PRINTF_MEMORY_RT_DATA_UPDATE_RATE("\tRemaining runtime "
				"(%.2fh %.2fmin %.2fms) = "
				"%.2fAs/%.2fA*1000 (rem cap = %.2fAs)\r\n",
				(float)batteryState.remaining_runtime_ms/1000.0/60.0/60.0,
				(float)batteryState.remaining_runtime_ms/1000.0/60.0,
				(float)batteryState.remaining_runtime_ms,
				batteryState.capacity_usable_remaining,
				(statusBMS.current)/1000.0,
				(batteryState.capacity_init - batteryState.capacity_t));
		PRINTF_MEMORY_RT_DATA_UPDATE_RATE("\tRefresh rate %ld ms "
				"(dynamic %.0lfms/%ldlines remaining)\r\n",
				memoryRtDataRefreshTime,
				(float)batteryState.remaining_runtime_ms,
				(uint32_t)((MEM_ADDR_RT_DATA_STOP-MEM_ADDR_RT_DATA_START)/
						MEM_SIZE_RT_DATA)-
				(nextMemoryRtLineAddressOffset/MEM_SIZE_RT_DATA));

		//// Print error bitmap
		PRINTF_CYCLIC_DEBUG_MSGS("ERR ");
		printBinary(statusBMS.error_map, UINT32_MAX, 32);
		PRINTF_CYCLIC_DEBUG_MSGS(", "); */

		// State of connection / slot
		if(statusBMS.slotState == SLOT_NONE)
			PRINTF_CYCLIC_DEBUG_MSGS("SLOT_NONE       , ");
		else if(statusBMS.slotState == SLOT_IN_CHARGER)
			PRINTF_CYCLIC_DEBUG_MSGS("SLOT_CHARGER , ");
		else if(statusBMS.slotState == SLOT_IN_SYSTEM_1)
			PRINTF_CYCLIC_DEBUG_MSGS("SLOT_SYSTEM1 , ");
		else if(statusBMS.slotState == SLOT_IN_SYSTEM_2)
			PRINTF_CYCLIC_DEBUG_MSGS("SLOT_SYSTEM2 , ");

		// State of BMS
		if(statusBMS.bms_state == BMS_CHARGING)
			PRINTF_CYCLIC_DEBUG_MSGS("CHARGING   , ");
		else if(statusBMS.bms_state == BMS_DISCHARGING)
			PRINTF_CYCLIC_DEBUG_MSGS("DISCHARGING, ");
		else if(statusBMS.bms_state == BMS_ERROR)
			PRINTF_CYCLIC_DEBUG_MSGS("BMS_ERROR      , ");
		else if(statusBMS.bms_state == BMS_IDLE)
			PRINTF_CYCLIC_DEBUG_MSGS("BMS_IDLE       , ");
		else
			PRINTF_CYCLIC_DEBUG_MSGS("BMS STATE %d   , ",
					statusBMS.bms_state);

		// State of BMS
		if(statusBMS.hotSwapRole == HOTSWAP_MAIN)
			PRINTF_CYCLIC_DEBUG_MSGS("ROLE_MAIN  , ");
		else if(statusBMS.hotSwapRole == HOTSWAP_GIVER)
			PRINTF_CYCLIC_DEBUG_MSGS("ROLE_GIVER , ");
		else if(statusBMS.hotSwapRole == HOTSWAP_TAKER)
			PRINTF_CYCLIC_DEBUG_MSGS("ROLE_TAKER , ");
		else if(statusBMS.hotSwapRole == HOTSWAP_ERROR)
			PRINTF_CYCLIC_DEBUG_MSGS("ROLE_ERROR , ");
		else
			PRINTF_CYCLIC_DEBUG_MSGS("ROLE %d    , ",
					statusBMS.hotSwapRole);

		// Print coding resistor readings
		//PRINTF_CYCLIC_DEBUG_MSGS("R_COD %ldmV\r\n", adc_coding_res_mV);

		// Print cell specific errors
		if(latched_error_map_ext_com & ERR_CAF_OL_ERR){
			PRINTF_CYCLIC_DEBUG_MSGS("OL_MAP ");
			printBinary((uint32_t)statusBMS.ol_map,
					0b0000111111111111, 16);
			PRINTF_CYCLIC_DEBUG_MSGS(", ");
		}
		if(latched_error_map_ext_com & ERR_CAF_CELL_UV){
			PRINTF_CYCLIC_DEBUG_MSGS("UV_MAP ");
			printBinary((uint32_t)statusBMS.uv_map,
					0b0000111111111111, 16);
			PRINTF_CYCLIC_DEBUG_MSGS(", ");
		}
		if(latched_error_map_ext_com & ERR_CAF_CELL_OV){
			PRINTF_CYCLIC_DEBUG_MSGS("OV_MAP ");
			printBinary((uint32_t)statusBMS.ov_map,
					0b0000111111111111, 16);
			PRINTF_CYCLIC_DEBUG_MSGS(", ");
		}
		if(latched_error_map_ext_com & ERR_CAF_BAL_UC){
			PRINTF_CYCLIC_DEBUG_MSGS("UC_MAP ");
			printBinary((uint32_t)statusBMS.bal_uc_map,
					0b0000111111111111, 16);
			PRINTF_CYCLIC_DEBUG_MSGS(", ");
		}
		if(latched_error_map_ext_com & ERR_CAF_BAL_OC){
			PRINTF_CYCLIC_DEBUG_MSGS("OC_MAP ");
			printBinary((uint32_t)statusBMS.bal_oc_map,
					0b0000111111111111, 16);
			PRINTF_CYCLIC_DEBUG_MSGS(", ");
		}

		// Print bitmaps
		//PRINTF_CYCLIC_DEBUG_MSGS(", BMS_ERROR 0b");
		//printBinary(latched_error_map_ext_com, UINT32_MAX, 32);
		diagnostic_printStatusFlags(latched_error_map_ext_com,
				latched_message_map, 0, 0);

		PRINTF_CYCLIC_DEBUG_MSGS("\r\n");

		// Reset latched bitmaps
		reset_latchedBitmaps(0, 1);

		//// CAN
		// Print received package data
		if(newCanMessageReceived != 0){
			//PRINTF_CYCLIC_DEBUG_MSGS("New CAN Message\r\n");
			// Count receive of specific message
			if(receivedCanHandshakeCount != 0){
				PRINTF_CYCLIC_DEBUG_MSGS("HANDSHAKE Count %3d\r\n",
						receivedCanHandshakeCount);
			}
			if(receivedCanSwapCount != 0){
				PRINTF_CYCLIC_DEBUG_MSGS("SWAP Count      %3d\r\n",
						receivedCanSwapCount);
			}
			if(receivedCanFinishCount != 0){
				PRINTF_CYCLIC_DEBUG_MSGS("FINISH Count      %3d\r\n",
						receivedCanFinishCount);
			}
			//PRINTF_CYCLIC_DEBUG_MSGS("\r\n");

			// Reset CAN
			newCanMessageReceived = 0;
			receivedCanHandshakeCount = 0;
			receivedCanSwapCount = 0;
			receivedCanFinishCount = 0;
		}
	}

	/// DEBUG UART: RECEIVED MESSAGE CHECK
	// Prepare buffer
	#define RX_BUFFER_UART_SIZE 100
	uint8_t rxBufferUART[RX_BUFFER_UART_SIZE];
	//uint16_t cmdLen = 0;
	static uint16_t full_rx_size = 0;
	size_t rx_size = RX_BUFFER_UART_SIZE;

	// Read buffer and store received data
	cyhal_uart_read(&cy_retarget_io_uart_obj,
			&rxBufferUART[full_rx_size], &rx_size);
	full_rx_size += rx_size;

	// If \r was received store command and print result
	if (rxBufferUART[full_rx_size - 1] == 0x0d) {
		printf("Received %d bytes from UART: ", full_rx_size);
		for(uint8_t i=0; i<full_rx_size-1; i++){
			printf(" %d (%c)", rxBufferUART[i], rxBufferUART[i]);
		}
		printf(" %d", rxBufferUART[full_rx_size-1]);
		printf("\r\n");
		full_rx_size = 0;
		//cyhal_uart_clear(&cy_retarget_io_uart_obj);
	}

	/// CAN COMMUNICATION
	// Send information about BMS state via CAN. Only from active BMS
	if(statusBMS.hotSwapRole == HOTSWAP_MAIN && (TIMER_COUNTER_1MS >
				(canLastMessageTimestamp + CAN_MINIMAL_MESSAGE_TIME +
						(rand() % 4)))){
		// Store current time as last message send time
		canLastMessageTimestamp = TIMER_COUNTER_1MS;

		// There are 3 types of messages
		// (switchOnVeto for HotSwap, CAN requests and
		// repeated status message sending)
		// which are handled with given priority.
		cy_en_canfd_status_t can_status = UINT8_MAX;
		if(TIMER_COUNTER_1MS > (canSwitchOnVetoRefreshTimestamp +
								CAN_SWICH_ON_VETO_REFRESH_TIME)){
			// Store current time as last execution time
			canSwitchOnVetoRefreshTimestamp = TIMER_COUNTER_1MS;

			// Send switch on veto with data 200 to signal
			// other BMS that this one is active
			targetData[0] = 200;
			targetData[1] = 1;
			/*can_status = CAN_TX_Request((ROBOT_CONTROL_CLASS_MASK | 0x00),
					BMS_SwitchOn_Veto, targetData, 2);*/
			if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
				can_status = CAN_TX_Request(BMS_SLOT_1_SWITCH_ON_VETO,
						targetData, 2);
			}
			else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
				can_status = CAN_TX_Request(BMS_SLOT_2_SWITCH_ON_VETO,
						targetData, 2);
			}
		}
		else if((isCanRequested_RT_Reset |
				 isCanRequested_RT_GetLine |
				 isCanRequested_RT_GetStoredLineNumber |
				 isCanRequested_RT_GetMaxLineNumber |
				 isCanRequested_RT_SetSamplingTime |
				 isCanRequested_RT_GetSamplingTime) != 0){
			if(isCanRequested_RT_Reset >= 1){
				// Reset line index of RT data recording
				memoryResetRtData();
				isCanRequested_RT_Reset = 0;
				PRINTF_MAIN_RT_CAN_REQUEST("\t CAN RT "
						"data line reset requested\r\n");
			}
			else if(isCanRequested_RT_GetLine == 1){
				// At the beginning of a line read the hole line to buffer,
				// to be transfered in parts
				if(CanRequested_LinesCurrentIndexPart == 0){
					// Get data line as byte array
					memoryGetRtData_byteArray(memoryRtDataLineBuffer, 1,
							CanRequested_LinesStartIndex+
							CanRequested_LinesCurrentIndex);
					/*memoryGetRtData(NULL, 1,
							CanRequested_LinesStartIndex+
							CanRequested_LinesCurrentIndex, 1);*/
					PRINTF_MAIN_RT_CAN_REQUEST("CAN RT data get lines requested, "
							"sending: start %u, stop %u, current %3u \r\n",
							CanRequested_LinesStartIndex,
							CanRequested_LinesNumber,
							CanRequested_LinesCurrentIndex);
					PRINTF_MAIN_RT_GETLINE_DEBUG("\tRead line %d to buffer "
							"%02X %02X %02X %02X %02X "
							"%02X %02X %02X %02X %02X \r\n",
							CanRequested_LinesStartIndex,
							memoryRtDataLineBuffer[0],
							memoryRtDataLineBuffer[1],
							memoryRtDataLineBuffer[2],
							memoryRtDataLineBuffer[3],
							memoryRtDataLineBuffer[4],
							memoryRtDataLineBuffer[5],
							memoryRtDataLineBuffer[6],
							memoryRtDataLineBuffer[7],
							memoryRtDataLineBuffer[8],
							memoryRtDataLineBuffer[9]);
				}

				// Determine length to be transmitted
				// (the last part of the current line
				// may not fill a whole CAN message)
				uint8_t dataLength = CAN_MAX_DATA_LENGTH;
				if((CanRequested_LinesCurrentIndexPart+1)*CAN_MAX_DATA_LENGTH >
					MEM_SIZE_RT_DATA){
					dataLength = MEM_SIZE_RT_DATA -
							CanRequested_LinesCurrentIndexPart*
							CAN_MAX_DATA_LENGTH;
				}
				PRINTF_MAIN_RT_GETLINE_DEBUG("\tSending message "
						"from buffer index %u with length %u \r\n",
						(CanRequested_LinesCurrentIndexPart*
								CAN_MAX_DATA_LENGTH), dataLength);

				// Send line part via CAN
				// Switch to Big Endian representation for CAN
				if (CanRequested_LinesCurrentIndexPart==0)
				{
					targetData[0] = memoryRtDataLineBuffer[1];
				    targetData[1] = memoryRtDataLineBuffer[0];
				    targetData[2] = memoryRtDataLineBuffer[3];
		    		targetData[3] = memoryRtDataLineBuffer[2];
	   				targetData[4] = memoryRtDataLineBuffer[5];
	   				targetData[5] = memoryRtDataLineBuffer[4];
					targetData[6] = memoryRtDataLineBuffer[7];
					targetData[7] = memoryRtDataLineBuffer[6];
				} else
				{
					targetData[0] = memoryRtDataLineBuffer[9];
				    targetData[1] = memoryRtDataLineBuffer[8];
				}
				can_status = CAN_TX_Request(BMS_RT_DATA_SENDLINEPART,
						targetData, dataLength);
				if(can_status == CY_CANFD_SUCCESS){
					CanRequested_LinesCurrentIndexPart++;
				}

				// Check if line is finished and next line must be send
				if(CanRequested_LinesCurrentIndexPart >
				MEM_SIZE_RT_DATA/CAN_MAX_DATA_LENGTH){
					CanRequested_LinesCurrentIndex++;
					CanRequested_LinesCurrentIndexPart = 0;
					PRINTF_MAIN_RT_GETLINE_DEBUG("\tProcessing next "
							"line %d \r\n", CanRequested_LinesCurrentIndex);

					// If the current line index is higher than
					// max request is finished
					if(CanRequested_LinesCurrentIndex > MEM_SIZE_MAX_LINES){
						isCanRequested_RT_GetLine = 0;
						PRINTF_MAIN_RT_GETLINE_DEBUG("\tProcessed last "
								"possible line. "
								"GetLine CAN request stopped \r\n");
					}

					// If the current line index is after
					// stop index the request is finished
					if(CanRequested_LinesCurrentIndex >
					CanRequested_LinesNumber-1){
						isCanRequested_RT_GetLine = 0;
						PRINTF_MAIN_RT_GETLINE_DEBUG("\tProcessed last "
								"requested line. "
								"GetLine CAN request finished \r\n");
					}
				}
			}
			else if(isCanRequested_RT_GetStoredLineNumber >= 1){
				uint16_t storedLineNumber =
						nextMemoryRtLineAddressOffset/MEM_SIZE_RT_DATA;
				targetData[0] = ((storedLineNumber) >> 8 & 0xFF);
				targetData[1] = ((storedLineNumber) & 0xFF);
				// Try to send number via CAN for max 5 times
				// before ignoring the request
				can_status = CAN_TX_Request(BMS_RT_DATA_GETSTOREDLINENUMBER,
						targetData, 2);
				if(can_status == CY_CANFD_SUCCESS)
					isCanRequested_RT_GetStoredLineNumber = 0;
				else
					isCanRequested_RT_GetStoredLineNumber++;
				if(isCanRequested_RT_GetStoredLineNumber > 5)
					isCanRequested_RT_GetStoredLineNumber = 0;
				PRINTF_MAIN_RT_CAN_REQUEST("\t CAN RT data "
						"get stored line number requested\r\n");
			}
			else if(isCanRequested_RT_GetMaxLineNumber >= 1){
				uint16_t maxLineNumber = MEM_SIZE_MAX_LINES;
				targetData[0] = ((maxLineNumber) >> 8 & 0xFF);
				targetData[1] = ((maxLineNumber) & 0xFF);
				// Try to send number via CAN for max 5 times
				// before ignoring the request
				can_status = CAN_TX_Request(BMS_RT_DATA_GETMAXLINENUMBER,
						targetData, 2);
				if(can_status == CY_CANFD_SUCCESS)
					isCanRequested_RT_GetMaxLineNumber = 0;
				else
					isCanRequested_RT_GetMaxLineNumber++;
				if(isCanRequested_RT_GetMaxLineNumber > 5)
					isCanRequested_RT_GetMaxLineNumber = 0;
				PRINTF_MAIN_RT_CAN_REQUEST("\t CAN RT data "
						"get max line number requested\r\n");
			}
			else if(isCanRequested_RT_SetSamplingTime >= 1){
				// Limit check and value setting
				if(CanRequested_SamplingTime == 0){
					memoryRtDataRefresh_isTimeFixed = 0;
					memoryRtDataRefreshTime = 0;
					PRINTF_MAIN_RT_CAN_REQUEST("CAN RT data "
							"set sampling time: "
							"Enabled dynamic sampling time");
				}else{
					memoryRtDataRefresh_isTimeFixed = 1;
					memoryRtDataRefreshTime = CanRequested_SamplingTime;
					PRINTF_MAIN_RT_CAN_REQUEST("CAN RT data "
							"set sampling time: "
							"Enabled fixed sampling time of %lu",
							memoryRtDataRefreshTime);
				}
				isCanRequested_RT_SetSamplingTime = 0;
			}
			else if(isCanRequested_RT_GetSamplingTime >= 1){
				uint32_t samplingTime = memoryRtDataRefreshTime;
				targetData[0] = ((samplingTime) >> 24 & 0xFF);
				targetData[1] = ((samplingTime) >> 16 & 0xFF);
				targetData[2] = ((samplingTime) >> 8 & 0xFF);
				targetData[3] = ((samplingTime) >> 0 & 0xFF);
				// Try to send number via CAN for max 5 times
				// before ignoring the request
				can_status = CAN_TX_Request(BMS_RT_DATA_GETSAMPLINGTIME,
						targetData, 4);
				if(can_status == CY_CANFD_SUCCESS)
					isCanRequested_RT_GetSamplingTime = 0;
				else
					isCanRequested_RT_GetSamplingTime++;
				if(isCanRequested_RT_GetSamplingTime > 5)
					isCanRequested_RT_GetSamplingTime = 0;
				PRINTF_MAIN_RT_CAN_REQUEST("\t CAN RT data "
						"get sampling time\r\n");
			}
		}
		else if(TIMER_COUNTER_1MS > (canDataRefreshTimestamp +
				CAN_DATA_REFRESH_TIME)){
			// Store current time as last execution time
			canDataRefreshTimestamp = TIMER_COUNTER_1MS;

			// Send current data and increment data index
			targetData[0] = (uint8_t)(statusBMS.SoC);
			targetData[1] = (((uint16_t)(statusBMS.voltage*1000.0)) >> 8 & 0xFF);
			targetData[2] = (((uint16_t)(statusBMS.voltage*1000.0)) & 0xFF);
			targetData[3] = (((int16_t)statusBMS.current) >> 8 & 0xFF);
			targetData[4] = (((int16_t)statusBMS.current) & 0xFF);
			targetData[5] = (uint8_t)sh_pub_soh;
			targetData[6] = (((uint16_t)(TIMER_COUNTER_1MS/1000)) >> 8 & 0xFF);
			targetData[7] = (((uint16_t)(TIMER_COUNTER_1MS/1000)) & 0xFF);
			can_status = CAN_TX_Request(BMS_SLOT_1_STATE+statusBMS.slotState-1,
					targetData, 8);
		}
#if (EXT_DATALOG_ENABLED)
		else if(TIMER_COUNTER_1MS > (datalog_refreshTimestamp +
				EXTERNAL_COMMUNICATION_TIME)){
			datalog_refreshTimestamp = TIMER_COUNTER_1MS;
			// statusBMS.current in mA & temperatures in deg.C
			for (int i = 0; i < 4; i++){
				targetData[i] = (((int32_t)(statusBMS.current*100.0))
						>> (8*(3-i)) & 0xFF);
			}
			targetData[4] = (((int16_t)(tle9012_nodes[0].temperatures[0]*100.0))
					>> 8 & 0xFF);
			targetData[5] = (((int16_t)(tle9012_nodes[0].temperatures[0]*100.0))
					& 0xFF);
			targetData[6] = (((int16_t)(tle9012_nodes[0].temperatures[3]*100.0))
					>> 8 & 0xFF);
			targetData[7] = (((int16_t)(tle9012_nodes[0].temperatures[3]*100.0))
					& 0xFF);
			can_status = CAN_TX_Request(BMS_SLOT_1_CURR_TEMP+
					statusBMS.slotState-1, targetData, 8);
			Cy_SysLib_Delay(1);
			for (int i = 0; i < 3; i++){
				for (int j = 0; j < 4; j++){
					targetData[j*2] =
							tle9012_nodes[0].cell_voltages[i*4+j] >> 8 & 0xFF;
					targetData[j*2+1] =
							tle9012_nodes[0].cell_voltages[i*4+j] & 0xFF;
				}
				can_status = CAN_TX_Request(BMS_SLOT_1_CELLV0_3+
						statusBMS.slotState-1+(16*i), targetData, 8);
				Cy_SysLib_Delay(1);
			}
		}
#endif
		// Turn on blue led if communication successful or off if failed
		if(can_status != UINT8_MAX){
			if(canSendSuccessCount > 2)
				cyhal_gpio_write(BMS_LED4_BL, 1);
			else{
				cyhal_gpio_write(BMS_LED4_BL, 0);
				//CAN_Reset();
			}
		}

		// Write debug message if CAN message send fails
		//if(can_status != CY_CANFD_SUCCESS){
		//	PRINTF_MAIN_DEBUG("CAN send failed %d\r\n", can_status);
		//}
	}
}

//****************************************************************************
// manage_timer - Check timer overflow and cycle time
//****************************************************************************
void manage_timer(){
	// Reset timer and all time storing variables before a overflow occurs
	if(Cy_TCPWM_Counter_GetCounter(TIMER_1MS_HW, TIMER_1MS_NUM) >
								   TIMER_1MS_config.period-5000){
		// Reset counter
		Cy_TCPWM_TriggerStopOrKill_Single(TIMER_1MS_HW, TIMER_1MS_NUM);
		Cy_TCPWM_Counter_Disable(TIMER_1MS_HW, TIMER_1MS_NUM);
		Cy_TCPWM_Counter_SetCounter(TIMER_1MS_HW, TIMER_1MS_NUM, 0);
		Cy_TCPWM_Counter_Enable(TIMER_1MS_HW, TIMER_1MS_NUM);
		Cy_TCPWM_TriggerStart_Single(TIMER_1MS_HW, TIMER_1MS_NUM);
#if (EXT_DATALOG_ENABLED)
		datalog_refreshTimestamp = 0;
#endif
		canDataRefreshTimestamp = 0;
		// Reset all other time storages //TODO low priority,
								   	   	 //does only happen after
								   	     //49.7 days of continuous run
	}
}

//****************************************************************************
// manage_slotState
// Check the coding ADC channel and determine what state system is in
//****************************************************************************
void manage_slotState(uint8_t ignoreDebounce){
	// Decide in which slot state the system is
	if(adc_coding_res_mV > CODING_RES_MAINBOARD_2_BELOW_mV){
		// Write debug message and determine CAN ID
		// only on initial slot change back to none
		if(statusBMS.slotState != SLOT_NONE){
			PRINTF_MAIN_DEBUG("BMS is not in base: %ldmV\r\n",
					adc_coding_res_mV);

		}
		// Change State
		statusBMS.slotState = SLOT_NONE;

		// Store time of last state change
		slotPlugDebounceTimestamp = TIMER_COUNTER_1MS;
	}
	// If time since last SLOT_NONE detection is greater than
	// SLOT_PLUG_DEBOUNCE_TIME check in which slot the system is now
	else if((ignoreDebounce == 1) ||
			(TIMER_COUNTER_1MS > (slotPlugDebounceTimestamp +
					SLOT_PLUG_DEBOUNCE_TIME))){
		// Determine slot based on current and coding resistor or
		// only based on coding resistor (see DETECT_SLOT_BASED_ON_CURRENT)
		// EXPERIMENTAL: Detect CHARGER if current is below zero
		#if DETECT_SLOT_BASED_ON_CURRENT == 1
			if(statusBMS.current < 0){
				if(statusBMS.slotState != SLOT_IN_CHARGER){
					PRINTF_MAIN_DEBUG("BMS is in charger: %ldmV\r\n",
							adc_coding_res_mV);

					// Change State
					statusBMS.slotState = SLOT_IN_CHARGER;
				}
			}
			else if(adc_coding_res_mV > CODING_RES_CHARGER_BELOW_mV &&
					adc_coding_res_mV < CODING_RES_MAINBOARD_1_BELOW_mV &&
					statusBMS.slotState != SLOT_IN_SYSTEM_1){
				PRINTF_MAIN_DEBUG("BMS is in IMR mainboard 1: %ldmV\r\n",
						adc_coding_res_mV);

				// Change State
				statusBMS.slotState = SLOT_IN_SYSTEM_1;
			}
			else if(adc_coding_res_mV > CODING_RES_MAINBOARD_1_BELOW_mV &&
					adc_coding_res_mV < CODING_RES_MAINBOARD_2_BELOW_mV &&
					statusBMS.slotState != SLOT_IN_SYSTEM_2){
				PRINTF_MAIN_DEBUG("BMS is in IMR mainboard 2: %ldmV\r\n",
						adc_coding_res_mV);

				// Change State
				statusBMS.slotState = SLOT_IN_SYSTEM_2;
			}
		// DEFAULT: Detect all slot state only based on coding resistor
		#else
			if(adc_coding_res_mV > CODING_RES_CHARGER_BELOW_mV &&
					adc_coding_res_mV < CODING_RES_MAINBOARD_1_BELOW_mV &&
					statusBMS.slotState != SLOT_IN_SYSTEM_1){
				PRINTF_MAIN_DEBUG("BMS is in IMR mainboard 1: %ldmV\r\n",
						adc_coding_res_mV);

				// Change State
				statusBMS.slotState = SLOT_IN_SYSTEM_1;
			}
			else if(adc_coding_res_mV > CODING_RES_MAINBOARD_1_BELOW_mV &&
					adc_coding_res_mV < CODING_RES_MAINBOARD_2_BELOW_mV &&
					statusBMS.slotState != SLOT_IN_SYSTEM_2){
				PRINTF_MAIN_DEBUG("BMS is in IMR mainboard 2: %ldmV\r\n",
						adc_coding_res_mV);

				// Change State
				statusBMS.slotState = SLOT_IN_SYSTEM_2;
			}
			else if(adc_coding_res_mV < CODING_RES_CHARGER_BELOW_mV &&
					statusBMS.slotState != SLOT_IN_CHARGER){
				PRINTF_MAIN_DEBUG("BMS is in charger: %ldmV\r\n",
						adc_coding_res_mV);

				// Change State
				statusBMS.slotState = SLOT_IN_CHARGER;
			}
		#endif
	}
}

/****************************************************************************
 * manage_shutdown
 * Check if shutdown button is pressed and
 * kill system by disabling shutdown pin
 ***************************************************************************/
void manage_shutdown(){
	// Check how long deep discharge threshold is exceeded
	uint8_t isDeepDischargeProtectionTriggered = 0;
	if(statusBMS.voltage > (float)DEEP_DISCHARGE_PROTECTION_V){
		deepDischargeTimestamp = TIMER_COUNTER_1MS;
	}
	else if(TIMER_COUNTER_1MS >
			(deepDischargeTimestamp + DEEP_DISCHARGE_PROTECTION_TIME)){
		isDeepDischargeProtectionTriggered = 1;
	}

	// If shutdown button is pressed or
	// deep discharge protection is activated...
	if((isShutdownTriggered == 0) &&
			(cyhal_gpio_read(BUTTON_SHUTDOWN) == 1 ||
					isDeepDischargeProtectionTriggered == 1)){
		PRINTF_MAIN_DEBUG("Shutdown triggered - Button %d, "
				"System voltage = %.2f "
				"(deep discharge protection below %.2f)\r\n",
				cyhal_gpio_read(BUTTON_SHUTDOWN), statusBMS.voltage,
				(float)DEEP_DISCHARGE_PROTECTION_V);
		// Mark time when shutdown was triggered and
		// activate software watchdog if it is not already running
		// (will shut down hard if shutdown takes longer than
		// MAXIMUM_SHUTDOWN_TIME)
		shutdownTriggeredTimestamp = TIMER_COUNTER_1MS;
		isWatchdogActive = 1;

		// If system is in a system slot, mark that shutdown is triggered.
		// In this state the HOTSWAP will be initialized and
		// lead to shutdown after give-over or error shutdown
		if((statusBMS.slotState == SLOT_IN_SYSTEM_1 ||
				statusBMS.slotState == SLOT_IN_SYSTEM_2) &&
				(statusBMS.bms_state == BMS_CHARGING ||
						statusBMS.bms_state == BMS_DISCHARGING)){
			isShutdownTriggered = 1;
		}
		// In any other state shut down directly
		else{
			PRINTF_MAIN_DEBUG("System is not in a "
					"'wait for shutdown' condition. Direct shutdown:\r\n");
			shutdownSystem();
		}
	}
	// Note: Hard shutdown after MAXIMUM_SHUTDOWN_TIME is performed
	// in software watchdog timer_watchdog_interrupt_handler()
}

//****************************************************************************
// keepHardwareWatchdogAlive - Send a kick to the hardware watchdog.
// This must be called faster than every HARDWARE_WATCHDOG_TIMEOUT ms
//****************************************************************************
void keepHardwareWatchdogAlive(){
	// Keep hardware watchdog alive
	#if ENABLE_HARDWARE_WATCHDOG == 1
		cyhal_wdt_kick(&wdt_obj);
	#endif /* #if defined (ENABLE_HARDWARE_WATCHDOG) */
}

//****************************************************************************
// shutdownSystem -
// Deactivate Watchdog and shutdown system by disabling shutdown pin
//****************************************************************************
void shutdownSystem(){
	PRINTF_MAIN_DEBUG("SHUTDOWN SYSTEM!\r\n");

	// Disable safety switch
	setSwitchIsolating();
	cyhal_gpio_write(BMS_LED2_OR, 0);

	// Set output state to OFF so it will be shown on the display
	statusBMS.outputState = OUTPUT_OFF;

	// Deactivate software watchdog to prevent it from triggering
	isWatchdogActive = 0;

	// Init time stamp for Green LED blinking
	uint32_t shutdownBlinkTimestamp = TIMER_COUNTER_1MS;

	// Keep hardware watchdog alive one last time.
	// If system does not shut down HARDWARE_WATCHDOG_TIMEOUT ms after this
	// the watchdog will reset the system
	keepHardwareWatchdogAlive();

	// Make display refresh for the last time and disable shutdown pin
	for(uint16_t i = 0; i < LAST_UPDATE_DISPLAY_NUM; i++){
		// Blink green LED
		if(TIMER_COUNTER_1MS >
			(shutdownBlinkTimestamp + BLINK_TIME_LED_GREEN_SHUTDOWN)){
			cyhal_gpio_toggle(BMS_LED1_GR);
			shutdownBlinkTimestamp = TIMER_COUNTER_1MS;
		}
		// Update display and send shutdown command
		// so it will refresh to main page and stop refreshing.
		updatePublicDisplayData(PUBLIC_COMMAND_SHUTDOWN);

		// When the last update is done the CM0+ core
		// will return the done command
		if(sh_pub_command == PUBLIC_COMMAND_DONE){
			// Send debug message and shut down system
			PRINTF_MAIN_DEBUG("Display finished update, shut down now\r\n");
			cyhal_system_delay_ms(10);
			cyhal_gpio_write(VCC_HOLD, 0);
			// End loop
			break;
		}

		// Limit the time between executions
		cyhal_system_delay_ms(LAST_UPDATE_DISPLAY_DELAY);
	}

	// Debug message if display refresh was not successful
	if(sh_pub_command != PUBLIC_COMMAND_DONE){
		PRINTF_MAIN_DEBUG("Display did not finish update %d\r\n",
				sh_pub_command);
		cyhal_system_delay_ms(10);
	}

	// Shut down system even if all else fails and wait till system is off
	while(1){
		cyhal_gpio_write(VCC_HOLD, 0);
		cyhal_system_delay_ms(BLINK_TIME_LED_GREEN_SHUTDOWN);
		cyhal_gpio_toggle(BMS_LED1_GR);
		PRINTF_MAIN_DEBUG("Disabled VCC_HOLD\r\n");

		/* NOTE: If following line is inactive:
		 * the system will stay in in shutdown
		 * if the power on button is held continuously.
		 * If it is enabled:
		 * the system will reboot and potentially start up / switch ON again.
		 */
		//NVIC_SystemReset();
	}
}

//****************************************************************************
// reset_safetySwitchTurnOn -
// Reset switch-on counter and reset state to make switch on possible again
//****************************************************************************
void reset_safetySwitchTurnOn(){
	PRINTF_MAIN_DEBUG("Safety switch reset triggered\r\n");
	// Reset number turn of tries
	safety_switch_retry_counter = 0;
	// Reset number of common errors
	numComErrors_TLE9012 = 0;
	// Reset can message counter so that system is in known state
	// (especially switch on veto counter would trigger
	// ERR_INT_SWITCH_ON_VETO_RECEIVED if not reset)
	resetCanMessageCounters();
	// Reset the direct save state set on error interrupt
	SafetySwitchInterrupt_SaveStateSet = DIRECT_SAVESTATE_ON_INTERRUPT;
	AnalogFrontendInterrupt_SaveStateSet = DIRECT_SAVESTATE_ON_INTERRUPT;
}

/****************************************************************************
 * reset_latchedBitmaps
 * Resets each bitmap which content needs to be accumulated
 * until a certain event happens.
 * Does not effect the main/actual versions of the maps
 ***************************************************************************/
void reset_latchedBitmaps(uint8_t reset_error_map_switch_on,
		uint8_t reset_error_map_ext_com){
	// Reset the map that latches errors
	// between switch ON of the safety switch if needed
	if(reset_error_map_switch_on != 0){
		latched_error_map_switch_on = 0;
	}
	// Reset the map that latches errors
	// between PC/CAN transmissions if needed
	if(reset_error_map_ext_com != 0){
		latched_error_map_ext_com = 0;
	}

	// Reset remaining maps
	latched_message_map = 0;
	statusBMS.ol_map = 0;
	statusBMS.uv_map = 0;
	statusBMS.ov_map = 0;
	statusBMS.bal_uc_map = 0;
	statusBMS.bal_oc_map = 0;
}

/****************************************************************************
 * calc_voltageChangeRate
 * Calculate the load voltage change rate if conditions are right
 * (external voltage larger than minimal and
 * at least on good measurement done before calculation)
 * Writes the current rate if switch is in pre-charge mode
 ***************************************************************************/
void calc_voltageChangeRate(float loadVoltage, float minLoadVoltage){
	// Calculate voltage change rate
	if(loadVoltage > minLoadVoltage){
		// If the last read was successful calculate the change rate
		if(lastExtVoltageMeasurement < (ADC_EXT_V_CHANGE_RATE_MAX-0.01)){
			// Calculate change rate
			statusBMS.voltageChangeRate_ms =
					(loadVoltage - lastExtVoltageMeasurement)/
					(float)(TIMER_COUNTER_1MS -
							adcLastVoltageChangeRateTimestamp);
			// Print change rate only during precharge
			if(statusBMS.switchState == SWITCH_PRECHARGE){
				PRINTF_MAIN_DEBUG("\tVoltage change rate %11.8fV/ms, "
						"%5.3f-%5.3f in %ldms\r\n",
						statusBMS.voltageChangeRate_ms, loadVoltage,
						lastExtVoltageMeasurement,
						TIMER_COUNTER_1MS - adcLastVoltageChangeRateTimestamp);
			}
		}
		// If this read is successful store time and last voltage
		lastExtVoltageMeasurement = loadVoltage;
		adcLastVoltageChangeRateTimestamp = TIMER_COUNTER_1MS;
	}
	else{
		// Set change rate variables to a implausible signal
		statusBMS.voltageChangeRate_ms = ADC_EXT_V_CHANGE_RATE_MAX;
		lastExtVoltageMeasurement = ADC_EXT_V_CHANGE_RATE_MAX;
	}
}

//****************************************************************************
// updatePublicDisplayData - Write all runtime data shown on display to public
//							 RAM area where it can be accessed by CM0+
// Returns 1 if CM0+ send a done back
//****************************************************************************
uint8_t updatePublicDisplayData(public_core_commands command){
	static uint8_t cell_V_index = 0;
	uint8_t returnRefreshDone = 0;

	// Limit data refresh time
	if((command == PUBLIC_COMMAND_SHUTDOWN) ||
			(TIMER_COUNTER_1MS > (updatePublicDisplayDataTimestamp +
								  UPDATE_PUBLIC_DISPLAY_DATA_TIME))){
		// Update public variables from shadow buffer
		// if resource is not used at the moment (non blocking)
		if (Cy_IPC_Sema_Status(SEMA_DISPLAY_VAR) != CY_IPC_SEMA_STATUS_LOCKED){
			if(Cy_IPC_Sema_Set(SEMA_DISPLAY_VAR, false) == CY_IPC_SEMA_SUCCESS){
				// Store time of last update
				updatePublicDisplayDataTimestamp = TIMER_COUNTER_1MS;

				// Store latest values in shadow variables
				// Error map
				sh_pub_error_map =
						latched_error_map_switch_on | statusBMS.error_map;

				// SoC and SoH
				sh_pub_soc = statusBMS.SoC;
				sh_pub_soh = statusBMS.SoH;

				// System voltage
				sh_pub_volt =
						(uint16_t)(tle9012_nodes[0].block_voltage*1000.0);

				// System current
				sh_pub_current = statusBMS.current;

				// Slot state
				sh_pub_slotState = statusBMS.slotState;

				// Output state
				sh_pub_outputState = statusBMS.outputState;

				// Cell Voltage and diagnostic
				// (3 MSBs are cellDiagnosticStates, rest is voltage in mV)
				for(uint8_t i = 0; i < NUM_CELLS; i++){
					/// Store voltage value of cell in shadow register
					sh_pub_cells[i] = tle9012_nodes[0].cell_voltages[i];
					// If voltage is to high set it to max
					// (this should never happen)
					if(tle9012_nodes[0].cell_voltages[i] >= CELL_DIAGNOSTIC_MAX_mV){
						sh_pub_cells[i] = CELL_DIAGNOSTIC_MAX_mV;
					}

					/// Add diagnostic state if needed
					if((statusBMS.ol_map & (1 << i)) != 0){
						sh_pub_cells[i] |= CELL_DIAG_OPEN_LOOP;
					}
					else if((statusBMS.uv_map & (1 << i)) != 0){
						sh_pub_cells[i] |= CELL_DIAG_UNDERVOLTAGE;
					}
					else if((statusBMS.ov_map & (1 << i)) != 0){
						sh_pub_cells[i] |= CELL_DIAG_OVERVOLTAGE;
					}
					else if((statusBMS.bal_uc_map & (1 << i)) != 0){
						sh_pub_cells[i] |= CELL_DIAG_BAL_UNDERVOLTAGE;
					}
					else if((statusBMS.bal_oc_map & (1 << i)) != 0){
						sh_pub_cells[i] |= CELL_DIAG_BAL_OVERVOLTAGE;
					}
					else if((statusBMS.bal_cell_map & (1 << i)) != 0){
						sh_pub_cells[i] |= CELL_DIAG_IS_BALANCING;
					}
				}

				// Update variables that are refreshed every cycle
				PUB_RAM_ACCESS(uint32_t, 	 PUB1_ERROR_MAP)   	 = sh_pub_error_map;
				PUB_RAM_ACCESS(uint8_t, 	 PUB2_SOC) 			 = sh_pub_soc;
				PUB_RAM_ACCESS(uint8_t, 	 PUB3_SOH) 			 = sh_pub_soh;
				PUB_RAM_ACCESS(uint16_t, 	 PUB4_VOLT) 		 = sh_pub_volt;
				PUB_RAM_ACCESS(int16_t, 	 PUB5_CURRENT)		 = sh_pub_current;
				PUB_RAM_ACCESS(outputStates, PUB6_OUTPUT_STATE)  = sh_pub_outputState;
				PUB_RAM_ACCESS(slotStates, 	 PUB7_SLOT_STATE)    = sh_pub_slotState;

				// Update variables that are not refreshed every cycle
				// (every cycle the next on will be refreshed)
				PUB_RAM_ACCESS(uint16_t,	 PUB8_CELL_V)     	 = sh_pub_cells[cell_V_index];
				PUB_RAM_ACCESS(uint8_t,		 PUB8_CELL_V_IDX)	 = cell_V_index;
				/*PRINTF_MAIN_DEBUG("Write C %d = %d\r\n",
						cell_V_index, sh_pub_cells[cell_V_index]);*/

				// Increment cell index and manage overflow
				cell_V_index++;
				if(cell_V_index >= NUM_CELLS){
					cell_V_index = 0;
				}

				// Get refresh state to determine if last refresh was done.
				// if not wait till it is. If so write new value
				sh_pub_command = PUB_RAM_ACCESS(uint8_t, PUB9_COMMAND);
				if((command == PUBLIC_COMMAND_SHUTDOWN) &&
						(sh_pub_command != PUBLIC_COMMAND_DONE)){
					PUB_RAM_ACCESS(uint8_t, PUB9_COMMAND) = command;
				}
				while (Cy_IPC_Sema_Clear(SEMA_DISPLAY_VAR, false) != CY_IPC_SEMA_SUCCESS){
					__WFE();
				}

				// Note that refresh was executed
				returnRefreshDone = 1;
			}
		}
	}

	// Return
	return returnRefreshDone;
}

/******************************************************************************
* Function Name: SAR_HW_interrupt
*******************************************************************************
* Summary:
* This function is the handler for SAR_HW interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
/*
void SAR_HW_interrupt(void)
{
    // Check if End-Of-Scan trigger has occurred.
    // If yes, set SAR_HW_isr_set flag to true
    if (Cy_SAR_GetInterruptStatus(SAR_HW) & CY_SAR_INTR_EOS)
    {
        SAR_HW_isr_set = true;
    }
    // Clear the interrupts
    Cy_SAR_ClearInterrupt(SAR_HW, CY_SAR_INTR);
}*/

//****************************************************************************
// debugButtonCheck
//****************************************************************************
void debugButtonCheck(){
	// Wait till semaphore is free and check button pressed
	static uint8_t btn_pressed = 0;

	// If button was detected as pressed send message
	// and increment counter (Block until semaphore is free)
	if(cyhal_gpio_read(CYBSP_SWITCH) == 1 && (btn_pressed == 0)){
		// Reset switch on counter to make switch on possible again
		reset_safetySwitchTurnOn();

		//// Trigger safe state
		/*
		saveStateEn_setLOW(); //cyhal_gpio_toggle(D_2ED4820_SAVE_STATE_EN);
		PRINTF_MAIN_DEBUG("D_2ED4820_SAVE_STATE_EN  %d \r\n",
				cyhal_gpio_read(D_2ED4820_SAVE_STATE_EN));*/

		//// Trigger watchdog to test kill of system.
		// Use first while to test software watchdog.
		// Use second while to test hardware watchdog
		/*while(1){
			1;
		}
		while(1){
		// Keep software watchdog alive
		// Increment main loop counter to keep watchdog handler inactive
			mainLoopCounter++;
		}*/

		//// Simulate communication errors
		/*
		PRINTF_MAIN_DEBUG("ComErr %d\r\n", numComErrors_TLE9012);
		numComErrors_TLE9012 = 99;
		PRINTF_MAIN_DEBUG("ComErr %d\r\n", numComErrors_TLE9012);

		// Test soft reset of controller
		NVIC_SystemReset();

		// Print stored RT data memory as CSV
		isWatchdogActive = 0;
		memoryGetRtData(NULL,
				nextMemoryRtLineAddressOffset/MEM_SIZE_RT_DATA, 0, 1);
		isWatchdogActive = 1;

		//// Send a UART message (Attempt to lock the semaphore)
		while (Cy_IPC_Sema_Set(SEMA_DEBUG_MSG, false) != CY_IPC_SEMA_SUCCESS){
			__WFE();
		}

		//// Read the ADC conversion result for corresponding ADC channel.
		PRINTF_MAIN_DEBUG("Channel 0 CodeRes   input: %ldmV\r\n",
				adc_coding_res_mV);
		PRINTF_MAIN_DEBUG("Channel 1 SWCurrent input: %ld, %ldmV\r\n",
				adc_safety_switch_current, adc_safety_switch_current_mV);

		//// Write RTC time
		Cy_RTC_GetDateAndTime(&dateTime);
		PRINTF_MAIN_DEBUG("Time: %02ld:%02ld:%02ld\r\n",
				dateTime.hour,dateTime.min,dateTime.sec);

		//// Block until semaphore is freed
		while (Cy_IPC_Sema_Clear(SEMA_DEBUG_MSG, false) != CY_IPC_SEMA_SUCCESS){
			__WFE();
		}

		//// Send switch on veto message to potential other BMS
		uint8_t targetData = 0;
		if(statusBMS.slotState == SLOT_IN_SYSTEM_1){
			CAN_TX_Request(BMS_CLASS_MASK+1, BMS_SwitchOn_Veto, &targetData, 1);
		}
		else if(statusBMS.slotState == SLOT_IN_SYSTEM_2){
			CAN_TX_Request(BMS_CLASS_MASK, BMS_SwitchOn_Veto, &targetData, 1);
		}*/

		// Mark button as pressed
		btn_pressed = 1;
	}
	else if(cyhal_gpio_read(CYBSP_SWITCH) == 0){
		// Reset button pressed
		btn_pressed = 0;
	}
}

#define SIZE_CYCLE_TIME_ARRAY 200

void debugRecordCycleTime(){
	static uint32_t lastCycleTimestamp = 0;
	static uint16_t cycleTimeArray[SIZE_CYCLE_TIME_ARRAY] = {0};
	static uint16_t  cycleTimeArrayIndex = 0;

	// Store SIZE_CYCLE_TIME_ARRAY times the cycle time
	cycleTimeArray[cycleTimeArrayIndex] =
			TIMER_COUNTER_1MS - lastCycleTimestamp;
	lastCycleTimestamp = TIMER_COUNTER_1MS;
	cycleTimeArrayIndex++;

	// If buffer runs over print all cycle times
	if(cycleTimeArrayIndex >= SIZE_CYCLE_TIME_ARRAY-1){
		PRINTF_MAIN_DEBUG("Cycle time records in ms: %d", cycleTimeArray[0]);
		for(uint8_t i = 1; i < SIZE_CYCLE_TIME_ARRAY; i++){
			PRINTF_MAIN_DEBUG(", %d", cycleTimeArray[i]);
		}
		PRINTF_MAIN_DEBUG("\r\n");
		cycleTimeArrayIndex = 0;
	}
}

/* [] END OF FILE */
