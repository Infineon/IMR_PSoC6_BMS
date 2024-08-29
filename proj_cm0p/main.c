/******************************************************************************
 * File Name:   main.c (CM0+)
 * Created on: 1 Jun 2024
 * Author: r. santeler (based on MTB example)
 *
 * Description: This is the source code for CM0+ in the PSoC6_BatteryManagement.
 *              Here the control of the E-Ink display, the buzzer and the LEDs on
 *              the PSoc6_ board is hosted. Code hosted on this processor must
 *              never be safety critical and is allowed to to use blocking layouts.
 *              The main functions are described in this file. The display design
 *              and control is found in PSoC6_BMS_EPD_ScreenConfig. Note that
 *              "ram_public.h" and "ipc_def.h" is used for communication between
 *              processors and the public RAM area.
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "stdio.h"
#include "cy_pdl.h"
#include "cycfg.h"
#include "cybsp.h"
#include "cyhal.h"
#include "ipc_def.h"
#include "ram_public.h"

#include <PSoC6_BMS_EPD_ScreenConfig.h>

/*******************************************************************************
* Macros
*******************************************************************************/
// #define BUZZER_COMPLEX_SOUND_ENABLE			(1)
#define CM0_REFRESH_PERIOD_MIN			     (1500)		// in ms. Minimum time between any display refresh
#define CM0_REFRESH_PERIOD					(10000)		// in ms. Time between regular display refresh
#define CM0_MAXIMUM_SHUTDOWN_TIME			 (6500)		// in ms. Determines how long the button must be pressed continuously to trigger hard shutdown. Note that this code also runs on CM4 to make sure a shutdown is possible even if one core stops working. The value for the CM4 core must be set in it main.c
#define CM0_MAIN_LOOP_DELAY					   (18)		// in ms. Limits the execution of the main loop
#define DISPLAY_PAGE_BUTTON_IGNORE_TIME		 (7000)		// in ms. Time that must pass after power on before the display page button will change the shown display screen. This is needed because any press of this button will be overrulled by state changes at beginning


#define TIMER_COUNTER_1MS 					Cy_TCPWM_Counter_GetCounter(TIMER_1MS_HW, TIMER_1MS_NUM)


/*******************************************************************************
* Piezo Buzzer Definitions
*******************************************************************************/
cyhal_pwm_t buzzer_pwm_obj;

#ifdef BUZZER_COMPLEX_SOUND_ENABLE
#include <tones.h>

uint16_t timeStampMusic = 0;
#endif


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void gpio_interrupt_button_display_handler(void* handler_arg, cyhal_gpio_event_t event);
void loadPublicDisplayData(public_core_commands command);
void print_bmp_to_uart();


/*******************************************************************************
* Global Variables
*******************************************************************************/
// Flags
uint8_t buttonPressed_Page = 0;
uint8_t updateDisplay_regular = 0;
uint8_t updateDisplay_stateChange = 0;
uint8_t updateDisplay_pageButtonPressed = 0;
uint8_t lastUpdateBeforeShutdownExecuted = 0;

// Timestamps for time checks
uint32_t cm0NextRegularDisplayRefreshTimestamp = 0;
uint32_t cm0RunTimeCounterTimestamp = 0;
uint32_t cm0RunTimeStartTimestamp = 0;
uint32_t cm0LastDisplayRefreshTimestamp = 0;

// External objects from display library
extern EPD_E2200CS021_pages_t current_page;
extern EPD_E2200CS021_status_t EPD_Status_Obj;
extern uint8_t *current_frame;						/* Pointer to the new frame that need to be written */
extern uint8_t previous_frame[PV_EINK_IMAGE_SIZE];	/* Buffer to the previous frame written on the display */

// String buffer for debug messages
char strBufUART[200];

// Shadow buffer for public variables
uint32_t  	 sh_pub_error_map_cm0 = 0;
uint8_t 	 sh_pub_soc_cm0 = 0;
uint8_t 	 sh_pub_soh_cm0 = 0;
uint16_t 	 sh_pub_volt_cm0 = 0;
int16_t 	 sh_pub_current_cm0 = 0;
outputStates sh_pub_outputState_cm0 = OUTPUT_OFF;
slotStates 	 sh_pub_slotState_cm0 = SLOT_NONE;
uint16_t 	 sh_pub_cells_cm0[NUM_CELLS];
public_core_commands 	 sh_pub_command_cm0;
// Last state of critical states (that should trigger a display refresh)
uint8_t outputState_changed;
uint8_t slotState_changed;

/*******************************************************************************
* Function Definitions
*******************************************************************************/

#ifdef BUZZER_COMPLEX_SOUND_ENABLE
// Note: To use this it needs to be implemented in main loop, without timer
void IRQ_buzzer_timer(void* callback_arg, cyhal_timer_event_t event) {
	int16_t current_freq = -1;
	current_freq = playMelody(timeStampMusic, melody_startup, noteDurations_startup);

	if (current_freq <= 0) {
		if (current_freq < 0)
				timeStampMusic = 0;
		cyhal_pwm_stop(&buzzer_pwm_obj);
		cyhal_timer_stop(&buzzer_timer_10s_obj);
	} else {
	    cyhal_pwm_set_duty_cycle(&buzzer_pwm_obj, 50, current_freq);			// Set a duty cycle of 10% and frequency of current_freq
	    cyhal_pwm_start(&buzzer_pwm_obj);
	}

	timeStampMusic++;
}
#endif

// Display page increment interrupt
cyhal_gpio_callback_data_t cb_data = {
    .callback     = gpio_interrupt_button_display_handler
};
void gpio_interrupt_button_display_handler(void* handler_arg, cyhal_gpio_event_t event) { 		// Interrupt handler callback function
	CY_UNUSED_PARAMETER(handler_arg);
	CY_UNUSED_PARAMETER(event);

	cyhal_gpio_write(LED_Blue, 0);
	buttonPressed_Page = 1;
}

//Init of display page increment button interrupt, buzzer pwm, and LEDs on the PSoC Controller board
void Initialize_GPIO(void) {
	cyhal_gpio_init(BUTTON_DISPLAY, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0);	// Replaced by Device Configurator
	cyhal_gpio_register_callback(BUTTON_DISPLAY, &cb_data);		    								// Register callback function - gpio_interrupt_button_display_handler and pass the value global_count
	cyhal_gpio_enable_event(BUTTON_DISPLAY, CYHAL_GPIO_IRQ_RISE, CYHAL_ISR_PRIORITY_DEFAULT, 1);	// Enable falling edge interrupt event with interrupt priority set to 3

    cyhal_pwm_init(&buzzer_pwm_obj, P2_2, NULL);		// Initialize PWM on the supplied pin and assign a new clock
    cyhal_pwm_set_duty_cycle(&buzzer_pwm_obj, 10, 1);	// Set a duty cycle of 50% and frequency of 1Hz

	cyhal_gpio_init(LED_Blue, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	cyhal_gpio_write(LED_Blue, CYBSP_LED_STATE_OFF);

    cyhal_gpio_init(LED_Green, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write(LED_Green, CYBSP_LED_STATE_OFF);

    cyhal_gpio_init(LED_Red, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write(LED_Red, CYBSP_LED_STATE_OFF);
}

void checkHardShutdownCondition(){
	// Kill system if shutdown button is continuously pressed for multiple seconds (this code also runs on CM4 to make sure a shutdown is possible even if one core stops working)
	static uint32_t shutdownButtonNotPressedTimestamp;
	if(cyhal_gpio_read(BUTTON_SHUTDOWN) == 0)
		shutdownButtonNotPressedTimestamp = TIMER_COUNTER_1MS;
	if(TIMER_COUNTER_1MS > (shutdownButtonNotPressedTimestamp + CM0_MAXIMUM_SHUTDOWN_TIME)){
		cyhal_gpio_write(VCC_HOLD, 0);
   	}
}

/******************************************************************************/
int main(void) {
    cy_rslt_t result = cybsp_init() ;		/* Initialize the device and board peripherals */

    if (result != CY_RSLT_SUCCESS)
        CY_ASSERT(0);

    /* Enable global interrupts */
    __enable_irq();

    Initialize_EPD(AMBIENT_TEMPERATURE_DEFAULT_C); // Set ambient temperature, in degree C, in order to perform temperature compensation of E-INK parameters. Todo: Make this dynamic
    Initialize_GPIO();

    // Set initial value of public variables
    PUB_RAM_ACCESS(uint32_t, 	 PUB1_ERROR_MAP) 	= (uint32_t)(0);
    PUB_RAM_ACCESS(uint8_t, 	 PUB2_SOC) 	  		= (uint8_t)(0);
    PUB_RAM_ACCESS(uint8_t, 	 PUB3_SOH) 			= (uint8_t)(0);
    PUB_RAM_ACCESS(uint16_t, 	 PUB4_VOLT) 		= (uint16_t)(0);
    PUB_RAM_ACCESS(int16_t, 	 PUB5_CURRENT) 		= (int16_t)(0);
    PUB_RAM_ACCESS(outputStates, PUB6_OUTPUT_STATE) = (outputStates)(0);
    PUB_RAM_ACCESS(slotStates, 	 PUB7_SLOT_STATE) 	= (slotStates)(0);
    PUB_RAM_ACCESS(uint16_t, 	 PUB8_CELL_V)		= (uint16_t)(0);
    PUB_RAM_ACCESS(uint8_t, 	 PUB8_CELL_V_IDX)	= (uint8_t)(0);
    PUB_RAM_ACCESS(uint8_t, 	 PUB9_COMMAND) 	    = (uint8_t)(0);

    /* Lock the semaphore to wait for CM4 to be init */
    Cy_IPC_Sema_Set(SEMA_START, 0);
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);	/* Enable CM4. CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */

    // Play startup sound
	//cyhal_pwm_set_duty_cycle(&buzzer_pwm_obj, 50, 500);			// Set a duty cycle of 10% and frequency of current_freq
	//cyhal_pwm_start(&buzzer_pwm_obj);
	//cyhal_system_delay_ms(250);
	//cyhal_pwm_set_duty_cycle(&buzzer_pwm_obj, 50, 1400);			// Set a duty cycle of 10% and frequency of current_freq
	//cyhal_system_delay_ms(250);
	//cyhal_pwm_stop(&buzzer_pwm_obj);
	//
	//cyhal_system_delay_ms(1000); // DELAY_AFTER_STARTUP_SCREEN_MS);
	//
	//cyhal_pwm_set_duty_cycle(&buzzer_pwm_obj, 50, 1200);			// Set a duty cycle of 10% and frequency of current_freq
	//cyhal_pwm_start(&buzzer_pwm_obj);
	//cyhal_system_delay_ms(250);
	//cyhal_pwm_set_duty_cycle(&buzzer_pwm_obj, 50, 350);			// Set a duty cycle of 10% and frequency of current_freq
	//cyhal_system_delay_ms(250);
	//cyhal_pwm_stop(&buzzer_pwm_obj);

    // Wait till CM4 unlocks the semaphore
    do {
    	checkHardShutdownCondition();
        //__WFE();
    }
    while (Cy_IPC_Sema_Status(SEMA_START) == CY_IPC_SEMA_STATUS_LOCKED);

    // Update display variables for the first time
    cyhal_system_delay_ms(100);
    loadPublicDisplayData(PUBLIC_COMMAND_REFRESH);

    // Update display for the first time
    first_screen_update();
    cyhal_system_delay_ms(100);

    // ---- MAIN LOOP CM0+ ----
    for (;;)
    {
    	// Check if on time counter must be refreshed (every second), and update run time
    	if(TIMER_COUNTER_1MS > (cm0RunTimeCounterTimestamp + 1000)){
    		// Store current time as last execution time
    		cm0RunTimeCounterTimestamp = TIMER_COUNTER_1MS;

        	// Update active seconds based on start time or reset it if output is not on
        	if(sh_pub_outputState_cm0 == OUTPUT_ON){
        		// If active seconds was previously reset there must have been a reset - store start time
        		if(EPD_Status_Obj.Active_Seconds == 254){
        			cm0RunTimeStartTimestamp = TIMER_COUNTER_1MS;
        			cm0NextRegularDisplayRefreshTimestamp = cm0RunTimeStartTimestamp + 1000;
        			EPD_Status_Obj.Active_Seconds = 0;
        		}
        	}
        	else{
        		// Reset run time
        		EPD_Status_Obj.Active_Seconds = 254;
        		EPD_Status_Obj.Active_Minutes = 0;
        		EPD_Status_Obj.Active_Hours = 0;
        	}

            // Calculate hours, minutes, and remaining seconds
        	uint32_t runTimeS = (TIMER_COUNTER_1MS - cm0RunTimeStartTimestamp) / 1000;
        	EPD_Status_Obj.Active_Hours   = (runTimeS / 3600);
            EPD_Status_Obj.Active_Minutes = (runTimeS - (3600*EPD_Status_Obj.Active_Hours)) / 60;
            EPD_Status_Obj.Active_Seconds = (runTimeS - (3600*EPD_Status_Obj.Active_Hours) - (EPD_Status_Obj.Active_Minutes * 60));
    	}


		// Check if display must be refreshed because display button is pressed (interrupt or currently pressed) and a page update isn't already triggered
		if((buttonPressed_Page == 1 || cyhal_gpio_read(BUTTON_DISPLAY)) && (updateDisplay_pageButtonPressed != 1)){
			// Set button pressed marker to  keep this active on startup till the DISPLAY_PAGE_BUTTON_IGNORE_TIME has passed
			buttonPressed_Page = 1;

			// Turn on user indicator LED
			cyhal_gpio_write(LED_Blue, 0);

			// Do not actually perform the display update until the startup phase is done
			if(TIMER_COUNTER_1MS > DISPLAY_PAGE_BUTTON_IGNORE_TIME){
				// Reset interrupt marker
				buttonPressed_Page = 0;

				// Mark that display must be updated and prevent this code to run until it is
				updateDisplay_pageButtonPressed = 1;
			}
		}

    	// Check if display must be refreshed because of regular refresh or press of display page button
    	if((EPD_Status_Obj.Active_Seconds % (CM0_REFRESH_PERIOD/1000)) == 0){//TIMER_COUNTER_1MS > cm0NextRegularDisplayRefreshTimestamp){ //|| ((Update_Display_Trigger == 1) && (TIMER_COUNTER_1MS > (cm0NextRegularDisplayRefreshTimestamp-CM0_REFRESH_PERIOD+CM0_REFRESH_PERIOD_MIN)))){
    		// Store current time as last execution time
    		cm0NextRegularDisplayRefreshTimestamp += CM0_REFRESH_PERIOD;

    		// Mark that display must be updated
    		updateDisplay_regular = 1;
    	}

    	// Check if display must be refreshed because a important state changed
    	if(slotState_changed != 0 || outputState_changed != 0){
    		// Reset changed marker
    		slotState_changed = 0;
    		outputState_changed = 0;

    		// Mark that display must be updated
    		updateDisplay_stateChange = 1;
    	}


    	// If the minimum display refresh time has passed and one of the update triggers is set, perform update and reset everything
    	loadPublicDisplayData(PUBLIC_COMMAND_IGNORE);
    	if((TIMER_COUNTER_1MS > (cm0LastDisplayRefreshTimestamp + CM0_REFRESH_PERIOD_MIN))
    	   && (updateDisplay_regular | updateDisplay_pageButtonPressed | updateDisplay_stateChange | (sh_pub_command_cm0 == PUBLIC_COMMAND_SHUTDOWN))
		   && (lastUpdateBeforeShutdownExecuted == 0)){
    		// If the refresh was done during shutdown, mark that the display must not be refreshed anymore and set page index to main page
			if(sh_pub_command_cm0 == PUBLIC_COMMAND_SHUTDOWN){
				current_page = 0;
				lastUpdateBeforeShutdownExecuted = 1;
			}

			// Increment display page index
			if(updateDisplay_stateChange == 1){
	    		// Let the main page be shown
	    		current_page = 0;
			}
			else if(updateDisplay_pageButtonPressed == 1){
				// Change shown display page
				current_page++;
				if (current_page >= TOTAL_PAGE_COUNT){
					current_page = 0;
				}
			}

    		// Update public variables and signalizes that the display is being updated
			loadPublicDisplayData(PUBLIC_COMMAND_REFRESH);

    		// Store current time as last execution time
    		cm0LastDisplayRefreshTimestamp = TIMER_COUNTER_1MS;

    		// Update display
    		current_page = show_page(current_page);
			mtb_e2200cs021_show_frame(previous_frame, current_frame, MTB_E2200CS021_FULL_2STAGE, 1);

			// Display update due to page button press: Turn off user indicator and reset flag only if it was actually executed (this could also be done in if cause above but then the LED does not stay on till update is finished -> flicker)
			if(updateDisplay_pageButtonPressed == 1 && updateDisplay_stateChange != 1){
				// Reset display update
				updateDisplay_pageButtonPressed = 0;

				// Reset user indicator LED
				cyhal_gpio_write(LED_Blue, 1);
			}

    		// Reset display update
			updateDisplay_regular = 0;
    		updateDisplay_stateChange = 0;
    		updateDisplay_pageButtonPressed = 0;

			// Update public variables and signalizes that the display has finished updating
			loadPublicDisplayData(PUBLIC_COMMAND_IGNORE);
    	}
    	else if(lastUpdateBeforeShutdownExecuted != 0){
    		// Update public variables and keep shutdown command
    		while(1){
    			//cyhal_gpio_write(LED_Red, 1);
    			cyhal_system_delay_ms(50);
    			loadPublicDisplayData(PUBLIC_COMMAND_DONE);
    		}
    	}
    	else{
    		// Update public variables normally if display is not updated
			loadPublicDisplayData(PUBLIC_COMMAND_IGNORE);
    	}

        // Kill system if shutdown button is continuously pressed for multiple seconds (this code also runs on CM4 to make sure a shutdown is possible even if one core stops working)
    	checkHardShutdownCondition();

    	// Check if debug button was detected
    	static uint8_t btn_pressed = 0;
    	if(cyhal_gpio_read(CYBSP_SWITCH) == 1 && (btn_pressed == 0)){
    		// DEBUG SCREENSHOT: If the debug button is pressed print a screenshot to debug UART
    		//print_bmp_to_uart();

			//// UART synched with SEMA_DEBUG_MSG (not used currently)
			//while (Cy_IPC_Sema_Set(  SEMA_DEBUG_MSG, 0) != CY_IPC_SEMA_SUCCESS){__WFE();}
			////sprintf(strBufUART, "*Test CM0+ - ui16 %d, ui32 %lu\r\n", PUB_RAM_ACCESS(uint16_t, PUB10_NUM16), PUB_RAM_ACCESS(uint32_t, PUB11_NUM32));
			//sprintf(strBufUART, "*Test CM0+\r\n");
			//Cy_SCB_UART_PutString(CYBSP_UART_HW, strBufUART);
			//while (Cy_IPC_Sema_Clear(SEMA_DEBUG_MSG, 0) != CY_IPC_SEMA_SUCCESS){__WFE();}

    		// Mark button as pressed
    		btn_pressed = 1;
    	}
    	else if(cyhal_gpio_read(CYBSP_SWITCH) == 0){
    		// Reset button pressed
    		btn_pressed = 0;
    	}

    	// Cycle time limitation
    	cyhal_system_delay_ms(CM0_MAIN_LOOP_DELAY);
    }
}

void loadPublicDisplayData(public_core_commands command){
	// Load shadow buffer from public variables
	while (Cy_IPC_Sema_Status(SEMA_DISPLAY_VAR)        == CY_IPC_SEMA_STATUS_LOCKED){__WFE();}
	while (Cy_IPC_Sema_Set(   SEMA_DISPLAY_VAR, 0) != CY_IPC_SEMA_SUCCESS){__WFE();}

	// Check if state changed
	if(sh_pub_outputState_cm0 != PUB_RAM_ACCESS(outputStates, PUB6_OUTPUT_STATE)){
		outputState_changed = 1;
	}
	if(sh_pub_slotState_cm0   != PUB_RAM_ACCESS(slotStates, 	 PUB7_SLOT_STATE)){
		slotState_changed = 1;
	}

	// Update variables
	sh_pub_error_map_cm0 = 		PUB_RAM_ACCESS(uint32_t, 	 PUB1_ERROR_MAP);
	sh_pub_soc_cm0 = 			PUB_RAM_ACCESS(uint8_t, 	 PUB2_SOC);
	sh_pub_soh_cm0 = 			PUB_RAM_ACCESS(uint8_t, 	 PUB3_SOH);
	sh_pub_volt_cm0 = 			PUB_RAM_ACCESS(uint16_t, 	 PUB4_VOLT);
	sh_pub_current_cm0 =		PUB_RAM_ACCESS(int16_t, 	 PUB5_CURRENT);
	sh_pub_outputState_cm0 = 	PUB_RAM_ACCESS(outputStates, PUB6_OUTPUT_STATE);
	sh_pub_slotState_cm0 = 		PUB_RAM_ACCESS(slotStates, 	 PUB7_SLOT_STATE);

	// Get current cell voltage/diagnostic from public ram area and store it in shadow buffer
	uint8_t cell_V_index = PUB_RAM_ACCESS(uint8_t, PUB8_CELL_V_IDX);
	sh_pub_cells_cm0[cell_V_index] = PUB_RAM_ACCESS(uint16_t, PUB8_CELL_V);

	// UART synched with SEMA_DEBUG_MSG
	//while (Cy_IPC_Sema_Set(  SEMA_DEBUG_MSG, 0) != CY_IPC_SEMA_SUCCESS){__WFE();}
	//sprintf(strBufUART, "*Test CM0+: Read C %d = %d\r\n", cell_V_index, sh_pub_cells_cm0[cell_V_index]);
	//Cy_SCB_UART_PutString(CYBSP_UART_HW, strBufUART);
	//while (Cy_IPC_Sema_Clear(SEMA_DEBUG_MSG, 0) != CY_IPC_SEMA_SUCCESS){__WFE();}

	// Return refresh done
	sh_pub_command_cm0 = PUB_RAM_ACCESS(uint8_t, PUB9_COMMAND);
	if(command != PUBLIC_COMMAND_IGNORE){
		PUB_RAM_ACCESS(uint8_t, PUB9_COMMAND) = command;
	}
	while (Cy_IPC_Sema_Clear(SEMA_DISPLAY_VAR,  0) != CY_IPC_SEMA_SUCCESS){__WFE();}
}

/******************************************************************************/
// print_bmp - Use this to print the current display frame as BMP8 to command line
//			   The output will interfere with output from CM4! Deactivate all debug messages before using this!
static void write_byte_to_uart(uint8_t Data, void * p) {
	// Print current byte to display buffer
	char strBufUART[20];
	sprintf(strBufUART, "%02X ", Data);
	Cy_SCB_UART_PutString(CYBSP_UART_HW, strBufUART);
}
void print_bmp_to_uart(){
	char strBufUART[10];
	// Print debug message
	strBufUART[0] = 'B';
	strBufUART[1] = 'M';
	strBufUART[2] = 'P';
	strBufUART[3] = ':';
	strBufUART[4] = '\r';
	strBufUART[5] = '\n';
	strBufUART[6] = '\0';
	Cy_SCB_UART_PutString(CYBSP_UART_HW, strBufUART);
	// Print BMP data
    GUI_BMP_Serialize(write_byte_to_uart, NULL);
    // Print new line to push buffer to UART
	strBufUART[0] = '\r';
	strBufUART[1] = '\n';
	strBufUART[2] = '\0';
	Cy_SCB_UART_PutString(CYBSP_UART_HW, strBufUART);
}

/* [] END OF FILE */
