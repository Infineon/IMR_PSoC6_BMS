/*
 * PSoC6_BMS_EPD_ScreenConfigs.c - E-Ink display menu settings and definition
 *
 *  Created on: 07.07.2023
 *      Author: m. schmidt and r. santeler
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

#include <PSoC6_BMS_EPD_ScreenConfig.h>
#include "ram_public.h"
#include "ipc_def.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define CELL_SOC_LOWER 		2.8	// The cell state symbol is printed mapping the cell voltage between CELL_SOC_LOWER and CELL_SOC_UPPER
#define CELL_SOC_UPPER 		4.2
#define MAX_SHOWN_ERRORS 	3   // Maximum number of errors that can be shown on display

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
uint8_t print_DispErrors(uint32_t error_map, uint8_t numErrors, uint8_t maxErrorsPerLine);
uint8_t countSetBits(uint32_t n);
float mapRangeToScale(float n, float lRange, float hRange, float scale);

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Shadow buffer for public variables*/

extern uint32_t		 sh_pub_error_map_cm0;
extern uint8_t 	 	 sh_pub_soc_cm0;
extern uint8_t 	 	 sh_pub_soh_cm0;
extern uint16_t 	 sh_pub_volt_cm0;
extern int16_t 	 	 sh_pub_current_cm0;
extern outputStates  sh_pub_outputState_cm0;
extern slotStates 	 sh_pub_slotState_cm0;
extern uint16_t 	 sh_pub_cells_cm0[NUM_CELLS];

uint16_t Full_SOC = 99;
uint16_t Full_SOH = 99;
uint16_t Full_Volt = 44200;
uint16_t Full_Current = 980;

EPD_E2200CS021_status_t EPD_Status_Obj = { 	.Active_Hours = 0,
											.Active_Minutes = 0,
											.Active_Seconds = 254,
											.OUTPUT_Status = OUTPUT_READY,
											.SLOT_Status = SLOT_NONE,
											.error_map = 0,
											.soc = 0,
											.soh = 0,
											.voltage_data = 0,
											.current_data = 0};

uint16_t Cell_SOC[NUM_CELLS] = { 	8, 20, 30, 40, 100, 100,
											75, 78, 31, 45, 76, 8};
uint16_t Cell_Volt[NUM_CELLS] = { 	3750, 4120, 2950, 3910, 2094, 1029,
											3049, 1092, 4180, 3600, 3832, 2930};

cyhal_spi_t spi;	/* HAL SPI object to interface with display driver */
EPD_E2200CS021_pages_t current_page = 0;

uint8_t *current_frame;		/* Pointer to the new frame that need to be written */
uint8_t previous_frame[PV_EINK_IMAGE_SIZE] = { 0 };		/* Buffer to the previous frame written on the display */

const mtb_e2200cs021_pins_t pins = {		/* Configuration structure defining the necessary pins to communicate with the E-ink display */
    .spi_mosi 	= EPD_PIN_DISPLAY_SPI_MOSI,
    .spi_miso 	= EPD_PIN_DISPLAY_SPI_MISO,
    .spi_sclk 	= EPD_PIN_DISPLAY_SPI_SCLK,
    .spi_cs 	= EPD_PIN_DISPLAY_CS,
    .reset 		= EPD_PIN_DISPLAY_RST,
    .busy 		= EPD_PIN_DISPLAY_BUSY,
    .discharge 	= EPD_PIN_DISPLAY_DISCHARGE,
    .enable 	= EPD_PIN_DISPLAY_EN
};

/*******************************************************************************
* Function Definitions
*******************************************************************************/
void Initialize_EPD(int8_t displayTempFactor) {
    /* Initialize SPI and EINK display */
    cy_rslt_t result = cyhal_spi_init(&spi, EPD_PIN_DISPLAY_SPI_MOSI,
            								EPD_PIN_DISPLAY_SPI_MISO,
											EPD_PIN_DISPLAY_SPI_SCLK,
											NC, NULL, 8, CYHAL_SPI_MODE_00_MSB, false);

    if (CY_RSLT_SUCCESS == result) {
        result = cyhal_spi_set_frequency(&spi, EPD_SPI_BAUDRATE_HZ);

        if (CY_RSLT_SUCCESS == result) {
            result = mtb_e2200cs021_init(&pins, &spi);

            mtb_e2200cs021_set_temp_factor(displayTempFactor);	/* Set ambient temperature, in degree C, in order to perform temperature compensation of E-INK parameters */
            current_frame = (uint8_t*) LCD_GetDisplayBuffer();

            GUI_Init();		/* Initialize EmWin driver*/
            //first_screen_update();

            //vTaskDelay(DELAY_AFTER_STARTUP_SCREEN_MS);
            //cyhal_system_delay_ms(DELAY_AFTER_STARTUP_SCREEN_MS);
        }
    }
}

/******************************************************************************/
void first_screen_update(void) {
    current_page = show_page(EPD_MainScreen);

    /* Update the display */
    mtb_e2200cs021_show_frame(previous_frame, current_frame,
                              MTB_E2200CS021_FULL_4STAGE, true);
}


/******************************************************************************/
void clear_screen(void) {
    GUI_SetColor(GUI_BLACK);
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
}

/******************************************************************************/
void show_top_legend(char legend_name[], EPD_E2200CS021_pages_t page) {
	/* Set foreground and background color and font size */
	GUI_SetFont(GUI_FONT_10_1);
	GUI_SetColor(GUI_BLACK);
	GUI_SetBkColor(GUI_WHITE);
	GUI_FillRect(0, 0, MTB_E2200CS021_DISPLAY_SIZE_X, TOP_LEGEND_HEIGHT);

	GUI_SetColor(GUI_WHITE);
	GUI_SetBkColor(GUI_BLACK);
	GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
	GUI_DispStringAt(legend_name, 6, (TOP_LEGEND_HEIGHT / 2));
	GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_DispStringAt("PAGE    /    ", MTB_E2200CS021_DISPLAY_SIZE_X, (TOP_LEGEND_HEIGHT / 2));
    GUI_DispDecAt((page + 1), MTB_E2200CS021_DISPLAY_SIZE_X - 22, (TOP_LEGEND_HEIGHT / 2), 1);
    GUI_DispDecAt(TOTAL_PAGE_COUNT, MTB_E2200CS021_DISPLAY_SIZE_X - 10, (TOP_LEGEND_HEIGHT / 2), 1);
}

/******************************************************************************/
void show_battery_page(uint16_t start_battery, uint16_t battery_voltage_data[]) {
	GUI_SetColor(GUI_BLACK);
	GUI_SetBkColor(GUI_WHITE);

	GUI_SetFont(GUI_FONT_13B_1);
	uint8_t Pos_Counter = 1;
	for (uint8_t i = start_battery; (i <= NUM_CELLS) && (Pos_Counter <= BATTERY_PER_PAGE); i++) {
		// Determine diagnostic, cell voltage and mapped SoC
		uint16_t cell_diagnotic_state = battery_voltage_data[i - 1] & CELL_DIAGNOSTIC_STATES_MAP;
		float cell_voltage    = (float)(battery_voltage_data[i - 1] & CELL_VOLTAGE_MAP) / 1000.0;
		Cell_SOC[i - 1] = (uint16_t)mapRangeToScale(cell_voltage, CELL_SOC_LOWER, CELL_SOC_UPPER, 100.0);

		// Print cell SoC
		GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
	    GUI_DispDecAt(i,	BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_X + 4,
	    					BATTERY_TEXT_TOP_LOCATION, 2);

	    GUI_DrawRect(	BATTERY_CENTER_LOCATION_X * Pos_Counter - (BATTERY_SYMBOL_SIZE_X / 2) - BATTERY_SYMBOL_HEAD_SIZE,
	    				MTB_E2200CS021_DISPLAY_SIZE_Y_HALF - (BATTERY_SYMBOL_SIZE_Y / 2) - 2,
						BATTERY_CENTER_LOCATION_X * Pos_Counter - (BATTERY_SYMBOL_SIZE_X / 2) + BATTERY_SYMBOL_HEAD_SIZE,
						MTB_E2200CS021_DISPLAY_SIZE_Y_HALF - (BATTERY_SYMBOL_SIZE_Y / 2) - 1);

	    GUI_DrawRoundedRect(BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_X + BATTERY_SYMBOL_SIZE_BORDER,
	    					MTB_E2200CS021_DISPLAY_SIZE_Y_HALF - (BATTERY_SYMBOL_SIZE_Y / 2),
							BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_BORDER,
							MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2), 2);

	    if (Cell_SOC[i - 1] >= 10) {
	    	GUI_FillRoundedRect(BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_X + BATTERY_SYMBOL_SIZE_BORDER + BATTERY_SYMBOL_SIZE_BORDER,
	    						MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - (BATTERY_SYMBOL_SIZE_Y / 5),
								BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_BORDER - BATTERY_SYMBOL_SIZE_BORDER,
								MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - BATTERY_SYMBOL_SIZE_BORDER, 2);
	    }
	    if (Cell_SOC[i - 1] >= 30) {
			GUI_FillRoundedRect(BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_X + BATTERY_SYMBOL_SIZE_BORDER + BATTERY_SYMBOL_SIZE_BORDER,
								MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - (BATTERY_SYMBOL_SIZE_Y / 5) * 2,
								BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_BORDER - BATTERY_SYMBOL_SIZE_BORDER,
								MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - (BATTERY_SYMBOL_SIZE_Y / 5) - BATTERY_SYMBOL_SIZE_BORDER, 2);
	    }
	    if (Cell_SOC[i - 1] >= 50) {
			GUI_FillRoundedRect(BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_X + BATTERY_SYMBOL_SIZE_BORDER + BATTERY_SYMBOL_SIZE_BORDER,
								MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - (BATTERY_SYMBOL_SIZE_Y / 5) * 3,
								BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_BORDER - BATTERY_SYMBOL_SIZE_BORDER,
								MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - (BATTERY_SYMBOL_SIZE_Y / 5) * 2 - BATTERY_SYMBOL_SIZE_BORDER, 2);
	    }
	    if (Cell_SOC[i - 1] >= 70) {
			GUI_FillRoundedRect(BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_X + BATTERY_SYMBOL_SIZE_BORDER + BATTERY_SYMBOL_SIZE_BORDER,
								MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - (BATTERY_SYMBOL_SIZE_Y / 5) * 4,
								BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_BORDER - BATTERY_SYMBOL_SIZE_BORDER,
								MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - (BATTERY_SYMBOL_SIZE_Y / 5) * 3 - BATTERY_SYMBOL_SIZE_BORDER, 2);
	    }
	    if (Cell_SOC[i - 1] >= 90) {
			GUI_FillRoundedRect(BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_X + BATTERY_SYMBOL_SIZE_BORDER + BATTERY_SYMBOL_SIZE_BORDER,
								MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - (BATTERY_SYMBOL_SIZE_Y / 5) * 5,
								BATTERY_CENTER_LOCATION_X * Pos_Counter - BATTERY_SYMBOL_SIZE_BORDER - BATTERY_SYMBOL_SIZE_BORDER,
								MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2) - (BATTERY_SYMBOL_SIZE_Y / 5) * 4 - BATTERY_SYMBOL_SIZE_BORDER, 2);
	    }

	    // Print cell diagnostic
	    GUI_SetTextAlign(GUI_TA_HCENTER | GUI_TA_VCENTER);
	    GUI_GotoXY(	BATTERY_CENTER_LOCATION_X * Pos_Counter - (BATTERY_SYMBOL_SIZE_X / 2) , 	BATTERY_TEXT_BOT_LOCATION_1);// - 6
	    //GUI_DispDecSpace(Cell_SOC[i - 1], 3);
	    //GUI_DispString("%");
	    if(cell_diagnotic_state == CELL_DIAG_OPEN_LOOP)
	    	GUI_DispString("OL");
	    else if(cell_diagnotic_state == CELL_DIAG_UNDERVOLTAGE)
	    	GUI_DispString("UV");
	    else if(cell_diagnotic_state == CELL_DIAG_OVERVOLTAGE)
	    	GUI_DispString("OV");
	    else if(cell_diagnotic_state == CELL_DIAG_BAL_UNDERVOLTAGE)
	    	GUI_DispString("BUV");
	    else if(cell_diagnotic_state == CELL_DIAG_BAL_OVERVOLTAGE)
	    	GUI_DispString("BOV");
	    else if(cell_diagnotic_state == CELL_DIAG_IS_BALANCING)
	    	GUI_DispString("BAL");
	    else
	    	GUI_DispString(" ");

	    // Print cell voltage
	    GUI_SetTextAlign(GUI_TA_HCENTER | GUI_TA_VCENTER);
	    GUI_GotoXY(	BATTERY_CENTER_LOCATION_X * Pos_Counter - (BATTERY_SYMBOL_SIZE_X / 2) - 2, 	BATTERY_TEXT_BOT_LOCATION_2);
	    GUI_DispFloatMin(cell_voltage, 2);
	    GUI_DispString("V");
	    Pos_Counter++;
	}
}

/******************************************************************************/
void show_main_page(EPD_E2200CS021_status_t status) {
	static uint8_t executionCounter;

	/* Draw Top - Show Up-Time for Charging/Discharging */
	GUI_SetColor(GUI_BLACK);
	GUI_SetBkColor(GUI_WHITE);

	GUI_SetFont(GUI_FONT_24B_1);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, MAIN_TEXT_TOP_LOCATION_1);

    switch(status.OUTPUT_Status) {
        	case OUTPUT_OFF:
        	   	GUI_DispString("OFF");
        	   	break;
        	case OUTPUT_READY:
            	GUI_DispString("READY");
            	break;
            case OUTPUT_OFF_TILL_RESTART:
            	GUI_DispString("INACTIVE");
            	break;
            case OUTPUT_PRECHARGE:
            	GUI_DispString("PRECHARGE");
            	break;
        	case OUTPUT_ON:
        	    GUI_DispDec(status.Active_Hours, 2);
        	    GUI_DispString(":");
        	    GUI_DispDec(status.Active_Minutes, 2);
        	    GUI_DispString(":");
        	    GUI_DispDec(status.Active_Seconds, 2);
            	break;
    }

	/* Draw First Information Line - Show Battery SOC & SOH */
	GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, 	MAIN_TEXT_BOT_LOCATION_1);
    GUI_DispString("SOC:");

    GUI_GotoXY(	MTB_E2200CS021_DISPLAY_SIZE_X_QUARTER + 10, 	MAIN_TEXT_BOT_LOCATION_1);
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_DispDecSpace(status.soc, 3);

    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispString("%  ");
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispString("SOH:");

	GUI_GotoXY(	MTB_E2200CS021_DISPLAY_SIZE_X_HALF + 33, 	MAIN_TEXT_BOT_LOCATION_1);
	GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    if(status.soh >= 99){
    	GUI_DispString("100");
    }
    else if(status.soh < 50){
    	GUI_DispString("<50");
    }
    else {
    	// Show SoH in steps of 5
		//if(status.soh < 60) GUI_DispString(">55");
		//else if(status.soh < 65) GUI_DispString(">60");
		//else if(status.soh < 70) GUI_DispString(">65");
		//else if(status.soh < 75) GUI_DispString(">70");
		//else if(status.soh < 80) GUI_DispString(">75");
		//else if(status.soh < 85) GUI_DispString(">80");
		//else if(status.soh < 90) GUI_DispString(">85");
		//else if(status.soh < 95) GUI_DispString(">90");
		//else GUI_DispString(">95");

    	// Show SoH in steps of 10
    	if(status.soh < 60) GUI_DispString(">50");
		else if(status.soh < 70) GUI_DispString(">60");
		else if(status.soh < 80) GUI_DispString(">70");
		else if(status.soh < 90) GUI_DispString(">80");
    	else GUI_DispString(">90");
    }


    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispString("%   ");

	/* Draw Second Information Line - Show Battery Voltage*/
	GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, 	MAIN_TEXT_BOT_LOCATION_2);
    GUI_DispString("Voltage:");

    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_GotoXY(	MTB_E2200CS021_DISPLAY_SIZE_X_HALF, 	MAIN_TEXT_BOT_LOCATION_2);
    GUI_DispFloatMin((float) status.voltage_data / 1000, 2);

    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispString(" V");

	/* Draw Third Information Line - Show Battery Current Or Error Code */
	GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, 	MAIN_TEXT_BOT_LOCATION_3);
    if(sh_pub_error_map_cm0 == 0){
		GUI_DispString("Current:");

		GUI_SetFont(GUI_FONT_16B_1);
		GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
		GUI_GotoXY(	MTB_E2200CS021_DISPLAY_SIZE_X_HALF, 	MAIN_TEXT_BOT_LOCATION_3);
		//////GUI_DispDecSpace(current_data, 3);
		GUI_DispFloatMin((float) status.current_data / 1000, 2);
		//GUI_DispString("-");

		GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
		GUI_DispString(" A");
    }
    else{
    	// Display error strings 2 out of 3 the times, the 3rd time show the error code
    	uint8_t numErrors = countSetBits(status.error_map);
    	if(((executionCounter % 3) == 0) && (numErrors > MAX_SHOWN_ERRORS)){
    		GUI_DispString("Error: 0x");
    		GUI_DispHex(status.error_map, 8);
    	}
    	else{
    		GUI_DispString("Error: ");
			print_DispErrors(status.error_map, numErrors, MAX_SHOWN_ERRORS);
    	}
    }

	/* Draw Fourth Information Line - Show Battery Voltage & Current Draw */
	GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, 	MAIN_TEXT_BOT_LOCATION_4);
    GUI_DispString("Slot:");

    switch(status.SLOT_Status) {
        	case SLOT_NONE:
        	   	GUI_DispString(" ---");
        	   	break;
        	case SLOT_IN_SYSTEM_1:
            	GUI_DispString(" System 1");
            	break;
        	case SLOT_IN_SYSTEM_2:
            	GUI_DispString(" System 2");
            	break;
        	case SLOT_IN_CHARGER:
            	GUI_DispString(" Charger");
            	break;
    }

    GUI_FillRect(	MTB_E2200CS021_DISPLAY_SIZE_X - (MAIN_SYMBOL_SIZE_X / 2) - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_HEAD_SIZE,
		   	   	   	MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + TOP_LEGEND_HEIGHT - (MAIN_SYMBOL_SIZE_Y / 2) - MAIN_SYMBOL_SIZE_BORDER - 2,
					MTB_E2200CS021_DISPLAY_SIZE_X - (MAIN_SYMBOL_SIZE_X / 2) - MAIN_SYMBOL_SIZE_BORDER + MAIN_SYMBOL_HEAD_SIZE,
					MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + TOP_LEGEND_HEIGHT - (MAIN_SYMBOL_SIZE_Y / 2) - MAIN_SYMBOL_SIZE_BORDER - 1);

	GUI_DrawRoundedRect(	MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_X - MAIN_SYMBOL_SIZE_BORDER,
							MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + TOP_LEGEND_HEIGHT - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 2),
							MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_BORDER,
							MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + TOP_LEGEND_HEIGHT + (MAIN_SYMBOL_SIZE_Y / 2), 2);

    if (status.soc >= 10) {
    	GUI_FillRoundedRect(MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_X,
    						MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 5),
							MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER,
							MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER, 2);
    }
    if (status.soc >= 30) {
    	GUI_FillRoundedRect(MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_X,
    						MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 5) * 2,
							MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER,
							MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 5), 2);
    }
    if (status.soc >= 50) {
    	GUI_FillRoundedRect(MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_X,
    						MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 5) * 3,
							MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER,
							MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 5) * 2, 2);
    }
    if (status.soc >= 70) {
    	GUI_FillRoundedRect(MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_X,
    						MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 5) * 4,
							MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER,
							MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 5) * 3, 2);
    }
    if (status.soc >= 90) {
    	GUI_FillRoundedRect(MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_X,
    						MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 5) * 5,
							MTB_E2200CS021_DISPLAY_SIZE_X - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER,
							MTB_E2200CS021_DISPLAY_SIZE_Y - MAIN_SYMBOL_SIZE_BORDER - MAIN_SYMBOL_SIZE_BORDER - (MAIN_SYMBOL_SIZE_Y / 5) * 4, 2);
    }

    // Increment execution counter
    executionCounter++;
}

/******************************************************************************/
void show_detail_page(EPD_E2200CS021_status_t status) {
	static uint8_t executionCounter;

	status.error_map = UINT32_MAX;

	/* Draw Top - Show Up-Time for Charging/Discharging */
	GUI_SetColor(GUI_BLACK);
	GUI_SetBkColor(GUI_WHITE);

	GUI_SetFont(GUI_FONT_24B_1);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, MAIN_TEXT_TOP_LOCATION_1);

    switch(status.OUTPUT_Status) {
        	case OUTPUT_OFF:
        	   	GUI_DispString("OFF");
        	   	break;
        	case OUTPUT_READY:
            	GUI_DispString("READY");
            	break;
            case OUTPUT_OFF_TILL_RESTART:
            	GUI_DispString("INACTIVE");
            	break;
            case OUTPUT_PRECHARGE:
            	GUI_DispString("PRECHARGE");
            	break;
        	case OUTPUT_ON:
        	    GUI_DispDec(status.Active_Hours, 2);
        	    GUI_DispString(":");
        	    GUI_DispDec(status.Active_Minutes, 2);
        	    GUI_DispString(":");
        	    GUI_DispDec(status.Active_Seconds, 2);
            	break;
    }

	/* Draw First Information Line - Show Battery SOC & SOH */
	GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, 	MAIN_TEXT_BOT_LOCATION_1);
    GUI_DispString("SOC:");

    GUI_GotoXY(	MTB_E2200CS021_DISPLAY_SIZE_X_QUARTER + 10, 	MAIN_TEXT_BOT_LOCATION_1);
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_DispDecSpace(status.soc, 3);

    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispString("%  ");
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispString("SOH:");

    GUI_GotoXY(	MTB_E2200CS021_DISPLAY_SIZE_X_HALF + 33, 	MAIN_TEXT_BOT_LOCATION_1);
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_DispDecSpace(status.soh, 3);

    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispString("%   ");

	/* Draw Second Information Line - Show Battery Voltage*/
	GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, 	MAIN_TEXT_BOT_LOCATION_2);
    GUI_DispString("Voltage:");

    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_GotoXY(	MTB_E2200CS021_DISPLAY_SIZE_X_HALF, 	MAIN_TEXT_BOT_LOCATION_2);
    GUI_DispFloatMin((float) status.voltage_data / 1000, 2);

    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispString(" V");

	/* Draw Third Information Line - Show Battery Current Or Error Code */
	GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, 	MAIN_TEXT_BOT_LOCATION_3);
    if(sh_pub_error_map_cm0 == 0){
		GUI_DispString("Current:");

		GUI_SetFont(GUI_FONT_16B_1);
		GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
		GUI_GotoXY(	MTB_E2200CS021_DISPLAY_SIZE_X_HALF, 	MAIN_TEXT_BOT_LOCATION_3);
		//////GUI_DispDecSpace(current_data, 3);
		GUI_DispFloatMin((float) status.current_data / 1000, 2);
		//GUI_DispString("-");

		GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
		GUI_DispString(" A");
    }
    else{
    	// Display error code and strings
    	uint8_t numErrors = countSetBits(status.error_map);
		GUI_DispString("Error: 0x");
		GUI_DispHex(status.error_map, 8);
		GUI_GotoXY(	MAIN_SYMBOL_SIZE_BORDER, 	MAIN_TEXT_BOT_LOCATION_4);
		print_DispErrors(status.error_map, numErrors, 6);
    }

    // Increment execution counter
    executionCounter++;
}

/******************************************************************************/
EPD_E2200CS021_pages_t show_page(EPD_E2200CS021_pages_t page) {
	// Clear the current screen buffer
	clear_screen();

	// Update data object for display
	EPD_Status_Obj.OUTPUT_Status = sh_pub_outputState_cm0;
	EPD_Status_Obj.SLOT_Status   = sh_pub_slotState_cm0;
	EPD_Status_Obj.error_map = sh_pub_error_map_cm0;
	EPD_Status_Obj.soc = sh_pub_soc_cm0;
	EPD_Status_Obj.soh = sh_pub_soh_cm0;
	EPD_Status_Obj.voltage_data = sh_pub_volt_cm0;
	EPD_Status_Obj.current_data = sh_pub_current_cm0;

	// Lock public access during refresh
	switch (page) {
		case EPD_MainScreen:
			show_top_legend("BMS Monitoring", (EPD_E2200CS021_pages_t) page);
			show_main_page(EPD_Status_Obj);

			return EPD_MainScreen;
			break;
		case EPD_Battery_Page1:
			show_top_legend("Battery Page 1", (EPD_E2200CS021_pages_t) page);
			show_battery_page(1, sh_pub_cells_cm0);

			return EPD_Battery_Page1;
			break;
		case EPD_Battery_Page2:
			show_top_legend("Battery Page 2", (EPD_E2200CS021_pages_t) page);
			show_battery_page(5, sh_pub_cells_cm0);

			return EPD_Battery_Page2;
			break;
		case EPD_Battery_Page3:
			show_top_legend("Battery Page 3", (EPD_E2200CS021_pages_t) page);
			show_battery_page(9, sh_pub_cells_cm0);

			return EPD_Battery_Page3;
			break;
		case EPD_DetailScreen:
			show_top_legend("Detail Page", (EPD_E2200CS021_pages_t) page);
			EPD_Status_Obj.OUTPUT_Status = sh_pub_outputState_cm0;
			EPD_Status_Obj.SLOT_Status   = sh_pub_slotState_cm0;

			show_detail_page(EPD_Status_Obj);

			return EPD_DetailScreen;
			break;
		default:
			return 0;
			break;
	}

	return 0;
}

/******************************************************************************/
uint8_t print_DispErrors(uint32_t error_map, uint8_t numErrors, uint8_t maxErrorsPerLine){
	uint8_t numShownErrors = 0;
	// Priority errors		----------------------
	if((error_map & ERR_INT_TLE9012_QUEUE_FAILED) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("IQF"); }
		else				   { GUI_DispString(",IQF"); }
		numShownErrors++;
	}
	if((error_map & ERR_INT_TLE9012_COM_ERROR) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("ICE"); }
		else				   { GUI_DispString(",ICE"); }
		numShownErrors++;
	}
	if((error_map & ERR_INT_OVERCURRENT) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("IOC"); }
		else				   { GUI_DispString(",IOC"); }
		numShownErrors++;
	}
	if((error_map & ERR_INT_SWITCH_ON_VETO_RECEIVED) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("ISV"); }
		else				   { GUI_DispString(",ISV"); }
		numShownErrors++;
	}
	if((error_map & ERR_INT_ADC_COM_TIMEOUT) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("IAT"); }
		else				   { GUI_DispString(",IAT"); }
		numShownErrors++;
	}
	if((error_map & ERR_INT_VBAT_OV) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("IOV"); }
		else				   { GUI_DispString(",IOV"); }
		numShownErrors++;
	}
	if((error_map & ERR_INT_VBAT_UV) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("IUV"); }
		else				   { GUI_DispString(",IUV"); }
		numShownErrors++;
	}


	// Cell errors		----------------------
	if((error_map & ERR_CAF_OL_ERR) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("COL"); }
		else				   { GUI_DispString(",COL"); }
		numShownErrors++;
	}
	if((error_map & ERR_CAF_CELL_UV) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("CUV"); }
		else				   { GUI_DispString(",CUV"); }
		numShownErrors++;
	}
	if((error_map & ERR_CAF_CELL_OV) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("COV"); }
		else				   { GUI_DispString(",COV"); }
		numShownErrors++;
	}


	// Medium importance system errors		----------------------
	if((error_map & ERR_CAF_EXT_T_ERR) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("COT"); }
		else				   { GUI_DispString(",COT"); }
		numShownErrors++;
	}
	if((error_map & ERR_CAF_INT_OT) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("FOT"); }
		else				   { GUI_DispString(",FOT"); }
		numShownErrors++;
	}
	if((error_map & ERR_CAF_BAL_UC) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("BUC"); }
		else				   { GUI_DispString(",BUC"); }
		numShownErrors++;
	}
	if((error_map & ERR_CAF_BAL_OC) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("BOC"); }
		else				   { GUI_DispString(",BOC"); }
		numShownErrors++;
	}
	if((error_map & ERR_CAF_PS_SLEEP) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("FPS"); }
		else				   { GUI_DispString(",FPS"); }
		numShownErrors++;
	}
	if((error_map & ERR_CAF_ADC_ERR) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("FAD"); }
		else				   { GUI_DispString(",FAD"); }
		numShownErrors++;
	}

	// Medium importance driver/safety switch errors		----------------------
	if((error_map & ERR_SSW_OVERCURRENT) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SOC"); }
		else				   { GUI_DispString(",SOC"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_SAVESTATE_ENABLED) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SSS"); }
		else				   { GUI_DispString(",SSS"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_VBAT_UNDERVOLTAGE) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SUV"); }
		else				   { GUI_DispString(",SUV"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_VDD_UNDERVOLTAGE) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SVD"); }
		else				   { GUI_DispString(",SVD"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_VBAT_OVERVOLTAGE) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SOV"); }
		else				   { GUI_DispString(",SOV"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_CHIP_OVERTEMPERATURE) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SOT"); }
		else				   { GUI_DispString(",SOT"); }
		numShownErrors++;
	}

	// Remaining low importance errors
	if((error_map & ERR_CAF_INT_IC_ERR) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("FIC"); }
		else				   { GUI_DispString(",FIC"); }
		numShownErrors++;
	}
	if((error_map & ERR_CAF_REG_CRC_ERR) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("FCE"); }
		else				   { GUI_DispString(",FCE"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_VDS_OVERVOLTAGE_A) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SAO"); }
		else				   { GUI_DispString(",SAO"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_VGS_UNDERVOLTAGE_A) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SAU"); }
		else				   { GUI_DispString(",SAU"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_VDS_OVERVOLTAGE_B) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SBO"); }
		else				   { GUI_DispString(",SBO"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_VGS_UNDERVOLTAGE_B) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SBU"); }
		else				   { GUI_DispString(",SBU"); }
		numShownErrors++;
	}
	if((error_map & ERR_SSW_CHARGEPUMP_UNDERVOLTAGE) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SCU"); }
		else				   { GUI_DispString(",SCU"); }
		numShownErrors++;
	}

	// Show interrupt errors
	if((error_map & ERR_SSW_INTERRUPT) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("SI"); }
		else				   { GUI_DispString(",SI"); }
		numShownErrors++;
	}
	if((error_map & ERR_CAF_TLE9012_INTERRUPT) && numShownErrors < maxErrorsPerLine){
		if(numShownErrors == 0){ GUI_DispString("FI"); }
		else				   { GUI_DispString(",FI"); }
		numShownErrors++;
	}

	// If there where more errors than can be shown signalizes it by adding dots
	if(numErrors > maxErrorsPerLine){ GUI_DispString("..."); }

	// Return the number of shown errors
	return numShownErrors;
}

// Function to get number of set bits in binary representation of positive integer n
// https://www.geeksforgeeks.org/count-set-bits-in-an-integer/
uint8_t countSetBits(uint32_t n){
    uint8_t count = 0;
    while (n) {
        count += n & 1;
        n >>= 1;
    }
    return count;
}

//****************************************************************************
// mapRangeToScale - Converts a range of numbers to a percentage scale
// n       number to convert
// lRange  lowest number of the range
// hRange  highest number in the range
// scale   percentage scale
//****************************************************************************
float mapRangeToScale(float n, float lRange, float hRange, float scale){
	// Input validation
	//if (n < lRange || n > hRange) { return -1.0; }

	// Map to scale
	float result = (n - lRange)/(hRange - lRange) * scale;

	// Return
	if(result < 0)
		return 0;
	else if(result > 100)
		return 100;
	else
		return result;
}

/******************************************************************************/
/* [] END OF FILE */

