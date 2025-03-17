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

#include "cy_pdl.h"
#include "cycfg.h"
#include "cybsp.h"
#include "display-eink-e2200cs021/release-v1.1.0/mtb_e2200cs021.h"

#include "GUI.h"
#include "LCDConf.h"
#include "ram_public.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/** Pin for the E-Ink SPI MOSI signal */
#define EPD_PIN_DISPLAY_SPI_MOSI    SPI_EPD_MOSI 	// (CYBSP_D11)
/** Pin for the E-Ink SPI MISO signal */
#define EPD_PIN_DISPLAY_SPI_MISO    SPI_EPD_MISO 	// (CYBSP_D12)
/** Pin for the E-Ink SPI SCLK signal */
#define EPD_PIN_DISPLAY_SPI_SCLK    SPI_EPD_SCLK 	// (CYBSP_D13)
/** Pin for the E-Ink SPI Cable Select signal */
#define EPD_PIN_DISPLAY_CS          SPI_EPD_CS  	// (CYBSP_D10)

/** Pin for the E-Ink Display Reset signal */
#define EPD_PIN_DISPLAY_RST         EPD_RST_L		// (CYBSP_D2)
/** Pin for the E-Ink Display Busy signal */
#define EPD_PIN_DISPLAY_BUSY        EPD_BUSY		// (CYBSP_D3)
/** Pin for the E-Ink Display Discharge signal */
#define EPD_PIN_DISPLAY_DISCHARGE   EPD_DISCHARGE	// (CYBSP_D5)
/** Pin for the E-Ink Display Enable signal */
#define EPD_PIN_DISPLAY_EN          EPD_VCC_EN		// (CYBSP_D4)

typedef struct {
	uint8_t	Active_Hours;
	uint8_t Active_Minutes;
	uint8_t Active_Seconds;
	outputStates OUTPUT_Status;
	slotStates	 SLOT_Status;
	uint32_t error_map;
	uint8_t soc;
	uint8_t soh;
	uint16_t voltage_data;
	int16_t current_data;

} EPD_E2200CS021_status_t;


typedef enum {		/* Different screens the E-INK display can show */
	EPD_MainScreen,
	EPD_Battery_Page1,
	EPD_Battery_Page2,
	EPD_Battery_Page3,
	EPD_DetailScreen
} EPD_E2200CS021_pages_t;

#define TOTAL_PAGE_COUNT					4			// Adjust to Number of Elements in EPD_E2200CS021_pages_t
#define AMBIENT_TEMPERATURE_DEFAULT_C       10			// Ambient temperature for adjustment of communication speed
#define EPD_SPI_BAUDRATE_HZ                	20000000	// Baudrate setting for SPI communication

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void Initialize_EPD (int8_t displayTempFactor);
void first_screen_update(void);
void clear_screen (void);
void show_top_legend (char legend_name[], EPD_E2200CS021_pages_t page);
EPD_E2200CS021_pages_t show_page (EPD_E2200CS021_pages_t page);

void show_main_page(EPD_E2200CS021_status_t status);
void show_battery_page (uint16_t start_battery, uint16_t battery_voltage_data[]);
void show_detail_page(EPD_E2200CS021_status_t status);


/*******************************************************************************
* Global Variables
*******************************************************************************/
#define TOP_LEGEND_HEIGHT							10U		// Height of top legend

#define MTB_E2200CS021_DISPLAY_SIZE_X_HALF			100U
#define MTB_E2200CS021_DISPLAY_SIZE_Y_HALF			48U
#define MTB_E2200CS021_DISPLAY_SIZE_X_QUARTER		50U
#define MTB_E2200CS021_DISPLAY_SIZE_Y_QUARTER		24U

/*******************************************************************************
* Main Page Screen Variables & Sizes
*******************************************************************************/
#define MAIN_SYMBOL_SIZE_BORDER				3U
#define MAIN_SYMBOL_HEAD_SIZE				6U

#define MAIN_SYMBOL_SIZE_X					MTB_E2200CS021_DISPLAY_SIZE_X_QUARTER
#define MAIN_SYMBOL_SIZE_Y					(MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + MTB_E2200CS021_DISPLAY_SIZE_Y_QUARTER)

#define MAIN_TEXT_TOP_LOCATION_1			MTB_E2200CS021_DISPLAY_SIZE_Y_QUARTER

#define MAIN_TEXT_BOT_LOCATION_1			MTB_E2200CS021_DISPLAY_SIZE_Y_HALF - 5
#define MAIN_TEXT_BOT_LOCATION_2			MAIN_TEXT_BOT_LOCATION_1 + 15
#define MAIN_TEXT_BOT_LOCATION_3			MAIN_TEXT_BOT_LOCATION_2 + 15
#define MAIN_TEXT_BOT_LOCATION_4			MAIN_TEXT_BOT_LOCATION_3 + 15

/*******************************************************************************
* Battery Page Screen Variables & Sizes
*******************************************************************************/
#define BATTERY_PER_PAGE					4U		// Battery cells shown per battery page

#define BATTERY_SYMBOL_SIZE_BORDER			3U
#define BATTERY_SYMBOL_HEAD_SIZE			6U

#define BATTERY_SYMBOL_SIZE_X				(MTB_E2200CS021_DISPLAY_SIZE_X / (BATTERY_PER_PAGE))
#define BATTERY_SYMBOL_SIZE_Y				MTB_E2200CS021_DISPLAY_SIZE_Y_HALF

#define BATTERY_CENTER_LOCATION_X			(MTB_E2200CS021_DISPLAY_SIZE_X / (BATTERY_PER_PAGE))
#define BATTERY_TEXT_TOP_LOCATION			(MTB_E2200CS021_DISPLAY_SIZE_Y_HALF - (BATTERY_SYMBOL_SIZE_Y / 2)) - 7U
#define BATTERY_TEXT_BOT_LOCATION_1			(MTB_E2200CS021_DISPLAY_SIZE_Y_HALF + (BATTERY_SYMBOL_SIZE_Y / 2)) + 7U
#define BATTERY_TEXT_BOT_LOCATION_2			BATTERY_TEXT_BOT_LOCATION_1 + 11U
