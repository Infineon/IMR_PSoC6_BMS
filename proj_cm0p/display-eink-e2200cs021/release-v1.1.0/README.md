# Display 2.7 Inch EPD (E2271CS021)

### Overview

This library provides functions for supporting the 2.7" EPD Display (E2271CS021). This is the same display as used on the CY8CKIT-028-EPD shield.

https://www.electronicsdatasheets.com/download/59018c25e34e2457312cc1ab.pdf?format=pdf


# Quick Start

* [EINK emWin application](#eink-emwin-application)
* [EINK driver usage without emWin](#eink-driver-usage-without-emwin)
* [EINK emWin FreeRTOS application](#eink-emwin-freertos-application)
* [EINK emWin mbed application](#eink-emwin-mbed-application)


## EINK emWin application
Follow the steps below in order to create a simple emWin application and display some text on it.
1. Create an empty application
2. Add this library to the application
3. Add emWin library to the application
4. Enable EMWIN_NOSNTS emWin library option by adding it to the Makefile COMPONENTS list:
```
COMPONENTS=EMWIN_NOSNTS
```
5. place the following code in the main.c file:
```cpp
#include "cyhal.h"
#include "cybsp.h"
#include "GUI.h"
#include "mtb_e2271cs021.h"
#include "LCDConf.h"

cyhal_spi_t spi;

// The pins used here work with the CY8CKIT-028-EPD shield and the CY8CKIT-062-BLE
// or CY8CKIT-062-WIFI-BT kit.
// If the display is being used on different hardware the mappings will be different.
const mtb_e2271cs021_pins_t pins =
{
    .spi_mosi  = CYBSP_D11,
    .spi_miso  = CYBSP_D12,
    .spi_sclk  = CYBSP_D13,
    .spi_cs    = CYBSP_D10,
    .reset     = CYBSP_D2,
    .busy      = CYBSP_D3,
    .discharge = CYBSP_D5,
    .enable    = CYBSP_D4,
    .border    = CYBSP_D6,
    .io_enable = CYBSP_D7
};

uint8_t previous_frame[PV_EINK_IMAGE_SIZE] = {0};
uint8_t *current_frame;

int main(void)
{
    cy_rslt_t result;

    // Initialize the device and board peripherals
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    // Initialize SPI and EINK display
    result = cyhal_spi_init(&spi, pins.spi_mosi, pins.spi_miso, pins.spi_sclk, NC, NULL, 8,
            CYHAL_SPI_MODE_00_MSB, false);
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_spi_set_frequency(&spi, 20000000);
    }

    result = mtb_e2271cs021_init(&pins, &spi);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    current_frame = (uint8_t*)LCD_GetDisplayBuffer();

    GUI_Init();
    GUI_SetTextMode(GUI_TM_NORMAL);
    GUI_SetFont(GUI_FONT_32B_1);
    GUI_SetBkColor(GUI_WHITE);
    GUI_SetColor(GUI_BLACK);
    GUI_SetTextAlign(GUI_ALIGN_HCENTER | GUI_ALIGN_HCENTER);
    GUI_Clear();

    GUI_DispStringAt("Hello World", LCD_GetXSize() / 2, (LCD_GetYSize() / 2) - (GUI_GetFontSizeY() / 2));

    // update the display
    mtb_e2271cs021_show_frame(previous_frame, current_frame, MTB_E2271CS021_FULL_4STAGE, true);

    while (1);
}
```
6. Build the application and program the kit.


## EINK driver usage without emWin
The EINK library can be also used standalone.
Follow the steps below to create a simple application which shows an interesting pattern on the display.
1. Create an empty application
2. Add this library to the application
3. Place the following code in the main.c file:
```cpp
#include "cyhal.h"
#include "cybsp.h"
#include "mtb_e2271cs021.h"

cyhal_spi_t spi;

// The pins used here work with the CY8CKIT-028-EPD shield and the CY8CKIT-062-BLE
// or CY8CKIT-062-WIFI-BT kit.
// If the display is being used on different hardware the mappings will be different.
const mtb_e2271cs021_pins_t pins =
{
    .spi_mosi  = CYBSP_D11,
    .spi_miso  = CYBSP_D12,
    .spi_sclk  = CYBSP_D13,
    .spi_cs    = CYBSP_D10,
    .reset     = CYBSP_D2,
    .busy      = CYBSP_D3,
    .discharge = CYBSP_D5,
    .enable    = CYBSP_D4,
    .border    = CYBSP_D6,
    .io_enable = CYBSP_D7
};

uint8_t previous_frame[PV_EINK_IMAGE_SIZE] = {0};
uint8_t current_frame[PV_EINK_IMAGE_SIZE] = {0};

int main(void)
{
    cy_rslt_t result;
    uint32_t i;

    // Initialize the device and board peripherals
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    // Initialize SPI and EINK display
    result = cyhal_spi_init(&spi, pins.spi_mosi, pins.spi_miso, pins.spi_sclk, NC, NULL, 8,
            CYHAL_SPI_MODE_00_MSB, false);
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_spi_set_frequency(&spi, 20000000);
    }

    result = mtb_e2271cs021_init(&pins, &spi);

    // Fill the EINK buffer with an interesting pattern
    for (i = 0; i < PV_EINK_IMAGE_SIZE; i++)
    {
        current_frame[i] = i % 0xFF;
    }

    // Update the display
    mtb_e2271cs021_show_frame(previous_frame, current_frame, MTB_E2271CS021_FULL_4STAGE, true);

    while (1);
}
```
4. Build the application and program the kit.


## EINK emWin FreeRTOS application
Follow the steps bellow in order to create a simple emWin application and display some text.
1. Create an empty application
2. Add this library to the application
3. Add the emwin, abstraction-rtos, and freertos libraries to the application
4. Enable EMWIN_OSNTS emWin library option by adding it to the Makefile COMPONENTS list:
```
COMPONENTS=EMWIN_OSNTS FREERTOS
```
5. Edit the FreeRTOSConfig.h file in the freertos library.  Or make a copy of it in the top level
   of your project directory.  Comment out the \#warning statement and update these definitions.
```
#define configUSE_MUTEXES                       (1u)
#define configUSE_RECURSIVE_MUTEXES             (1u)
#define configUSE_COUNTING_SEMAPHORES           (1u)
#define configSUPPORT_STATIC_ALLOCATION         (1u)
#define configSUPPORT_DYNAMIC_ALLOCATION        (1u)
#define configTOTAL_HEAP_SIZE                   (1024*48u)
#define configUSE_TRACE_FACILITY                (1u)
```
6. Place the following code in the main.c file:
```cpp
#include "FreeRTOS.h"
#include "GUI.h"
#include "LCDConf.h"
#include "cybsp.h"
#include "cyhal.h"
#include "mtb_e2271cs021.h"
#include "task.h"

cyhal_spi_t spi;

// The pins used here work with the CY8CKIT-028-EPD shield and the CY8CKIT-062-BLE
// or CY8CKIT-062-WIFI-BT kit.
// If the display is being used on different hardware the mappings will be different.
const mtb_e2271cs021_pins_t pins =
{
    .spi_mosi  = CYBSP_D11,
    .spi_miso  = CYBSP_D12,
    .spi_sclk  = CYBSP_D13,
    .spi_cs    = CYBSP_D10,
    .reset     = CYBSP_D2,
    .busy      = CYBSP_D3,
    .discharge = CYBSP_D5,
    .enable    = CYBSP_D4,
    .border    = CYBSP_D6,
    .io_enable = CYBSP_D7
};

uint8_t previous_frame[PV_EINK_IMAGE_SIZE] = {0};
uint8_t *current_frame;

void guiTask(void *arg)
{
    current_frame = (uint8_t*)LCD_GetDisplayBuffer();

    GUI_Init();
    GUI_SetTextMode(GUI_TM_NORMAL);
    GUI_SetFont(GUI_FONT_32B_1);
    GUI_SetBkColor(GUI_WHITE);
    GUI_SetColor(GUI_BLACK);
    GUI_SetTextAlign(GUI_ALIGN_HCENTER | GUI_ALIGN_HCENTER);
    GUI_Clear();

    GUI_DispStringAt("Hello World!", LCD_GetXSize() / 2, (LCD_GetYSize() / 2) - (GUI_GetFontSizeY() / 2));

    // update the display
    mtb_e2271cs021_show_frame(previous_frame, current_frame, MTB_E2271CS021_FULL_4STAGE, true);
}

int main(void)
{
    cy_rslt_t result;

    // Initialize the device and board peripherals
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    // Initialize SPI and EINK display
    result = cyhal_spi_init(&spi, pins.spi_mosi, pins.spi_miso, pins.spi_sclk, NC, NULL, 8,
            CYHAL_SPI_MODE_00_MSB, false);
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_spi_set_frequency(&spi, 20000000);
    }

    result = mtb_e2271cs021_init(&pins, &spi);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    xTaskCreate(guiTask, "GUI Task", 1024*10,  0,  1,  0);
    vTaskStartScheduler();

    while (1); // Will never get here
}
```
7. Build the application and program the kit.


## EINK emWin mbed application
Follow the steps bellow in order to create a simple emWin application and display some text.
1. Create a new application
```
mbed new tft_app
cd tft_app
```
2. Add this library to the application
```
mbed add http://devops-git.aus.cypress.com/repo-staging/display-eink-e2271cs021.git
```
3. Add the emwin library to the application
```
mbed add http://devops-git.aus.cypress.com/repo-staging/emwin.git
```
4. Enable EMWIN_OSNTS emWin library option by adding it to the mbed_app.json file
```
"target_overrides": {
        "*":{
        "target.components_add": ["EMWIN_OSNTS"],
```
5. Place the following code in the main.cpp file:
```cpp
#include "GUI.h"
#include "LCDConf.h"
#include "cy_pdl.h"
#include "cy_utils.h"
#include "cybsp.h"
#include "cycfg.h"
#include "mbed.h"
#include "mtb_e2271cs021.h"

cyhal_spi_t spi;

// The pins used here work with the CY8CKIT-028-EPD shield and the CY8CKIT-062-BLE
// or CY8CKIT-062-WIFI-BT kit.
// If the display is being used on different hardware the mappings will be different.
const mtb_e2271cs021_pins_t pins =
{
    .spi_mosi  = CYBSP_D11,
    .spi_miso  = CYBSP_D12,
    .spi_sclk  = CYBSP_D13,
    .spi_cs    = CYBSP_D10,
    .reset     = CYBSP_D2,
    .busy      = CYBSP_D3,
    .discharge = CYBSP_D5,
    .enable    = CYBSP_D4,
    .border    = CYBSP_D6,
    .io_enable = CYBSP_D7
};

uint8_t previous_frame[PV_EINK_IMAGE_SIZE] = {0};
uint8_t *current_frame;

int main(void)
{
    cy_rslt_t result;

    // Initialize the device and board peripherals
    cybsp_init();

    __enable_irq();

    // Initialize SPI and EINK display
    result = cyhal_spi_init(&spi, pins.spi_mosi, pins.spi_miso, pins.spi_sclk, NC, NULL, 8,
            CYHAL_SPI_MODE_00_MSB, false);
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_spi_set_frequency(&spi, 20000000);
    }

    result = mtb_e2271cs021_init(&pins, &spi);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    current_frame = (uint8_t*)LCD_GetDisplayBuffer();

    GUI_Init();
    GUI_SetTextMode(GUI_TM_NORMAL);
    GUI_SetFont(GUI_FONT_32B_1);
    GUI_SetBkColor(GUI_WHITE);
    GUI_SetColor(GUI_BLACK);
    GUI_SetTextAlign(GUI_ALIGN_HCENTER | GUI_ALIGN_HCENTER);
    GUI_Clear();

    GUI_DispStringAt("Hello World!", LCD_GetXSize() / 2, (LCD_GetYSize() / 2) - (GUI_GetFontSizeY() / 2));

    // update the display
    mtb_e2271cs021_show_frame(previous_frame, current_frame, MTB_E2271CS021_FULL_4STAGE, true);

    ThisThread::sleep_for(osWaitForever);
}
```
7. Build the application and program the kit.
```
mbed compile --target CY8CKIT_062_BLE --toolchain GCC_ARM --flash
or
mbed compile --target CY8CKIT_062_WIFI_BT --toolchain GCC_ARM --flash
```


### More information

* [API Reference Guide](https://cypresssemiconductorco.github.io/display-eink-e2271cs021/html/index.html)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Cypress Semiconductor GitHub](https://github.com/cypresssemiconductorco)
* [ModusToolbox](https://www.cypress.com/products/modustoolbox-software-environment)
* [PSoC 6 Code Examples using ModusToolbox IDE](https://github.com/cypresssemiconductorco/Code-Examples-for-ModusToolbox-Software)
* [PSoC 6 Middleware](https://github.com/cypresssemiconductorco/psoc6-middleware)
* [PSoC 6 Resources - KBA223067](https://community.cypress.com/docs/DOC-14644)

---
© Cypress Semiconductor Corporation, 2019-2021.
