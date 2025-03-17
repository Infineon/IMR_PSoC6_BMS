/*********************************************************************
*                SEGGER Microcontroller GmbH                         *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2018  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.48 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software  has been licensed to  Cypress Semiconductor Corporation,
whose registered  office is situated  at 198 Champion Ct. San Jose, CA
95134 USA  solely for the  purposes of creating  libraries for Infineon
PSoC3 and  PSoC5 processor-based devices,  sublicensed and distributed
under  the  terms  and  conditions  of  the  Infineon  End User License
Agreement.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Microcontroller Systems LLC
Licensed to:              Cypress Semiconductor Corp, 198 Champion Ct., San Jose, CA 95134, USA
Licensed SEGGER software: emWin
License number:           GUI-00319
License model:            Services and License Agreement, signed June 10th, 2009
Licensed platform:        Any Cypress platform (Initial targets are: PSoC3, PSoC5)
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2009-06-12 - 2022-07-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display configuration file for GUIDRV_Bitplains to be
              used without any display controller. This implementation
              has been written and tested with the RSKR32C111 with
              Visualization Add On.
---------------------------END-OF-HEADER------------------------------
*/

#include <display-eink-e2200cs021/release-v1.1.0/configs/emwin/emwin.h>

#if defined(EMWIN_ENABLED)

#include <stddef.h>
#include "GUI.h"
#include "GUIDRV_BitPlains.h"
#include "string.h"
#include <display-eink-e2200cs021/release-v1.1.0/configs/emwin/LCDConf.h>
#include <mtb_e2200cs021_hw_interface.h>

/*********************************************************************
*
*       Layer configuration
*
**********************************************************************
*/
//
// Physical display size
//
#define XSIZE_PHYS MTB_DISPLAY_SIZE_X
#define YSIZE_PHYS MTB_DISPLAY_SIZE_Y

//
// Color conversion
//   The color conversion functions should be selected according to
//   the color mode of the target display. Details can be found in
//   the chapter "Colors" in the emWin user manual.
//
#define COLOR_CONVERSION GUICC_1

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
//
// Checking display driver configuration
//
#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
  #error Color conversion not defined!
#endif

//
// Number of bytes per line
//
#define BYTES_PER_LINE ((XSIZE_PHYS + 7) / 8)

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
//
// Data arrays to be used by the display driver
//
static U8 _aPlain_0[BYTES_PER_LINE * YSIZE_PHYS];

//
// Structure to be passed to the driver
//
static struct {
  U8 * apVRAM[8];
} _VRAM_Desc = {{
  _aPlain_0,
}};

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/
void LCD_X_Config(void)
{
  GUI_DEVICE * pDevice;
  CONFIG_BITPLAINS Config = {0};
  //
  // Set display driver and color conversion for 1st layer
  //
  pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_BITPLAINS, COLOR_CONVERSION, 0, 0);
  /* Configure Bitplains library */
  Config.Mirror = 1;
  GUIDRV_BitPlains_Config(pDevice, &Config);
  //
  // Display driver configuration
  //
  if (LCD_GetSwapXY())
  {
    LCD_SetSizeEx (0, YSIZE_PHYS, XSIZE_PHYS);
    LCD_SetVSizeEx(0, YSIZE_PHYS, XSIZE_PHYS);
  }
  else
  {
    LCD_SetSizeEx (0, XSIZE_PHYS, YSIZE_PHYS);
    LCD_SetVSizeEx(0, XSIZE_PHYS, YSIZE_PHYS);
  }
  //
  // Initialize VRAM access off driver
  //
  LCD_SetVRAMAddrEx(0, (void *)&_VRAM_Desc);
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task, the routine needs to be adapted to
*   the display controller. Note that the commands marked
*   "optional" are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Zero based layer index
*   Cmd        - Command to be executed
*   pData      - Pointer to a data structure.
*
* Return Value:
*   < -1 - Error
*     -1 - The command is not handled.
*      0 - OK.
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData)
{
  GUI_USE_PARA(LayerIndex);
  GUI_USE_PARA(pData);

  return -1;
}

/*********************************************************************
*
*       LCD_GetDisplayBuffer
*
* Purpose:
*   This function returns the pointer to EmWin's display buffer

* Parameter:
*   None
*
* Return Value:
*   U8* - Pointer to the EmWin's display buffer
*/
U8* LCD_GetDisplayBuffer(void)
{
  /* Return pointer to the display buffer */
  return _aPlain_0;
}

#endif /* defined(EMWIN_ENABLED) */

/*************************** End of file ****************************/
