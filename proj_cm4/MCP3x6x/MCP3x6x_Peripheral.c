/********************************************************************************
* Company:
*   Microchip Technology Inc.
*
* File Name:
*   MCP3x6x_Peripheral.cpp
*
* Summary:
*   MCP3x6x(R) Delta-Sigma ADC Temperature Sensor peripheral function.
*
* Description:
*   This file contains the source code for the MCP3x6x(R) Delta-Sigma ADC
*   Temperature Sensor peripheral function for use with the Arduino Zero Board 
*   and the ATSAMD21G18 MCU.   
********************************************************************************/

/********************************************************************************
* Copyright (C) 2023 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
************************************************************************************************************************************************************/

/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/
/*                                                              Section: Included Files                                                                    */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

#include "MCP3x6x_Peripheral.h"                             // Peripheral definitions header-file.

#include "stdio.h"
#include "stdint.h"

#include "MCP3x6x_ADC.h"                                    // ADC definitions header-file.
#include "MCP3x6x_Type.h"                                   // Variable definitions header-file.

/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/
/*                                                    Section: MCP3x6x(R) Peripheral Functions                                                             */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

/***********************************************************************************************************************************************************
  Function:
    float TEMP_SNSR(uint32_t)

  Remarks:
    - ADC Temperature Sensor Function.
    - See prototype in MCP3x6x_Peripheral_Definitions.h.
 */

float TEMP_SNSR(uint32_t TEMP_SNSR_OUTPUT)
{
    float TEMP_DEG_C;                                                                             // Temperature Sensor calculated floating-point value in °C.

    TEMP_DEG_C = (((float)(0x00FFFFFF & TEMP_SNSR_OUTPUT)*((float)0.00040096)*2) - 269.13);       // See MCP3564(R) datasheet for Temp Sensor Eqn.
                                                                                                  // Temp(C) = (.00040096 x ADCDATA x VREF) - 269.13. 
    return TEMP_DEG_C;                                                                            // Return calcualted temperature in °C.                                                                                                                   
}

/***********************************************************************************************************************************************************
 End of File
 */
