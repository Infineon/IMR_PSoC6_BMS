/*******************************************************************************
* Company:
*   Microchip Technology Inc.
*
* File Name:
*   MCP3x6x_SPI_Definitions.h
*
* Summary:
*   MCP3x6x(R) Delta-Sigma ADC SPI Read-CMD, Write-CMD, Fast-CMD, STATUS Byte 
*   Retrieval and CRC Calculation function prototypes. 
*
* Description:
*   This file contains the function prototypes for the MCP3x6x(R) Delta-Sigma ADC 
*   SPI Read-CMD, Write-CMD, Fast-CMD, STATUS Byte Retrieval and CRC Calculation 
*   functions for use with the Arduino Zero Board and the ATSAMD21G18 MCU.  
********************************************************************************/

/*******************************************************************************
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
/*                                                          Section: Arduino Included Files                                                                */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

#ifndef MCP3x6x_SPI_H_
#define MCP3x6x_SPI_H_


#include "stdint.h"
#include "MCP3x6x_Type.h"
#include "MCP3x6x_Target.h"

/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/
/*                                                      Section: MCP3x6x(R) SPI Definitions                                                                */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

// #define ENABLE_SPI_CRC                                  // Enable Communication-CRC.
#define DISABLE_SPI_CRC                                 // Disable Communication-CRC.

//#define ENABLE_ADCCONV_FASTCMD                          // Enable ADC Conversion Fast-CMD.
#define DISABLE_ADCCONV_FASTCMD                         // Disable ADC Conversion Fast-CMD.

/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/
/*                                                  Section: Arduino Zero SPI Pin Definitions (ATSAMD21G18 SPI Pins)                                       */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

#define PIN_SPI_nCS  10                        // Arduino Zero Pin D10 is MCP3564 nCS.
 
 /* NOTE:                                     		
 1.)ATSAMD21G18 Pin 24 is MCP3564 SCK (available on Arduino Zero SPI pin header).
 2.)ATSAMD21G18 Pin 23 is MCP3564 SDI (available on Arduino Zero SPI pin header).
 3.)ATSAMD21G18 Pin 22 is MCP3564 SDO (available on Arduino Zero SPI pin header).	
  */ 
  
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/
/*                                                Section: MCP3x6x(R) SPI Write/Read Command Definitions                                                   */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

#define _WRT_CTRL_  0b01000010                // MCP3564(R) Write-CMD Command-Byte.
#define _RD_CTRL_   0b01000001                 // MCP3564(R) Read-CMD Command-Byte.

/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/
/*                                                    Section: MCP3x6x(R) Fast-CMD Definitions                                                             */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

#define _DONT_CARE0_  0b01000000              // Don't-Care Fast-CMD.
#define _DONT_CARE1_  0b01000100              // Don't-Care Fast-CMD.
#define _DONT_CARE2_  0b01001000              // Don't-Care Fast-CMD.
#define _DONT_CARE3_  0b01001100              // Don't-Care Fast-CMD.
#define _DONT_CARE4_  0b01010000              // Don't-Care Fast-CMD.
#define _DONT_CARE5_  0b01010100              // Don't-Care Fast-CMD.
#define _DONT_CARE6_  0b01011000              // Don't-Care Fast-CMD.
#define _DONT_CARE7_  0b01011100              // Don't-Care Fast-CMD.
#define _DONT_CARE8_  0b01100000              // Don't-Care Fast-CMD.
#define _DONT_CARE9_  0b01100100              // Don't-Care Fast-CMD.
#define _ADC_CONV_STRST_   0b01101000          // ADC Conversion-Start/Restart Fast-CMD (Overwrites ADC_MODE[1:0] = 0b11).
#define _ADC_STBY_MODE_    0b01101100           // ADC Standby Mode Fast-CMD (Overwrites ADC_MODE[1:0] = 0b10).
#define _ADC_SHTDN_MODE_   0b01110000          // ADC Shutdown Mode Fast-CMD (Overwrites ADC_MODE[1:0] = 0b00).
#define _DEV_FSHTDN_MODE_  0b01110100         // Device Full-Shutdown Mode Fast-CMD (Overwrites CONFIG0[7:0] = 0x00).
#define _DEV_FRESET_    0b01111000              // Device Full-Reset Fast-CMD (Overwrites entire register map to default settings).
#define _RTRV_STATUS_   0b01111100             // Don't-Care Fast-CMD to retrieve STATUS Byte.

/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/
/*                    Section: MCP3x6x(R) SPI Bus Write/Read, Conversion-Start, Retieve Status, Fast-CMD and CRC Function Prototypes                       */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

void SPI_WRT(uint8_t, uint32_t);                      // SPI Write-CMD Function Prototype.

void SPI_FASTCMD(uint8_t);                            // SPI Fast-CMD Function Prototype.

uint32_t SPI_RD(uint8_t, _u16data_t*, uint64_t*);     // SPI Read-CMD Function Prototype.

uint64_t SPI_COMMCRC(_u64data_t);                     // SPI Communication-CRC Function Prototype.

uint8_t CONV_START(uint8_t);                          // ADC Conversion-Start Prototype.

uint8_t RETRIEVE_STATUS(void);                        // ADC STATUS Byte Retrieval Function Prototype.


#endif /* MCP3x6x_SPI_H_ */
/************************************************************************************************************************************************************
 End of File
 */
