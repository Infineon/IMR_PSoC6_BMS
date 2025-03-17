/*******************************************************************************
* Company:
*   Microchip Technology Inc.
*
* File Name:
*   MCP3x6x_Type_Definitions.h
*
* Summary:
*   MCP3x6x(R) Delta-Sigma ADC SPI Read-CMD, Write-CMD and CRC Calculation function 
*   variable declarations. 
*
* Description:
*   This file contains the variable declarations for the MCP3x6x(R) Delta-Sigma ADC 
*   SPI Read-CMD, Write-CMD and CRC functions for use with the Arduino Zero Board 
*   and the ATSAMD21G18 MCU.  
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
/*                                                 Section: MCP3x6x(R) ADC Variable Type-Definitions                                                       */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

#ifndef MCP3x6x_TYPE_H_
#define MCP3x6x_TYPE_H_


typedef unsigned long long int _uint64_t;         // 64-bit Unsigned Long Long Integer Type-Def.  

typedef union                                   // 16-bit Union-Structure for Communication-CRC.
{
    struct
    {
        uint8_t LOW;                              // Low-Byte of 8-, 16- and 24-bit MCP3564 Registers.
        uint8_t HIGH;                             // High-Byte of 16- and 24-bit MCP3564 Registers.
    }BYTE;
    uint16_t WORD;                                // Communication-CRC 16-bit unsigned integer data value. 
} _u16data_t;

typedef union                                   // 32-bit Union of Structures for ADC Conversion Data.
{
    struct
    {
        uint8_t LOW;                              // Low-Byte of 8-, 16- and 24-bit MCP3564 Registers.
        uint8_t HIGH;                             // High-Byte of 16- and 24-bit MCP3564 Registers.
        uint8_t UPPER;                            // Upper-Byte of 24-bit MCP3564 Registers.
        uint8_t STATUS;                           // STATUS Byte of MCP3564.
    }BYTE;                                        // ADC Conversion Data Byte Structure.
    struct
    {
        uint16_t LOW;                             // Low-Word of 16- and 24-bit MCP3564 Registers.
        uint16_t HIGH;                            // High-Word (high-byte) of 24-bit MCP3564 Registers.
    }WORD;                                        // ADC Conversion Data Word Structure. 
    uint32_t DWORD;                               // ADC Signal Convserion 32-bit unsigned integer data value. 
    float FLTWORD;                                // Temperature Sensor 32-bit floating point data value. 
} _u32data_t;

typedef union                                   // 64-bit Union of Structures for CRC Calculation.
{
    struct
    {
        uint8_t BYTE0;                            // Byte0 of 64-bit CRC Dividend.                    
        uint8_t BYTE1;                            // Byte1 of 64-bit CRC Dividend.
        uint8_t BYTE2;                            // Byte2 of 64-bit CRC Dividend.
        uint8_t BYTE3;                            // Byte3 of 64-bit CRC Dividend.
        uint8_t BYTE4;                            // Byte4 of 64-bit CRC Dividend.
        uint8_t BYTE5;                            // Byte5 of 64-bit CRC Dividend.
        uint8_t BYTE6;                            // Byte6 of 64-bit CRC Dividend.
        uint8_t BYTE7;                            // Byte7 of 64-bit CRC Dividend.            
    }BYTE;                                        // CRC Dividend Byte Structure. 
    struct
    {
        uint16_t LOW;                             // Low-Word of 64-bit CRC Dividend.
        uint16_t HIGHL;                           // Low-Word of High-Double-Word of 64-bit CRC Dividend. 
        uint16_t HIGHH;                           // High-Word of High-Double-Word of 64-bit CRC Dividend.
        uint16_t UPPER;                           // Upper-Word of 64-bit CRC Dividend.
    }WORD;                                        // CRC Dividend Word Structure. 
    struct
    {
        uint16_t LOW;                             // Low-Word of 64-bit CRC Dividend.
        uint32_t HIGH;                            // High-Double-Word of 64-bit CRC Dividend.
        uint16_t UPPER;                           // Upper-Word of 64-bit CRC Dividend.
    }DWORD;                                       // CRC Dividend Double-Word Structure. 
    uint64_t QWORD;                               // CRC Dividend 64-bit unsigned long long integer data value.                                                                 
} _u64data_t;

#endif /* MCP3x6x_TYPE_H_ */
/************************************************************************************************************************************************************
 End of File
 */
