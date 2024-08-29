/********************************************************************************
* Company:
*   Microchip Technology Inc.
*
* File Name:
*   MCP3x6x_ADC.cpp
*
* Summary:
*   MCP3x6x(R) Delta-Sigma ADC Configuration Bit setting definitions and Configuration 
*   Register initialization and Conversion-Start functions.
*
* Description:
*   This file contains the source code for the MCP3x6x(R) Delta-Sigma ADC 
*   Configuration Register initialization and Convsersion-Start functions 
*   for use with the Arduino Zero Board and the ATSAMD21G18 MCU.  
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
/*                                                             Section: Included Files                                                                     */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/


#include "stdio.h"
#include "stdint.h"

#include "MCP3x6x_SPI.h"                                  // SPI definitions header-file. 
#include "MCP3x6x_Type.h"                                 // Variable definitions header-file.
#include "MCP3x6x_ADC.h"                                  // ADC definitions header-file.

/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/
/*                                                    Section: MCP3x6x(R) ADC Configuration Function                                                       */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

/***********************************************************************************************************************************************************
  Function:
    void MCP3x6x_CONFIG(void)

  Remarks:
    - ADC Configuration Function. 
    - See prototype in MCP3x6x_ADC_Definitions.h.
 */

void MCP3x6x_CONFIG(void)
{
    //GAINCAL --> (8,253,056 / 8,388,607 = 1.615894%) Gain Error w/2.048V Input --> (0b01111011110111010000000 = 0x7DEE80).
    SPI_WRT(_GAINCAL_, GAINCAL_CFG);                                  // GAINCAL_CFG defined in "MCP3x6x_ADC_Definitions.h" file.                               
    
    //OFFSETCAL --> +62 Counts of Offset Cancellation (Measured Offset is Negative) --> (0b000000000000000000000000 = 0x00003E).
    SPI_WRT (_OFFSETCAL_, OFFSETCAL_CFG);                             // OFFSETCAL_CFG defined in "MCP3x6x_ADC_Definitions.h" file.                                
    
    //TIMER --> Disabled --> (0b000000000000000000000000 = 0x000000).
    SPI_WRT(_TIMER_, TIMER_CFG);                                      // TIMER_CFG defined in "MCP3x6x_ADC_Definitions.h" file.                                     
    
    //SCAN --> Disabled --> (0b000000000000000000000000 = 0x000000).
    SPI_WRT(_SCAN_, SCAN_CFG);                                        // SCAN_CFG defined in "MCP3x6x_ADC_Definitions.h" file.                                      
    
    //MUX --> VIN+ = CH0, VIN- = CH1 --> (0b00000001 = 0x01).
    SPI_WRT(_MUX_, MUX_CFG);                                          // MUX_CFG defined in "MCP3x6x_ADC_Definitions.h" file.                                       
    
    //IRQ --> IRQ Mode = Hi-Z IRQ Output, Fast-CMD = Enabled, Conversion-Start Pulse = Disabled --> (0b00000000 = 0x72).
    SPI_WRT(_IRQ_, IRQ_CFG);                                          // IRQ_CFG defined in "MCP3x6x_ADC_Definitions.h" file.                                       
    
    //CONFIG3 --> Conv. Mod = One-Shot Conv. Mode, FORMAT = 24b, CRC_FORMAT = 16b, CRC-COM = Enabled, OFFSETCAL = Enabled, GAINCAL = Enabled --> (0b10000111 = 0x87).
    SPI_WRT(_CONFIG3_, CONFIG3_CFG);                                  // CONFIG3_CFG Register Configuration defined in "MCP3x6x_ADC_Definitions.h" file.                                   
    
    //CONFIG2 --> BOOST = 1x, GAIN = 1x, AZ_MUX = ON --> (0b10001111 = 0x8F).
    SPI_WRT(_CONFIG2_, CONFIG2_CFG);                                  // CONFIG2_CFG defined in "MCP3x6x_ADC_Definitions.h" file.                                   
    
    //CONFIG1 --> AMCLK = MCLK, OSR = 98304 --> (0b00111100 = 0x3C).
    SPI_WRT(_CONFIG1_, CONFIG1_CFG);                                  // CONFIG1_CFG defined in "MCP3x6x_ADC_Definitions.h" file.                                   
    
    //CONFIG0 --> CONFIG[7:6] = Partial Shutdown Mode, CLK_SEL = INTOSC w/o CLKOUT, CS_SEL = No Bias, ADC_MODE = Standby Mode --> (0b11100010 = 0xE2).
    SPI_WRT(_CONFIG0_, CONFIG0_CFG);                                  // CONFIG0_CFG defined in "MCP3x6x_ADC_Definitions.h" file. 

    //LOCK --> 0xA5 enables write-access, LOCK --> 0x00 disables write-access and enables Register Configuration CRC.
    SPI_WRT(_LOCK_, LOCK_CFG);                                        // LOCK_CFG defined in "MCP3x6x_ADC_Definitions.h" file. 
}

/************************************************************************************************************************************************************
 End of File
 */
