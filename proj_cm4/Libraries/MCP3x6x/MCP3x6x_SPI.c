/********************************************************************************
* Company:
*   Microchip Technology Inc.
*
* File Name:
*   MCP3x6x_SPI.cpp
*
* Summary:
*   MCP3x6x(R) Delta-Sigma ADC SPI Read-CMD, Write-CMD, Fast-CMD, STATUS Byte 
*   Retrieval and CRC Calculation functions. 
*
* Description:
*   This file contains the source code for the MCP3x6x(R) Delta-Sigma ADC SPI 
*   Read-CMD, Write-CMD, Fast-CMD, STATUS Byte Retrieval and CRC Calculation 
*   functions for use with the Arduino Zero Board and the ATSAMD21G18 MCU.  
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

#include "MCP3x6x_SPI.h"                                                // SPI definitions header-file.

#include "stdio.h"
#include "stdint.h"

#include "MCP3x6x_ADC.h"                                                // ADC definitions header-file.
#include "MCP3x6x_Type.h"                                               // Variable definitions header-file.
#include "MCP3x6x_Target.h"												// Controller specific commands - interface to hardware.

/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/
/*                     Section: MCP3x6x(R) SPI Bus Write/Read, Conversion-Start, Retieve Status, Fast-CMD and CRC Function Prototypes                      */
/***********************************************************************************************************************************************************/
/***********************************************************************************************************************************************************/

/************************************************************************************************************************************************************
  Function:
    void SPI_WRT(uint8_t, uint32_t)

  Remarks:
    - SPI Write-CMD Function. 
    - See prototype in MCP3x6x_SPI_Definitions.h.
 */

void SPI_WRT(uint8_t REG_ADDR, uint32_t REG_CFG)
{
    _u32data_t WR_DATA;                                                             // REG_ADDR Register write-data variable.

    WR_DATA.DWORD = REG_CFG;                                                        // REG_ADDR Register write-data value. 

    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _LOCK_) 
    {
       set_CS_SPI_LOW();                                             // Assert nCS Pin low.
        
        spiTransfer((REG_ADDR << 2) | _WRT_CTRL_);                                 // Write-CMD of REG_ADDR Register.
        spiTransfer(WR_DATA.BYTE.LOW);                                             // 8-bit REG_ADDR Register write-data byte.
        
        set_CS_SPI_HIGH();                                            // Assert nCS Pin high.
    } 
    if (REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_) 
    {
       set_CS_SPI_LOW();                                             // Assert nCS Pin low.
        
        spiTransfer((REG_ADDR << 2) | _WRT_CTRL_);                                 // Write-CMD of REG_ADDR Register.
        spiTransfer(WR_DATA.BYTE.UPPER);                                           // 24-bit REG_ADDR Register write-data upper-byte.
        spiTransfer(WR_DATA.BYTE.HIGH);                                            // 24-bit REG_ADDR Register write-data high-byte.
        spiTransfer(WR_DATA.BYTE.LOW);                                             // 24-bit REG_ADDR Register write-data low-byte.
        
        set_CS_SPI_HIGH();                                            // Assert nCS Pin high
    }  
} 

/***********************************************************************************************************************************************************
  Function:
    uint32_t SPI_RD(uint8_t, _u16data_t&, uint64_t&)

  Remarks:
    - SPI Read-CMD Function. 
    - See prototype in MCP3x6x_SPI_Definitions.h.
 */

uint32_t SPI_RD(uint8_t REG_ADDR, _u16data_t* COMM_CRC, uint64_t* CALC_CRC)
{
    _u32data_t RD_DATA;                                                             // REG_ADDR Register read-data variable.                                                
    
    _u64data_t CRC_DATA_64b;                                                        
    CRC_DATA_64b.QWORD = 0x0000000000000000;                                        // Reset CRC_DIVD_64b dividend value.

    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _RESERVED_C_ || REG_ADDR == _LOCK_)    
    {
       set_CS_SPI_LOW();                                             // Assert nCS Pin low.
        
        RD_DATA.BYTE.STATUS = spiTransfer((REG_ADDR << 2) | _RD_CTRL_);            // Read-CMD of REG_ADDR Register.
        RD_DATA.BYTE.LOW = spiTransfer(0x00);                                      // Read 8-bit REG_ADDR Register data.
    
        #ifdef ENABLE_SPI_CRC                                                       // Insert SPI Communication-CRC code if enabled. 
            COMM_CRC->BYTE.HIGH = spiTransfer(0x00);                                // Read CRC high-byte.
            COMM_CRC->BYTE.LOW = spiTransfer(0x00);                                 // Read CRC low-byte.

            CRC_DATA_64b.BYTE.BYTE5 = RD_DATA.BYTE.STATUS;                          // Load STATUS Byte to CRC_DIVD_64b high-high-word high-byte.
            CRC_DATA_64b.BYTE.BYTE4 = RD_DATA.BYTE.LOW;                             // Load 8-bit DATA-Byte to CRC_DIVD_64b high-high-word low-byte.
        #endif
    
        set_CS_SPI_HIGH();                                            // Assert nCS Pin high.
    }
    else if (REG_ADDR == _RESERVED_E_ || REG_ADDR == _CRCCFG_)
    {            
       set_CS_SPI_LOW();                                             // Assert nCS Pin low.
        
        RD_DATA.BYTE.STATUS = spiTransfer((REG_ADDR << 2) | _RD_CTRL_);            // Read-CMD of REG_ADDR Register.
        RD_DATA.BYTE.HIGH = spiTransfer(0x00);                                     // Read 16-bit REG_ADDR Register data high-byte.
        RD_DATA.BYTE.LOW = spiTransfer(0x00);                                      // Read 16-bit REG_ADDR Register data low-byte.
    
        #ifdef ENABLE_SPI_CRC                                                       // Insert SPI Communication-CRC code if enabled.
            COMM_CRC->BYTE.HIGH = spiTransfer(0x00);                                // Read CRC high-byte.
            COMM_CRC->BYTE.LOW = spiTransfer(0x00);                                 // Read CRC low-byte.

            CRC_DATA_64b.WORD.UPPER = RD_DATA.BYTE.STATUS;                          // Load STATUS Byte to CRC_DIVD_64b upper-word.
            CRC_DATA_64b.WORD.HIGHH = RD_DATA.WORD.LOW;                             // Load 16-bit DATA-Byte to CRC_DIVD_64b high-high-word.
        #endif 
    
        set_CS_SPI_HIGH();                                            // Assert nCS Pin high.
    }
    else if (REG_ADDR == _ADCDATA_ || REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_ || REG_ADDR == _RESERVED_B_)
    {    
       set_CS_SPI_LOW();                                             // Assert nCS Pin low.

        RD_DATA.BYTE.STATUS = spiTransfer((REG_ADDR << 2) | _RD_CTRL_);            // Read-CMD of REG_ADDR Register.
        RD_DATA.BYTE.UPPER = spiTransfer(0x00);                                    // 24-bit REG_ADDR Register data upper-byte.
        RD_DATA.BYTE.HIGH = spiTransfer(0x00);                                     // 24-bit REG_ADDR Register data high-byte.
        RD_DATA.BYTE.LOW = spiTransfer(0x00);                                      // 24-bit REG_ADDR Register data low-byte.
    
        #ifdef ENABLE_SPI_CRC                                                       // Insert SPI Communication-CRC code if enabled
            COMM_CRC->BYTE.HIGH = spiTransfer(0x00);                                // CRC high-byte.
            COMM_CRC->BYTE.LOW = spiTransfer(0x00);                                 // CRC low-byte.
      
            CRC_DATA_64b.DWORD.HIGH = RD_DATA.DWORD;                                // Message Data w/STATUS Byte.  
        #endif 
    
        set_CS_SPI_HIGH();                                            // Assert nCS Pin high.
    }
    else
    {
        RD_DATA.DWORD = 0;                                                       // Return NULL Byte if REG_ADDR is invalid.
    }
    
    #ifdef ENABLE_SPI_CRC
        *CALC_CRC = SPI_COMMCRC(CRC_DATA_64b);                                       // Insert CRC-Calculation if enabled.
    #endif 

    return RD_DATA.DWORD;                                                           // Return REG_ADDR Register data. 
}

/***********************************************************************************************************************************************************
  Function:
    uint8_t CONV_START(uint8_t)

  Remarks:
    - ADC Conversion-Start Function.   
    - See prototype in MCP3x6x_SPI_Definitions.h.
 */

 uint8_t CONV_START(uint8_t MUX_CHNL)
 {
    uint8_t CONV_START_STATUS_BYTE = 0;                                                 // STATUS Byte for CONV_START() function. NOTE: Manual change over original lib required!
    uint16_t STATUS_LOOP_COUNT;                                                     // STATUS Byte retrieval Loop-Count. 
    
    SPI_WRT(_MUX_, MUX_CHNL);                                                       // ADC Conversion on channel MUX_CHNL.  
    
    #ifdef ENABLE_ADCCONV_FASTCMD
      SPI_FASTCMD(_ADC_CONV_STRST_);                                                // Alternative -- Send ADC-Converstion-Start/Restart Fast-CMD.    
    #else
      SPI_WRT(_CONFIG0_, (CONFIG0_CFG | ADC_MODE_CONV));                            // Write ADC_MODE[1:0] = ADC Conversion Mode.
    #endif

    DELAY_MS(100);                                                                     // Delay 100ms per MCP3x6x(R) Data-Ready(DR) STATUS Bit Errata.

    STATUS_LOOP_COUNT = 0x00;                                                       // Reset STATUS Byte Loop-Count to 0. 

    while((CONV_START_STATUS_BYTE != 0x13) && STATUS_LOOP_COUNT < 65535)            // Wait until Data-Ready(DR) Bit of STATUS Byte is set.
    {
        CONV_START_STATUS_BYTE = RETRIEVE_STATUS();                                 // Read STATUS Byte for Data-Ready(DR) Bit.
        STATUS_LOOP_COUNT++;                                                        // STATUS Byte test count limited to 65535. 
    }
    
    return CONV_START_STATUS_BYTE;                                                  // Return STATUS Byte for CONV_START() function. 
 }

/***********************************************************************************************************************************************************
  Function:
    uint8_t RETRIEVE_STATUS(void)

  Remarks:
    - ADC STATUS Byte Retrieval Function.
    - See prototype in MCP3x6x_SPI_Definitions.h.
 */

uint8_t RETRIEVE_STATUS(void)
{
    uint8_t RETRIEVE_STATUS_STATUS_BYTE;                                            // STATUS Byte for RETRIEVE_STATUS() function. 

   set_CS_SPI_LOW();                                                 // Assert nCS Pin low.

    RETRIEVE_STATUS_STATUS_BYTE = spiTransfer(_RTRV_STATUS_);                      // Retrieve STATUS Byte.
    
    set_CS_SPI_HIGH();                                                // Assert nCS Pin high.

    return RETRIEVE_STATUS_STATUS_BYTE;                                             // Return STATUS Byte for RETRIEVE_STATUS() function. 
}

/***********************************************************************************************************************************************************
  Function:
    void SPI_FASTCMD(uint8_t)

  Remarks:
    - SPI Fast-CMD Function.
    - See prototype in MCP3x6x_SPI_Definitions.h.
 */

void SPI_FASTCMD(uint8_t FAST_CMD)
{
   set_CS_SPI_LOW();                                                  // Assert nCS Pin low.

    spiTransfer(FAST_CMD);                                                         // Send Fast-CMD.
    
    set_CS_SPI_HIGH();                                                // Assert nCS Pin high.
} 

/***********************************************************************************************************************************************************
  Function:
    uint64_t SPI_COMCRC(_u64data_t)

  Remarks:
    - SPI Communication-CRC Algorithm.  
    - See prototype in MCP3x6x_SPI_Definitions.h.
 */

uint64_t SPI_COMMCRC(_u64data_t CRC_DIVD_64b)
{
    /****************************************************************************************/
    /*************************** "Appended Zeros" CRC Calculation ***************************/
    /****************************************************************************************/
    
    _uint64_t CRC_POLY_START_64b = 0xC002800000000000;                              // 0x18005 CRC-Polynomial left-justified for 64-bit CRC calculation starting point.
    _uint64_t CRC_POLY_END_64b = 0x00000000C0028000;                                // 0x18005 CRC-Polynomial left-shifted for 64-bit CRC calculation ending point.     
    _uint64_t bitptr_64b = 0x8000000000000000;                                      // Bit-Pointer to find "1's" in the CRC_DVID_64b Dividend value.    

    while(CRC_POLY_START_64b != CRC_POLY_END_64b)
    {
        if((CRC_DIVD_64b.QWORD & bitptr_64b) == bitptr_64b)                         // If Dividend bit is a '1'?
        {
            CRC_DIVD_64b.QWORD = (CRC_DIVD_64b.QWORD ^ CRC_POLY_START_64b);         // Compute XOR for CRC.
        }
        else                                                                        // If Dividend bit is a '0'?
        {   
            CRC_POLY_START_64b = (CRC_POLY_START_64b >> 1);                         // Right-shift polynomial for XOR computation.
            bitptr_64b = (bitptr_64b >> 1);                                         // Right-shift bit-pointer to next bit of Dividend. 
        }
    }

    return CRC_DIVD_64b.QWORD;
} 

/************************************************************************************************************************************************************
 End of File
 */
