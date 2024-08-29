/***********************************************************************************************//**
 * \file mtb_e2200cs021.h
 *
 * Description: This file is the implementation of the public E-Ink interface.
 *
 * For the details of E-INK display hardware and driver interface, see the
 * documents available at the following website:
 * https://www.pervasivedisplays.com/product/2-71-e-ink-display/
 *
 ***************************************************************************************************
 * \copyright
 * Copyright 2018-2021 Cypress Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************************************/

#include "mtb_e2200cs021.h"

//--------------------------------------------------------------------------------------------------
// mtb_e2200cs021_init
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_e2200cs021_init(const mtb_e2200cs021_pins_t* pins, cyhal_spi_t* spi)
{
    CY_ASSERT(pins != NULL);
    CY_ASSERT(spi != NULL);

    cy_rslt_t rslt = MTB_E2200CS021_InitDriver(pins, spi);

    // Initialize the E-INK display hardware and associated PSoC components
    if (CY_RSLT_SUCCESS == rslt)
    {
        Pv_EINK_Init(pins);
    }

    mtb_e2200cs021_set_temp_factor(MTB_E2200CS021_DEFAULT_TEMP_FACTOR);

    return rslt;
}


//--------------------------------------------------------------------------------------------------
// mtb_e2200cs021_free
//--------------------------------------------------------------------------------------------------
void mtb_e2200cs021_free()
{
    MTB_E2200CS021_FreeDriver();
}


//--------------------------------------------------------------------------------------------------
// mtb_e2200cs021_set_temp_factor
//--------------------------------------------------------------------------------------------------
void mtb_e2200cs021_set_temp_factor(int8_t temperature)
{
    // Perform temperature compensation of E-INK parameters
    Pv_EINK_SetTempFactor(temperature);
}


//--------------------------------------------------------------------------------------------------
// mtb_e2200cs021_power
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_e2200cs021_power(bool power)
{
    pv_eink_status_t pwrStatus = (power)
        ? Pv_EINK_HardwarePowerOn()
        : Pv_EINK_HardwarePowerOff();

    return (pwrStatus == PV_EINK_RES_OK)
        ? CY_RSLT_SUCCESS
        : MTB_E2200CS021_RSLT_ERROR_POWER;
}


//--------------------------------------------------------------------------------------------------
// mtb_e2200cs021_clear
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_e2200cs021_clear(bool background, bool powerCycle)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    // If power cycle operation requested, turn on E-INK power
    if (powerCycle)
    {
        result = mtb_e2200cs021_power(true);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        pv_eink_frame_data_t* frame = (background == MTB_E2200CS021_WHITE_BACKGROUND)
            ? PV_EINK_WHITE_FRAME_ADDRESS
            : PV_EINK_BLACK_FRAME_ADDRESS;

        // Two consecutive display updates to reduce ghosting
        Pv_EINK_FullStageHandler(frame, PV_EINK_STAGE4);
        Pv_EINK_FullStageHandler(frame, PV_EINK_STAGE4);

        // If power cycle operation requested, turn off E-INK power
        if (powerCycle)
        {
            result = mtb_e2200cs021_power(false);
        }
    }

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_e2200cs021_show_frame
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_e2200cs021_show_frame(uint8_t* prevFrame, uint8_t* newFrame,
                                    mtb_e2200cs021_update_t updateType, bool powerCycle)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    // If power cycle operation requested, turn on E-INK power
    if (powerCycle)
    {
        result = mtb_e2200cs021_power(true);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        if (updateType == MTB_E2200CS021_PARTIAL)
        {
            // Update the display with changes from previous frame
            Pv_EINK_PartialStageHandler(prevFrame, newFrame);
        }
        else
        {
            // Stage 1: update the display with the inverted version of the previous frame
            Pv_EINK_FullStageHandler(prevFrame, PV_EINK_STAGE1);

            if (updateType == MTB_E2200CS021_FULL_4STAGE)
            {
                // Stage 2: update the display with an all white frame
                Pv_EINK_FullStageHandler(prevFrame, PV_EINK_STAGE2);
                // Stage 3: update the display with the inverted version of the new frame
                Pv_EINK_FullStageHandler(newFrame, PV_EINK_STAGE3);
            }

            // Stage 4: update the display with the new frame
            Pv_EINK_FullStageHandler(newFrame, PV_EINK_STAGE4);
        }

        // If power cycle operation requested, turn off E-INK power
        if (powerCycle)
        {
            result = mtb_e2200cs021_power(false);
        }
    }

    if (CY_RSLT_SUCCESS == result)
    {
        memcpy(prevFrame, newFrame, MTB_E2200CS021_FRAME_SIZE);
    }

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_e2200cs021_image_to_frame_buffer
//--------------------------------------------------------------------------------------------------
void mtb_e2200cs021_image_to_frame_buffer(
    uint8_t* frameBuffer, uint8_t* image, uint8_t* imgCoordinates)
{
    // Counter variable for horizontal pixel lines
    uint16_t pixelLineCounter;

    // Variable that stores the starting location of the current pixel line
    uint16_t startLocation;

    // Variable that stores the length of the pixel line
    uint16_t lineLength;

    // Do this for all horizontal pixel lines between Y1 and Y2 coordinates
    for (pixelLineCounter  = imgCoordinates[MTB_E2200CS021_IMG_Y1];
         pixelLineCounter <= imgCoordinates[MTB_E2200CS021_IMG_Y2];
         pixelLineCounter++)
    {
        // Find the starting location of the pixel line
        startLocation = (pixelLineCounter * PV_EINK_HORIZONTAL_SIZE) +
                        imgCoordinates[MTB_E2200CS021_IMG_X1];

        // Find the length of the pixel line to be copied
        lineLength = imgCoordinates[MTB_E2200CS021_IMG_X2] - imgCoordinates[MTB_E2200CS021_IMG_X1];

        // Copy the pixel line from the image to the same location in the frame buffer
        memcpy(&frameBuffer[startLocation], &image[startLocation], lineLength);
    }
}


//--------------------------------------------------------------------------------------------------
// mtb_e2200cs021_text_to_frame_buffer
//--------------------------------------------------------------------------------------------------
void mtb_e2200cs021_text_to_frame_buffer(
    uint8_t* frameBuffer, char* string, mtb_e2200cs021_font_t* fontInfo, uint8_t* fontCoordinates)
{
    // Range of ASCII printable characters: "!" to "~"
    const uint8_t MTB_E2200CS021_ASCII_MIN = (0x21);
    const uint8_t MTB_E2200CS021_ASCII_MAX = (0x7E);

    // Pointer variable for current character under conversion
    char* currentChar;

    // Variable for storing the location in the frame buffer to which the font data is printed
    uint16_t printLocation;
    // Variable for storing the character location in the font
    uint16_t charLocation;

    // Variable that stores the pixel data byte location in the frame buffer
    uint16_t frameByte;
    // Variable that stores the pixel data byte location in the font array
    uint16_t fontByte;

    // Counter variable for the vertical loop
    uint8_t verticalCounter;
    // Counter variable for the horizontal loop
    uint8_t horizontalCounter;

    // Repeat this process until the null character of the string is reached
    for (currentChar = string; *currentChar != '\0'; currentChar++)
    {
        // Check if X and Y coordinates are between the font's max limits
        if ((fontCoordinates[MTB_E2200CS021_FONT_X] < (fontInfo->xSpan)) &&
            (fontCoordinates[MTB_E2200CS021_FONT_Y] < (fontInfo->ySpan)))
        {
            // Find the actual pixel location in the frame buffer where the pixel data should be
            // copied
            printLocation = (fontInfo->xOffset) +
                            (fontCoordinates[MTB_E2200CS021_FONT_X] * (fontInfo->xSize)) +
                            (((fontCoordinates[MTB_E2200CS021_FONT_Y] * (fontInfo->ySize)) +
                              (fontInfo->yOffset)) * PV_EINK_HORIZONTAL_SIZE);

            // Check if the current character is a printable ASCII character between "!" and "~"
            if ((*currentChar <= MTB_E2200CS021_ASCII_MAX) &&
                (*currentChar >= MTB_E2200CS021_ASCII_MIN))
            {
                // Find the location of pixel data corresponding to the current character from the
                // font array
                charLocation = (*currentChar - MTB_E2200CS021_ASCII_MIN) *
                               ((fontInfo->ySize) * (fontInfo->xSize));

                // Vertical copying loop
                for (verticalCounter = 0; verticalCounter < (fontInfo->ySize);
                     verticalCounter++)
                {
                    // Horizontal copying loop
                    for (horizontalCounter = 0;
                         horizontalCounter < (fontInfo->xSize);
                         horizontalCounter++)
                    {
                        // Calculate the current pixel data byte location of the frame buffer
                        frameByte = horizontalCounter +
                                    (printLocation + (PV_EINK_HORIZONTAL_SIZE * verticalCounter));

                        // Calculate the current pixel data byte location of the font
                        fontByte = charLocation + (verticalCounter * (fontInfo->xSize)) +
                                   horizontalCounter;

                        // Copy the pixel data for background, invert for white background
                        frameBuffer[frameByte] =
                            ((fontInfo->color) == MTB_E2200CS021_WHITE_BACKGROUND)
                            ? ~(fontInfo->fontData[fontByte])
                            : fontInfo->fontData[fontByte];
                    }
                }
            }
            // For space and non-printable ASCII characters
            else
            {
                // Vertical copying loop
                for (verticalCounter = 0;
                     verticalCounter < (fontInfo->ySize);
                     verticalCounter++)
                {
                    // Horizontal copying loop
                    for (horizontalCounter = 0;
                         horizontalCounter < (fontInfo->xSize);
                         horizontalCounter++)
                    {
                        // Calculate the current pixel data byte location of the frame buffer
                        frameByte = horizontalCounter +
                                    (printLocation + (PV_EINK_HORIZONTAL_SIZE * verticalCounter));

                        // Copy pixels to create a space for font's background
                        frameBuffer[frameByte] =
                            ((fontInfo->color) == MTB_E2200CS021_WHITE_BACKGROUND)
                            ? MTB_E2200CS021_CLEAR_TO_WHITE
                            : MTB_E2200CS021_CLEAR_TO_BLACK;
                    }
                }
            }
        }
        // Move on to the next character by incrementing the horizontal location
        fontCoordinates[MTB_E2200CS021_FONT_X]++;

        // Reset the X location and increment the Y location if the X span is reached
        if (fontCoordinates[MTB_E2200CS021_FONT_X] >= (fontInfo->xSpan))
        {
            fontCoordinates[MTB_E2200CS021_FONT_X] = 0;
            fontCoordinates[MTB_E2200CS021_FONT_Y]++;
        }
        // End the function if the Y span is reached
        if (fontCoordinates[MTB_E2200CS021_FONT_Y] > (fontInfo->ySpan))
        {
            break;
        }
    }
}
