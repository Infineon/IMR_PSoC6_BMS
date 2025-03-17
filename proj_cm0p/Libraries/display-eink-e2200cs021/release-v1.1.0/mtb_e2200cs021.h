/***********************************************************************************************//**
 * \file mtb_e2200cs021.h
 *
 * Description: This file is the public interface of the EE2200CS021
 * E-Ink display.
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

#pragma once

#include "cy_result.h"
#include "mtb_e2200cs021_display.h"
#include "mtb_e2200cs021_hw_interface.h"
#include "mtb_e2200cs021_pervasive_hardware_driver.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * \addtogroup group_board_libs E-INK
 * \{
 * APIs for controlling the E-INK display on the board.
 */

// Macros for background color options
/** White background color for \ref mtb_e2200cs021_clear() */
#define MTB_E2200CS021_WHITE_BACKGROUND      (true)
/** Black background color for \ref mtb_e2200cs021_clear() */
#define MTB_E2200CS021_BLACK_BACKGROUND      (false)

// Macros of font coordinates
/** X font start coordinate for \ref mtb_e2200cs021_text_to_frame_buffer() */
#define MTB_E2200CS021_FONT_X                (0x00)
/** Y font start coordinate for \ref mtb_e2200cs021_text_to_frame_buffer() */
#define MTB_E2200CS021_FONT_Y                (0x01)

// Macros of image coordinates
/** X image start coordinate for \ref mtb_e2200cs021_image_to_frame_buffer() */
#define MTB_E2200CS021_IMG_X1                (0x00)
/** X image end coordinate for \ref mtb_e2200cs021_image_to_frame_buffer() */
#define MTB_E2200CS021_IMG_X2                (0x01)
/** Y image start coordinate for \ref mtb_e2200cs021_image_to_frame_buffer() */
#define MTB_E2200CS021_IMG_Y1                (0x02)
/** Y image end coordinate for \ref mtb_e2200cs021_image_to_frame_buffer() */
#define MTB_E2200CS021_IMG_Y2                (0x03)

// Definitions of byte-level colors
/** Value to clear an 8 pixel section of the screen to white. */
#define MTB_E2200CS021_CLEAR_TO_WHITE        (0xFF)
/** Value to clear an 8 pixel section of the screen to black. */
#define MTB_E2200CS021_CLEAR_TO_BLACK        (0x00)

/** Number of bytes needed for a full image (pixel count / 8). */
#define MTB_E2200CS021_FRAME_SIZE           ((MTB_E2200CS021_DISPLAY_SIZE_X * \
                                              MTB_E2200CS021_DISPLAY_SIZE_Y) / 8)

/** Default temperature compensation factor used during initalization. Call
 * \ref mtb_e2200cs021_set_temp_factor() if a different temp is needed. */
#define MTB_E2200CS021_DEFAULT_TEMP_FACTOR   (20)

/** Error changing power state for display. */
#define MTB_E2200CS021_RSLT_ERROR_POWER      (0x01CF) // (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_E2271CS021, 0))

/** Structure that contains font information */
typedef struct
{
    uint8_t* fontData;  /**< Pointer to the font pixel data array */
    uint8_t  xOffset;   /**< X offset of the font in pixels */
    uint8_t  yOffset;   /**< Y offset of the font in pixels */
    uint8_t  xSize;     /**< X size of one font data in bytes */
    uint8_t  ySize;     /**< Y size of one font data in bytes */
    uint8_t  xSpan;     /**< Number of characters that fit the screen horizontally */
    uint8_t  ySpan;     /**< Number of characters that fit the screen vertically */
    bool     color;     /**< Color of the font : true  = black characters in white background,
                                                 false = white characters in black background */
} mtb_e2200cs021_font_t;

/** Different ways the E-INK display can be updated */
typedef enum
{
    /** Update the display with only the changes from previous frame */
    MTB_E2200CS021_PARTIAL,
    /** Full update with four stages:
     * Stage 1: update the display with the inverted version of the previous frame.
     * Stage 2: update the display with an all white frame.
     * Stage 3: update the display with the inverted version of the new frame.
     * Stage 4: update the display with the new frame.
     */
    MTB_E2200CS021_FULL_4STAGE,
    /** Full update with only stages 1 and 4. Stages 2 and 3 are skipped */
    MTB_E2200CS021_FULL_2STAGE
} mtb_e2200cs021_update_t;


// Power and initialization functions
/**
 * Initialize the E-INK display hardware, starts the required hardware
 * components, and sets a default temperature compensation factor.
 * Note: This function does not turn on the E-INK display.
 * @param[in] pins  Pointer to the structure defining the pins used to communicate with the display
 * @param[in] spi   Pointer to an already intialized SPI block to use for
 * communication. Note: The cable select pin should NOT be initialized as part
 * of the SPI interface. It is toggled manually.
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
cy_rslt_t mtb_e2200cs021_init(const mtb_e2200cs021_pins_t* pins, cyhal_spi_t* spi);

/**
 * Frees any hardware resources reserved for the E-INK display.
 */
void mtb_e2200cs021_free();

/**
 * Set temperature in order to perform temperature compensation of E-INK parameters.
 * Note: This function does not turn on the E-INK display.
 * @param[in] temperature    The ambient temperature, in degree C, the display is operating in
 */
void mtb_e2200cs021_set_temp_factor(int8_t temperature);

/**
 * Turns on/off the E-INK display power.
 * Note: This function can not be used to clear the E-INK display. The display
 * will retain the previously written frame even when it's turned off.
 * @param[in] power    Should the power be turned on to the display
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
cy_rslt_t mtb_e2200cs021_power(bool power);

// Display update functions
/**
 * Clears the display to all white or all black pixels.
 *
 * Note1: The E-INK display should be powered on (using \ref mtb_e2200cs021_power())
 * before calling this function if "power_cycle" is false. Otherwise the display
 * won't be cleared.
 *
 * Note2: This function is intended to be called only after a reset/power up.
 * Use \ref mtb_e2200cs021_show_frame() to clear the display if you know the frame
 * that has been written to the display.
 * @param[in] background   \ref MTB_E2200CS021_WHITE_BACKGROUND or
 *                         \ref MTB_E2200CS021_BLACK_BACKGROUND
 * @param[in] power_cycle   true for automatic power cycle. False otherwise
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
cy_rslt_t mtb_e2200cs021_clear(bool background, bool power_cycle);

/**
 * Updates the E-INK display with the provided frame stored in the flash or
 * RAM. Afterward the new frame data is copied to the previous frame buffer.
 *
 * Note: If the previous frame data changes from the actual frame previously
 * written to the display, considerable ghosting may occur.
 *
 * If the "power_cycle" parameter is false, the E-INK display should be powered on
 * (using \ref mtb_e2200cs021_power()) before calling this function. Otherwise the display
 * won't be updated.
 * @param[in] prev_frame   Pointer to the previous frame written on the display
 * @param[in] new_frame    Pointer to the new frame that need to be written
 * @param[in] update_type  Full update (2/4 stages) or partial update
 * @param[in] power_cycle  "true" for automatic power cycle, "false" for manual
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
cy_rslt_t mtb_e2200cs021_show_frame(
    uint8_t* prev_frame, uint8_t* new_frame, mtb_e2200cs021_update_t update_type, bool power_cycle);

// Frame buffer operations
/**
 * Copies pixel block data between the specified x and y coordinates
 * from an image (typically stored in the flash) to a frame buffer in RAM.
 *
 * Copying is done at byte level. Pixel level operations are not supported.
 *
 * This function does not update the E-INK display. After frame buffer update,
 * use \ref mtb_e2200cs021_show_frame() to update the display if required.
 *
 * NOTE: This is a blocking function that can take as many as
 * \ref MTB_E2200CS021_FRAME_SIZE RAM write cycles to complete.
 * @param[in,out] frame_buffer    Pointer to the frame buffer array in RAM
 * @param[in]     image           Pointer to the image array (typically in flash)
 * @param[in]     img_coordinates Pointer to a 4-byte array of image coordinates
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
void mtb_e2200cs021_image_to_frame_buffer(uint8_t* frame_buffer, uint8_t* image,
                                          uint8_t* img_coordinates);
/**
 * Converts a text input (string) to pixel data and stores it at the specified
 * coordinates of a frame buffer.
 *
 * Notes: This function only supports printable ASCII characters from "!" to "~"
 *
 * X and Y locations used in this function are font coordinates, rather than the
 * pixel coordinates used in \ref mtb_e2200cs021_image_to_frame_buffer(). See the
 * mtb_e2200cs021_fonts.h for details.
 *
 * This function does not update the E-INK display. After frame buffer update,
 * use \ref mtb_e2200cs021_show_frame() to update the display if required.
 *
 * NOTE: This is a blocking function that can take as many as
 * \ref MTB_E2200CS021_FRAME_SIZE RAM write cycles to complete.
 * @param[in,out] frame_buffer     Pointer to the frame buffer array in RAM
 * @param[in]     string           Pointer to the string
 * @param[in]     font_info        Structure that stores font information
 * @param[in]     font_coordinates Pointer to the 2-byte array containing X and
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
void mtb_e2200cs021_text_to_frame_buffer(
    uint8_t* frame_buffer, char* string, mtb_e2200cs021_font_t* font_info,
    uint8_t* font_coordinates);

/** \} group_board_libs */

#if defined(__cplusplus)
}
#endif
