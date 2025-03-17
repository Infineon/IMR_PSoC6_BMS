/*
 * tle9012_interpreter.h
 *
 *	Library for TLE9012 Li-Ion battery monitoring and balancing system IC.
 *	See main file Lib_TLE9012.c for how to use guide
 *	Created by R. Santeler @ Infineon 2023
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

#ifndef TLE9012_INTERPRETER_H__
#define TLE9012_INTERPRETER_H__

#include <stdint.h>
#include "tle9012_target.h"

 //****************************************************************************
 // Definitions
 //****************************************************************************
#define RX_RING_BUFFER_SIZE 300

#define PACKAGE_ID_FRAME_IDX    			0	// The ID frame is always at first position
#define PACKAGE_ADR_FRAME_IDX				1	// The address frame is always after the ID
#define PACKAGE_R_COMMAND_CRC_FRAME_IDX 	2	// Read commands have no data,
												// therefore the CRC comes next
#define PACKAGE_DATA0_FRAME_IDX				2	// All replies have data at index 2 and 3
#define PACKAGE_DATA1_FRAME_IDX 			3	// All replies have data at index 2 and 3
#define PACKAGE_CRC_FRAME_IDX 				4	// All replies have CRC  at index 4
#define PACKAGE_W_REPLY_FRAME_IDX 			5	// Write relies have a special reply frame
												// at index 5

#define SIZE_WRITE_COMMAND 		7
#define SIZE_READ_COMMAND 		4
#define SIZE_READ_PACKAGE 		5
#define SIZE_MINIMAL_RX_SIZE 	SIZE_READ_COMMAND

#define ID_FRAME_ID_MASK				0b00111111
#define ID_FRAME_ID_BROADCAST			0b00111111
#define ID_FRAME_ISWRITE_MASK			0b10000000
#define REPLY_FRAME_CRC_ERROR_MASK		0b00100000
#define REPLY_FRAME_REG_ADR_ERROR_MASK  0b00010000
#define REPLY_FRAME_GENERAL_FAULT_MASK  0b00001000
#define REPLY_FRAME_3BIT_CRC_MASK       0b00000111

//****************************************************************************
// Data structures
//****************************************************************************
typedef enum { InterpreterSuccess = 0U, InterpreterFail = 1U } InterpreterResult;

//****************************************************************************
// Interpreter and management
//****************************************************************************
//uint16_t receive_index; 		// The index of the last byte in the receive ring buffer,
								// new bytes will be added after this index
//uint16_t interpreter_index; 	// The index of the currently processed byte in
								// the receive ring buffer
extern InterpreterResult rx_buffer_state; // The last state of the receive ISR
// The uart rx fifo ring buffer being filled by e.g. the uart ISR
extern uint8_t rx_ring_buffer[RX_RING_BUFFER_SIZE];

// Function Prototypes
void resetInterpreter();
void dumpReceiveBuffer();
InterpreterResult addToReceiveRingBuffer_tle9012(uint8_t data);
void tle9012_manageInterpreter();

#endif /* TLE9012_INTERPRETER_H__ */
