/*
 * tle9012_interpreter.c
 *
 *	Library for TLE9012 Li-Ion battery monitoring and balancing system IC. See main file Lib_TLE9012.c for how to use guide
 *	Created by R. Santeler @ Infineon 2023
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <stdlib.h> // includes strtol
#include "tle9012_interpreter.h"
#include "tle9012_registers.h"

//****************************************************************************
// Definitions and Objects for the interpreter system
//****************************************************************************
#define DEBUGPRINTF_INTERP_PERF(p_frm, ...)	//DEBUGPRINTF(p_frm, ##__VA_ARGS__)	// De/activate performance debug messages of interpreter system
#define DEBUGPRINTF_ERROR(p_frm, ...)		DEBUGPRINTF(p_frm, ##__VA_ARGS__)
#define DEBUGPRINTF_INCOMPLETE(p_frm, ...)  //DEBUGPRINTF(p_frm, ##__VA_ARGS__)
#define DEBUGPRINTF_CRC_FAIL(p_frm, ...)  	//DEBUGPRINTF(p_frm, ##__VA_ARGS__)
#define DEBUGPRINTF_INFO(p_frm, ...)		//DEBUGPRINTF(p_frm, ##__VA_ARGS__)
#define DEBUGPRINTF_SUCCESS(p_frm, ...)		//DEBUGPRINTF(p_frm, ##__VA_ARGS__)

typedef enum { I_PARSE_READ_RETURN = 0, I_PARSE_SUCCESS, I_PACKET_INCOMPLETE, I_PARSE_FAILED } interpreterStates;

// Objects for the interpreter system
uint16_t receive_index = 0; // The index of the last byte in the receive ring buffer, new bytes will be added after this index
uint16_t interpreter_index = 0; // The index of the last processed byte in the receive ring buffer, will parse the one ofter this next
InterpreterResult rx_buffer_state = InterpreterSuccess; // The last state of the receive ISR
uint8_t  rx_ring_buffer[RX_RING_BUFFER_SIZE] = {0};
// {0x1E, 0x81, 0x4, 0x50, 0x0, 0x18, 0x0};             // Write commands
// {0x1E,  0x1, 0xB, 0x63, 0x1,  0xB, 0x0, 0x1, 0x59};  // Read command with reply
// {0x1E, 0x1, 0xB, 0x63};                              // Single read command
// {0x1, 0xB, 0x0, 0x1, 0x59};                          // Single read reply

//****************************************************************************
// Inline Functions
//****************************************************************************
static inline void incrementInterpreterIndex(uint16_t* index) {
	(*index)++;
	if (*index > RX_RING_BUFFER_SIZE - 1) {
		(*index) = 0;
	}
}
static inline void decrementInterpreterIndex(uint16_t* index) {
	if ((*index) >= 1)
		(*index)--;
	else
		(*index) = RX_RING_BUFFER_SIZE - 1;
}
static inline void incrementInterpreterIndexBy(uint16_t* index, uint8_t num) {
	for (uint8_t i = 0; i < num; i++) {
		incrementInterpreterIndex(index);
	}
}
static inline uint16_t getInterpreterAbsoluteIndex(uint16_t start_index, uint16_t relative_index) {
	return (start_index + relative_index) % RX_RING_BUFFER_SIZE;
}



//****************************************************************************
// Functions
//****************************************************************************
//****************************************************************************
// resetInterpreter - Resets the indices and first ring buffer element
//****************************************************************************
void resetInterpreter(){
	receive_index = 0;
	interpreter_index = 0;
	rx_ring_buffer[0] = 0;
}

//****************************************************************************
// addToReceiveRingBuffer_tle9012 - Takes a new byte to be added to the uart
// receive ring buffer.
//
// Takes a 8bit data byte
// Returns 0 on success or1 if buffer is full
//****************************************************************************
InterpreterResult addToReceiveRingBuffer_tle9012(uint8_t data) {
	// Determine index of next item
	incrementInterpreterIndex(&receive_index);

	// If the next item index does not collide with currently processed item index...
	if (receive_index != interpreter_index) {
		// ... store received byte item
		rx_ring_buffer[receive_index] = data;

		// Return OK
		//printf("\tAdd %02X to buffer at %d\r\n", data, receive_index);
		return InterpreterSuccess;
	}

	// Revert to last item index
	decrementInterpreterIndex(&receive_index);

	// Return FAIL
	//printf("\tAdd  to buffer FAILED: Buffer overflow!!!!\r\n");
	return InterpreterFail;
}


//****************************************************************************
// Returns the number of uninterpreted bytes in the ring buffer
//****************************************************************************
uint16_t getInterpreterIndexDifference() {
	// If the indices match there is no difference
	if (interpreter_index == receive_index) {
		//DEBUGPRINTF("Same\r\n");
		return 0;
	}
	// If the receive index is higher than the interpret index a normal difference can be applied
	else if (receive_index > interpreter_index) {
		//DEBUGPRINTF("Normal\r\n");
		return (receive_index - interpreter_index);
	}
	// If the receive index is lower than the interpret index the leap over must be accounted for
	else if (receive_index < interpreter_index) {
		//DEBUGPRINTF("Over\r\n");
		return ((receive_index + RX_RING_BUFFER_SIZE) - interpreter_index);
	}

	// Should never come here
	return -1;
}


//****************************************************************************
// Return data byte at the over leap correct index inside a package
//****************************************************************************
static inline uint8_t getPackageByte(uint16_t package_start_index, uint16_t relative_index) {
	return rx_ring_buffer[(package_start_index + relative_index) % RX_RING_BUFFER_SIZE];
}


//****************************************************************************
// calcPackageCRC8 - Calculates the crc8 sum direct in rx_ring_buffer at the given start, for the given length
// Returns the checksum
// TODO get faster code and document reference
//****************************************************************************
unsigned char calcPackageCRC8(uint16_t crc_start_index, uint8_t length) {
	unsigned long crc;
	int i, bit;

	crc = 0xFF;
	for (i = 0; i < length; i++) {
		crc ^= getPackageByte(crc_start_index, i);
		for (bit = 0; bit < 8; bit++) {
			if ((crc & 0x80) != 0) {
				crc <<= 1;
				crc ^= 0x1D;
			}
			else {
				crc <<= 1;
			}
		}
	}

	return (~crc) & 0xFF;
}


//****************************************************************************
// Debug function to display all bytes in buffer
//****************************************************************************
void dumpReceiveBuffer() {
	DEBUGPRINTF("Buffer dump - receive index %d, interpreter index %d: ", receive_index, interpreter_index);
	for (int i = 0; i <= RX_RING_BUFFER_SIZE - 1; i++){
		DEBUGPRINTF("0x%02X, ", getPackageByte(0, i));
	}
	DEBUGPRINTF("\r\n");
}

//****************************************************************************
// Used to set error/ready bit and write data to shadow register.
// Checks register index node ID and isSuccessful to determine what states shall be set.
//****************************************************************************
static inline void setRegistersStateAndData(uint8_t nodeID, uint8_t regIdx, uint16_t data, uint8_t isSuccessful) {
	if (regIdx < NUM_REGISTERS) {
		if (isSuccessful == 1) {
			if (nodeID > 0 && nodeID <= TLE9012_NODES_SIZE) {
				setRegisterCacheDataByIndex_tle9012(nodeID, regIdx, data);
				setRegisterReady_tle9012(nodeID, regIdx, 0);
				resetRegisterError_tle9012(nodeID, regIdx, 0);

			}
			else if (nodeID == ID_FRAME_ID_BROADCAST) {
				DEBUGPRINTF_INFO("\t\t\t\tNodeID %d detected as Broadcast - write data to every Node\r\n", nodeID);
				for (int i = 0; i < TLE9012_NODES_SIZE; i++) {
					setRegisterCacheDataByIndex_tle9012(i+1, regIdx, data);
				}
				setRegisterReady_tle9012(nodeID, regIdx, 1);
				resetRegisterError_tle9012(nodeID, regIdx, 1);
			}
			else {
				DEBUGPRINTF_ERROR("\t\t\t\tNodeID %d is 0 or exceeded size of NodeArray - Ignoring package\r\n", nodeID);
			}
		}
		else {
			if (nodeID > 0 && nodeID <= TLE9012_NODES_SIZE) {
				setRegisterReady_tle9012(nodeID, regIdx, 0);
				setRegisterError_tle9012(nodeID, regIdx, 0);
			}
			else if (nodeID == ID_FRAME_ID_BROADCAST) {
				DEBUGPRINTF_INFO("\t\t\t\tNodeID %d detected as Broadcast - write data to every Node\r\n", nodeID);
				for (int i = 0; i < TLE9012_NODES_SIZE; i++) {
					setRegisterReady_tle9012(nodeID, regIdx, 0);
					setRegisterError_tle9012(nodeID, regIdx, 0);
				}
			}
			else {
				DEBUGPRINTF_ERROR("\t\t\t\tNodeID %d is 0 or exceeded size of NodeArray - Ignoring package\r\n", nodeID);
			}
		}
	}
	else {
		DEBUGPRINTF_ERROR("\t\t\t\tRegister index %d exceeded number of registers: Ignore message \r\n", regIdx);
	}
}


//****************************************************************************
// Takes the already verified frames of a package (starting with id frame) and
// extracts its components, interprets data, acts based on read or write, checks
// reply fame if needed, converts data, sets ready/error bit and stores result in
// node object. The package index is allowed to overflow the maximum buffer size.
// It will always be corrected because the relative index must be corrected anyway.
// Returns number of bytes that the index must be incremented or 0 if it failed.
//****************************************************************************
uint16_t interpretDataPackage(uint16_t package_index) {
	// Extract node ID and read/write info from "ID Frame"
	DEBUGPRINTF_INFO("\t\t\t\tID Frame at: index=%d+%d, byte=0x%02X\r\n", package_index, PACKAGE_ID_FRAME_IDX, getPackageByte(package_index, PACKAGE_ID_FRAME_IDX));
	uint8_t id_frame = getPackageByte(package_index, PACKAGE_ID_FRAME_IDX);
	uint8_t nodeID = id_frame & ID_FRAME_ID_MASK;
	uint8_t isWrite = id_frame & ID_FRAME_ISWRITE_MASK;

	// Extract Address from "Address frame"
	uint8_t regAdr = getPackageByte(package_index, PACKAGE_ADR_FRAME_IDX);
	uint8_t regIdx = getRegisterIndexFromAddress(regAdr);

	// Extract data from "Data frame"
	uint16_t data = ((uint16_t)getPackageByte(package_index, PACKAGE_DATA0_FRAME_IDX) << 8) | getPackageByte(package_index, PACKAGE_DATA1_FRAME_IDX);

	// If its a write data package check the reply frame, else store the data as read
	if (isWrite) {
		// WRITE PACKAGE: Check reply Frame
		uint8_t reply_frame = getPackageByte(package_index, PACKAGE_W_REPLY_FRAME_IDX);

		// If reply frame reports an error set error bit and return
		uint8_t isSuccessful = 0;
		if (reply_frame == 0) {
			// Store data to shadow register set ready bit and reset error bit
			DEBUGPRINTF_SUCCESS("\t\t\t\tStore write data: NodeID=%d, RegAdr=0x%02X, Data=0x%04X, Reply=0x%02X\r\n", nodeID, regAdr, data, reply_frame);

			// Mark successful - no error
			isSuccessful = 1;
		}
		else if(reply_frame & REPLY_FRAME_CRC_ERROR_MASK){
			// Print debug info about reply frame
			DEBUGPRINTF_ERROR("\t\t\t\tWrite command reply indicated - CRC checked register error:  NodeID=%d, RegAdr=0x%02X, ReplyByte=0x%02X (CRC3calc=0x%02X)\r\n", nodeID, regAdr, reply_frame, calcCRC3((uint16_t)(reply_frame>>3)));
			//dumpReceiveBuffer();

			// Mark not successful - CRC checked wrong
			isSuccessful = 0;
		}
		else if (reply_frame & REPLY_FRAME_REG_ADR_ERROR_MASK) {
			// Print debug info about reply frame
			DEBUGPRINTF_ERROR("\t\t\t\tWrite command reply indicated - Register address for write command invalid: NodeID=%d, RegAdr=0x%02X, ReplyByte=0x%02X (CRC3calc=0x%02X)\r\n", nodeID, regAdr, reply_frame, calcCRC3((uint16_t)(reply_frame>>3)));
			//dumpReceiveBuffer();

			// Mark not successful - wrong register address
			isSuccessful = 0;
		}
		else if (reply_frame & REPLY_FRAME_GENERAL_FAULT_MASK) {
			// Print debug info about reply frame
			//DEBUGPRINTF_ERROR("\t\t\t\tWrite command reply indicated - Fault in general diagnostics register. Will ignore error: NodeID=%d, RegAdr=0x%02X, ReplyByte=0x%02X (CRC3calc=0x%02X)\r\n", nodeID, regAdr, reply_frame, calcCRC3((uint16_t)(reply_frame>>3)));
			generalDiagnosticProblemDetected = 1;
			//dumpReceiveBuffer();

			// Mark successful - a replied problem in general diagnostic register is not recognized as error (based on information from datasheet - not sure if write command is ok in this case or not TODO)
			isSuccessful = 1;
		}
		else {
			// Print debug info about reply frame
			DEBUGPRINTF_ERROR("\t\t\t\tWrite command reply indicated unknown error: NodeID=%d, RegAdr=0x%02X, ReplyByte=0x%02X (CRC3calc=0x%02X)\r\n", nodeID, regAdr, reply_frame, calcCRC3((uint16_t)(reply_frame>>3)));
			//dumpReceiveBuffer();

			// Mark not successful - unknown error can not be distinguished
			isSuccessful = 0;
		}

		// Do not store the data if the reset of a diagnostic register is written
		if(regIdx == GEN_DIAG && isWrite == 1)
			isSuccessful = 0;

		// Write the actual data to register and set error/ready bit
		setRegistersStateAndData(nodeID, regIdx, data, isSuccessful);

		// Return size of write command (there are no write packages) to increase to interpreter index
		return SIZE_WRITE_COMMAND;
	}
	else {
		// READ PACKAGE:
		DEBUGPRINTF_SUCCESS("\t\t\t\tStore read data: NodeID=%d, RegAdr=0x%02X, Data=0x%04X\r\n", nodeID, regAdr, data);
		// Store data to shadow register set ready bit and reset error bit
		setRegistersStateAndData(nodeID, regIdx, data, 1);

		// Return size of read package to increase the interpreter index to next package
		return SIZE_READ_PACKAGE;
	}
}


//****************************************************************************
// Takes the package start index and the size of the data that shall be crc8
// checked and compares the result with the should-be CRC sum at the next byte.
// Returns true if crc matched and false if not
//****************************************************************************
static inline uint8_t checkDataPackageCRC(uint16_t index_crc_start, uint8_t crc_data_size) {
	DEBUGPRINTF_INFO("\t\t\tCalc CRC from index %d for %d bytes = 0x%02X. Compare to 0x%02X at index %d\r\n", index_crc_start, crc_data_size, calcPackageCRC8(index_crc_start, crc_data_size), getPackageByte(index_crc_start, crc_data_size), (index_crc_start + crc_data_size) % RX_RING_BUFFER_SIZE);
	return calcPackageCRC8(index_crc_start, crc_data_size) == getPackageByte(index_crc_start, crc_data_size);
}


//****************************************************************************
// Manages the Interpreter - Checks if whole data packages were receive to the
// ring buffer, interprets the packages. The interpreted data will be stored in
// the shadow registers and post-processed if needed.
// Additionally the ready bit is set after data is written successfully.
//****************************************************************************
void tle9012_manageInterpreter() {
	//uint64_t debugStartTime = TIMER_COUNTER_1MS;
	DEBUGPRINTF_INFO("\r\nStart manageInterpreter at interpreter_index=%d and receive_index=%d\r\n", interpreter_index, receive_index);

	// Check last state of the RX buffer
	if (rx_buffer_state != InterpreterSuccess) {
		DEBUGPRINTF("\tUART RX buffer overflow!\r\n");
	}

	// Get number of bytes in buffer
	uint16_t numBytesInBuffer = getInterpreterIndexDifference();
	//uint16_t initialNumBytesInBuffer = numBytesInBuffer;

	// Initial interpreter state
	interpreterStates iState;

	// Debug
	if (!(numBytesInBuffer >= SIZE_MINIMAL_RX_SIZE)) { DEBUGPRINTF_INFO("\r\n\tSKIPED interpreter loop - bytes in fifo MIN=%d, IS=%d ---\r\n\r\n", SIZE_MINIMAL_RX_SIZE, numBytesInBuffer); }

	// Interpret ring buffer as long as there is the minimum size of any message in buffer or a bigger incomplete message is found
	while (numBytesInBuffer >= SIZE_MINIMAL_RX_SIZE) {
		DEBUGPRINTF_INFO("\r\n\tSTART interpreter loop - bytes in fifo MIN=%d, IS=%d ---\r\n", SIZE_MINIMAL_RX_SIZE, numBytesInBuffer);

		// The interpreter_index points to the last processed byte in order to make parts of the code simpler and faster.
		// But during the interpreter run the actual index of the first new byte is needed. Therefore it is safely incremented here and used throughout the interpreter routine.
		uint16_t next_interpreter_index = getInterpreterAbsoluteIndex(interpreter_index, 1);

		// Reset parse state to tryParsing
		iState = I_PARSE_READ_RETURN;

		// TRY COMMAND PARSE: If "Synchronization Frame" is detected at first byte - New data package is assumed -> check if it is by checking crc sum
		if (getPackageByte(next_interpreter_index, 0) == FRAME_SYNC) {
			DEBUGPRINTF_INFO("\t\tAssumed SYNC frame at %d: Check CRC to check if this is a command\r\n", next_interpreter_index);
			// If read detected (isWrite bit is not set)
			if ((getPackageByte(next_interpreter_index, 1) & ID_FRAME_ISWRITE_MASK) == 0) {
				DEBUGPRINTF_INFO("\t\t\tRead command?\r\n");
				// If remaining bytes in buffer are below read command size stop this loop and try next time
				if (numBytesInBuffer < SIZE_READ_COMMAND) {
					DEBUGPRINTF_INCOMPLETE("\t\t\t\tMessage read com incomplete MIN=%d IS=%d (LAST=0x%02X)\r\n", SIZE_READ_COMMAND, numBytesInBuffer, getPackageByte(receive_index, 0));
					iState = I_PACKET_INCOMPLETE;
					// Stop loop and hope that next time the whole message is in buffer
					break;
				}
				// Else if CRC8 of this and next 2 bytes (sync, nodeID, regAdr) match with 4th byte (CRC8 byte)
				else if (checkDataPackageCRC(next_interpreter_index, 3)) {
					// Set interpreter index to after this command - the replies will be read in "Reply package is assumed"
					incrementInterpreterIndexBy(&interpreter_index, SIZE_READ_COMMAND);
					next_interpreter_index = getInterpreterAbsoluteIndex(interpreter_index, 1);
					DEBUGPRINTF_INFO("\t\t\t\tCRC matches = read command confirmed - moving interpreter index by %d to %d and try parse\r\n", SIZE_READ_COMMAND, interpreter_index);
					// State is still tryParsing - next section, try reply parse will become active
				}
				// Else: If the CRC is wrong there is either a invalid package or a data package with a device id similar to the sync frame.
				//       In this case the next section will try to parse it as data package because the state still is on I_PARSE_READ_RETURN.
				else {
					DEBUGPRINTF_CRC_FAIL("\t\t\t\tCRC does not match - must be a data package or a invalid command \r\n");
				}
			}
			// Else write detected (isWrite bit is set)
			else {
				DEBUGPRINTF_INFO("\t\t\tWrite command?\r\n");
				// If remaining bytes in buffer are below write command size stop this loop and try next time
				if (numBytesInBuffer < SIZE_WRITE_COMMAND) {
					DEBUGPRINTF_INCOMPLETE("\t\t\t\tMessage write com incomplete MIN=%d IS=%d (LAST=0x%02X)\r\n", SIZE_WRITE_COMMAND, numBytesInBuffer, getPackageByte(receive_index, 0));
					iState = I_PACKET_INCOMPLETE;
					// Stop loop and hope that next time the whole message is in buffer
					break;
				}
				// Else if CRC8 of this and next 4 bytes (sync, nodeID, regAdr, data1, data0) match with 6th byte (CRC8 byte)
				else if (checkDataPackageCRC(next_interpreter_index, 5)) {
					DEBUGPRINTF_INFO("\t\t\t\tCRC matches = write command confirmed - parse package and increment index\r\n");
					uint8_t package_size = interpretDataPackage(interpreter_index + 2);
					// Parse was successful, no further processing of this data needed - set parse state to: I_PARSE_SUCCESS
					if (package_size > 0) {
						iState = I_PARSE_SUCCESS;
						// Set interpreter index to after this command
						incrementInterpreterIndexBy(&interpreter_index, package_size);
						DEBUGPRINTF_SUCCESS("\t\t\t\tParse successful, moving interpreter index by %d to %d (0x%02X)\r\n", package_size, interpreter_index, getPackageByte(interpreter_index, 0));
					}
					else {
						DEBUGPRINTF_INFO("\t\t\t\tParse failed\r\n");
						iState = I_PARSE_FAILED;
					}
				}
				// Else: If the CRC is wrong this must be a invalid package.
				else {
					DEBUGPRINTF_CRC_FAIL("\t\t\t\tCRC does not match - must be a invalid command \r\n");
					// TODO: Test if code works if the state is set to failed parse here. Should skip tryParsing
				}
			}
		}


		// TRY REPLY PARSE: If - Reply package is assumed:    If Package parse state is not successParse yet and CRC8 of this and next 3 bytes (nodeID, regAdr, data0, data1) match with 5th byte (CRC8 byte)
		if (iState == I_PARSE_READ_RETURN) {
			DEBUGPRINTF_INFO("\t\tAssumed reply: Try to parse \r\n");
			// Refresh number of bytes in buffer
			numBytesInBuffer = getInterpreterIndexDifference();
			// If remaining bytes in buffer are below reply size stop this loop and try next time
			if (numBytesInBuffer < SIZE_READ_PACKAGE) {
				DEBUGPRINTF_INCOMPLETE("\t\t\tMessage incomplete MIN=%d IS=%d (LAST=0x%02X)\r\n", SIZE_READ_PACKAGE, numBytesInBuffer, getPackageByte(receive_index, 0));
				iState = I_PACKET_INCOMPLETE;
				// Stop loop and hope that next time the whole message is in buffer
				break;
			}
			// If CRC8 of this and next 3 bytes (nodeID, regAdr, data0, data1) match with 5th byte (CRC8 byte)
			else if (checkDataPackageCRC(next_interpreter_index, 4)) {
				DEBUGPRINTF_INFO("\t\t\tCRC matches - parse package and increment index \r\n");
				uint8_t package_size = interpretDataPackage(next_interpreter_index);
				// Parse was successful, no further processing of this data needed - set parse state to: I_PARSE_SUCCESS
				if (package_size > 0) {
					iState = I_PARSE_SUCCESS;
					// Set interpreter index to after this command
					incrementInterpreterIndexBy(&interpreter_index, package_size);
					DEBUGPRINTF_SUCCESS("\t\t\t\tParse successful, moving interpreter index by %d to %d (0x%02X)\r\n", package_size, interpreter_index, getPackageByte(interpreter_index, 0));
				}
				else {
					DEBUGPRINTF_INFO("\t\t\t\tParse failed\r\n");
					iState = I_PARSE_FAILED;
				}
			}
			// Else: If the CRC is still wrong the package is invalid. In this case the next section will discard all bytes until the sync frame is found.
			else {
				DEBUGPRINTF_CRC_FAIL("\t\t\tCRC does not match - must be invalid data\r\n");
				iState = I_PARSE_FAILED;
			}
		}

		// DISCARD UNPARSABLE BYTES: If parse not successful invalid data assumed and ignored
		if (iState == I_PARSE_FAILED) {
			//DEBUGPRINTF_INFO("\t\tDiscard invalid data until SYNC frame is found\r\n");
			//// Refresh number of bytes in buffer
			//numBytesInBuffer = getInterpreterIndexDifference();
			//// Increment interpreter index until "Synchronization Frame" is found or end of buffer is reached. This ignores all broken data
			//for(uint16_t i = 0; i <= numBytesInBuffer; i++){
			//	// If synchronization frame is found at any point except the first (could be initial SYNC frame), stop
			//	if(i > 0 && getPackageByte(interpreter_index+1, 0) == FRAME_SYNC){
			//		DEBUGPRINTF_INFO("\t\t\tFound SYNC frame\r\n");
			//		break; // Todo: Test if "continue;" works also!
			//	}
			//	// If end of recieved data is reached, stop
			//	if(interpreter_index == receive_index){
			//	    DEBUGPRINTF_INFO("\t\t\tDiscarded all bytes currently in buffer\r\n");
			//		break;
			//	}
			//	DEBUGPRINTF_ERROR("\t\t\tDiscard byte index %d: 0x%02X \r\n", interpreter_index+1, getPackageByte(interpreter_index+1, 0));
			//	// Increment interpreter index
			//	incrementInterpreterIndex(&interpreter_index);
			//}
			// Note: The above loop cannot handle additional bytes between read responses or crc errors in responses. However it would allow for faster processing
			// If end of received data is reached, stop
			if (interpreter_index == receive_index) { break; }
			DEBUGPRINTF_ERROR("\t\t\tDiscard byte index %d: 0x%02X \r\n", next_interpreter_index, getPackageByte(interpreter_index, 1));
			// Increment interpreter index
			incrementInterpreterIndex(&interpreter_index);
		}

		// Refresh number of bytes in buffer
		numBytesInBuffer = getInterpreterIndexDifference();

		// Check if data received during loop
		//if (numBytesInBuffer > initialNumBytesInBuffer)
		//	DEBUGPRINTF_ERROR("\tnumBytesInBuffer increased during interp, ISR RX?\r\n");
	}

	DEBUGPRINTF_INTERP_PERF("manageInterpreter took %ld ms\r\n", (uint32_t)(TIMER_COUNTER_1MS-debugStartTime));
}

