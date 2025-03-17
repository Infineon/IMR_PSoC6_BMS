/*
 * BMS_TLE9012.c
 *
 *	Library for TLE9012 Li-Ion battery monitoring and balancing system IC.
 *	Created by R. Santeler @ Infineon 2024
 *
 *  VERSION 1.00
 *
 *  USAGE (see "example.c_" for details):
 * 			- Implement target/uC specific functions in tle9012_target.h/.c
 * 			- Implement ISR for Interrupt and its behavior in main code
 * 				(see examples.c_ for hint on how to implement.
 * 				Note that the ERR interrupt pin must be configured before it is active)
 * 			- Configure number of expected nodes, etc
 * 				in tle9012_registers.h SETTINGS section.
 * 				NOTE: This library was developed on a single IC system.
 * 				Multi-node init and broadcast commands are implemented but untested.
 * 			- Init TL9012 with TLE9012_Init() command and use set...() commands and
 * 				tle9012_writeRegisterByIndex to configure it
 * 			- Use library commands to write (tle9012_writeRegisterByIndex()) and
 * 				read (tle9012_readRegisterByIndex()) register data via the queue.
 * 				Use checkRegisterQueued_tle9012(), checkRegisterReady_tle9012 and
 * 				checkRegisterError_tle9012 before requesting the register data by
 * 				getRegisterCacheDataByIndex_tle9012().
 * 			- Convert functions like tle9012_convertBlockVoltage or
 * 				tle9012_convertPrimaryCellVoltages write their result into the
 * 				tle9012_nodes[] array.
 * 				Here the index represents the node ID-1
 * 				(e.g.: NoteID 1 is on index 0, NodeID 2 is on 1, etc)
 * 			- Call tle9012_manageQueue(1) and tle9012_manageInterpreter() every cycle.
 * 				It handles the actual TLE9012 RX and TX and keeps watchdog running!
 * 			- The function parameter isBlocking  can be used to execute a function and
 * 				wait till the result from the tle9012 has been received
 * 				(all queue items done).
 * 			- The function parameter isBroadcast can be used to read/write from/to
 * 				all configured nodes.
 * 				NOTE: This library was developed on a single IC system.
 * 				Multi-node init and broadcast commands are implemented but untested.
 *
 *  NOTE:	Always use this library in combination with
 *  		the datasheet/user manual of the chip.
 *  		Due to the vast amount of functions and subsystems in the IC
 *  		the content of its documentation is essential to use this library properly.
 *  NOTE:	This library was developed on a single IC system.
 *  		Multi-node init and broadcast commands are implemented but untested.
 *  NOTE:	Never directly write to or read from TLE9012 UART.
 *  		Everything must be passed thru Queue/Interpreter system.
 *  NOTE:	Access to many functions of the IC are simplified with "high level" functions
 *  		in this file. However, due to the vast possibilities of the IC
 *  		some functions must be addressed by lower level functions,
 *  		found in registers.c/.h. E.g. use tle9012_writeRegisterByIndex and
 *  		tle9012_readRegisterByIndex
 *  NOTE:	The content of registers.c/.h consists of helpful lower level definitions and
 *  		helper functions needed for the higher and lower level features
 *  NOTE:	Muti-read configuration must be set with the function
 *  		tle9012_writeMultiReadConfig!
 *  		Ignoring this will break the queue/interpreter system!
 *  		See description of tle9012_writeMultiReadConfig()!
 *  NOTE:	The register GEN_DIAG must be used to get most errors.
 *  		Use tle9012_clearProblemDetected() and tle9012_clearStatusFlags(1, 0, 0)
 *  		to clear errors. In addition multiple bitmaps for cell errors are available.
 *  NOTE:	tle9012_problemDetected() must be used for some functions to check
 *  		if an action was successful (see description of function).
 *  		After the check clearProblemDetected() should be used to reset the flag
 *  		(also clearStatusFlags() resets this).
 *  		Also the general error bit in every write request is requested with this
 *  		(generalDiagnosticProblemDetected - see interpreter)
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h> // includes strtol
#include <math.h>

#include "BMS_TLE9012.h"
#include "tle9012_registers.h"
#include "tle9012_target.h"
#include "tle9012_data.h"
#include "tle9012_queue.h"
#include "tle9012_interpreter.h"

#define DEBUGPRINTF_READ_WRITE(p_frm, ...)	//DEBUGPRINTF(p_frm, ##__VA_ARGS__)

// Used to check if an action was successful and if failure flags are set
uint8_t tle9012_problemDetected = 0;

uint8_t generalDiagnosticProblemDetected = 0;

// Make indices available for debug
extern uint16_t receive_index;
extern uint16_t interpreter_index;
extern uint8_t queue_index_current;
extern uint8_t queue_index_latest;

/****************************************************************************
 * tle9012_initUART
 *
 * initialize the UART device. Must be done at start of main.
 * Returns 0 if successful
 ****************************************************************************/
uint8_t tle9012_initUART(void)
{
	return uartInit();
}

/****************************************************************************
 * tle9012_deviceInit
 * Initialize all TLE9012 devices.
 * Sends wakeup, assigns all IDs up until TLE9012_NODES_SIZE,
 * sets initial values and performs configuration updates
 * if device is reset or re-woken
 *
 * reinitializeConfig:
 * If this is set to 1 all configuration registers are written from
 * shadow cache back to the devices.
 * This is necessary e.g. if device went to sleep and lost its settings
 * (this way the configuration does not need to be re-performed)
 ****************************************************************************/
void tle9012_deviceInit(uint8_t reinitializeConfig)
{
	DEBUGPRINTF("TLE9012 Init:\r\n");

	// Reset queue and interpreter
	resetQueue();
	resetInterpreter();

	// Wakeup first device - will wake all
	tle9012_sendWakeup(1);

	// Store time of wake (equal to watchdog refresh)
	timeLastWatchdogRefresh = TIMER_COUNTER_1MS;

	// Assign all devices and reset settings
	for(uint8_t i = 1; i <= TLE9012_NODES_SIZE; i++){
		//DEBUGPRINTF("\tAssign ID to node %d\r\n", i);
		tle9012_assignID(i, IS_FINAL_NODE, ENABLE_ALL_ADC);

		// Fake reply frame/byte (seems like write to node 0 do not trigger reply)
		addToReceiveRingBuffer_tle9012(0);

		/// Apply init configuration
		// Set all ready bits to 1 and all error bits to 1
		tle9012_nodes[i-1].reg_items_ready_bitmap = UINT64_MAX;
		tle9012_nodes[i-1].reg_items_error_bitmap = UINT64_MAX;
		tle9012_nodes[i-1].reg_items_queued_bitmap = 0;

		// Write all configuration registers from shadow cache back to the devices if needed
		if(reinitializeConfig){
			tle9012_writeRegisterByIndex(i, PART_CONFIG,
					tle9012_nodes[i-1].reg_data[PART_CONFIG] , 0, 0);
			tle9012_writeRegisterByIndex(i, MEAS_CTRL,
					tle9012_nodes[i-1].reg_data[MEAS_CTRL]   , 0, 0);
			tle9012_writeRegisterByIndex(i, OP_MODE,
					tle9012_nodes[i-1].reg_data[OP_MODE]     , 0, 0);
			tle9012_writeRegisterByIndex(i, FAULT_MASK,
					tle9012_nodes[i-1].reg_data[FAULT_MASK]  , 0, 0);
			tle9012_writeRegisterByIndex(i, OL_OV_THR,
					tle9012_nodes[i-1].reg_data[OL_OV_THR]   , 0, 0);
			tle9012_writeRegisterByIndex(i, OL_UV_THR,
					tle9012_nodes[i-1].reg_data[OL_UV_THR]   , 0, 0);
			tle9012_writeRegisterByIndex(i, TEMP_CONF,
					tle9012_nodes[i-1].reg_data[TEMP_CONF]   , 0, 0);
			tle9012_writeRegisterByIndex(i, INT_OT_WARN_CONF,
					tle9012_nodes[i-1].reg_data[INT_OT_WARN_CONF], 0, 0);
			tle9012_writeRegisterByIndex(i, BAL_PWM,
					tle9012_nodes[i-1].reg_data[BAL_PWM]     , 0, 0);
			tle9012_writeRegisterByIndex(i, BAL_CURR_THR,
					tle9012_nodes[i-1].reg_data[BAL_CURR_THR], 0, 0);
			tle9012_writeRegisterByIndex(i, BAL_SETTINGS,
					tle9012_nodes[i-1].reg_data[BAL_SETTINGS], 0, 0);
			tle9012_writeRegisterByIndex(i, RR_CONFIG,
					tle9012_nodes[i-1].reg_data[RR_CONFIG]   , 0, 0);
			tle9012_writeRegisterByIndex(i, MULTI_READ_CFG,
					tle9012_nodes[i-1].reg_data[MULTI_READ_CFG], 0, 0);

			DEBUGPRINTF("\t Reinit TLE9012 config registers:\r\n");
		}
	}

	// Make sure queue and interpreter are finished
	do{
		tle9012_manageInterpreter();
	}while(tle9012_manageQueue(1) != QUEUE_FINISHED);
}

/****************************************************************************
 * tle9012_sendWakeup
 * Sends the wakeup pattern to the ICs (not ID specific)
 *
 * nodeID 	-> 	The ID of the node that shall be changed.
 * 				Range 1 to TLE9012_NODES_SIZE
 ****************************************************************************/
void tle9012_sendWakeup(uint8_t nodeID)
{
    if (isEnabled()) {
    	// Compose write frame and send data
		tle9012_tx_buffer[0] = FRAME_WAKE;
		tle9012_tx_buffer[1] = FRAME_WAKE;
		tle9012_tx_buffer_size = 2;
		uartWrite(tle9012_tx_buffer_size, 1);
		tle9012_nodes[nodeID-1].timeLastCommandSent = TIMER_COUNTER_1MS;
		DEBUGPRINTF("TLE9012 WakeUp: NodeID %d\r\n", nodeID);

		timeLastWatchdogRefresh = TIMER_COUNTER_1MS;

		// Wait wakeup is done
		DELAY_MS(10); // TODO: check if this is necessary
    }
    else {
        tle9012_problemDetected = 1;
        DEBUGPRINTF("\r\n[ERROR]: sendWakeup - Device is disabled\r\n");
    }
}

/*********************************************************************************
 * tle9012_assignID
 * After first wakeup every TLE9012 node in chain
 * must be assigned a ID starting from 1
 * The last node must be assigned with the FinalNode flag because
 * only it creates the reply frame.
 * Without the FinalNode set right there will never be any answer of any tle9012!
 *
 * nodeID		 -> The ID that shall be assigned. Range 1 to TLE9012_NODES_SIZE
 * isFinalNode   -> If 1 this will be marked as the last node
 * enableAllADC  -> If 1 PCVM is done for each channel,
 * 					independent of PART_CONFIG setup (ignore PART_CONFIG)
 *********************************************************************************/
void tle9012_assignID(uint8_t nodeID, uint8_t isFinalNode, uint8_t enableAllADC)
{
	DEBUGPRINTF("TLE9012 AssignID: NodeID %d\r\n", nodeID);

	// Reset node ID of object(must be 0 to address the next non assigned node)
	tle9012_nodes[nodeID-1].ID = 0;

	// Set ID in data
	uint16_t data = nodeID;
	//DEBUGPRINTF("\r\n");

	// If this is the final node add flag
	if(isFinalNode == 1){
		DEBUGPRINTF("\tAssign final node to ID %d\r\n", nodeID);
		data |= FN__FN ;
	}

	// Enable ADCs
	if(enableAllADC == 1){
		DEBUGPRINTF("\tEnable all ADCs on node ID %d\r\n", nodeID);
		data |= EN_ALL_ADC__ALL_ADC;
	}

	// Send assign message out
	sendWrite(&tle9012_nodes[nodeID-1], reg_items[CONFIG_MAIN].address, data, 0);

	DELAY_MS(10); // TODO: check if this is necessary

	// Store ID
	tle9012_nodes[nodeID-1].ID = nodeID;
}

/****************************************************************************
 * blockTillQueueFinished
 * This will block until the queue is finished and a register index is ready.
 * Will execute manageQueue and manageInterpreter continuously until
 * conditions are met or timeout happened.
 *
 * nodeID	-> 	The ID of the node that shall be changed.
 * 				Range 1 to TLE9012_NODES_SIZE
 * reg_idx	-> 	The index of the register that shall be checked
 * 				see register.h definitions.  E.g. PART_CONFIG etc
 *
 * Returns the requestResult:
 * REQUEST_OK, REQUEST_ERROR,
 * REQUEST_DONE_BUT_REGISTER_NOT_READY or REQUEST_TIMEOUT
 ****************************************************************************/
static inline requestResult blockTillQueueFinished(uint8_t nodeID, uint8_t reg_idx){
	QueueResult result;

	// Try to wait for max TIMEOUT_COMMAND + safety and
	// manage queue and interpreter until the queue is finished
	uint32_t startTime = TIMER_COUNTER_1MS;
	for(uint32_t i = 0; i<UINT32_MAX; i++){
		// Execute management of queue and interpreter
		result = tle9012_manageQueue(1);
		tle9012_manageInterpreter();
		// If queue is finished with all items the request should be ready
		if(result == QUEUE_FINISHED){
			DEBUGPRINTF_READ_WRITE("Queue finished after %ld loops and %ld ms\r\n",
					i, (uint32_t)(TIMER_COUNTER_1MS - startTime));
			// Check if result is not ready, has an error or else is OK
			if(checkRegisterReady_tle9012(nodeID, reg_idx, 0) == 0){
				DEBUGPRINTF("[ERROR]: Queue finished, "
						"but register not ready on Node %d with reg_idx %d\r\n",
						nodeID, reg_idx);
				return REQUEST_DONE_BUT_REGISTER_NOT_READY;
			}
			else if(checkRegisterError_tle9012(nodeID, reg_idx, 0) == 1){
				DEBUGPRINTF("[ERROR]: Queue finished, "
						"but register has error on Node %d with reg_idx %d\r\n",
						nodeID, reg_idx);
				return REQUEST_ERROR;
			}
			else
				return REQUEST_OK;
			//break;
		}
		else if (result == QUEUE_TIMEOUT){
			DEBUGPRINTF("[ERROR]: Request queue timeout on Node %d "
					"with reg_idx %d\r\n", nodeID, reg_idx);
			return REQUEST_TIMEOUT;
		}


		// If blocking takes to long stop execution
		if(TIMER_COUNTER_1MS > (startTime + TIMEOUT_COMMAND+50)){
			DEBUGPRINTF("[ERROR]: Request blocking timeout "
					"after %ld ms (%ld loops) on Node %d with reg_idx %d "
					"(receive_idx %d, interp_idx %d, queue_idx_cur %d, "
					"queue_idx_last %d)\r\n",
					(uint32_t)(TIMER_COUNTER_1MS - startTime),
					i, nodeID, reg_idx, receive_index, interpreter_index,
					queue_index_current, queue_index_latest);
			//return REQUEST_TIMEOUT;
		}
		// Block 1ms at a time
		//DELAY_MS(1);
	}

	// Should never get here
	DEBUGPRINTF("[ERROR]: blockTillQueueFinished reached end - this should never be\r\n");
	return REQUEST_ERROR;
}

/****************************************************************************
 * tle9012_writeRegisterByIndex
 * Set all 16 bits of a register on a node directly.
 * Register cache will ultimately be updated by the interpreter
 * if a positive reply of the node is received.
 *
 * nodeID		-> 	The ID of the node that shall be changed.
 * 					Range 1 to TLE9012_NODES_SIZE
 * reg_idx		-> 	The index of the register that shall be set
 * 					see register.h definitions.  E.g. PART_CONFIG etc
 * data     	-> 	The full 16 bit data of the register.
 * 					Previous state will be overwritten completely
 * 					Note: Always check registers.h to find
 * 					the right setting and use the defined values
 * isBlocking	-> 	Wait until the result is ready or error,
 * 					otherwise the command will only be queued.
 * 					This will block the CPU, but if this returns REQUEST_OK
 * 					the result can directly be used
 * isBroadcast  -> 	If set to 1 this will effect all nodes
 *
 * Note: NEVER WRITE DIRECTLY TO ONE OF THE FOLLOWING REGISTER INDECES
 * 		 MULTI_READ 	- Does not allow direct write
 * 		 MULTI_READ_CFG	- Must be written with tle9012_writeMultiReadConfig()
 * 		 				  Direct write and subsequent read of
 * 		 				  MULTI_READ will result in catastrophic bugs
 * 		 				  (e.g. simultaneous UART TX and RX, overflows, etc)
 * 		 				  -> see tle9012_writeMultiReadConfig() description
 *
 * Note: With this you cannot queue a write and a read of the same register
 * at the same time. Use a state machine to switch between write and read
 * when checkRegisterQueued_tle9012 returns 0;
 *
 * Returns the requestResult:
 * If the result is REQUEST_OK the command can be considered successful.
 * Else it can neither be considered successful nor unsuccessful
 * (reply could be corrupted).
 ****************************************************************************/
requestResult tle9012_writeRegisterByIndex(uint8_t nodeID, uint8_t reg_idx,
		uint16_t data, uint8 isBlocking, uint8_t isBroadcast){
	// Debug
	/*if(reg_idx != GEN_DIAG){
		DEBUGPRINTF_READ_WRITE("Write node %d register index %d with data 0x%04X, "
				"blocking %d and isBroadcast %d ready %d\r\n",
				nodeID, reg_idx, data, isBlocking, isBroadcast,
				checkRegisterReady_tle9012(nodeID, reg_idx, isBroadcast));
	}*/

	// Check if a write request is already in queue
	// (it is when ready bit of register is 0 - not ready)
	if(checkRegisterReady_tle9012(nodeID, reg_idx, isBroadcast) == 0){
		return REQUEST_REGISTER_NOT_READY;
	}
	else if(checkRegisterQueued_tle9012(nodeID, reg_idx, isBroadcast) == 1){
		return REQUEST_REGISTER_ALREADY_QUEUED;
	}
	else{
		QueueResult queueResult;

		// Add request to queue
		queueResult = addToQueue_tle9012(nodeID, 1, reg_idx, data, isBroadcast);
		if(queueResult != QUEUE_SUCCESS){
			return REQUEST_ADD_FAILED;
		}

		// If blocking was requested, wait till the result is finished
		if(isBlocking == 1){
			return blockTillQueueFinished(nodeID, reg_idx);
		}
		else{
			return REQUEST_ADDED;
		}
	}

	// The function can only get here if blocking was 1 and
	// the queue request did not finish in time
	return REQUEST_TIMEOUT;
}

/****************************************************************************
 * tle9012_readRegisterByIndex
 * Read all 16 bits of a register on a node directly.
 * Register cache will ultimately be updated by the interpreter
 * if a positive reply of the node is received.
 *
 * nodeID		-> 	The ID of the node that shall be changed.
 * 					Range 1 to TLE9012_NODES_SIZE
 * reg_idx		-> 	The index of the register that shall be set
 * 					see register.h definitions.  E.g. PART_CONFIG etc
 * isBlocking	-> 	Wait until the result is ready or error,
 * 					otherwise the command will only be queued.
 * 					This will block the CPU, but if this returns REQUEST_OK
 * 					the result can directly be used
 * isBroadcast  -> 	If set to 1 this will effect all nodes
 *
 * Note: Register index MULTI_READ will be treated differently
 * but can be used the same way as all others...
 * BUT: Using MULTI_READ without proper prior use of
 * tle9012_writeMultiReadConfig() will result in catastrophic bugs
 * (e.g. simultaneous UART TX and RX, overflows, etc) ->
 * see tle9012_writeMultiReadConfig() description
 *
 * Note: With this you cannot queue a write and a read
 * of the same register at the same time.
 * Use a state machine to switch between write and read
 * when checkRegisterQueued_tle9012 returns 0;
 *
 * Returns the requestResult.
 * If the result is REQUEST_OK the command can be considered successful.
 * Else it can neither be considered successful nor not successful
 * (reply could be corrupted).
 ****************************************************************************/
requestResult tle9012_readRegisterByIndex(uint8_t nodeID, uint8_t reg_idx,
		uint8 isBlocking, uint8_t isBroadcast){
	// Debug
	if(reg_idx != GEN_DIAG){
		DEBUGPRINTF_READ_WRITE("Read node %d register index %d "
				"blocking %d and isBroadcast %d\r\n", nodeID, reg_idx,
				isBlocking, isBroadcast);
	}
	// Check if a read request is already in queue
	// (it is when ready bit of register is 0 - not ready)
	if(checkRegisterReady_tle9012(nodeID, reg_idx, isBroadcast) == 0){
		return REQUEST_REGISTER_NOT_READY;
	}
	else if(checkRegisterQueued_tle9012(nodeID, reg_idx, isBroadcast) == 1){
		return REQUEST_REGISTER_ALREADY_QUEUED;
	}
	else{
		QueueResult queueResult;

		// Add request to queue
		queueResult = addToQueue_tle9012(nodeID, 0, reg_idx, 0, isBroadcast);
		if(queueResult != QUEUE_SUCCESS){
			return REQUEST_ADD_FAILED;
		}

		// If blocking was requested, wait till the result is finished
		if(isBlocking == 1){
			return blockTillQueueFinished(nodeID, reg_idx);
		}
		else{
			return REQUEST_ADDED;
		}
	}

	// The function can only get here if blocking was 1 and
	// the queue request did not finish in time
	return REQUEST_TIMEOUT;
}

/****************************************************************************
 * tle9012_writeMultiReadConfig
 * Takes the select properties of MULTI_READ_CFG,
 * stores which registers will be requested and send the write command.
 *
 * NOTE: NEVER CHANGE MULTI_READ_CFG WITHOUT THIS FUNCTION!
 * WHEN MULTI_READ IS QUEUED IT WILL WAT FOR EXACTLY WHATS BEEN SET HERE
 * (too less -> waits for something that will never come,
 * too much will receive more while already sending next = collision)
 *
 * nodeID		-> 	The ID of the node that shall be changed.
 * 					Range 1 to TLE9012_NODES_SIZE
 * ...__sel		-> 	Use the MULT_READ_CFG macros in register.h
 * 					like: PCVM_SEL__RES_CELL_11_0, BVM_SEL__RESULT,
 * 					EXT_TEMP_SEL__RES_TMP0_4, EXT_TEMP_R_SEL__R_DIAG_RES,
 * 					INT_TEMP_SEL__INT_TMP_RES, SCVM_SEL__RESULT,
 * 					STRESS_PCVM_SEL__RESULT
 * 					(triggers all possible multi read registers).
 * 					Use intellisense on parameter name
 *                 	Note:	pcvm_sel = 0-> No PCVM read,
 *                 			PCVM_SEL__RES_CELL_11->Only cell 11 PCVM read,
 *                 			PCVM_SEL__RES_CELL_11_0->All cell PCVM read,
 *                 			4->Cell 11-8 PCVM read
 *                 			(not all combinations are mentioned in data sheet,
 *                 			in between are possible)
 * isBlocking	-> 	Wait until the result is ready or error,
 * 					otherwise the command will only be queued.
 * 					This will block the CPU, but if this returns REQUEST_OK
 * 					the result can directly be used
 *
 * Returns the requestResult from tle9012_writeRegisterByIndex
 ****************************************************************************/
requestResult tle9012_writeMultiReadConfig(uint8_t nodeID, uint16_t pcvm_sel,
		uint16_t bvm_sel, uint16_t ext_temp_sel, uint16_t ext_temp_r_sel,
		uint16_t int_temp_sel, uint16_t scvm_sel,
		uint16_t stress_pcvm_sel, uint8 isBlocking){
	uint8_t num_list_items = 0;

	// For each register that will be read store its reg index inside the list
	// Store each cell that will be read in PCVM as bit
    for(uint8_t i = 0; i<pcvm_sel; i++){
    	multi_read_cfg_reg_idx_list[num_list_items] = PCVM_11-i;
    	num_list_items++;
	}
    if(bvm_sel != 0){
    	multi_read_cfg_reg_idx_list[num_list_items] = BVM;
    	num_list_items++;
    }
    uint8_t num_temp = ext_temp_sel >> 5;
    // Store each temp sensor that will be read in as bit
    for(uint8_t i = 0; i<num_temp; i++){
    	multi_read_cfg_reg_idx_list[num_list_items] = EXT_TEMP_0+i;
    	num_list_items++;
	}
    if(ext_temp_r_sel != 0){
    	multi_read_cfg_reg_idx_list[num_list_items] = EXT_TEMP_R_DIAG;
    	num_list_items++;
    }
    if(int_temp_sel != 0){
    	multi_read_cfg_reg_idx_list[num_list_items] = INT_TEMP;
    	num_list_items++;
    }
    if(scvm_sel != 0){
    	multi_read_cfg_reg_idx_list[num_list_items] = SCVM_HIGH;
    	num_list_items++;
    	multi_read_cfg_reg_idx_list[num_list_items] = SCVM_LOW;
    	num_list_items++;
    }
    if(stress_pcvm_sel != 0){
    	multi_read_cfg_reg_idx_list[num_list_items] = STRESS_PCVM;
    	num_list_items++;
    }

    // Store number of elements in list
    multi_read_cfg_reg_idx_list_num = num_list_items;

    // Print debug
	//for(uint8_t i = 0; i < multi_read_cfg_reg_idx_list_num; i++)
	//	DEBUGPRINTF("MultiRead %d: Index %d\r\n", i, multi_read_cfg_reg_idx_list[i]);

    // Write multi read config
    return tle9012_writeRegisterByIndex(nodeID, MULTI_READ_CFG,
    		pcvm_sel | bvm_sel | ext_temp_sel | ext_temp_r_sel |
			int_temp_sel | scvm_sel | stress_pcvm_sel, isBlocking, 1);
}

/****************************************************************************
 * tle9012_setConfig
 * Set one certain config in a register.
 * Usually it is better to use tle9012_writeRegisterByIndex() and
 * write the whole register as needed.
 *
 * Note: The register must be read before this is used!
 * 			This function will use the last read result in shadow register
 *			for the rest of the settings.
 *		 Always check registers.h to find the right subreg and
 *		 	setting and use the defined value.
 *		 Some settings can be found on multiple positions of the register
 *		 	(e.g. bitmaps for cell errors or balancing).
 *		 	These must be constructed manually (e.g. 0b0000000000100000)
 *
 * Example: requestResult res =
 * 			tle9012_setConfig(1, RR_CONFIG__M_NR_ERR_BAL_OC,
 * 							  M_NR_ERR_BAL_OC__DISABLE , 1, 0);
 *
 * nodeID		-> 	The ID of the node that shall be changed.
 * 					Range 1 to TLE9012_NODES_SIZE
 * subreg       -> 	The subreg_item of the setting that shall be changed
 * 					e.g. PART_CONFIG__EN_CELLi
 * setting      -> 	The setting that shall be set to above subregister
 * 					e.g. EN_CELLi__NO_CELL_ATTACHED
 * 					Note: Always check registers.h to find
 * 					the right setting and use the defined value
 * isBlocking	-> 	Wait until the result is ready or error,
 * 					otherwise the command will only be queued.
 * 					This will block the CPU, but if this returns REQUEST_OK
 * 					the result can directly be used
 * isBroadcast  -> 	If set to 1 this will effect all nodes
 ****************************************************************************/
requestResult tle9012_setConfig(uint8_t nodeID, subreg_item subreg,
		uint16_t setting, uint8_t isBlocking, uint8_t isBroadcast)
{
    uint16_t reg_data;

	// Check if setting bits are in mask
	if (((subreg.mask | setting) & ~subreg.mask) == 0)
	{
		// Get current state of register
		reg_data = getRegisterCacheDataByIndex_tle9012(nodeID, subreg.reg_index);
		// Clear subregister bits
		reg_data &= ~subreg.mask;
		// Set setting bits
		reg_data |= setting;

		// Write whole register
		requestResult res =
				tle9012_writeRegisterByIndex(nodeID, subreg.reg_index,
						reg_data, isBlocking, isBroadcast);

		// Return result of register write
		return res;
	}

	// Else return error
	DEBUGPRINTF("[ERROR]: Tried to set bits out of "
			"subregister mask (wrong setting for subreg)\r\n");
	return REQUEST_ERROR;
}

/****************************************************************************
 * tle9012_checkConfig
 * Check if one certain config is set in a register.
 *
 * Note: The register must be read before this is used!
 * 		 	This function will use the last read result in shadow register
 * 			for the compare.
 *		 Always check registers.h to find the right subreg and
 *		 	setting and use the defined value.
 *		 Some settings can be found on multiple positions of the register
 *		 	(e.g. bitmaps for cell errors or balancing).
 *		 	These must be constructed manually (e.g. 0b0000000000100000)
 *
 * Example: uint8_t res = tle9012_checkConfig(1, RR_CONFIG__M_NR_ERR_BAL_OC,
 * 											  M_NR_ERR_BAL_OC__ENABLE, 1, 0);
 *
 * nodeID		-> 	The ID of the node that shall be checked.
 * 					Range 1 to TLE9012_NODES_SIZE
 * subreg       -> 	The subreg_item of the setting that shall be changed
 * 					e.g. PART_CONFIG__EN_CELLi
 * setting      -> 	The setting that shall be set to above subregister
 * 					e.g. EN_CELLi__NO_CELL_ATTACHED
 * 					Note: Always check registers.h to find
 * 					the right setting and use the defined value
 * isBlocking	-> 	Wait until the result is ready or error,
 * 					otherwise the command will only be queued.
 * 					This will block the CPU, but if this returns REQUEST_OK
 * 					the result can directly be used
 * isBroadcast  -> 	If set to 1 this will effect all nodes
 *
 * Return 0 if setting is not active,
 * 1 if setting is active and 2 if an error occurred
 ****************************************************************************/
uint8_t tle9012_checkConfig(uint8_t nodeID, subreg_item subreg,
		uint16_t setting, uint8_t isBlocking, uint8_t isBroadcast)
{
    uint16_t reg_data;

	// Check if setting bits are in mask
	if (((subreg.mask | setting) & ~subreg.mask) == 0)
	{
		// Get current state of register
		reg_data = getRegisterCacheDataByIndex_tle9012(nodeID, subreg.reg_index);
		// Get only bits in mask
		reg_data &= subreg.mask;
		// If the remaining bits are the same as the wanted setting bits
		if(reg_data == setting){
			return 1;
		}
		else{
			return 0;
		}

	}
	// Else return error
	DEBUGPRINTF("[ERROR]: Tried to check bits out of "
			"subregister mask (wrong setting for subreg)\r\n");
	return 2;
}

/****************************************************************************
 * tle9012_clearStatusFlags
 * Try to reset errors and check if they return
 *
 * nodeID		-> 	The ID of the node that shall be changed.
 * 					Range 1 to TLE9012_NODES_SIZE
 * isBlocking	-> 	Wait until the result is ready or error,
 * 					otherwise the command will only be queued.
 * 					This will block the CPU, but if this returns REQUEST_OK
 * 					the result can directly be used
 * isBroadcast  -> 	If set to 1 this will effect all nodes
 ****************************************************************************/
requestResult tle9012_clearStatusFlags(uint8_t nodeID,
		uint8_t isBlocking, uint8_t isBroadcast)
{
	return tle9012_writeRegisterByIndex(1, GEN_DIAG, 0x0000,
			isBlocking, isBroadcast);
}

/****************************************************************************
 * tle9012_readAllRegistersToCache
 * If multiple registers shall be read/written use this beforehand
 * The register data is stored in tle9012_node.reg_data[reg_index]
 *
 * nodeID		-> 	The ID of the node that shall be changed.
 * 					Range 1 to TLE9012_NODES_SIZE
 * isBlocking	-> 	Wait until the result is ready or error,
 * 					otherwise the command will only be queued.
 * 					This will block the CPU, but if this returns REQUEST_OK
 * 					the result can directly be used
 * isBroadcast  -> 	If set to 1 this will effect all nodes
 * 					NOTE: This might overflow queue or ring buffer
 * 					for multiple nodes! Must be extend as needed
 *
 * Returns REQUEST_OK if all registers where read/queued successful.
 * If errors occur only the last error is returned.
 ****************************************************************************/
requestResult tle9012_readAllRegistersToCache(uint8_t nodeID,
		uint8_t isBlocking, uint8_t isBroadcast){  // TODO does not work in Broadcast
	requestResult result = REQUEST_OK;
	requestResult tempResult;
	// Read all registers except the last one non-blocking
	for(uint8_t i = 0; i < NUM_REGISTERS-1; i++){
		if(i != MULTI_READ){
			tempResult = tle9012_readRegisterByIndex(nodeID, i,
					isBlocking, isBroadcast);
			if(tempResult != REQUEST_OK)
				result = tempResult;
		}
	}
	// Read the last one blocking if requested
	tempResult = tle9012_readRegisterByIndex(nodeID, NUM_REGISTERS-1,
			isBlocking, isBroadcast);
	if(tempResult != REQUEST_OK)
		result = tempResult;

	// Return result
	return result;
}

/****************************************************************************
 * tle9012_readPrimaryCellVoltages
 * Read and convert all of the Primary Cell Voltage channels and
 * store results in tle9012_node item.
 *
 * Note: 	The PCVM_BIT_DIVIDER must be adjusted if
 * 				the default precision of 16bit changed.
 * 			The channel must be activated and the measurement
 * 				must be triggered before it can be read.
 *
 * node				-> 	The tle9012_node object representing
 * 						one TLE9012 instance
 * ignoreCellMap	-> 	Each bit (from 0 LSB to 11 MSB) of this mask
 * 						represents a cell. If set to 1 the result
 * 						will not be converted for this cell.
 * toCellIdx  		-> 	The highest cell index that shall be read
 * 						(e.g. 2 reads cell 0,1 and 2).
 * 						Maximum is 11 - equals all 12 cells.
 * 						NO LOGIC CHECK IN PLACE
 * 						HIGHER NUMBER WILL RESULT IN OVERFLOW
 * checkIfQueued	-> 	Will not perform the action if
 * 						the used PCVM_x register is currently queued
 * 						to be read
 *
 * Result is stored in tle9012_node.cell_voltages[cellsIdx]
 ****************************************************************************/
requestResult tle9012_convertPrimaryCellVoltages(uint8_t nodeID,
		uint16_t ignoreCellMap, uint8_t toCellIdx, uint8_t checkIfQueued){
	requestResult res = REQUEST_OK;

	// Check every cell
	uint16_t compareBit = 1;
	for(uint16_t i = 0; i <= toCellIdx; i++){
		// If current cell is not marked to be ignored...
		if((ignoreCellMap & (compareBit << (i))) == 0){
			// ... read cell data
			tle9012_convertPrimaryCellVoltageByIndex(nodeID, i, checkIfQueued);
			if(res != REQUEST_OK){
				return res;
			}
			//printf("Update %d \r\n", i);
		}
		else{
			//printf("I %d\r\n", i);
		}
	}

	return res;
}

/****************************************************************************
 * tle9012_readPrimaryCellVoltageByIndex
 * Read and convert one of the Primary Cell Voltage channels and
 * store result in tle9012_node item
 *
 * Note: 	The PCVM_BIT_DIVIDER must be adjusted if
 * 				the default precision of 16bit changed.
 * 			The channel must be activated and the measurement must be
 * 				triggered before it can be read.
 *
 * node				-> 	The tle9012_node object representing
 * 						one TLE9012 instance
 * cellsIdx			-> 	Index of the cell that shall be read.
 * 						Maximum is 11 - equals all 12 cells.
 * 						NO LOGIC CHECK IN PLACE
 * 						HIGHER NUMBER WILL RESULT IN OVERFLOW
 * checkIfQueued	-> 	Will not perform the action if
 * 						the used PCVM_x register is currently queued
 * 						to be read
 *
 * Result is stored in tle9012_node.cell_voltages[cellsIdx]
 ****************************************************************************/
requestResult tle9012_convertPrimaryCellVoltageByIndex(uint8_t nodeID,
		uint8_t cellsIdx, uint8_t checkIfQueued){
	// If register is not ready or on error state return error
	if(checkRegisterReady_tle9012(nodeID, PCVM_0 + cellsIdx, 0) != 1)
		return REQUEST_REGISTER_NOT_READY;
	else if(checkIfQueued == 1 &&
			checkRegisterQueued_tle9012(nodeID, PCVM_0 + cellsIdx, 0) == 1)
		return REQUEST_REGISTER_ALREADY_QUEUED;
	else if(checkRegisterError_tle9012(nodeID, PCVM_0 + cellsIdx, 0) != 0)
		return REQUEST_ERROR;

	// Read register
	uint16_t cellVoltageRaw = 0;

	// Get result
	cellVoltageRaw = getRegisterCacheDataByIndex_tle9012(nodeID, PCVM_0 + cellsIdx);

	// Convert register data
	tle9012_nodes[nodeID-1].cell_voltages[cellsIdx] =
			(int16_t)(((PCVM_FSR/PCVM_BIT_DIVIDER) * (double)cellVoltageRaw)*1000.0);

	// Return ok
	return REQUEST_OK;
}

/****************************************************************************
 * tle9012_convertBlockVoltage
 * Read and convert one of the Block Voltage BVM and
 * store result in tle9012_node item
 *
 * Note: 	BVM_BIT_DIVIDER must be adjusted if
 * 			the default precision of 16bit changed.
 *
 * node				-> 	The tle9012_node object representing
 * 						one TLE9012 instance
 * checkIfQueued	-> 	Will not perform the action if
 * 						the used BVM register is currently queued
 * 						to be read
 *
 * Result is stored in tle9012_node.block_voltage
 ****************************************************************************/
requestResult tle9012_convertBlockVoltage(uint8_t nodeID, uint8_t checkIfQueued){
	// If register is not ready or on error state return error
	if(checkRegisterReady_tle9012(nodeID, BVM, 0) != 1)
		return REQUEST_REGISTER_NOT_READY;
	else if(checkIfQueued == 1 && checkRegisterQueued_tle9012(nodeID, BVM, 0) == 1)
		return REQUEST_REGISTER_ALREADY_QUEUED;
	else if(checkRegisterError_tle9012(nodeID, BVM, 0) != 0)
		return REQUEST_ERROR;

	// Read register
	uint16_t blockVoltageRaw = 0;

	// Get result
	blockVoltageRaw = getRegisterCacheDataByIndex_tle9012(nodeID, BVM);

	// Convert register data
	tle9012_nodes[nodeID-1].block_voltage =
			(BVM_FSR/BVM_BIT_DIVIDER) * (double)blockVoltageRaw;

	/*DEBUGPRINTF("tle9012_convertBlockVoltage: %.2f - R%d Q%d E%d\r\n",
			tle9012_nodes[nodeID-1].block_voltage,
			checkRegisterReady_tle9012(nodeID, BVM, 0),
			checkRegisterQueued_tle9012(nodeID, BVM, 0),
			checkRegisterError_tle9012(nodeID, BVM, 0));*/

	// Return ok
	return REQUEST_OK;
}

/****************************************************************************
 * tle9012_convertTemperatures
 * Read and convert all of the temperature sensor channels (0-3) and
 * store result in tle9012_node item
 *
 * Note: TEMP_R, TEMP_INTC and TEMP_NOMINATOR must be adjusted if the
 * 		 	default precision of 16bit changed or a different sensor is used.
 * 		 The channel must be activated and the measurement
 * 		 	must be triggered before it can be read
 *
 * node			-> 	The tle9012_node object representing one TLE9012 instance
 * toSensIdx	-> 	The highest sensors index that shall be read
 * 					(e.g. 2 reads sensor 0,1 and 2)
 * checkIfQueued->	Will not perform the action if the used
 * 					EXT_TEMP_x register is currently queued to be read
 *
 * Result is stored in tle9012_node.temperatures[cellsIdx]
 ****************************************************************************/
requestResult tle9012_convertTemperatures(uint8_t nodeID, uint8_t toSensIdx,
		uint8_t checkIfQueued){
	requestResult res = REQUEST_OK;
	requestResult returnRes = REQUEST_OK;

	// Check every sensor till up to toSensIdx
	for(uint8_t i = 0; i <= toSensIdx; i++){
		// Read cell data
		res = tle9012_convertTemperatureByIndex(nodeID, i, checkIfQueued);
		//printf("Update ETMP %d, %d\r\n", i, res); //checkRegisterReady_tle9012(nodeID, EXT_TEMP_0 + i, 0)
		// If result not OK and not marked as "old" (no new result)
		// latch error till the end. This will only report the last error
		if(res != REQUEST_OK && res != REQUEST_REGISTER_DATA_INVALID_OR_OLD)
			res = returnRes;
	}
	return returnRes;
}

/****************************************************************************
 * tle9012_convertTemperatureByIndex
 * Read and convert one of the temperature sensor channels (0-3) and
 * store result in tle9012_node item
 *
 * Note: TEMP_R, TEMP_INTC and TEMP_NOMINATOR must be adjusted if the
 * 			default precision of 16bit changed or a different sensor is used.
 * 		 The channel must be activated and the measurement must be
 * 		 	triggered before it can be read
 * 		 If the VALID flag of the temperature sensor is not set
 * 		 	the error bit in the error bitmap is set
 *
 * node			-> The tle9012_node object representing one TLE9012 instance
 * channelIdx	-> Index of the channel that shall be read
 * checkIfQueued-> Will not perform the action if the used
 * 				   EXT_TEMP_x register is currently queued to be read
 *
 * Result is stored in tle9012_node.temperatures[cellsIdx]
 ****************************************************************************/
requestResult tle9012_convertTemperatureByIndex(uint8_t nodeID,
		uint8_t channelIdx, uint8_t checkIfQueued){
	// Get register index of requested temp register
	uint8_t reg_temp_idx = EXT_TEMP_0 + channelIdx;

	// If register is not ready or on error state return error
	if(checkRegisterReady_tle9012(nodeID, reg_temp_idx, 0) != 1)
		return REQUEST_REGISTER_NOT_READY;
	else if(checkIfQueued == 1 && checkRegisterQueued_tle9012(nodeID, reg_temp_idx, 0) == 1)
		return REQUEST_REGISTER_ALREADY_QUEUED;
	else if(checkRegisterError_tle9012(nodeID, reg_temp_idx, 0) != 0)
		return REQUEST_ERROR;

	// Get register result
	uint16_t tempRaw = getRegisterCacheDataByIndex_tle9012(nodeID, reg_temp_idx);

	// If valid bit is not set the data is old or invalid
	if((tempRaw & VALID__NEW_RESULT_STORED) == 0){
		// Mark register as error
		setRegisterError_tle9012(nodeID, reg_temp_idx, 0);
		return REQUEST_REGISTER_DATA_INVALID_OR_OLD;
	}

	// Extract INTC - internal current source power
	uint8_t ext_temp_z_INTC = (tempRaw & EXT_TEMP_0__INTC.mask) >> 10;

	// Extract result bits (assume same mask for all temp registers)
	tempRaw &= EXT_TEMP_0__RESULT.mask;

	// Convert register data to resistance value in kOhm
	double resistance =
			(((double)tempRaw * (TEMP_FSR * pow(4.0, ext_temp_z_INTC) ) ) /
					TEMP_DENOMINATOR - TEMP_R) /1000.0;

	// Convert resistance to temperature with interpolated lookup table
	tle9012_nodes[nodeID-1].temperatures[channelIdx] =
			interpLookupTable_tle9012(LT_NTC_10k, LT_NTC_10K_SIZE, resistance);

	/*DEBUGPRINTF("%d -> %d INTC = %lf ohm = %lf C\r\n",
			getRegisterCacheDataByIndex_tle9012(nodeID, reg_temp_idx),
			ext_temp_z_INTC, resistance,
			tle9012_nodes[nodeID-1].temperatures[channelIdx]);
	if(channelIdx == 4){
		printf("tempraw: %d\r\n", tempRaw);
		double res = interpLookupTableReverse_tle9012(LT_NTC_10k, LT_NTC_10K_SIZE, 25.0F);
		double raw = ((res*1000.0+TEMP_R)*TEMP_DENOMINATOR)/TEMP_NOMINATOR;
		printf("Res=%f raw=%f\r\n", res, raw);
	}*/

	// Return ok
	return REQUEST_OK;
}

/****************************************************************************
 * tle9012_convertExternalTemperatureThreshold
 * Used to convert the temperature threshold in Celsius
 * to the format that need to be written to TEMP_CONF register
 ****************************************************************************/
uint16_t  tle9012_convertExternalTemperatureThreshold(double th_degree_C,
		uint8_t ext_temp_z_INTC){
	double e_tmp_th_resistance =
			interpLookupTableReverse_tle9012(LT_NTC_10k,
					LT_NTC_10K_SIZE, th_degree_C);
	double e_tmp_th_raw =
			((e_tmp_th_resistance*1000.0+TEMP_R)*TEMP_DENOMINATOR)/
			(TEMP_FSR * pow(4.0, ext_temp_z_INTC) );
	//printf("Res=%f raw=%f\r\n", e_tmp_th_resistance, e_tmp_th_raw);

	// Limit to max and min if needed
	if(e_tmp_th_raw < 250){
		printf("convertExternalTemperatureThreshold: "
				"Calculated threshold turned out to be lower than 250. "
				"Use other ext_temp_z_INTC! %.2f\r\n", e_tmp_th_raw);
		return 250;
	}
	else if(e_tmp_th_raw > 800){
		printf("convertExternalTemperatureThreshold: "
				"Calculated threshold turned out to be higher than 800. "
				"Use other ext_temp_z_INTC! %.2f\r\n", e_tmp_th_raw);
		return 800;
	}
	else{
		return (uint16_t)e_tmp_th_raw;
	}
}

/****************************************************************************
 * tle9012_convertInternalTemperatureByIndex
 * Read and convert one of the two temperature sensor channels (0-1) and
 * store result in tle9012_node item
 *
 * Note: TEMP_R, TEMP_INTC and TEMP_NOMINATOR must be adjusted if the
 * 		 	default precision of 16bit changed or a different sensor is used.
 * 		 If the VALID flag of the temperature sensor is not set
 * 		 	the error bit in the error bitmap is set
 *
 * node			-> 	The tle9012_node object representing one TLE9012 instance
 * channelIdx	-> 	Index of the channel that shall be read (0 or 1)
 * checkIfQueued-> 	Will not perform the action if the used
 * 					EXT_TEMP_x register is currently queued to be read
 *
 * Result is stored in tle9012_node.temperaturesInternal[cellsIdx]
 ****************************************************************************/
requestResult tle9012_convertInternalTemperatureByIndex(uint8_t nodeID,
		uint8_t channelIdx, uint8_t checkIfQueued){
	// Get register index of requested temp register
	uint16_t reg_temp_idx = 0;
	if(channelIdx == 0)
		reg_temp_idx = INT_TEMP;
	else if(channelIdx == 1)
		reg_temp_idx = INT_TEMP_2;
	else
		return REQUEST_ERROR;

	// If register is not ready or on error state return error
	if(checkRegisterReady_tle9012(nodeID, reg_temp_idx, 0) != 1)
		return REQUEST_REGISTER_NOT_READY;
	else if(checkIfQueued == 1 && checkRegisterQueued_tle9012(nodeID, reg_temp_idx, 0) == 1)
		return REQUEST_REGISTER_ALREADY_QUEUED;
	else if(checkRegisterError_tle9012(nodeID, reg_temp_idx, 0) != 0)
		return REQUEST_ERROR;

	// Get register result
	uint16_t tempRaw = getRegisterCacheDataByIndex_tle9012(nodeID, reg_temp_idx);

	// If valid bit is not set the data is old or invalid
	if((tempRaw & VALID__NEW_RESULT_STORED) == 0){
		// Mark register as error
		setRegisterError_tle9012(nodeID, reg_temp_idx, 0);
		return REQUEST_REGISTER_DATA_INVALID_OR_OLD;
	}

	// Extract result bits (assume same mask for all temp registers)
	tempRaw &= INT_TEMP__RESULT.mask;

	// Convert register data to temperature
	tle9012_nodes[nodeID-1].temperaturesInternal[channelIdx] =
			-TEMP_INT_LSB*(double)tempRaw+547.3;

	// Threshold calculation
	// (-TC + 547.3)/TEMP_INT_LSB = tempRaw

	// Return ok
	return REQUEST_OK;
}

//****************************************************************************
//TODO: Create convert function for SCVM, BAVM and AVM
//****************************************************************************

/****************************************************************************
 * tle9012_printRegister
 * Print the register data and all its sub-registers separately
 * in HEX, DEC and BIN
 *
 * nodeID	-> 	The ID of the node that shall be printed.
 * 				Range 1 to TLE9012_NODES_SIZE
 * reg_idx	-> 	The index of the register that shall be printed
 * 				see register.h definitions.  E.g. PART_CONFIG etc
 ****************************************************************************/
void tle9012_printRegister(uint8_t nodeID, uint8_t reg_index)
{
    uint16_t reg_addr;
	const char *reg_name;
    //const char *bit_name;
	//int bit_num;

    if(reg_index >= 0 && reg_index <= NUM_REGISTERS)
    {
        reg_name = reg_items[reg_index].name;
        reg_addr = reg_items[reg_index].address;

        DEBUGPRINTF("\r\nRegister '%-16s' adr 0x%04X, idx %2d: 0x%04X\r\n",
        		reg_name, reg_index, reg_addr,
				tle9012_nodes[nodeID-1].reg_data[reg_index]);

        // Check every subreg index till subregs of this register are done
        for(uint8_t subreg_idx = 0; subreg_idx < NUM_SUBREGISTERS-1; subreg_idx++){
        	// Stop iteration if wanted register is done, else print register
        	if(subreg_items[subreg_idx]->reg_index > reg_index){
        		break;
        	}
        	else if(subreg_items[subreg_idx]->reg_index == reg_index){
        		// Extract subreg data from register
        		uint16_t data = tle9012_nodes[nodeID-1].reg_data[reg_index];
        		data &= subreg_items[subreg_idx]->mask;

        		// Shift subreg data to LSB
        		uint16_t maskShifted = subreg_items[subreg_idx]->mask;
        		uint8_t numBitShift = 0;
        		for(numBitShift = 0; numBitShift <= 15; numBitShift++){
        			// Check if shift is done
        			if((maskShifted & 1) == 1){ break; }
        			// Shift bits
        			maskShifted = maskShifted >> 1;
        		}
        		// Shift data
        		uint16_t dataShifted = data >> numBitShift;

        		// Print data
        		DEBUGPRINTF("\tSubregister '%-18s' at %2d: 0x%04X | %6d | 0b",
        				subreg_items[subreg_idx]->name, numBitShift,
						dataShifted, dataShifted);

        		/// Print data in binary
        		// Get the MSB of mask
        		uint16_t compare = 0b1000000000000000;
        		// Run backward through bits and print bit if it is in mask
        		for(int8_t numBit = 15; numBit >= 0; numBit--){
        			// Extract bit
        			uint16_t selectedBit = compare & subreg_items[subreg_idx]->mask;

        			// If the current bit is part of mask
        			if( selectedBit >= 1){
        				// If bit is a 1 write a one
        				if((data & selectedBit) == 0){
        					DEBUGPRINTF("0");
        				}
        				else{
        					DEBUGPRINTF("1");
        				}
        				/*DEBUGPRINTF("A: %d %d %d %d\r\n",numBit, compare,
        						selectedBit, data & selectedBit);*/
        			}
        			// Double compare to select next bit
        			compare = compare>>1;
        		}
        		DEBUGPRINTF("\r\n");
        	}
        }
    }
    else
    {
        DEBUGPRINTF("\r\n"); // new line
        DEBUGPRINTF("[ERROR]: Invalid register number");
    }
}

/****************************************************************************
 * tle9012_printAllRegisters
 * Print all register data and all their sub-registers separately
 * in HEX, DEC and BIN
 *
 * nodeID	->	The ID of the node that shall be printed.
 * 				Range 1 to TLE9012_NODES_SIZE
 ****************************************************************************/
void tle9012_printAllRegisters(uint8_t nodeID)
{
	for(int i = 0; i < NUM_REGISTERS; i++){
		tle9012_printRegister(nodeID, i);
	}
	DEBUGPRINTF("\r\n");
}

/****************************************************************************
 * tle9012_printStatusFlags
 * check failure bits in diagnosis registers and report failures to debug
 ****************************************************************************/
uint8_t last_GEN_DIAG;
void tle9012_printStatusFlags(uint8_t nodeID, uint8_t newLineForEachError,
		uint8_t onlyPrintIfDataChanged)
{
	uint16_t gen_diag_data = getRegisterCacheDataByIndex_tle9012(nodeID, GEN_DIAG);

	if((gen_diag_data & GEN_DIAG_RESETABLE_FLAGS) != 0 &&
			(onlyPrintIfDataChanged == 0 || (onlyPrintIfDataChanged == 1 &&
					last_GEN_DIAG != gen_diag_data))){
		if(newLineForEachError == 0)
			DEBUGPRINTF("[FAILURE]: TLE9012 -");

		if(gen_diag_data & PS_ERR_SLEEP__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" PS_ERR_SLEEP");
		}
		if(gen_diag_data & ADC_ERR__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" ADC_ERR");
		}
		if(gen_diag_data & OL_ERR__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" OL_ERR");
		}
		if(gen_diag_data & INT_IC_ERR__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" INT_IC_ERR");
		}
		if(gen_diag_data & REG_CRC_ERR__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" REG_CRC_ERR");
		}
		if(gen_diag_data & EXT_T_ERR__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" EXT_T_ERR");
		}
		if(gen_diag_data & INT_OT__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" INT_OT");
		}
		if(gen_diag_data & CELL_UV__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" CELL_UV");
		}
		if(gen_diag_data & CELL_OV__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" CELL_OV");
		}
		if(gen_diag_data & BAL_ERR_UC__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" BAL_ERR_UC");
		}
		if(gen_diag_data & BAL_ERR_OC__ERR_OCCURED){
			if(newLineForEachError)DEBUGPRINTF("\r\n[FAILURE]: ");
			DEBUGPRINTF(" BAL_ERR_OC");
		}
		DEBUGPRINTF("\r\n");

		// Store last state of registers
		last_GEN_DIAG = gen_diag_data;
	}
}

/****************************************************************************
 * tle9012_isProblemDetected
 * Check problem flags in diagnosis registers
 *
 * Return 1 if the library detected a logic failure or
 * Return 2 if a TLE9012 reported a fail in general diagnostic register
 * 		(automatically returned by every write to a register)
 ****************************************************************************/
uint8_t tle9012_isProblemDetected(){
	// If any error was detected return its code
	if(tle9012_problemDetected == 1 )
		return 1;
	else if(generalDiagnosticProblemDetected == 1)
		return 2;

	// Else return 0
	return 0;
}

/****************************************************************************
 * tle9012_clearProblemDetected
 * Clear all problem flags
 *
 * (library logic failure and TLE9012 fail in general diagnostic register
 * 		(automatically checked/set every write to a register)
 ****************************************************************************/
void    tle9012_clearProblemDetected(){
	tle9012_problemDetected = 0;
	generalDiagnosticProblemDetected = 0;
}

/****************************************************************************
 * tle9012_getFailureBitmap
 * Get a uint32 with all set failure bits
 ****************************************************************************/
uint32_t tle9012_getFailureBitmap(uint8_t nodeID){
	uint32_t bitmap = 0;
	uint16_t gen_diag_data =
			getRegisterCacheDataByIndex_tle9012(nodeID, GEN_DIAG);

	if(gen_diag_data & OL_ERR__ERR_OCCURED)
		bitmap |= Fail_SYS_OL_ERR;

	if(gen_diag_data & INT_IC_ERR__ERR_OCCURED)
		bitmap |= Fail_SYS_INT_IC_ERR;

	if(gen_diag_data & REG_CRC_ERR__ERR_OCCURED)
		bitmap |= Fail_SYS_REG_CRC_ERR;

	if(gen_diag_data & EXT_T_ERR__ERR_OCCURED)
		bitmap |= Fail_SYS_EXT_T_ERR;

	if(gen_diag_data & INT_OT__ERR_OCCURED)
		bitmap |= Fail_SYS_INT_OT;

	if(gen_diag_data & CELL_UV__ERR_OCCURED)
		bitmap |= Fail_SYS_CELL_UV;

	if(gen_diag_data & CELL_OV__ERR_OCCURED)
		bitmap |= Fail_SYS_CELL_OV;

	if(gen_diag_data & BAL_ERR_UC__ERR_OCCURED)
		bitmap |= Fail_SYS_BAL_ERR_UC;

	if(gen_diag_data & BAL_ERR_OC__ERR_OCCURED)
		bitmap |= Fail_SYS_BAL_ERR_OC;

	if(gen_diag_data & PS_ERR_SLEEP__ERR_OCCURED)
		bitmap |= Fail_SYS_PS_ERR_SLEEP;

	if(gen_diag_data & ADC_ERR__ERR_OCCURED)
		bitmap |= Fail_SYS_ADC_ERR;

	return bitmap;
}

/****************************************************************************
 * tle9012_isFailureFlagActive
 * Check specific failure bits in diagnosis registers and return state
 ****************************************************************************/
uint8_t tle9012_isFailureFlagActive(uint8_t nodeID, FailureFlags_tle9012 Fail){
	uint16_t gen_diag_data =
			getRegisterCacheDataByIndex_tle9012(nodeID, GEN_DIAG);

	switch(Fail){
		case Fail_SYS_PS_ERR_SLEEP:
			if(gen_diag_data & PS_ERR_SLEEP__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_ADC_ERR:
			if(gen_diag_data & ADC_ERR__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_OL_ERR:
			if(gen_diag_data & OL_ERR__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_INT_IC_ERR:
			if(gen_diag_data & INT_IC_ERR__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_REG_CRC_ERR:
			if(gen_diag_data & REG_CRC_ERR__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_EXT_T_ERR:
			if(gen_diag_data & EXT_T_ERR__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_INT_OT:
			if(gen_diag_data & INT_OT__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_CELL_UV:
			if(gen_diag_data & CELL_UV__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_CELL_OV:
			if(gen_diag_data & CELL_OV__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_BAL_ERR_UC:
			if(gen_diag_data & BAL_ERR_UC__ERR_OCCURED)
				return 1;
			break;
		case Fail_SYS_BAL_ERR_OC:
			if(gen_diag_data & BAL_ERR_OC__ERR_OCCURED)
				return 1;
			break;
	}
	return 0;
}
