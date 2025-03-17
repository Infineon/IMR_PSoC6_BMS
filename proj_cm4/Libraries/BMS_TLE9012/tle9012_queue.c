/*
 * tle9012_queue.c
 *
 *	Library for TLE9012 Li-Ion battery monitoring and balancing system IC.
 *	See main file Lib_TLE9012.c for how to use guide
 *	Created by R. Santeler @ Infineon 2024
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
#include <ctype.h>
#include <stdlib.h> // includes strtol
#include "tle9012_queue.h"
#include "tle9012_target.h"
#include "tle9012_registers.h"

//****************************************************************************
// Definitions and Objects for the queuing system
//****************************************************************************
// De/activate common debug messages of queuing system
#define DEBUGPRINTF_QUEUE(p_frm, ...)		//DEBUGPRINTF(p_frm, ##__VA_ARGS__)
// De/activate performance debug messages of queuing system
#define DEBUGPRINTF_QUEUE_PERF(p_frm, ...)	//DEBUGPRINTF(p_frm, ##__VA_ARGS__)
uint8_t queue_index_latest = 0;  	// The index of the last item in the queue,
									// new items will be added after this index
uint8_t queue_index_current = 0; 	// The index of the currently processed item in the queue
// First item in queue must be set to wait for next item
queue_item queue_items[QUEUE_ITEMS_MAX] = {{0,0,0,0,0,WAIT_FOR_NEXT_ITEM}};
uint32_t timeLastQueueCommandSent = 0;

//****************************************************************************
// Inline Functions
//****************************************************************************
static inline void incrementQueueIndex(uint8_t* index){
	(*index)++;
	if(*index > QUEUE_ITEMS_MAX-1){
		(*index) = 0;
	}
}
static inline void decrementQueueIndex(uint8_t* index){
    if((*index) >= 1)
	    (*index)--;
    else
        (*index) = QUEUE_ITEMS_MAX-1;
}

//****************************************************************************
// Functions
//****************************************************************************
//****************************************************************************
// resetQueue
// Resets the indices, the first queue item and the last CommandSent
//****************************************************************************
void resetQueue(){
	queue_index_latest = 0;
	queue_index_current = 0;
	queue_items[0].state = WAIT_FOR_NEXT_ITEM;
	timeLastQueueCommandSent = TIMER_COUNTER_1MS;
}

//****************************************************************************
// Adds a new queue item to be processed. Check if the queue will overflow and
// returns QueueFail if the request could not be added
//****************************************************************************
QueueResult addToQueue_tle9012(uint8_t nodeID, uint8_t isWrite,
		uint8_t registerIdx, uint16_t data, uint8_t isBroadcast){
	// Determine index of next item
	incrementQueueIndex(&queue_index_latest);

	// If the next item index does not collide with currently processed item index...
	if(queue_index_latest != queue_index_current){
		// ... store new queue item
		queue_items[queue_index_latest].nodeID = nodeID;
		queue_items[queue_index_latest].isWrite = isWrite;
		queue_items[queue_index_latest].isBroadcast = isBroadcast;
		queue_items[queue_index_latest].regIndex = registerIdx;
		queue_items[queue_index_latest].data = data;
		queue_items[queue_index_latest].state = EXECUTE_TX;

		/*DEBUGPRINTF_QUEUE("\tAdd queue item: id %d, isWrite %d, "
				"register %d, data %d\r\n",
				queue_items[queue_index_latest].nodeID,
				queue_items[queue_index_latest].isWrite,
				queue_items[queue_index_latest].regIndex,
				queue_items[queue_index_latest].data);*/

		DEBUGPRINTF_QUEUE("\tAdd  queue item at index %d: "
				"Register %d in state %d\r\n", queue_index_latest,
				queue_items[queue_index_latest].regIndex,
				queue_items[queue_index_latest].state);

		// Mark register as queued
		setRegisterQueued_tle9012(nodeID, registerIdx, isBroadcast);

		// Return OK
		return QUEUE_SUCCESS;
	}

	// Mark register action failed
	setRegisterError_tle9012(nodeID, registerIdx, isBroadcast);

    // Revert to last item index
    decrementQueueIndex(&queue_index_latest);

	// Return FAIL
	DEBUGPRINTF_QUEUE("\tAdd  queue item FAILED: Queue overflow!!!!\r\n");
	return QUEUE_FAIL;
}

uint64_t txTime = 0;

//****************************************************************************
// Manages queue items
// Resets the ready bit of the register, sends the request
// and waits till ready bit is set again (from FIFO interpreter).
// After this it goes on with the next
// queue item or waits till a new one comes in.
//****************************************************************************
QueueResult tle9012_manageQueue(uint8_t keepAlive){
	//uint32_t debugStartTime = TIMER_COUNTER_1MS;

	DEBUGPRINTF_QUEUE("tle9012_manageQueue: current index %d, "
			"latest index %d, state %d\r\n", queue_index_current,
			queue_index_latest, queue_items[queue_index_current].state);

	// If the time since last wakeup exceeded timeout wake device - most registers will lose their state! Refresh of setup needed
	if(TIMER_COUNTER_1MS > (timeLastWatchdogRefresh + TLE9012_WATCHDOG_MAXIMUM_TIME)){
		DEBUGPRINTF("[ERROR]: Last TLE9012 watchdog refresh was %ld ms ago. "
				"Will reinitialize to wake devices!\r\n",
				TIMER_COUNTER_1MS - timeLastWatchdogRefresh);

		// Reinitialize all tle9012 ICs
		tle9012_deviceInit(1);

		// Return that device was reinitialized so that
		// main application can reconfig the TL9012 as seen fit
		return QUEUE_DEVICE_REINIT;
	}

	// Check if current queue item is waiting (for RX or next queue item to be added)
	if(queue_items[queue_index_current].state == WAIT_FOR_RX){
		DEBUGPRINTF_QUEUE("\tWAIT_FOR_RX on queue index %d: NodeId %d, "
				"RegisterIdx %d, Write %d, ready %d\r\n",
				queue_index_current,
				queue_items[queue_index_current].nodeID,
				queue_items[queue_index_current].regIndex,
				queue_items[queue_index_current].isWrite,
				checkRegisterReady_tle9012(queue_items[queue_index_current].nodeID,
						queue_items[queue_index_current].regIndex,
						queue_items[queue_index_current].isBroadcast));
		// Check ready bit of register until it is ready
		if(checkRegisterReady_tle9012(queue_items[queue_index_current].nodeID,
				queue_items[queue_index_current].regIndex,
				queue_items[queue_index_current].isBroadcast)){
			/*DEBUGPRINTF_QUEUE("\tReceived queue index %d: "
					"NodeId %d, RegisterIdx %d, Write %d, Data 0x%04X\r\n",
					queue_index_current, queue_items[queue_index_current].nodeID,
					queue_items[queue_index_current].regIndex,
					queue_items[queue_index_current].isWrite,
					getRegisterCacheDataByIndex_tle9012(
							&tle9012_nodes[queue_items[queue_index_current].nodeID-1],
							queue_items[queue_index_current].regIndex));*/

			// Set register index as not queued
			resetRegisterQueued_tle9012(queue_items[queue_index_current].nodeID,
					queue_items[queue_index_current].regIndex,
					queue_items[queue_index_current].isBroadcast);

			// If there is another item in queue execute it ...
			// TODO: Code restructure (this code is also in next block...)
			if(queue_index_latest != queue_index_current){
			    DEBUGPRINTF_QUEUE("\tRegister data ready: Check next queue item\r\n");
				incrementQueueIndex(&queue_index_current);
			}
			// ... else skip till a new item comes in
			else{
			    DEBUGPRINTF_QUEUE("\tRegister data ready: Wait for next queue item\r\n");
				queue_items[queue_index_current].state = WAIT_FOR_NEXT_ITEM;
				DEBUGPRINTF_QUEUE_PERF("manageQueue took %ld ms\r\n",
						(uint32_t)(TIMER_COUNTER_1MS-debugStartTime));
				return QUEUE_FINISHED;
			}
		}
		// If time since last send command is to long, stop with error
		/*DEBUGPRINTF_QUEUE("\r\nTimeout %ld %ld \r\n",
				TIMER_COUNTER_1MS,
				tle9012_nodes[queue_items[queue_index_current].nodeID-1].timeLastCommandSent);*/
		else if(TIMER_COUNTER_1MS > (timeLastQueueCommandSent + TIMEOUT_COMMAND)){
			DEBUGPRINTF_QUEUE("\r\n[ERROR]: "
					"Timeout tle9012_manageQueue took %ld ms (max=%d)\r\n",
					(uint32_t)(TIMER_COUNTER_1MS - timeLastQueueCommandSent),
					TIMEOUT_COMMAND);
			// Mark register as ready
			setRegisterReady_tle9012(
					queue_items[queue_index_current].nodeID,
					queue_items[queue_index_current].regIndex,
					queue_items[queue_index_current].isBroadcast);
			// Mark register as error
			setRegisterError_tle9012(
					queue_items[queue_index_current].nodeID,
					queue_items[queue_index_current].regIndex,
					queue_items[queue_index_current].isBroadcast);
			// Set register index as not queued
			resetRegisterQueued_tle9012(
					queue_items[queue_index_current].nodeID,
					queue_items[queue_index_current].regIndex,
					queue_items[queue_index_current].isBroadcast);

			// If there is another item in queue execute it ...
			if(queue_index_latest != queue_index_current){
			    DEBUGPRINTF_QUEUE("\tRegister data ready: Check next queue item\r\n");
				incrementQueueIndex(&queue_index_current);
			}
			// ... else skip till a new item comes in
			else{
			    DEBUGPRINTF_QUEUE("\tRegister data ready: Wait for next queue item\r\n");
				queue_items[queue_index_current].state = WAIT_FOR_NEXT_ITEM;
				DEBUGPRINTF_QUEUE_PERF("manageQueue took %ld ms\r\n",
						(uint32_t)(TIMER_COUNTER_1MS-debugStartTime));
			}

			// Return result
			DEBUGPRINTF_QUEUE_PERF("manageQueue took %ld ms\r\n",
					(uint32_t)(TIMER_COUNTER_1MS-debugStartTime));
			return QUEUE_TIMEOUT;
		}

	}
	else if(queue_items[queue_index_current].state == WAIT_FOR_NEXT_ITEM){
		// If there is another item in queue execute it, else skip till a new item occurs
		if(queue_index_latest != queue_index_current){
			// Switch to next item in queue
			DEBUGPRINTF_QUEUE("\tNew  queue item  detected\r\n");
			incrementQueueIndex(&queue_index_current);
		}
		else{
			DEBUGPRINTF_QUEUE_PERF("manageQueue took %ld ms\r\n",
					(uint32_t)(TIMER_COUNTER_1MS-));
			return QUEUE_FINISHED;
		}
	}


	// Check if current item is to be transmitted
	if(queue_items[queue_index_current].state == EXECUTE_TX){
		// Check if the device must be kept alive by refresh of the watchdog counter
		// If so this has higher priority and the TX of the current queue item will be delayed
		if(keepAlive == 1 && TIMER_COUNTER_1MS >
		(timeLastWatchdogRefresh + TLE9012_WATCHDOG_REFRESH_TIME)){
			sendWatchdogUpdate(1, 0); // TODO: check if this works as broadcast
		}
		else{
			// Send command
			DEBUGPRINTF_QUEUE("\tSend queue index %d: NodeId %d, "
					"RegisterIdx %d, Write %d, Data 0x%04X\r\n",
					queue_index_current,
					queue_items[queue_index_current].nodeID,
					queue_items[queue_index_current].regIndex,
					queue_items[queue_index_current].isWrite,
					queue_items[queue_index_current].data);
			if(queue_items[queue_index_current].isWrite == 1){
				sendWrite(tle9012_nodes,
						reg_items[queue_items[queue_index_current].regIndex].address,
						queue_items[queue_index_current].data,
						queue_items[queue_index_current].isBroadcast);
			}
			else{
				sendRead(tle9012_nodes,
						reg_items[queue_items[queue_index_current].regIndex].address,
						queue_items[queue_index_current].isBroadcast);
			}

			// Mark register as not ready
			resetRegisterReady_tle9012(queue_items[queue_index_current].nodeID,
					queue_items[queue_index_current].regIndex,
					queue_items[queue_index_current].isBroadcast);
			// Mark register as no error
			/*resetRegisterError_tle9012(queue_items[queue_index_current].nodeID,
					queue_items[queue_index_current].regIndex,
					queue_items[queue_index_current].isBroadcast);*/

			// Store time of send
			timeLastQueueCommandSent = TIMER_COUNTER_1MS;

			// Change state to wait
			queue_items[queue_index_current].state = WAIT_FOR_RX;
		}
	}

	DEBUGPRINTF_QUEUE_PERF("manageQueue took %ld ms\r\n",
			(uint32_t)(TIMER_COUNTER_1MS-debugStartTime));
	return QUEUE_SUCCESS;
}
