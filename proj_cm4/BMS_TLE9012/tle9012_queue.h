/*
 * tle9012_queue.h
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

#ifndef TLE9012_QUEUE_H__
#define TLE9012_QUEUE_H__

#include <stdint.h>
#include "tle9012_registers.h"

//****************************************************************************
// Definitions for the queuing system
//****************************************************************************
#define QUEUE_ITEMS_MAX 65

//****************************************************************************
// Data structures
//****************************************************************************
typedef enum {QUEUE_SUCCESS = 0U, QUEUE_FINISHED = 1U, QUEUE_FAIL = 2U, QUEUE_TIMEOUT = 3U, QUEUE_DEVICE_REINIT = 4U} QueueResult;
typedef enum {EXECUTE_TX = 0U, WAIT_FOR_RX = 1U, WAIT_FOR_NEXT_ITEM = 2U} queueItemState;
typedef struct {
    uint8_t  nodeID;
    uint8_t  isWrite;
    uint8_t  isBroadcast;
    uint8_t  regIndex;
    uint16_t data;
    queueItemState state;
} queue_item;

//****************************************************************************
// Queue and management
//****************************************************************************
// The command queue (used as ring)
extern queue_item queue_items[QUEUE_ITEMS_MAX];

// The time of the last send command (used to determine timeouts)
extern uint32_t timeLastQueueCommandSent;

// Function Prototypes
void resetQueue();
QueueResult addToQueue_tle9012(uint8_t nodeID, uint8_t isWrite, uint8_t registerIdx, uint16_t data, uint8_t isBroadcast);
QueueResult tle9012_manageQueue(uint8_t keepAlive);



#endif /* TLE9012_QUEUE_H__ */
