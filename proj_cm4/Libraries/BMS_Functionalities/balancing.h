/*
 * balancing.c
 * 		Abstraction off all battery balancing related functions
 * 		that need to be accessed from main.
 * 		The implemented algorithm sorts the cells by voltage and
 * 		returns a threshold (determineBalancingThreshold()).
 * 		All cells above this must be balanced.
 * 		The function manage_balancing() determines the a cell map,
 * 		which is used in main.
 *
 *  Created on: 23 May 2024
 *      Author: r. santeler
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

#ifndef BALANCING_H_
#define BALANCING_H_


/*******************************************************************************
* DEFINITIONS
*******************************************************************************/


/*******************************************************************************
 * VARIABLES
*******************************************************************************/


/*******************************************************************************
* FUNCTIONS
*******************************************************************************/
//void swap(sortBB_struct* xp, sortBB_struct* yp);
//void prepareCellSort(uint16_t* BBs, sortBB_struct* sortBB, uint8_t n);
//void selectionCellSort(sortBB_struct* sortBB, uint8_t n);
//uint16_t determineBalancingThreshold(uint16_t* batteryArray);
void manage_balancing();

#endif /* BALANCING_H_ */
