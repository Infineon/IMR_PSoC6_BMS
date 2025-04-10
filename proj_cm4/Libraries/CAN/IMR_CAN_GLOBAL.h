/******************************************************************************
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
******************************************************************************/

#ifndef IMR_CAN_GLOBAL_H_
#define IMR_CAN_GLOBAL_H_

#include "stdint.h"

typedef struct IMR_CAN_MESSAGE_STRUCT {
	uint32_t CAN_ID;		// Stores CAN ID for message identification
	uint8_t  CAN_DATA[8];	// Stores the CAN data associated with
							// the corresponding CAN Message
} IMR_CAN_MESSAGE_STRUCT_t;

typedef enum IMR_CAN_MESSAGE_IDS {
    EMERGENCY_STOP = 0x80,

    BMS_SLOT_1_STATE = 0x100,
    BMS_SLOT_2_STATE = 0x101,

#if (EXT_DATALOG_ENABLED)
	BMS_SLOT_1_CURR_TEMP  = 0x110,
	BMS_SLOT_2_CURR_TEMP  = 0x111,
	BMS_CHARGER_CURR_TEMP = 0x112,

	BMS_SLOT_1_CELLV0_3  = 0x120,
	BMS_SLOT_2_CELLV0_3  = 0x121,
	BMS_CHARGER_CELLV0_3 = 0x122,

	BMS_SLOT_1_CELLV4_7  = 0x130,
	BMS_SLOT_2_CELLV4_7  = 0x131,
	BMS_CHARGER_CELLV4_7 = 0x132,

	BMS_SLOT_1_CELLV8_11  = 0x140,
	BMS_SLOT_2_CELLV8_11  = 0x141,
	BMS_CHARGER_CELLV8_11 = 0x142,
#endif

    BMS_REDUCE_POWER_FOR_HS = 0x180,
    BMS_ENABLE_POWER_FOR_HS = 0x181,

    BMS_SLOT_1_HS_HANDSHAKE = 0x182,
    BMS_SLOT_1_HS_SWAP = 0x183,
    BMS_SLOT_1_HS_FINISH = 0x184,
    BMS_SLOT_1_SWITCH_ON_VETO = 0x185,

    BMS_SLOT_2_HS_HANDSHAKE = 0x192,
    BMS_SLOT_2_HS_SWAP = 0x193,
    BMS_SLOT_2_HS_FINISH = 0x194,
    BMS_SLOT_2_SWITCH_ON_VETO = 0x195,

    BMS_RT_DATA_RESTART = 0x199,
    BMS_RT_DATA_GETLINES = 0x19A,
    BMS_RT_DATA_SENDLINEPART = 0x19B,
    BMS_RT_DATA_GETSTOREDLINENUMBER = 0x19C,
    BMS_RT_DATA_GETMAXLINENUMBER = 0x19D,
    BMS_RT_DATA_SETSAMPLINGTIME = 0x19E,
    BMS_RT_DATA_GETSAMPLINGTIME = 0x19F,

    PD_FRONT_CHANNEL_OUTPUT_CURRENT = 0x280,
    PD_BACK_CHANNEL_OUTPUT_CURRENT = 0x281,

    PD_FRONT_CHANNEL_SET = 0x300,
    PD_BACK_CHANNEL_SET = 0x301,

    MOT_FL_SPEED_COMMAND = 0x380,
    MOT_FR_SPEED_COMMAND = 0x381,
    MOT_BL_SPEED_COMMAND = 0x382,
    MOT_BR_SPEED_COMMAND = 0x383,

    MOT_FL_ENCODER_DATA = 0x400,
    MOT_FR_ENCODER_DATA = 0x401,
    MOT_BL_ENCODER_DATA = 0x402,
    MOT_BR_ENCODER_DATA = 0x403,

	BAR_GRAPH_BACK = 0x406,
	BAR_GRAPH = 0x407,

    ROBOT_VELOCITY_COMMAND = 0x41C,
    ROBOT_ODOMETRY_ESTIMATE = 0x440,

    IMU_DATA = 0x460,

    LED_LAYER_1_FRONT = 0x480,
	LED_LAYER_1_LEFT = 0x481,
	LED_LAYER_1_BACK = 0x482,
	LED_LAYER_1_RIGHT = 0x483,

	LED_LAYER_2_FRONT = 0x484,
	LED_LAYER_2_LEFT = 0x485,
	LED_LAYER_2_BACK = 0x486,
	LED_LAYER_2_RIGHT = 0x487,

	LED_LAYER_3_FRONT = 0x488,
	LED_LAYER_3_LEFT = 0x489,
	LED_LAYER_3_BACK = 0x48A,
	LED_LAYER_3_RIGHT = 0x48B,

    LED_ALL = 0x48F,

    CALIBRATION_REQUEST_ALL_MOTORS = 0x540,
    CALIBRATION_REQUEST_IMU = 0x541,

	COMMIT_HASH = 0x600
} IMR_CAN_MESSAGE_IDS_t;

typedef enum CAN_STATUS_t {
    CAN_SUCCESS,
    CAN_ERROR,
    CAN_BUSY,
    CAN_NOT_ACCEPTABLE,
    CAN_DISABLED
} CAN_STATUS_t;

// Sending LED Effects to SENSOR_LED
typedef enum IMR_CAN_SENSOR_LED_COMMANDS {
    LED_MODE_OFF	= 	0x4A,
    LED_MODE_CHASER	= 	0x4B,
    LED_MODE_PULSE  = 	0x4C,
    LED_MODE_STEADY	= 	0x4F
} IMR_CAN_SENSOR_LED_COMMANDS_t;

/*****************************************************************************/

CAN_STATUS_t CAN_TX_Request(uint32_t CAN_ID, uint8_t* Target_Data,
		uint8_t Target_Data_Length);

#endif /* IMR_CAN_GLOBAL_H_ */
