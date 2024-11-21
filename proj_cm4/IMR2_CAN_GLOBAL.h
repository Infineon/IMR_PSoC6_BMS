/*
 * IMR2_CAN_GLOBAL.h - Abstraction off all CAN receive and transmit actions. Valid incoming messages set flags that are handled and reset in main.
 *
 *  Created on: 17.07.2023
 *      Author: m. schmidt and r. santeler
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef IMR2_CAN_GLOBAL_H_
#define IMR2_CAN_GLOBAL_H_

#define CAT(x, y) CAT_(x, y)
#define CAT_(x, y) x ## y

#include "cybsp.h"
#include "cy_utils.h"


/*******************************************************************************
* Definitions
*******************************************************************************/
#define CAN_IRQ_RX_MESSAGE_HANDLER         		canfd_rx_callback	/* IRQ Event Source Name */
#define CAN_MAX_DATA_LENGTH						6					/* Maximum size of the data to be analyzed */

/*******************************************************************************
* CAN Commands and IDs
*******************************************************************************/
typedef enum IMR_CAN_SENSOR_LED_COMMANDS {
	SENSOR_LED_Effect_Off			= 0x4A,		// Sending LED Effects to SENSOR_LED
	SENSOR_LED_Effect_ChaserLight	= 0x4B,
	SENSOR_LED_Effect_PulsingLight  = 0x4C,
	SENSOR_LED_Effect_Color			= 0x4D,
	SENSOR_LED_Effect_Alert			= 0x4E
} IMR_CAN_SENSOR_LED_COMMANDS_t;

typedef enum IMR_CAN_INVERTER_COMMANDS {
	INVERTER_TargetSpeed			= 0x1D,		// Sending TargetSpeed from ROBOT_CONTROL to INVERTER
	MOTOR_CALIBRATION				= 0x6F
} IMR_CAN_INVERTER_COMMANDS_t;

typedef enum IMR_CAN_ROBOT_CONTROL_COMMANDS {
	ROBOT_CONTROL_EncoderValue		= 0x0E,		// Sending EncoderValues from INVERTER to ROBOT_CONTROL
	ROBOT_CONTROL_Trajectory		= 0x2A		// Sending TrajectoryCommands from EXTERNAL to ROBOT_CONTROL
} IMR_CAN_ROBOT_CONTROL_COMMANDS_t;

typedef enum IMR_CAN_JETSON_COMMANDS {
	JETSON_Odometrics				= 0x2B,
	JETSON_Wheelspeed_FL			= 0x2C,
	JETSON_Wheelspeed_FR			= 0x2D,
	JETSON_Wheelspeed_BL			= 0x2E,
	JETSON_Wheelspeed_BR			= 0x2F
} IMR_CAN_JETSON_COMMANDS_t;

// All command send and received by the BMS
typedef enum IMR_CAN_BMS_COMMANDS {
	BMS_HS_Handshake 				= 0xB0,
	BMS_HS_Swap 					= 0xB1,
	BMS_HS_Finish 					= 0xB2,
	BMS_ReducePowerForHS			= 0xB3,
	BMS_EnablePowerForHS			= 0xB4,
	BMS_SwitchOn_Veto				= 0xB5,
	BMS_Data_SOC					= 0xB6,
	BMS_Data_Voltage				= 0xB7,
	BMS_Data_Current				= 0xB8,
	BMS_RT_Data_Restart				= 0xB9,
	BMS_RT_Data_GetLines			= 0xC0,
	BMS_RT_Data_SendLinePart		= 0xC1,
	BMS_RT_Data_GetStoredLineNumber	= 0xC2,
	BMS_RT_Data_GetMaxLineNumber	= 0xC3,
	BMS_RT_Data_SetSamplingTime		= 0xC4,
	BMS_RT_Data_GetSamplingTime		= 0xC5
} IMR_CAN_BMS_COMMANDS_t;

typedef enum IMR_CAN_POWER_COMMANDS {
	POWER_CH_Output_Current			= 0xF0,
	POWER_CH_Switch_On 				= 0xF1,
	POWER_CH_Switch_Off				= 0xF2,
	POWER_Reserved4 				= 0xF3
} IMR_CAN_POWER_COMMANDS_t;

typedef struct IMR_CAN_MESSAGE_STRUCT {
	uint8_t CAN_COMMAND;			// Stores the COMMAND to identify the message type per board
	uint8_t CAN_SENDER_ID;			// Stores the senders CAN ID for message identification
	uint8_t CAN_DATA[6];			// Stores the CAN data associated with the corresponding CAN COMMAND
} IMR_CAN_MESSAGE_STRUCT_t;

// ID of certain CAN devices
typedef enum IMR_CAN_CLASS_MASKS {
	ROBOT_CONTROL_CLASS_MASK 		= 0x10,		// Robot Control CAN Logic: (8 ... 1) 0001 XXXX			Mask: 0x10
	INVERTER_CLASS_MASK 			= 0x60,     // Inverter CAN Logic: 		(8 ... 1) 0110 00XX			Mask: 0x60
	SENSOR_LED_CLASS_MASK 			= 0xC0,     // Sensor LED CAN Logic: 	(8 ... 1) 1100 XXXX			Mask: 0xC0
	BMS_CLASS_MASK					= 0x3C,		// BMS CAN Logic:			(8 ... 1) 0011 110X			Mask: 0x3C
	POWER_CLASS_MASK				= 0xA8,		// POWER CAN Logic:			(8 ... 1) 1010 100X			Mask: 0xA8
	JETSON_CAN_ID					= 0x11,		// JETSON ID: 0x11
	TRAJECTORY_CMD_ID				= 0x23,		// Gamepad & autonomous navigation control
	ENCODER_DATA_CLASS_MASK			= 0x14		// Encoder data from motor: (8 ... 1) 0001 01XX
} IMR_CAN_CLASS_MASKS_t;


/*******************************************************************************
* Variables
*******************************************************************************/
extern IMR_CAN_MESSAGE_STRUCT_t IMR_CAN_RX;

extern uint8_t newCanMessageReceived;
extern uint16_t canSendErrorCount;
extern uint16_t canSendSuccessCount;
extern uint8_t receivedCanHandshakeCount;
extern uint8_t receivedCanSwapCount;
extern uint8_t receivedCanFinishCount;
extern uint8_t receivedCanSwitchOnVetoCount;
extern uint8_t receivedCanSwitchOnVetoSoC;
extern uint8_t receivedCanSwitchOnVetoActiveState;

extern uint8_t isCanRequested_RT_Reset;
extern uint8_t isCanRequested_RT_GetLine;
extern uint8_t isCanRequested_RT_GetStoredLineNumber;
extern uint8_t isCanRequested_RT_GetMaxLineNumber;
extern uint8_t isCanRequested_RT_SetSamplingTime;
extern uint8_t isCanRequested_RT_GetSamplingTime;

extern uint16_t CanRequested_LinesStartIndex;
extern uint16_t CanRequested_LinesNumber;
extern uint16_t CanRequested_LinesCurrentIndex;
extern uint16_t CanRequested_LinesCurrentIndexPart;
extern uint32_t CanRequested_SamplingTime;



/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void CAN_IRQ_RX_MESSAGE_HANDLER(bool msg_valid, uint8_t msg_buf_fifo_num, cy_stc_canfd_rx_buffer_t* canfd_rx_buf);
cy_en_canfd_status_t CAN_TX_Request(uint32_t Target_CAN_ID, uint16_t CAN_Command, uint8_t* Target_Data, uint8_t Target_Data_Length);

void set_canfd_ID();
void set_canfd_ID_based_on_slot();
void resetCanMessageCounters();
void CAN_Reset();

cy_en_canfd_status_t init_canfd();
cy_en_canfd_status_t refresh_canfd(void);


#endif /* IMR2_CAN_GLOBAL_H_ */


