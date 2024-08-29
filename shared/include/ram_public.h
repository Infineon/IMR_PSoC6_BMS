/*
 * ram_public.h - Define public variables used by CM0+ and CM4
 *
 *  Created on: 24.07.2023
 *      Author: r. santeler
 */

#ifndef SHARED_INCLUDE_RAM_PUBLIC_H_
#define SHARED_INCLUDE_RAM_PUBLIC_H_

// Include diagnostic enumerations and definitions
#include "../../proj_cm4/diagnostic.h"

/* Enumerations used only for public variables*/
typedef enum {OUTPUT_OFF = 0U, OUTPUT_READY, OUTPUT_PRECHARGE, OUTPUT_ON, OUTPUT_OFF_TILL_RESTART} outputStates;
typedef enum {SLOT_NONE = 0U, SLOT_IN_SYSTEM_1, SLOT_IN_SYSTEM_2, SLOT_IN_CHARGER} slotStates;
typedef enum {PUBLIC_COMMAND_IGNORE = 0, PUBLIC_COMMAND_REFRESH , PUBLIC_COMMAND_SHUTDOWN, PUBLIC_COMMAND_DONE} public_core_commands;


/* The size of the Cortex-M0+ application image at the start of FLASH */
#define RAM_CM0P_SIZE 0x20000
/* Size of unprotected public RAM to be reserved */
#define RAM_PUBLIC_SIZE 0x1000
/* Origin of unprotected public RAM */
#define PUB_RAM_ORIGIN (0x08000000 + RAM_CM0P_SIZE)

#define NUM_CELLS 12

// Address definition of every element in public ram. The size of the last element must always be the start of the next one (size must stack up)
// To add a new element:
//    - Create a new line below last element, copy last element address macro down and change type in sizeof to new element type:
//      #define PUBX_VARXX 	(PUBX-1_VARXX + sizeof(PUBX_VARXX_TYPE))
//    - Initialize the element in one of the main.c files like this:
//      PUB_RAM_ACCESS(uint16_t, PUB4_NUM16) = (uint16_t)(65533);
//	  - Use PUB_RAM_ACCESS Macro to read and write variable:
//      PUB_RAM_ACCESS(PUBX_VARXX_TYPE, PUBX_VARXX)
#define PUB1_ERROR_MAP		(PUB_RAM_ORIGIN)
#define PUB2_SOC 			(PUB1_ERROR_MAP	   + sizeof(uint32_t) 	 )
#define PUB3_SOH 			(PUB2_SOC 		   + sizeof(uint8_t)	 )
#define PUB4_VOLT 			(PUB3_SOH 		   + sizeof(uint8_t)	 )
#define PUB5_CURRENT		(PUB4_VOLT 		   + sizeof(uint16_t)	 )
#define PUB6_OUTPUT_STATE	(PUB5_CURRENT      + sizeof(uint16_t)	 )
#define PUB7_SLOT_STATE		(PUB6_OUTPUT_STATE + sizeof(outputStates))
#define PUB8_CELL_V			(PUB7_SLOT_STATE   + sizeof(slotStates)  )
#define PUB8_CELL_V_IDX		(PUB8_CELL_V   	   + sizeof(uint16_t)	 )
#define PUB9_COMMAND 		(PUB8_CELL_V_IDX   + sizeof(uint8_t)	 )

// Macro function to get or set value in public RAM
// 		Read  access:	uint16_t test = PUB_RAM_ACCESS(uint16_t, PUB4_NUM16);
// 		Write access:	PUB_RAM_ACCESS(uint16_t, PUB4_NUM16) = (uint16_t)(65535);
#define PUB_RAM_ACCESS(type, addr) 		        (*((type *)(addr)))
//#define PUB_RAM_ACCESS_ARRAY(type, addr, index)  (((type *)(addr))[index])

#endif /* SHARED_INCLUDE_RAM_PUBLIC_H_ */
