/**************************************************************************************************
	Filename:			 UPLBSApp.h
	Revised:				$Date: 2015-02-02$
	Revision:			 $Revision: 1 $
	Author:				 LAN Chen
	
	Description:		This file contains the Cardroid application
									definitions and prototypes.

**************************************************************************************************/

#ifndef UPLBSAPP_H
#define UPLBSAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_aes.h"
#include "hal_types.h"

/*********************************************************************
 * MACROS
 */
//Set all GPIO into 3-state
#define DEV_ALL_GPIO_INPUT_3STATE() st( P0SEL &= 0;	P0DIR &= 0; P0INP |= 0xFF;\
										P1SEL &= 0; P1DIR &= 0; P1INP |= 0xFF; P2INP |= 0xFF;)

// Set GPIO into 3-state
#define DEV_GPIO_P0_INPUT_3STATE(pins) st( P0SEL &= ~(pins); P0DIR &= ~(pins); \
										P0INP |= (pins);)
													
#define DEV_GPIO_P1_INPUT_3STATE(pins) st( P1SEL &= ~(pins); P1DIR &= ~(pins); \
										P1INP |= (pins);)
													
// Set device status
#define SET_DEV_STAT(frq, pwr)		((uint8) ((frq)<<2 | (pwr)&0x03 ))

// Modify device frequency
#define MODIFY_DEV_ADV_FRQ(stat,frq)	((uint8) ((stat)&0x03 | (frq)<<2))

// Modify device power
#define MODIFY_POWER_LEVEL(stat,pwr)	((uint8) ((stat)&0xFC | (pwr)&0x03))

// Get device tx frequency
#define GET_ADV_FRQ(stat)			((uint8)((stat) >> 2))

// Get device tx power
#define GET_POWER_LEVEL(stat)		((uint8)((stat) & 0x03))


/*********************************************************************
 * CONSTANTS
 */
// APP Task Events
//Common events
#define DEV_WORKING_EVT						0x0001
#define AUTO_DISCONNECT_EVT					0x0040

#if ( defined BLE_BEACON )
#define START_MANUAL_ADV_EVT					0x0002
#define BAT_LOW_VOLT_CHECK_EVT				0x0004
#elif ( defined CARDROID )
#define INT_PROCESS_EVT						0x0002
#define START_BLE_SCAN_EVT					0x0004
#define SWITCH_GAP_ROLE_EVT					0x0008
#define RESTORE_MOVING_CHECK_EVT				0x0010
#define BAT_LOW_VOLT_CHECK_EVT				0x0020
#define CHARGE_DETECT_EVT						0x0040
#define UART_WAKE_UP_EVT						0x0080
#else
#define START_BLE_SCAN_EVT					0x0002
#endif	/* BLE_BEACON */

#define PERFORM_PERODIC_EVT					0x0100


//Information position in advertisement data
#define UUID_LOW_BIT_POS_IN_ADV_DATA			5
#define PROTO_VERN_POS_IN_ADV_DATA			11
#define DEVICE_STATUS_POS_IN_ADV_DATA		14
#define BAT_PERCENT_POS_IN_ADV_DATA			17
#define BEACON_TYPE_POS_IN_ADV_DATA			20
#define POSITION_STAT_POS_IN_ADV_DATA		20

enum
{
	BEACON_DEV,
	CARDROID_DEV,
	IO_DETECT_DEV,
	BASE_DEV,
	CARD_DEV
};
/*********************************************************************
 * FUNCTIONS
 */

// Task Initialization for the BLE Application
extern void UPLBSApp_Init( uint8 task_id );

// Task Event Processor for the BLE Application
extern uint16 UPLBSApp_ProcessEvent( uint8 task_id, uint16 events );

void set_GAP_state(uint8 set_role);
uint8 get_GAP_state(void);

void set_GAP_init_task(uint8 task_id);

/*
void sys_sleep_ms(uint32 val);
bool get_sleep_state(void);*/
#ifdef __cplusplus
}
#endif

#endif /* UPLBSAPP_H */
