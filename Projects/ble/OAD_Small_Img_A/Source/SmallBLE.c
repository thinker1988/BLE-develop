/**************************************************************************************************
	Filename:			smallBLE.c
	Revised:			$Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
	Revision:			$Revision: 23333 $

	Description:		This file contains the Simple BLE Peripheral sample application
					for use with the CC2540 Bluetooth Low Energy Protocol Stack.

	Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

	IMPORTANT: Your use of this Software is limited to those specific rights
	granted under the terms of a software license agreement between the user
	who downloaded the software, his/her employer (which must be your employer)
	and Texas Instruments Incorporated (the "License").	You may not use this
	Software unless you agree to abide by the terms of the License. The License
	limits your use, and you acknowledge, that the Software may not be modified,
	copied or distributed unless embedded on a Texas Instruments microcontroller
	or used solely and exclusively in conjunction with a Texas Instruments radio
	frequency transceiver, which is integrated into your product.	Other than for
	the foregoing purpose, you may not use, reproduce, copy, prepare derivative
	works of, modify, distribute, perform, display or sell this Software and/or
	its documentation for any purpose.

	YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
	PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
	INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
	NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
	TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
	NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
	LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
	INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
	OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
	OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
	(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

	Should you have any questions regarding your right to use this Software,
	contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_lcd.h"

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#if defined FEATURE_OAD
	#include "oad.h"
	#include "oad_target.h"
#endif

#include "UPLBSSerial.h"
#include "smallBLE.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define OAD_DEV_UUID								0xABAB

// What is the advertising interval when device is discoverable (units of 625us, 800=500ms)
#define SML_BLE_ADVERTISING_INTERVAL				800

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define SML_BLE_DESIRED_MIN_CONN_INTERVAL		80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define SML_BLE_DESIRED_MAX_CONN_INTERVAL		800

// Slave latency to use if automatic parameter update request is enabled
#define SML_BLE_DESIRED_SLAVE_LATENCY			0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define SML_BLE_DESIRED_CONN_TIMEOUT				1000

// Whether to enable automatic parameter update request when a connection is formed
#define SML_BLE_ENABLE_UPDATE_REQUEST			FALSE

// LED blink cycle and percentage
#define SMALL_IMG_LED_BLINK_DUTY					30

#define SMALL_IMG_LED_BLINK_CYCLE				1000

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 smallBLE_TaskID;	 // Task ID for internal task/event processing

static gaprole_States_t small_BLE_state = GAPROLE_INIT;

// BLE MAC address
static uint8 small_BLE_MAC[B_ADDR_LEN];

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
	// complete name
	0x0C,	 // length of this data
	GAP_ADTYPE_LOCAL_NAME_COMPLETE,
	'S',
	'M',
	'A',
	'L',
	'L',
	' ',
	'I',
	'M',
	'A',
	'G',
	'E',

	// connection interval range
	0x05,	 // length of this data
	GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
	LO_UINT16( SML_BLE_DESIRED_MIN_CONN_INTERVAL ),	 // 100ms
	HI_UINT16( SML_BLE_DESIRED_MIN_CONN_INTERVAL ),
	LO_UINT16( SML_BLE_DESIRED_MAX_CONN_INTERVAL ),	 // 1s
	HI_UINT16( SML_BLE_DESIRED_MAX_CONN_INTERVAL ),

	// Tx power level
	0x02,	 // length of this data
	GAP_ADTYPE_POWER_LEVEL,
	0			 // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
	// Flags; this sets the device to use limited discoverable
	// mode (advertises for 30 seconds at a time) instead of general
	// discoverable mode (advertises indefinitely)
	0x02,	 // length of this data
	GAP_ADTYPE_FLAGS,
	GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

	// service UUID, to notify central devices what services are included
	// in this peripheral
	0x03,	 // length of this data
	GAP_ADTYPE_16BIT_MORE,			// some of the UUID's, but not all
	LO_UINT16( OAD_DEV_UUID ),
	HI_UINT16( OAD_DEV_UUID),
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "SMALL IMG";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void smallBLEStateNotificationCB( gaprole_States_t newState );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t smallBLE_PeripheralCBs =
{
	smallBLEStateNotificationCB,	// Profile State Change Callbacks
	NULL							// When a valid RSSI is read from controller (not used by application)
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn			SmallBLE_Init
 *
 * @brief	 Initialization function for the Simple BLE Peripheral App Task.
 *					This is called during initialization and should contain
 *					any application specific initialization (ie. hardware
 *					initialization/setup, table initialization, power up
 *					notificaiton ... ).
 *
 * @param	 task_id - the ID assigned by OSAL.	This ID should be
 *										used to send messages and set timers.
 *
 * @return	none
 */
void SmallBLE_Init( uint8 task_id )
{
	smallBLE_TaskID = task_id;

	// Setup the GAP Peripheral Role Profile
	{		
		uint8 initial_advertising_enable = TRUE;

		// By setting this to zero, the device will go into the waiting state after
		// being discoverable for 30.72 second, and will not being advertising again
		// until the enabler is set back to TRUE
		uint16 gapRole_AdvertOffTime = 0;

		uint8 enable_update_request = SML_BLE_ENABLE_UPDATE_REQUEST;
		uint16 desired_min_interval = SML_BLE_DESIRED_MIN_CONN_INTERVAL;
		uint16 desired_max_interval = SML_BLE_DESIRED_MAX_CONN_INTERVAL;
		uint16 desired_slave_latency = SML_BLE_DESIRED_SLAVE_LATENCY;
		uint16 desired_conn_timeout = SML_BLE_DESIRED_CONN_TIMEOUT;

		// Set the GAP Role Parameters
		GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
		GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

		GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
		GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

		GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
		GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
		GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
		GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
		GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
	}

	// Set the GAP Characteristics
	uint8 ename = TRUE;
	GGS_SetParameter( GGS_W_PERMIT_DEVICE_NAME_ATT, 1,&ename );
	
	GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

	// Set advertising interval
	{
		uint16 advInt = SML_BLE_ADVERTISING_INTERVAL;

		GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
		GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
		GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
		GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
	}

	// Initialize GATT attributes
	GGS_AddService( GATT_ALL_SERVICES );				// GAP
	GATTServApp_AddService( GATT_ALL_SERVICES );		// GATT attributes
#if defined FEATURE_OAD
	VOID OADTarget_AddService();						// OAD Profile
#endif

	// Enable clock divide on halt
	// This reduces active current while radio is active and CC254x MCU is halted
	HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )
	// Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
	HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );
#endif // defined ( DC_DC_P0_7 )

	HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF);
	UPLBS_Serial_Init();

	// Setup a delayed profile startup
	osal_set_event( smallBLE_TaskID, SMALL_BLE_START_DEVICE_EVT  );
}

/*********************************************************************
 * @fn			SmallBLE_ProcessEvent
 *
 * @brief	 Simple BLE Peripheral Application Task event processor.	This function
 *					is called to process all events for the task.	Events
 *					include timers, messages and any other user defined events.
 *
 * @param	 task_id	- The OSAL assigned task ID.
 * @param	 events - events to process.	This is a bit map and can
 *									 contain more than one event.
 *
 * @return	events not processed
 */
uint16 SmallBLE_ProcessEvent( uint8 task_id, uint16 events )
{
	VOID task_id; // OSAL required parameter that isn't used in this function

	if ( events & SYS_EVENT_MSG )
	{
		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}

	if ( events & SMALL_BLE_START_DEVICE_EVT )
	{
		PrintString("\r\nSSS");
		// Start the Device
		VOID GAPRole_StartDevice( &smallBLE_PeripheralCBs );		
		osal_start_reload_timer( smallBLE_TaskID, SMALL_BLE_LED_BLINK_EVT, SMALL_IMG_LED_BLINK_CYCLE);

		return ( events ^ SMALL_BLE_START_DEVICE_EVT);
	}

	if ( events & SMALL_BLE_LED_BLINK_EVT)
	{
		HalLedBlink(HAL_LED_ALL,1,SMALL_IMG_LED_BLINK_DUTY,SMALL_IMG_LED_BLINK_CYCLE);
		
		return (events ^ SMALL_BLE_LED_BLINK_EVT);
	}

	return 0;
}

/*********************************************************************
 * @fn			smallBLEStateNotificationCB
 *
 * @brief	 Notification from the profile of a state change.
 *
 * @param	 newState - new state
 *
 * @return	none
 */
static void smallBLEStateNotificationCB( gaprole_States_t newState )
{
	switch ( newState )
	{
		case GAPROLE_STARTED:
		{
			GAPRole_GetParameter(GAPROLE_BD_ADDR, small_BLE_MAC);
			break;
		}

		case GAPROLE_ADVERTISING:
			break;

		case GAPROLE_CONNECTED:
			break;

		case GAPROLE_WAITING:
			break;

		case GAPROLE_WAITING_AFTER_TIMEOUT:
			break;

		case GAPROLE_ERROR:
			break;

		default:
			break;

	}

	small_BLE_state = newState;

	VOID small_BLE_state;
}

/*********************************************************************
 * @fn		serial_data_proccess
 *
 * @brief		process serial read data
 *
 * @return	none
 */
void serial_data_proccess(uint8 type, uint8 *buffer, uint16 length)
{
#if (defined HAL_UART) && (HAL_UART == TRUE)
	uint8 cmd_status = CMD_SUCCESS;

	switch(SET_OUTPUT_TYPE(type))
	{
		case BLE_DEV_ADDR:
		{
			LBSsend(BLE_DEV_ADDR,small_BLE_MAC,sizeof(small_BLE_MAC));
			break;
		}
		case BLE_DEV_VERSION:
			break;
			
		case BLE_DEV_SET_PARAMS:
			break;
			
		case BLE_DEV_GET_PARAMS:
			break;
			
		case BLE_DEV_CAR_SCAN_DATA:
			break;
			
		case BLE_DEV_START_SCAN:
			break;
			
		case BLE_DEV_SCAN_RSLT:
			break;
			
		case BLE_DEV_UPGRADE:
		case BLE_DEV_REBOOT:
		{
			HAL_SYSTEM_RESET();
			break;
		}
		default:
			break;
	}
	
	VOID cmd_status;
#endif	/* HAL_UART && HAL_UART == TRUE */
};


/*********************************************************************
*********************************************************************/
