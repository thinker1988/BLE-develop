/********************************************************************
	Filename:	BLECore.c
	Revised:		Date: 2015-08-12
	Revision:	1.0 

*********************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"

#include "hal_led.h"
#include "hal_lcd.h"

#include "BLECore.h"
#include "Com433.h"
#include "GMProc.h"
#include "Pktfmt.h"
#include "Cc112x.h"

#if ( defined USE_BLE_STACK && defined ALLOW_BLE_ADV )
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "peripheral.h"
#endif	// USE_BLE_STACK && ALLOW_BLE_ADV




/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
 
/********** BLE peripheral params define**********/
#if ( defined USE_BLE_STACK && defined ALLOW_BLE_ADV )
// Adv type
#define BLE_CORE_DEV_TYPE			0xC9C9

// Adv interval (*0.625ms)
#define BLE_CORE_ADV_INTVL			8000		//(units of 625us, 8000*0.625

// Min connect event interval (*1.25ms)
#define BLE_CORE_MIN_CON_INTVL		80

// Max connect event interval (*1.25ms)
#define BLE_CORE_MAX_CON_INTVL		80

// Slave latency = slave ignore event times
#define BLE_CORE_SLAVE_LATENCY		0

// Connection time out (*10ms)
#define BLE_CORE_CON_TIMEOUT		50

// Enable connection parameter update
#define BLE_CORE_UPDATE_FLAG		TRUE

// Begin update parameters after connection (*1s)
#define BLE_CORE_UPDATE_PAUSE		1
#endif	// USE_BLE_STACK && ALLOW_BLE_ADV

/**********Other params define**********/
// BLE MAC store address in FLASH
#define BLE_MAC_ADDR_IN_FLASH		0x780E

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID for internal task/event processing
uint8 BLECore_TaskId;


/*********************************************************************
 * EXTERNAL VARIABLES
 */

#if ( !defined USE_CC112X_RF )
// TEN RF send data buf & length
extern uint8 rfsndbuf[];
extern uint8 rfsndlen;

#endif	// ! USE_CC112X_RF

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Random seeds
static uint32 next=1;

// Device state refer to desiged state machine
static sysstate_t Devstate = SYS_BOOTUP;

// 
static wsintstate_t wsint = WS_INT_DISABLE;

static bool sleepflag = FALSE;

// Already in work mode flag, use this flag to avoid RF setting during waiting IDLE_PWR_HOLD_PERIOD
static bool workflg = FALSE;

// Already in setup mode flag, use this flag to stay in setup mode during NO_OPERATION_WAIT_PERIOD
static bool setupflg = FALSE;

#if ( defined USE_BLE_STACK && defined ALLOW_BLE_ADV )
// Advertisement data (max size = 31 bytes, though this is best kept short to conserve power while advertisting)
static uint8 BLECoreAdvData[] =
{
	// Flags; this sets the device to use limited discoverable mode (advertises for 30 seconds at a time),
	// instead of general discoverable mode (advertises indefinitely)
	0x02,	 // length of this data
	GAP_ADTYPE_FLAGS,
	GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
	
	// service UUID, to notify central devices what services are included in this peripheral
	0x03,	// length of this data
	GAP_ADTYPE_16BIT_MORE,			//special UUID the same with entry device filtering UUID 
	LO_UINT16(BLE_CORE_DEV_TYPE),
	HI_UINT16(BLE_CORE_DEV_TYPE),
};

// Scan response data (max size = 31 bytes)
static uint8 BLECoreScanRespData[] =
{
	// complete name
	0x0B,	// length of this data
	GAP_ADTYPE_LOCAL_NAME_COMPLETE,
	'G','D','E','-','D','E','V','-','2','3',

	// connection interval range
	0x05,	// length of this data
	GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
	LO_UINT16( BLE_CORE_MIN_CON_INTVL ),
	HI_UINT16( BLE_CORE_MIN_CON_INTVL ),
	LO_UINT16( BLE_CORE_MAX_CON_INTVL ),
	HI_UINT16( BLE_CORE_MAX_CON_INTVL ),
};
#endif	// USE_BLE_STACK && ALLOW_BLE_ADV

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void StopAllTimer(uint8 task_id);

static void SYS_WS_INT_PIN_Enable(void);
static void SYS_WS_INT_PIN_Disable(void);

static void SetSysStateByFlag(uint8 task_id, bool slpflg);


static void c_srand(uint32 seed);
static uint32 c_rand(void);

static void ReadBLEMac(uint8 * mac_addr);

#if ( defined USE_BLE_STACK && defined ALLOW_BLE_ADV )
static void init_gap_periph_role(void);
static void init_gap_periph_params(void);
static void BLECorePeriphNotiCB(gaprole_States_t newState);
#endif	// USE_BLE_STACK && ALLOW_BLE_ADV

/*********************************************************************
 * PROFILE CALLBACKS
 */
// BLE peripheral profile callbacks
#if ( defined USE_BLE_STACK && defined ALLOW_BLE_ADV )
static gapRolesCBs_t BLECore_PeriphCBs =
{
	BLECorePeriphNotiCB,	// Profile State Change Callbacks
	NULL					// When a valid RSSI is read from controller (not used by application)
};
#endif	// USE_BLE_STACK && ALLOW_BLE_ADV

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn		BLECore_Init
 *
 * @brief	Initialization function for the BLECore app task.
 *
 * @param	task_id - the ID assigned by OSAL. This ID should be used to send messages and set timers.
 *
 * @return	none
 */
void BLECore_Init( uint8 task_id )
{
	uint8 BleMac[B_ADDR_LEN];
	BLECore_TaskId = task_id;

	// Init debug port and RF port
	Com433_Init();

	// Read reverse order MAC address into buffer, use it as random seed
	ReadBLEMac(BleMac);
	c_srand(BUILD_UINT32(BleMac[0],BleMac[1],BleMac[2],BleMac[3]));
	
#if ( defined USE_BLE_STACK && defined ALLOW_BLE_ADV )
	HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_23_DBM);

	init_gap_periph_role();
	init_gap_periph_params();
	VOID GAPRole_StartDevice( &BLECore_PeriphCBs );
#endif	// USE_BLE_STACK & ALLOW_BLE_ADV

	// Start working
	osal_set_event( task_id, BLE_SYS_WORKING_EVT );

	return;
}

/*********************************************************************
 * @fn		BLECore_ProcessEvent
 *
 * @brief	BLECore task event processor.	This function is called to process all events for the task.
 *				Events include timers, messages and any other user defined events.
 *
 * @param	task_id - The OSAL assigned task ID.
 * @param	events - events to process. This is a bit map and can contain more than one event.
 *
 * @return	events not processed
 */
uint16 BLECore_ProcessEvent( uint8 task_id, uint16 events )
{ 
	VOID task_id; // OSAL required parameter that isn't used in this function
	
	if ( events & SYS_EVENT_MSG )
	{
		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}

	if ( events & BLE_SYS_WORKING_EVT )
	{
		sys_working(task_id, GetSysState());

		return ( events ^ BLE_SYS_WORKING_EVT );
	}

	if ( events & RF_DATA_PROC_EVT)
	{
		RF_working(task_id, GetRFstate());

		return (events ^ RF_DATA_PROC_EVT);
	}

	if ( events & GM_DATA_PROC_EVT)
	{
		GM_working(task_id, GetGMSnState());

		return (events ^ GM_DATA_PROC_EVT);
	}

	
	if (events & HG_SWITCH_EVT)
	{
		SYS_WS_INT_Cfg(task_id, GetIntState());

		return (events ^ HG_SWITCH_EVT);
	}

	return 0;
}


/*********************************************************************
 * @fn		sys_working
 *
 * @brief	Function works like a main thread which controls system working process
 *				and main state machines.
 *
 * @param	task_id - The OSAL assigned task ID, like progress ID.
 * @param	newDevstate -New system state machine.
 *
 * @return	none
 *
 */
void sys_working(uint8 task_id, sysstate_t newDevstate)
{
	Devstate = newDevstate;

	switch (newDevstate)
	{
		case SYS_BOOTUP:
			SYS_WS_INT_Cfg(task_id, WS_INT_DISABLE);
			// Do not enter power saving
			PowerHold(task_id);
			RF_working(task_id, RF_PRESET);
#if ( !defined GME_WORKING )
			GM_working(task_id, GMSnTest);
#endif	// !GME_WORKING
			break;
		case SYS_WORKING:
			if (workflg == FALSE)
			{
				workflg = TRUE;
				PowerHold(task_id);
				RF_working(task_id, GetRFstate());
				osal_start_timerEx(task_id, BLE_SYS_WORKING_EVT, IDLE_PWR_HOLD_PERIOD);
			}
			else
			{
				workflg = FALSE;
				SetSysState(SYS_SLEEPING);
				osal_set_event(task_id, BLE_SYS_WORKING_EVT);
			}
			break;
		case SYS_SLEEPING:
			RF_working(task_id, RF_SLEEP);
			PowerSave(task_id);
			// First start GM data normal read, use random delay
			if (osal_get_timeoutEx(task_id, GM_DATA_PROC_EVT) == 0)
				osal_start_timerEx(task_id, GM_DATA_PROC_EVT, c_rand()*(GM_READ_EVT_PERIOD/MILSEC_IN_SEC)/MAX_RANDOM_SECONDS);
			break;
		case SYS_DORMANT:
			RF_working(task_id, RF_SLEEP);
			GM_working(task_id, GMSnSleep);
			StopAllTimer(task_id);
			PowerSave(task_id);
			break;
		case SYS_SETUP:
			if (setupflg == TRUE)
			{
				setupflg = FALSE;
				InitDevID();
				SetSysState(SYS_WORKING);
				SetRFstate(RF_PRESET);
				osal_set_event(task_id, BLE_SYS_WORKING_EVT);
			}
			else
			{
				setupflg = TRUE;
				StopAllTimer(task_id);
				// Send presetup data
				RF_working(task_id, RF_PRESET);
				osal_start_timerEx(task_id, BLE_SYS_WORKING_EVT, NO_OPERATION_WAIT_PERIOD);
			}
		case SYS_UPGRADE:
		default:
			break;
	}
}

/*********************************************************************
 * @fn		SYS_WS_INT_PIN_Enable
 *
 * @brief	Enable working state interrupt.
 *
 * @param	none.
 *
 * @return	none
 */
void SYS_WS_INT_Cfg(uint8 task_id, wsintstate_t curintst)
{
	wsint = curintst;
	switch (wsint)
	{
		case WS_INT_DISABLE:
			SET_P0_INT_DISABLE();
			DEV_EN_INT_DISABLE();
			SYS_WS_INT_PIN_Disable();
			break;
		case WS_INT_DETECT:
			SET_P0_INT_DISABLE();
			DEV_EN_INT_DISABLE();

			SYS_WS_INT_PIN_Enable();
			// Low indicate inverted.
			sleepflag = ( DEV_EN_INT_PIN == 0 ? TRUE: FALSE);
			wsint = WS_INT_CONFIRM;
			osal_start_timerEx(task_id, HG_SWITCH_EVT, SWITCH_WAIT_SHAKE_PERIOD);
			break;
		case WS_INT_CONFIRM:
			wsint = WS_INT_DETECT;
			if ((DEV_EN_INT_PIN == 0 && sleepflag == TRUE) || (DEV_EN_INT_PIN !=0  && sleepflag == FALSE))
			{
				SetSysStateByFlag(task_id, sleepflag);
			}
			else
			{
				osal_set_event(task_id, HG_SWITCH_EVT);
			}
			break;
		default:
			break;
	}

	return;
}

void SetSysState( sysstate_t newDevstate)
{
	Devstate = newDevstate;
}

sysstate_t GetSysState( void )
{
	return Devstate;
}

void SetIntState( wsintstate_t newintstate)
{
	wsint = newintstate;
}

wsintstate_t GetIntState( void )
{
	return wsint;
}


/*********************************************************************
 * @fn		PowerSave
 *
 * @brief		Set system power mode to PM3
 *
 * @param	none
 * @return	none
 */
void PowerSave(uint8 task_id)
{
#if ( defined POWER_SAVING )
	(void)osal_pwrmgr_task_state(task_id, PWRMGR_CONSERVE);
	osal_pwrmgr_device( PWRMGR_BATTERY );
#endif
	VOID task_id;
}

/*********************************************************************
 * @fn		PowerHold
 *
 * @brief	Hold system power
 *
 * @param	task_id - Task to hold power
 *
 * @return	none
 */
void PowerHold(uint8 task_id)
{
#if ( defined POWER_SAVING )
	osal_pwrmgr_device( PWRMGR_ALWAYS_ON);
	(void)osal_pwrmgr_task_state(task_id, PWRMGR_HOLD);
#endif
	VOID task_id;
}


/*********************************************************************
 * @fn		Com433Handle
 *
 * @brief	COM port call back function
 *
 * @param	port - serial port.
 * @param	pBuffer - read buffer.
 * @param	length - read buffer length.
 *
 * @return	none.
 */
void Com433Handle(uint8 port,uint8 *pBuffer, uint16 length)
{
	// No func will be called in CC112X RF from working port (recieve data by interrupt)
	if (port == COM433_WORKING_PORT)
	{

#if ( !defined USE_CC112X_RF )

#if ( defined TEN_DEBUG_MODE )
		// Print serial data directly in debug mode
		Com433Write(COM433_DEBUG_PORT, pBuffer, length);
#else	// !TEN_DEBUG_MODE
		GMSPktForm(pBuffer,length);
#endif	// TEN_DEBUG_MODE

#endif	// !USE_CC112X_RF

	}
	else if (port == COM433_DEBUG_PORT)
	{
#if ( defined USE_CC112X_RF && defined GME_WORKING )
		GMSPktForm(pBuffer, length);
#elif ( !defined USE_CC112X_RF && defined TEN_DEBUG_MODE )
		Com433Write(COM433_WORKING_PORT, pBuffer, length);
#endif	// USE_CC112X_RF && GME_WORKING
	}

	return;
}

/*********************************************************************
 * PRIVATE FUNCTIONS
 */

/*********************************************************************
 * @fn		StopAllTimer
 *
 * @brief	Stop all timer in given task ID
 *
 * @param	task_id - task ID of timer to stop.
 *
 * @return	none
 */
static void StopAllTimer(uint8 task_id)
{
	//osal_start_timerEx(task_id, BLE_SYS_WORKING_EVT, MIN_IN_HOUR*MILSEC_IN_MIN);
	osal_stop_timerEx(task_id, RF_DATA_PROC_EVT);
	osal_stop_timerEx(task_id, GM_DATA_PROC_EVT);
}

/*********************************************************************
 * @fn		SYS_WS_INT_PIN_Enable
 *
 * @brief	Enable working state interrupt.
 *
 * @param	none.
 *
 * @return	none
 */
static void SYS_WS_INT_PIN_Enable(void)
{
	DEV_EN_INT_PINSEL &= (uint8) ~DEV_EN_INT_IE;
	DEV_EN_INT_PINDIR &= (uint8) ~DEV_EN_INT_IE;
	DEV_EN_INT_PININP &= (uint8) ~DEV_EN_INT_IE;
}

/*********************************************************************
 * @fn		SYS_WS_INT_PIN_Disable
 *
 * @brief	Disable working state interrupt.
 *
 * @param	none.
 *
 * @return	none
 */
static void SYS_WS_INT_PIN_Disable(void)
{
	DEV_EN_INT_PINSEL &= (uint8) ~DEV_EN_INT_IE;
	DEV_EN_INT_PINDIR &= (uint8) ~DEV_EN_INT_IE;
	DEV_EN_INT_PININP |= (uint8) DEV_EN_INT_IE;
}

static void SetSysStateByFlag(uint8 task_id, bool slpflg)
{
	bool changestateflag = FALSE;

	if (slpflg == TRUE)
		SET_P0_INT_RISING_EDGE();	// Confirm inverted, start rising edge detect
	else
		SET_P0_INT_FALLING_EDGE();

	switch (GetSysState())
	{
		case SYS_BOOTUP:
			// system begin working
			if (slpflg == FALSE)
			{
				SetRFstate(RF_PRESET);
				SetSysState(SYS_WORKING);
			}
			else
			{
				SetSysState(SYS_DORMANT);
			}
			changestateflag = TRUE;
			break;
		case SYS_WORKING:
		case SYS_SLEEPING:
			if (slpflg == TRUE )
			{
				SetSysState(SYS_DORMANT);
				changestateflag = TRUE;
			}
			break;
		case SYS_DORMANT:
			if (slpflg == FALSE )
			{
				SetSysState(SYS_BOOTUP);
				changestateflag = TRUE;
			}
			break;
		case SYS_SETUP:
		case SYS_UPGRADE:
			break;
	}

	if (changestateflag == TRUE)
		osal_set_event(task_id, BLE_SYS_WORKING_EVT);

	if (GetSysState() != SYS_BOOTUP)
	{
		SET_P0_INT_ENABLE();
		DEV_EN_INT_ENABLE();
	}

}

/*********************************************************************
 * @fn		c_srand
 *
 * @brief	Set random seed
 *
 * @param	seed - random seed
 *
 * @return	none
 */
static void c_srand(uint32 seed)
{
	next=seed;
}

/*********************************************************************
 * @fn		c_rand
 *
 * @brief	c_rand and c_srand to calc random number
 *
 * @param	none
 *
 * @return	1~32767
 */
static uint32 c_rand(void)
{
	next=next*1103515245+12345;  
	return (uint32)(next/65536)%32768;  
}


/*********************************************************************
 * @fn		ReadBLEMac
 *
 * @brief	Read reverse own mac address in flash. Use *(unsigned char *) to read address
 *
 * @param	none
 *
 * @return	none
 */
static void ReadBLEMac(uint8 *mac_addr)  
{
	uint8 i;
	for (i=0;i<B_ADDR_LEN; i++)
		mac_addr[i]=XREG(BLE_MAC_ADDR_IN_FLASH+i);
	
	return ;  
}

#if ( defined USE_BLE_STACK && defined ALLOW_BLE_ADV )

/*********************************************************************
 * @fn		init_gap_peripheral_role
 *
 * @brief		Set gap peripheral role.
 *
 * @param	none
 *
 * @return	none
 */
static void init_gap_periph_role(void)
{
	uint8 initial_advertising_enable = TRUE;
	uint16 gapRole_AdvertOffTime = BLE_CORE_ADV_INTVL;
	uint8 enable_update_request = BLE_CORE_UPDATE_FLAG;
	uint16 desired_min_interval = BLE_CORE_MIN_CON_INTVL;
	uint16 desired_max_interval = BLE_CORE_MAX_CON_INTVL;
	uint16 desired_slave_latency = BLE_CORE_SLAVE_LATENCY;
	uint16 desired_conn_timeout = BLE_CORE_CON_TIMEOUT;

	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
	GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
	GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
	GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
	GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
	GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
	GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );

	GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( BLECoreAdvData ), BLECoreAdvData );

	GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( BLECoreScanRespData ), BLECoreScanRespData );
}

/*********************************************************************
 * @fn		init_gap_peripheral_params
 *
 * @brief		Init peripheral advertising interval.
 *
 * @param	none
 *
 * @return	none
 */
static void init_gap_periph_params(void)
{
	// Set advertising interval
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, BLE_CORE_ADV_INTVL );
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, BLE_CORE_ADV_INTVL );

	// Set parameter updates waiting time
	GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, BLE_CORE_UPDATE_PAUSE );
}


/*********************************************************************
 * @fn		BLECorePeriphNotiCB
 *
 * @brief	Notification from the profile of a state change.
 *
 * @param	newState - new state
 *
 * @return	none
 */
static void BLECorePeriphNotiCB( gaprole_States_t newState )
{
	switch ( newState )
	{
		case GAPROLE_STARTED:
		case GAPROLE_ADVERTISING:
		case GAPROLE_CONNECTED:
		case GAPROLE_WAITING:
		case GAPROLE_WAITING_AFTER_TIMEOUT:
		case GAPROLE_ERROR:
		{
			break;
		}
		default:
			break;
	} 
}
#endif	// USE_BLE_STACK && ALLOW_BLE_ADV


/*********************************************************************
 * ISR function
 */
#if ( !defined HW_VERN ) || ( HW_VERN == 0 )
HAL_ISR_FUNCTION(GMDRDYIsr, P1INT_VECTOR)
#elif ( HW_VERN >= 1 )
HAL_ISR_FUNCTION(GMDRDYIsr, P0INT_VECTOR)
#endif
{
	HAL_ENTER_ISR();

#if 0
	if ((GM_DRDY_INT_PXIFG & GM_DRDY_INT_IE) && (GM_DRDY_INT_PXIEN & GM_DRDY_INT_IE))
		osal_set_event(BLECore_TaskId, GM_DRDY_INT_INT_EVT);
#endif

#if ( HW_VERN >= 1 )
	if ((DEV_EN_INT_PXIFG & DEV_EN_INT_IE) && (DEV_EN_INT_PXIEN & DEV_EN_INT_IE))
		osal_set_event(BLECore_TaskId, HG_SWITCH_EVT);
#endif

	DEV_EN_INT_PXIFG = 0;
	DEV_EN_INT_PXIF = 0;
#if 0
	// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
	GM_DRDY_INT_PXIFG = 0;
	GM_DRDY_INT_PXIF = 0;
#endif
	CLEAR_SLEEP_MODE();
	
	HAL_EXIT_ISR();
}
