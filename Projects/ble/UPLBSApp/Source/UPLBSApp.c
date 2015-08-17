/**************************************************************************************************
	Filename:			UPLBSApp.c
	Revised:			$Date: 2015-02-02$
	Revision:			$Revision: 1 $
	Author:			LAN Chen


	Description:		This file contains the UPLBS device application 
					for use with the CC2541 Bluetooth Low Energy Protocol Stack.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "ll.h"
#include "hci.h"

#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

#include "hal_crc.h"
#include "hal_types.h"
#include "hal_led.h"

#if ( defined CARDROID ) || ( defined IO_DETECT ) || ( defined BLE_BASE )
#include "central.h"
#endif	/* CARDROID || IO_DETECT || BLE_BASE */

#if ( defined CARDROID ) || ( defined BLE_BEACON )
#include "peripheral.h"
#include "hal_adc.h"
#endif	/* CARDROID ||BLE_BEACON */

#include "UPLBSApp.h"
#include "UPLBSAcc.h"
#include "UPLBSProfile.h"
#include "UPLBSUpgrade.h"
#include "UPLBSSerial.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
/*************UUID, ADV & STATUS PARAMS START****************/
// Beacon device type
#define BEACON_DEV_TYPE						0xD000

//CARDROID device type
#define CARDROID_DEV_TYPE						0xB000

enum
{
	UPLBS_BLE_STATE_IDLE,
	UPLBS_BLE_STATE_SCANNING,
	UPLBS_BLE_STATE_CONNECTING,
	UPLBS_BLE_STATE_CONNECTED,
	UPLBS_BLE_STATE_DISCONNECTING,
	UPLBS_BLE_STATE_ADVERTISING
};

/**************UUID, ADV & STATUS PARAMS ENDS***************/

#define AUTO_DISCONNECT_TIME					5000

/***************PERIPHERAL PARAMS START*******************/
#if ( defined CARDROID ) || (defined BLE_BEACON)
// Device default type and adv params
#if ( defined BLE_BEACON )
#define DEV_DEFAULT_TYPE						BEACON_DEV_TYPE
#define DEV_DEFAULT_ADV_ENABLE				FALSE
#define DEV_ADVERTISING_INTERVAL				8000		//(units of 625us, 8000*0.625ms=5s)
#elif ( defined CARDROID )
#define DEV_DEFAULT_TYPE						CARDROID_DEV_TYPE
#define DEV_DEFAULT_ADV_ENABLE				TRUE
#define DEV_ADVERTISING_INTERVAL				4800		//(units of 625us, 4800*0.625ms=3s)
#endif	/* BLE_BEACON */

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEV_DISCOVERABLE_MODE					GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED

// Device default logic address
#define DEV_DEFAULT_ADDR						0

// GAP DATA Type
#define GAP_PROTOCOL_VERSION					0x20
#define GAP_DEVICE_STATUS						0x21
#define GAP_BATTERY_PERCENT					0x22
#define GAP_BASE_STATION_TYPE					0x23
#define GAP_POSITION_STATE					0x24

// Connection handle
#define INVALID_CONNHANDLE					0xFFFF

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEV_DESIRED_MIN_CONN_INTERVAL		80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEV_DESIRED_MAX_CONN_INTERVAL		80

// Slave latency to use if automatic parameter update request is enabled
#define DEV_DESIRED_SLAVE_LATENCY			0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEV_DESIRED_CONN_TIMEOUT				50

// Whether to enable automatic parameter update request when a connection is formed
#define DEV_ENABLE_UPDATE_REQUEST			TRUE

// Parameter update waiting time after connection (in seconds)
#define DEV_PARAMS_UPDATE_PAUSE				1
#endif	/* CARDROID ||BLE_BEACON */
/*******************PERIPHERAL PARAMS END*******************/

/*******************ADC PARAMS START***********************/
/*
 * The maximum ADC value for the battery voltage level is 511 for a 10-bit conversion.
 * The ADC value is references vs. 1.25v and this maximum value corresponds to a voltage of 3.75v.
 * For a coin cell battery 3.0v = 100%.	The minimum operating voltage of the CC2540 is 2.0v so 2.0v = 0%.
 *
 * To convert a voltage to an ADC value use: 
 * Beacon:	(v/3)/1.25 * 511 = adc	3.0V = 409 ADC	2.0V = 273 ADC
 * CARDROID:  (v*100)/370/1.25*2047 	4.2V = 1929 ADC	3.3V = 1560 ADC
 */
#if ( defined BLE_BEACON )
// Battery check period in ms <default 86400000, i.e. 24h>
#define BAT_VOLT_CHECK_CYCLE 					86400000

#define BATT_ADC_LEVEL_MAX					409
#define BATT_ADC_LEVEL_MIN					273
#elif ( defined CARDROID )
// Battery check every 12h or 1 min
#define BAT_VOLT_CHECK_CYCLE 					43200000
#define BAT_VOLT_CHECK_SHORT_CYCLE			60000

#define BATT_ADC_LEVEL_MAX					1929
#define BATT_ADC_LEVEL_MIN					1560
#endif	/* BLE_BEACON */
/*******************ADC PARAMS ENDS*********************/


/*******************LED PARAMS START*********************/
// LED fast blink times and interval(ms) when Start Scan
#define START_SCAN_BLINK_TIMES				5
#define START_SCAN_BLINK_CYCLE				400

// LED slow blink times and interval(ms) when Finish Scan
#define FINISH_SCAN_BLINK_TIMES				5
#define FINISH_SCAN_BLINK_CYCLE				1000

//LED blink once and interval(ms) when find beacon
#define LED_BLINK_ONCE_TIMES					1
#define LED_BLINK_ONCE_CYCLE					50

// LED on percentage during above blink circle (%)
#define NORMAL_LED_ON_PERCENTAGE				50

// LED blink times and interval(ms) when low voltage warning
#define LOW_VOLT_WARN_BLINK_TIMES			3
#define LED_VOLT_WARN_BLINK_CYCLE			2000

// LED on percentage during battery warning blink circle (%) 
#define VOLT_WARN_LED_ON_PERCENTAGE			75

/*******************LED PARAMS ENDS*********************/

/*****************CENTRAL PARAMS START********************/
#if ( defined CARDROID ) || (defined IO_DETECT ) || ( defined BLE_BASE )
// Max recieve scan results
#define DEV_DEFAULT_SCAN_RESPONSE			10

// CC2541 radio scan duration in ms
#if ( defined BLE_BASE )
// Base use 3s scan duration
#define DEV_DEFAULT_SCAN_DURATION			3000
#else
#define DEV_DEFAULT_SCAN_DURATION			1000 
#endif	/* BLE_BASE */
// Discovey mode (limited, general, all)
#define DEV_DEFAULT_DISC_MODE					DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEV_DEFAULT_DISC_ACTIVE_SCAN			FALSE

// TRUE to use white list during discovery
#define DEV_DEFAULT_DISC_WHITE_LIST			FALSE

// TRUE to use high scan duty cycle when creating link
#define DEV_DEFAULT_LINK_HIGH_DUTY_CYCLE	TRUE

// TRUE to use white list when creating link
#define DEV_DEFAULT_LINK_WHITE_LIST			FALSE


#if ( defined CARDROID )
#define FILTER_DATA_LEN						BEACON_TYPE_POS_IN_ADV_DATA
#elif ( defined IO_DETECT ) || ( defined BLE_BASE )
#define FILTER_DATA_LEN						POSITION_STAT_POS_IN_ADV_DATA
#endif	/* CARDROID */

// Discovery states
enum
{
	UPLBS_BLE_DISC_STATE_IDLE,		// Idle
	UPLBS_BLE_DISC_STATE_SVC,		// Service discovery
	UPLBS_BLE_DISC_STATE_CHAR		// Characteristic discovery
};
#endif	/* CARDROID || IO_DETECT || BLE_BASE */

/********************CENTRAL PARAMS END********************/


#if ( defined IO_DETECT ) || ( defined BLE_BASE )
// Connecting time out, equal 2 cycle of CARDROID/CARD beacon cycle
#define CONN_TIME_OUT							6000

#if ( !defined IMAGE_VERSION_NUM) || ( IMAGE_VERSION_NUM == 0 )
// Avoid wifi device uboot startup (during which do not allow serial output)
#define DEV_WAIT_WIFI_BOOT_DELAY				5000
#elif ( IMAGE_VERSION_NUM == 1 )
// Avoid wifi device serial input
#define DEV_WAIT_WIFI_BOOT_DELAY				25000
#endif	/* !IMAGE_VERSION_NUM ||  IMAGE_VERSION_NUM = 0*/

#endif	/* IO_DETECT || BLE_BASE */

/***************OTHER CARDROID PARAMS START*****************/
#if ( defined CARDROID )
#if ( !defined IMAGE_VERSION_NUM) || ( IMAGE_VERSION_NUM == 0 )
// Use WiFi position instead of BLE position
#define TEMP_USE_WIFI_POS_TIME				40
#endif	/* !IMAGE_VERSION_NUM ||  IMAGE_VERSION_NUM = 0*/

// Maximum number of scan list to send
#define MAX_SCAN_LIST_LEN						8

// Last duration of beacon scanning
#define SCAN_LAST_PERIOD						30

// Tx power level count (0 , -6, -23)
#define POWER_LEVEL_COUNT						3

// Lower than this LED will blink
#define BAT_LOW_VOLT_WARN_PERCENT 			10

// Higher than this LED will always on
#define BAT_ALMOST_FULL_PERCENT 				90
#endif /* CARDROID */
/***************OTHER CARDROID PARAMS END******************/

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
// Task ID for internal task/event processing
static uint8 UPLBS_TaskId;

// UPLBS BLE state, contains peripheral and central state
static uint8 UPLBSBLEState = UPLBS_BLE_STATE_IDLE;

// UPLBS BLE system address
static uint8 OwnAddress[B_ADDR_LEN];

// GAP GATT Attributes
#if ( defined BLE_BEACON )
static uint8 DevAttName[GAP_DEVICE_NAME_LEN] = "BEACON";
#elif ( defined IO_DETECT )
static uint8 DevAttName[GAP_DEVICE_NAME_LEN] = "ACCESS";
#elif ( defined CARDROID )
static uint8 DevAttName[GAP_DEVICE_NAME_LEN] = "CARDROID";
#elif ( defined BLE_BASE )
static uint8 DevAttName[GAP_DEVICE_NAME_LEN] = "BASE STATION";
#endif	/* BLE_BEACON */

#if ( defined CARDROID ) || (defined BLE_BEACON)
//Connection handle in peripheral mode
static uint16 UPLBSPeripheralConnHandle = GAP_CONNHANDLE_INIT;

// System working state
static uint8 SysState=SYS_PERIPHERAL;

// GAP - Advertisement data (max size = 31 bytes, though this is best kept short to conserve power while advertisting)
static uint8 DevAdvertData[] =
{
	// Flags; this sets the device to use limited discoverable mode (advertises for 30 seconds at a time),
	// instead of general discoverable mode (advertises indefinitely)
	0x02,	 // length of this data
	GAP_ADTYPE_FLAGS,
	DEV_DISCOVERABLE_MODE,
	
	// service UUID, to notify central devices what services are included in this peripheral
	0x05,	// length of this data
	GAP_ADTYPE_32BIT_MORE,			//special UUID the same with entry device filtering UUID 
	LO_UINT16(DEV_DEFAULT_TYPE),
	HI_UINT16(DEV_DEFAULT_TYPE),
	LO_UINT16(DEV_DEFAULT_ADDR),
	HI_UINT16(DEV_DEFAULT_ADDR),

	// Protocol version
	0x04,	// length of this data
	GAP_PROTOCOL_VERSION,
#if ( defined IMAGE_VERSION_NUM )
	IMAGE_VERSION_NUM,
#else
	0x00,
#endif	/* IMAGE_VERSION_NUM */

	// Device state
	0x02,	// length of this data
	GAP_DEVICE_STATUS,
	SET_DEV_STAT(DEV_ADVERTISING_INTERVAL/8*5/1000,HCI_EXT_TX_POWER_0_DBM),

	// Battery percent
	0x02,
	GAP_BATTERY_PERCENT,
	100,		// 100%

	// Position state
	0x02,
#if ( defined BLE_BEACON )
	GAP_BASE_STATION_TYPE,
#else
	GAP_POSITION_STATE,
#endif	/* BLE_BEACON */
	TRUE,
};

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 DevScanRspData[] =
{
	// complete name
	0x07,	// length of this data
	GAP_ADTYPE_LOCAL_NAME_COMPLETE,
	'L','B','S','A','P','P',

	// connection interval range
	0x05,	// length of this data
	GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
	LO_UINT16( DEV_DESIRED_MIN_CONN_INTERVAL ),
	HI_UINT16( DEV_DESIRED_MIN_CONN_INTERVAL ),
	LO_UINT16( DEV_DESIRED_MAX_CONN_INTERVAL ),
	HI_UINT16( DEV_DESIRED_MAX_CONN_INTERVAL ),
};

#endif	/* CARDROID ||BLE_BEACON */

#if (defined IO_DETECT ) || ( defined BLE_BASE )
// UPLBS connection handle
static uint16 UPLBSCentralConnHandle = GAP_CONNHANDLE_INIT;

// Use to count scan times
static uint32 ScanCounts;

//UPLBS discovery state
static uint8 UPLBSDiscState = UPLBS_BLE_DISC_STATE_IDLE;

// UPLBS service start & end handle
static uint8 UPLBSSvcStartHdl;
static uint8 UPLBSSvcEndHdl;

// UPLBS characteristic handle
static uint8 UPLBSCharHdl=0;

// CARDROID/CARD waiting list counts & info struct
static uint8 ConnListCount = 0;

static struct
{
	uint8 conn_addr[B_ADDR_LEN];
	uint8 conn_addr_type;
	int8 conn_rssi;
	bool conn_state;
}ConnList[DEV_DEFAULT_SCAN_RESPONSE];

// Current connection device addr
static struct
{
	uint8 cur_conn_addr[B_ADDR_LEN];
	uint8 old_conn_addr[B_ADDR_LEN];
}ConnDevAddr;

// Scan beacon device counts & device info struct
static uint8 ScanDevCount = 0;

static struct
{
	uint8 scan_dev_addr[B_ADDR_LEN];
	uint8 scan_dev_bat;
	uint8 scan_dev_power;
	int8 scan_dev_rssi;
}ScanDev[DEV_DEFAULT_SCAN_RESPONSE];

#endif	/* IO_DETECT || BLE_BASE */

#if ( defined BLE_BEACON )
// Beacon type
static uint8 BeaconType;

// Tx power level
static int8 Txpw=HCI_EXT_TX_POWER_0_DBM;
#endif	/* BLE_BEACON */

#if ( defined CARDROID )

#if ( IMAGE_VERSION_NUM == 1 )
// Power flag
static uint8 ExtPwrFlg = FALSE;
#endif

// Uart power on
static uint8 RxTmpPwrOn = FALSE;

// Car moving direction
static uint8 CarMovDir = CAR_DIR_UDF;

// Role switching state
static bool RoleSwitching = FALSE;

// Scan times
static uint8 ScanTime = 0;

// 8051 IAR default use 1 byte aligned
// Scan result list
static struct
{
	uint8 baseaddr[B_ADDR_LEN];
	uint8 basetype;
	uint8 scantimes[POWER_LEVEL_COUNT];
	int8 rssival[POWER_LEVEL_COUNT];
}ScanList[MAX_SCAN_LIST_LEN];

// Calculate RSSI value of each beacon
static int16 RssiSum[MAX_SCAN_LIST_LEN][POWER_LEVEL_COUNT];

// Number of scan results and scan result index
static uint8 DevInScanList = 0;

// Beacon found in once scan duration
static uint8 FoundBeacon = FALSE;

// Scan list been sent
static uint8 ScanListSent = TRUE;
#endif	/* CARDROID */

#if ( defined BLE_BASE )
// Wireless data receive from Cardroid
static uint8 PosData[UPLBS_UART_TX_BUF_SIZE];

// Receive data length
static uint8 TotLen=0;

static bool PosDataFinish=FALSE;

// Device detect 
static uint8 LBSBcnDetctCnts = 0;
#endif	/* BLE_BASE */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void init_pair_mode(void);
static void UPLBS_ProcessOSALMsg(osal_event_hdr_t * pMsg);

#if ( defined (BLINK_LEDS) ) && ( HAL_LED == TRUE )
static void BlinkLed(uint8 leds, uint8 numBlinks, uint8 percent, uint16 period);
#endif	/* BLINK_LEDS && HAL_LED==TRUE */

#if ( defined CARDROID ) || (defined BLE_BEACON)
static void init_gap_peripheral_role(void);
static void init_gap_peripheral_params(void);
static void init_UPLBS_profile(void);

static void UPLBSProcessGAPMsg( gapEventHdr_t *pMsg );

static void PeripheralNotificationCB( gaprole_States_t newState );
static void UPLBSCharChangeCB(uint8 paramID);

static uint8 get_batt_percent(void);
static void bat_volt_check(void);

static void ble_start_advertising(void);
static void ble_stop_advertising(void);
#endif	/* CARDROID ||BLE_BEACON */

#if ( defined CARDROID ) || (defined IO_DETECT) || ( defined BLE_BASE )
static void init_gap_central_role(void);
static void init_gap_central_params(void);

static void CentralEventProcessCB(gapCentralRoleEvent_t * pEvent);

static bool start_ble_scan(void);
static bool filter_32bit_uuid_by_dev_type(uint16 dev_type,uint8 * pData,uint8 dataLen);
#endif	/* CARDROID || IO_DETECT || BLE_BASE */

#if (defined IO_DETECT) || ( defined BLE_BASE )
static bool add_dev_to_conn_list(gapDeviceInfoEvent_t devinfo);
static bool add_dev_to_detect_list(gapDeviceInfoEvent_t devinfo);

static void UPLBSProcessGATTMsg(gattMsgEvent_t * pMsg);
static void UPLBSGATTStartDiscService( void );

static void clear_conn_list(void);

static void PrintDevInfo(gapDeviceInfoEvent_t devinfo);

#endif	/* IO_DETECT || BLE_BASE */

#if ( defined BLE_BASE )
static void clear_pos_data(void);
#endif

#if ( defined BLE_BEACON )
static void send_power_change(void);
#endif	/* BLE_BEACON */

#if ( defined CARDROID )
static void GAP_role_swtich(void);

static bool add_dev_to_scan_list(gapDeviceInfoEvent_t devinfo);
static void calc_avg_rssi(void);

static void clear_scan_list(void);
#endif	/* CARDROID */
	
/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Peripheral Role Callbacks
#if ( defined CARDROID ) || (defined BLE_BEACON)
static gapRolesCBs_t UPLBS_PeripheralCBs =
{
	PeripheralNotificationCB,	// Profile State Change Callbacks
	NULL							// When a valid RSSI is read from controller (not used by application)
};

// UPLBS GATT Profile Callbacks
static UPLBSProfileCBs_t UPLBS_Profile_CharChangeCBs =
{
	UPLBSCharChangeCB		// Charactersitic value change callback
};
#endif	/* CARDROID ||BLE_BEACON */

// GAP Central Role Callbacks
#if ( defined CARDROID ) || (defined IO_DETECT ) || ( defined BLE_BASE )
static gapCentralRoleCB_t UPLBS_CentralCBs =
{
	NULL,						// RSSI callback
	CentralEventProcessCB		// Event callback
};
#endif	/* CARDROID || IO_DETECT || BLE_BASE */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn		UPLBSApp_Init
 *
 * @brief		Initialization function for the UPLBS App Task.
 *				This is called during initialization and should contain
 *				any application specific initialization (ie. hardware
 *				initialization/setup, table initialization, power up
 *				notificaiton ... ).
 *
 * @param	 task_id - the ID assigned by OSAL.	This ID should be
 *				used to send messages and set timers.
 *
 * @return	none
 */
void UPLBSApp_Init( uint8 task_id )
{
	UPLBS_TaskId = task_id;
	
#if	( defined POWER_SAVING )
	system_power_saving(UPLBS_TaskId);
#else
	system_power_hold(UPLBS_TaskId);
#endif	/* POWER_SAVING */

	UPLBS_Serial_Init();

#if ( defined BLE_BEACON )
	DEV_GPIO_P0_INPUT_3STATE(0xF3);//leave UART (P0.2 P0.3)
	DEV_GPIO_P1_INPUT_3STATE(0xFF);
#elif ( defined CARD )
	DEV_GPIO_P0_INPUT_3STATE(0xB3);//leave UART (P0.2 P0.3) Bat check (P0.6)
	DEV_GPIO_P1_INPUT_3STATE(0xF0);//leave LED (P1.0 P1.1) Power off (P1.2) G-sensor INT (P1.3)

/*	P1SEL &= (uint8) ~ BV(2);
	P1DIR |= (uint8) BV(2);
	P1_2 = 0;*/

	UPLBS_Acc_Init();
#elif ( defined CARDROID )

#if ( !defined IMAGE_VERSION_NUM) || ( IMAGE_VERSION_NUM == 0 )
	DEV_GPIO_P0_INPUT_3STATE(0x73);//leave UART (P0.2 P0.3) Bat check (P0.7)
	DEV_GPIO_P1_INPUT_3STATE(0xF4);//leave Power off (P1.0) G-sensor INT (P1.1) LED (P1.3)
#elif ( IMAGE_VERSION_NUM == 1 )
	DEV_GPIO_P0_INPUT_3STATE(0x31);//leave Red LED (P0.1) UART (P0.2 P0.3) Charge detect(P0.6)  Bat check (P0.7)
	DEV_GPIO_P1_INPUT_3STATE(0xF0);//leave Power off (P1.0) Run LED (P1.1) G-sensor INT (P1.2) Handshake (P1.3)
#endif	/* IMAGE_VERSION_NUM == 0 */

	UPLBS_WiFi_Init();
	UPLBS_Acc_Init();
#elif ( defined IO_DETECT )
	UPLBS_WiFi_Init();
	wifi_dev_power_on();
#endif	/* BLE_BEACON */
	ReadOwnMac(OwnAddress);

#if ( defined (BLINK_LEDS) ) && ( HAL_LED == TRUE )
	HalLedSet(HAL_LED_ALL, HAL_LED_MODE_BLINK);
//	BlinkLed(HAL_LED_ALL,0,0,0);
#endif	/* BLINK_LEDS && HAL_LED==TRUE */

#if ( defined CARDROID ) || ( defined BLE_BEACON )
	// Always set tx power to 0dBm
	HCI_EXT_SetTxPowerCmd( HCI_EXT_TX_POWER_0_DBM);

	// modifiy the battery percent byte in advertisement data before advertising
	HalAdcInit();
	bat_volt_check();

	// Setup the GAP Peripheral Role Profile and params
	init_gap_peripheral_role();
	init_gap_peripheral_params();

	// Set UPLBS profile & callback
	init_UPLBS_profile();
	
	// Enable clock divide on halt
	// This reduces active current while radio is active and CC254x MCU is halted
	HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
#endif	/* CARDROID || BLE_BEACON */
	
#if ( defined CARDROID ) || ( defined IO_DETECT ) || ( defined BLE_BASE )
	// Setup Central Role Profile and params
	init_gap_central_role();
	init_gap_central_params();
	
	// Initialize GATT Client
	VOID GATT_InitClient();
	// Register to receive incoming ATT Indications/Notifications
	GATT_RegisterForInd( UPLBS_TaskId );
#endif	/*	CARDROID || IO_DETECT || BLE_BASE */

	// Setup the GAP Bond Manager
	init_pair_mode();

	// Set the GAP Characteristics
	GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, DevAttName );
	
	// Initialize GATT attributes
	GGS_AddService( GATT_ALL_SERVICES );				// GAP
	GATTServApp_AddService( GATT_ALL_SERVICES );		// GATT attributes
	

#if ( defined IO_DETECT ) || ( defined BLE_BASE )
	osal_start_timerEx(UPLBS_TaskId, DEV_WORKING_EVT, 10);//DEV_WAIT_WIFI_BOOT_DELAY);
#elif ( defined CARDROID )
	osal_start_timerEx(UPLBS_TaskId, DEV_WORKING_EVT, ACC_SOFT_RESET_DELAY);
#else
	osal_set_event(UPLBS_TaskId, DEV_WORKING_EVT);
#endif	/* IO_DETECT */
	return;
}

/*********************************************************************
 * @fn		UPLBSApp_ProcessEvent
 *
 * @brief	 DEV Application Task event processor.	This function
 *		is called to process all events for the task.	Events
 *		include timers, messages and any other user defined events.
 *
 * @param	 task_id - The OSAL assigned task ID.
 * @param	 events - events to process.	This is a bit map and can
 *						 contain more than one event.
 *
 * @return	events not processed
 */
uint16 UPLBSApp_ProcessEvent( uint8 task_id, uint16 events )
{
	VOID task_id; // OSAL required parameter that isn't used in this function

	if ( events & SYS_EVENT_MSG )
	{
		uint8 *pMsg;

		if ( (pMsg = osal_msg_receive(UPLBS_TaskId)) != NULL )
		{
			UPLBS_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
			VOID osal_msg_deallocate( pMsg );// Release the OSAL message
		}

		return (events ^ SYS_EVENT_MSG);
	}

	if ( events & DEV_WORKING_EVT )
	{
#if ( defined BLE_BEACON )
		// Start the peripheral mode
		VOID GAPRole_StartDevice( &UPLBS_PeripheralCBs );

		osal_start_timerEx(UPLBS_TaskId, START_MANUAL_ADV_EVT, DEV_ADVERTISING_INTERVAL/8*5);
		osal_start_reload_timer( UPLBS_TaskId, BAT_LOW_VOLT_CHECK_EVT, BAT_VOLT_CHECK_CYCLE);		
#elif ( defined CARDROID )
		VOID GAPRole_StartDevice( &UPLBS_PeripheralCBs );

		osal_start_reload_timer( UPLBS_TaskId, BAT_LOW_VOLT_CHECK_EVT, BAT_VOLT_CHECK_CYCLE);

		set_Int_Process_TaskId(UPLBS_TaskId);
		uart_wakeup_Int_cfg();
		if (acc_check() == TRUE)
			PrintIntValue("\r\nTmpr",get_env_tmpr(),10);
		else
			PrintString("\r\nACC NC");
		
#if ( IMAGE_VERSION_NUM == 1 )
		Hal_PD_Int_Cfg(ExtPwrFlg);
#endif	/* IMAGE_VERSION_NUM == 1 */

#elif ( defined IO_DETECT ) || (defined BLE_BASE )
		// Start the central mode
		VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &UPLBS_CentralCBs );
#endif	/* BLE_BEACON */

		return ( events ^ DEV_WORKING_EVT );
	}
	
	if ( events & PERFORM_PERODIC_EVT )
	{
#if ( defined BLE_BEACON )
		// Stop advertising after success
		ble_stop_advertising();
#endif	/* BLE_BEACON */

#if ( defined CARDROID ) && ( IMAGE_VERSION_NUM == 1 )
		if (ExtPwrFlg == FALSE)
			system_power_saving(UPLBS_TaskId);
#endif	/* CARDROID */

#if ( defined BLE_BASE)
		uint8 chstt = (PosDataFinish==TRUE?SYS_SENT_OVER:SYS_DATA_RESEND);

		write_data_by_handle(UPLBSCentralConnHandle,UPLBSCharHdl,&chstt,1,UPLBS_TaskId);
		if (PosDataFinish== TRUE)
		{
			PrintString("\r\n");
			LBSsend(BLE_DEV_CAR_SCAN_DATA,PosData,TotLen);
			PosDataFinish = FALSE;
			clear_pos_data();
		}
#endif	/* BLE_BASE */
		return (events ^ PERFORM_PERODIC_EVT);
	}

	if ( events & AUTO_DISCONNECT_EVT )
	{
#if ( defined CARDROID ) || ( defined BLE_BEACON )	
		GAPRole_TerminateConnection();
#elif ( defined IO_DETECT ) || ( defined BLE_BASE )
		if ( UPLBSBLEState == UPLBS_BLE_STATE_CONNECTED )
			GAPCentralRole_TerminateLink( UPLBSCentralConnHandle );
		else if ( UPLBSBLEState == UPLBS_BLE_STATE_CONNECTING )
		{
			HCI_LE_CreateConnCancelCmd();
			clear_conn_list();
			osal_start_timerEx(UPLBS_TaskId,START_BLE_SCAN_EVT,DEV_DEFAULT_SCAN_DURATION/2);
		}
#endif	/* CARDROID || BLE_BEACON */

		return (events ^ AUTO_DISCONNECT_EVT);
	}

#if ( defined CARDROID ) || ( defined IO_DETECT ) || ( defined BLE_BASE )
	if ( events & START_BLE_SCAN_EVT)
	{
		start_ble_scan();

		return (events ^ START_BLE_SCAN_EVT);
	}
#endif	/* CARDROID || IO_DETECT || BLE_BASE */

#if ( defined CARDROID ) || ( defined BLE_BEACON )
	if ( events & BAT_LOW_VOLT_CHECK_EVT)
	{
		bat_volt_check();

		return (events ^ BAT_LOW_VOLT_CHECK_EVT);
	}
#endif	/* CARDROID || BLE_BEACON */

#if ( defined BLE_BEACON )
	if ( events & START_MANUAL_ADV_EVT)
	{
//		system_power_hold(task_id);
		ble_start_advertising();

		return (events ^ START_MANUAL_ADV_EVT);
	}
#endif	/* BLE_BEACON */

#if ( defined CARDROID )
	if ( events & INT_PROCESS_EVT)
	{
		detect_motion_state();
	
		return (events ^ INT_PROCESS_EVT);
	}
	
	if ( events & SWITCH_GAP_ROLE_EVT )
	{
		GAP_role_swtich();

		return (events ^ SWITCH_GAP_ROLE_EVT);
	}

	if ( events & RESTORE_MOVING_CHECK_EVT )
	{
		restore_moving_check();

		return (events ^ RESTORE_MOVING_CHECK_EVT);
	}

	if ( events & CHARGE_DETECT_EVT)
	{
#if ( IMAGE_VERSION_NUM == 1 )
		ExtPwrFlg = !ExtPwrFlg;
		if ( ExtPwrFlg== FALSE)
		{
			osal_stop_timerEx(UPLBS_TaskId,BAT_LOW_VOLT_CHECK_EVT);
			RxTmpPwrOn= FALSE;
			uart_wakeup_Int_cfg();
			HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF);
			osal_start_reload_timer( UPLBS_TaskId, BAT_LOW_VOLT_CHECK_EVT, BAT_VOLT_CHECK_CYCLE);
			system_power_saving(UPLBS_TaskId);
		}
		else
		{
			system_power_hold(UPLBS_TaskId);
			RxTmpPwrOn= TRUE;
			uart_resume_cfg();
			osal_stop_timerEx(UPLBS_TaskId,BAT_LOW_VOLT_CHECK_EVT);
			bat_volt_check();
			osal_start_reload_timer(UPLBS_TaskId,BAT_LOW_VOLT_CHECK_EVT,BAT_VOLT_CHECK_SHORT_CYCLE);
		}
		Hal_PD_Int_Cfg(ExtPwrFlg);
#endif	/* IMAGE_VERSION_NUM==1 */

		return (events ^ CHARGE_DETECT_EVT);
	}

	if ( events & UART_WAKE_UP_EVT)
	{
#if ( !defined CARD ) && ( IMAGE_VERSION_NUM == 1 )
		if ( ExtPwrFlg== FALSE && RxTmpPwrOn== TRUE)
#else
		if (RxTmpPwrOn== TRUE)
#endif	/* IMAGE_VERSION_NUM==1 */
		{
			system_power_saving(UPLBS_TaskId);
			RxTmpPwrOn = FALSE;
			uart_wakeup_Int_cfg();
		}
		else
		{
			system_power_hold(UPLBS_TaskId);
			uart_resume_cfg();
			RxTmpPwrOn = TRUE;
			osal_start_timerEx(UPLBS_TaskId,UART_WAKE_UP_EVT,3000);
		}
		return (events ^ UART_WAKE_UP_EVT);
	}
#endif	/* CARDROID */

	// Discard unknown events
	return 0;
}

/*********************************************************************
 * @fn		init_pair_mode
 *
 * @brief	 	Init gap pair mode.
 *
 * @param	none
 *
 * @return	none
 */
static void init_pair_mode(void)
{
	uint8 pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING;
			
	GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
}

/*********************************************************************
 * @fn		UPLBS_ProcessOSALMsg
 *
 * @brief	 Process an incoming task message.
 *
 * @param	pMsg - message to process
 *
 * @return	none
 */
static void UPLBS_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
	switch ( pMsg->event )
	{
		case GATT_MSG_EVENT:
#if (defined IO_DETECT ) || ( defined BLE_BASE )
			// Process GATT message
			UPLBSProcessGATTMsg( (gattMsgEvent_t *) pMsg );
#endif	/* IO_DETECT  || BLE_BASE */
			break;

		case GAP_MSG_EVENT:
#if ( defined CARDROID ) || ( defined BLE_BEACON )
			UPLBSProcessGAPMsg( (gapEventHdr_t *) pMsg );
#endif	/* CARDROID || BLE_BEACON */
			break;
		default:
			break;
	}
}

#if ( defined (BLINK_LEDS) ) && ( HAL_LED == TRUE )
/*********************************************************************
 * @fn		BlinkLed
 *
 * @brief	 	
 *
 * @param	none
 *
 * @return	none
 */
static void BlinkLed(uint8 leds, uint8 numBlinks, uint8 percent, uint16 period)
{
#if ( defined CARDROID ) && ( IMAGE_VERSION_NUM == 1 )
	if (ExtPwrFlg == FALSE && numBlinks*period > 0)
	{
		system_power_hold(UPLBS_TaskId);
		osal_start_timerEx(UPLBS_TaskId,PERFORM_PERODIC_EVT,numBlinks*period);
	}
#endif	/* IMAGE_VERSION_NUM==1 */
	HalLedBlink(leds,numBlinks,percent,period);

}
#endif	/* BLINK_LEDS && HAL_LED==TRUE */

#if ( defined CARDROID ) || (defined BLE_BEACON)

/*********************************************************************
 * @fn		init_gap_peripheral_role
 *
 * @brief		Set gap peripheral role.
 *
 * @param	none
 *
 * @return	none
 */
static void init_gap_peripheral_role(void)
{
	uint8 initial_advertising_enable = DEV_DEFAULT_ADV_ENABLE;
	uint16 gapRole_AdvertOffTime = DEV_ADVERTISING_INTERVAL;
	uint8 enable_update_request = DEV_ENABLE_UPDATE_REQUEST;
	uint16 desired_min_interval = DEV_DESIRED_MIN_CONN_INTERVAL;
	uint16 desired_max_interval = DEV_DESIRED_MAX_CONN_INTERVAL;
	uint16 desired_slave_latency = DEV_DESIRED_SLAVE_LATENCY;
	uint16 desired_conn_timeout = DEV_DESIRED_CONN_TIMEOUT;

	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
	GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
	GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
	GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
	GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
	GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
	GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );

	GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( DevAdvertData ), DevAdvertData );

	GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( DevScanRspData ), DevScanRspData );
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
static void init_gap_peripheral_params(void)
{
	// Set advertising interval
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEV_ADVERTISING_INTERVAL );
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEV_ADVERTISING_INTERVAL );

	// Set parameter updates waiting time
	GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEV_PARAMS_UPDATE_PAUSE );
}

/*********************************************************************
 * @fn		init_UPLBS_profile
 *
 * @brief		Init UPLBS profile.
 *
 * @param	none
 *
 * @return	none
 */
static void init_UPLBS_profile(void)
{
	uint8 init_val=0;
	
	UPLBSProfile_AddService(UPLBS_SERVICE);
	
	UPLBSProfile_SetParameter(UPLBS_CHAR_VERN,1,&init_val);
	UPLBSProfile_SetParameter(UPLBS_CHAR_SYS_INFO,1,&init_val);
	UPLBSProfile_SetParameter(UPLBS_CHAR_TEMPR,1,&init_val);
	UPLBSProfile_SetParameter(UPLBS_CHAR_ACC_PARAM,1,&init_val);
	UPLBSProfile_SetParameter(UPLBS_CHAR_RUN_STATE,1,&SysState);
#if ( defined BLE_BEACON )
	UPLBSProfile_SetParameter(UPLBS_CHAR_BEACON_TYPE,1,&BeaconType);
#else
	UPLBSProfile_SetParameter(UPLBS_CHAR_PARKING_STATE,1,&CarMovDir);
#endif	/* BLE_BEACON */

	// Register callback with UPLBS GATT profile
	VOID UPLBSProfile_RegisterAppCBs( &UPLBS_Profile_CharChangeCBs );

	return;
}

/*********************************************************************
 * @fn		UPLBSProcessGAPMsg
 *
 * @brief	 	Process GAP messages
 *
 *@param	pMsg - message from GAP
 *
 * @return	none
 */
static void UPLBSProcessGAPMsg( gapEventHdr_t *pMsg )
{
	// Advertising data change OK
	if (((gapEventHdr_t *)pMsg)->opcode == GAP_ADV_DATA_UPDATE_DONE_EVENT)
	{
		gapAdvDataUpdateEvent_t *pPkt = (gapAdvDataUpdateEvent_t *)pMsg;

		if ( pPkt->adType == TRUE && pPkt->hdr.status == SUCCESS )
		{
#if ( defined CARDROID )
			ble_start_advertising();
#else
			// modified by lanchen 15.7.29
			//osal_start_timerEx(UPLBS_TaskId, START_MANUAL_ADV_EVT, DEV_ADVERTISING_INTERVAL/8*5);
			ble_start_advertising();
#endif	/* CARDROID */
		}
		else
			PrintString("\r\nCH ADV ERR");
	}

	return;
}

/*********************************************************************
 * @fn		PeripheralNotificationCB
 *
 * @brief	 	Notification from the profile of a state change.
 *
 * @param	newState - new state
 *
 * @return	none
 */
static void PeripheralNotificationCB( gaprole_States_t newState )
{
	switch ( newState )
	{
		case GAPROLE_STARTED:
			break;
		case GAPROLE_ADVERTISING:
		{
			UPLBSBLEState = UPLBS_BLE_STATE_ADVERTISING;
//			PrintString("A");
#if ( defined BLE_BEACON ) && ( ! defined SCAN_DEBUG )
			// modified by lanchen 15.7.29
//			ble_stop_advertising();
			osal_start_timerEx(UPLBS_TaskId, PERFORM_PERODIC_EVT, DEV_ADVERTISING_INTERVAL/8*10-1000);
#endif	/* BLE_BEACON && ! SCAN_DEBUG */
			break;
		}
		case GAPROLE_CONNECTED:
		{
			UPLBSBLEState = UPLBS_BLE_STATE_CONNECTED;
//			PrintString("\r\nC");
			
			osal_start_timerEx(UPLBS_TaskId, AUTO_DISCONNECT_EVT, AUTO_DISCONNECT_TIME);
#if ( defined CARDROID )
			GAPRole_GetParameter( GAPROLE_CONNHANDLE, &UPLBSPeripheralConnHandle );
			// don't restart advertising when connection is closed
			ble_stop_advertising();
			if (ScanListSent == FALSE)
				BLE_noti_send_data(UPLBSPeripheralConnHandle,SCAN_DATA_HDL_IDX,(uint8 *)ScanList,\
						sizeof(ScanList)/MAX_SCAN_LIST_LEN*DevInScanList);
#endif	/* CARDROID */
			break;
		}
		case GAPROLE_WAITING:
		case GAPROLE_WAITING_AFTER_TIMEOUT:
		// Stop adv, disconnnect, time out process
		{
			UPLBSBLEState = UPLBS_BLE_STATE_IDLE;
//			PrintString("\r\nW");
#if ( defined CARDROID )
			// Advertising stopped or disconnected during GAP role change
			if ( RoleSwitching == TRUE )
			{
				osal_set_event(UPLBS_TaskId, SWITCH_GAP_ROLE_EVT);
				break;
			}
			else if (UPLBSPeripheralConnHandle!=GAP_CONNHANDLE_INIT)// Normal after disconnect
			{
				uint8 oldsentstate =DevAdvertData[POSITION_STAT_POS_IN_ADV_DATA];
				
				// Scan data sent success
				if (ScanListSent==TRUE && oldsentstate == FALSE)
				{
					DevAdvertData[POSITION_STAT_POS_IN_ADV_DATA] = ScanListSent;
					GAP_UpdateAdvertisingData(UPLBS_TaskId, TRUE, sizeof(DevAdvertData), DevAdvertData);
				}
				else 
					ble_start_advertising();
			}
#elif ( defined BLE_BEACON )
			// After advertising  stopped successfully
			if (UPLBSPeripheralConnHandle == GAP_CONNHANDLE_INIT)
				send_power_change();
#endif	/* CARDROID */

			// Reset connection handle after disconnected
			if (UPLBSPeripheralConnHandle != GAP_CONNHANDLE_INIT)
				UPLBSPeripheralConnHandle = GAP_CONNHANDLE_INIT;

			break;
		}
		case GAPROLE_ERROR:
		{
			UPLBSBLEState = UPLBS_BLE_STATE_IDLE;
			PrintString("\r\nERR");
			break;
		}
		default:
			break;
	} 
}

/*********************************************************************
 * @fn		UPLBSCharChangeCB
 *
 * @brief		Callback from UPLBS Profile indicating a value change
 *
 * @param	paramID - parameter ID of the value that was changed.
 *
 * @return	none
 */
static void UPLBSCharChangeCB( uint8 paramID )
{
	uint8 newValue;

	switch( paramID )
	{
		case UPLBS_CHAR_SYS_INFO:
		{
			uint32 sys_clk;
			
			UPLBSProfile_GetParameter( UPLBS_CHAR_SYS_INFO, &newValue );
			sys_clk = osal_GetSystemClock();
			BLE_noti_send_data(UPLBSPeripheralConnHandle,BOOT_TIME_HDL_IDX,(uint8 *)&sys_clk,sizeof(sys_clk));
			break;
		}
		case UPLBS_CHAR_TEMPR:
		{	
			UPLBSProfile_GetParameter( UPLBS_CHAR_TEMPR, &newValue );
#if ( defined CARDROID )
			int8 tmpr;

			tmpr = get_env_tmpr();
			BLE_noti_send_data(UPLBSPeripheralConnHandle,TEMPR_VAL_HDL_IDX,(uint8 *)&tmpr,sizeof(tmpr));
#endif	/* CARDROID */
			break;
		}
		case UPLBS_CHAR_ACC_PARAM:
		{
			UPLBSProfile_GetParameter( UPLBS_CHAR_ACC_PARAM, &newValue );
			break;
		}
		case UPLBS_CHAR_RUN_STATE:
		{
			UPLBSProfile_GetParameter( UPLBS_CHAR_RUN_STATE, &newValue );
			switch (newValue)
			{
				case SYS_PERIPHERAL:// Do nothing
				case SYS_CENTRAL:
					break;
				case SYS_DATA_RESEND://Data resend
#if ( defined CARDROID )
					BLE_noti_send_data(UPLBSPeripheralConnHandle,SCAN_DATA_HDL_IDX,(uint8 *)ScanList,\
							sizeof(ScanList)/MAX_SCAN_LIST_LEN*DevInScanList);
#endif	/* CARDROID */
					break;
				case SYS_SENT_OVER://Data recieved OK
					newValue = SYS_PERIPHERAL;
#if ( defined CARDROID )
					ScanListSent = TRUE;
					GAPRole_TerminateConnection();
#endif	/* CARDROID */
					UPLBSProfile_SetParameter( UPLBS_CHAR_RUN_STATE, 1, &newValue );

					break;
				case SYS_REBOOT:// Reset
					HAL_SYSTEM_RESET();
					break;
				case SYS_UPGRADE:// Prepare upgrade
#if ( defined OAD_UPGRADE )
					prepare_oad_upgrade();
#endif	/* OAD_UPGRADE */
					break;
				default:
					break;
			}	
			
			break;
		}
		case UPLBS_CHAR_PARKING_STATE:
		{
			UPLBSProfile_GetParameter( UPLBS_CHAR_PARKING_STATE, &newValue );
			break;
		}
		default:
			break;
	}
}

/*********************************************************************
 * @fn			get_batt_percent
 *
 * @brief	 get battery percent
 *
 * @param	none
 *
 * @return	battery percent
 */
static uint8 get_batt_percent(void)
{
	uint16 adc;
	uint8 percent;

	// Configure ADC and perform a read
	HalAdcSetReference( HAL_ADC_REF_125V );
	
#if ( defined BLE_BEACON )
	adc = HalAdcRead( HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_10 );
#elif ( defined CARDROID )
	#if ( defined CARD )
	// Cardroid use different adc channel and resolution
	adc = HalAdcRead( HAL_ADC_CHN_AIN6,HAL_ADC_RESOLUTION_12);
	#else
	adc = HalAdcRead( HAL_ADC_CHN_AIN7,HAL_ADC_RESOLUTION_12);
	#endif /* CARD */
#endif	/* BLE_BEACON */

	if (adc >= BATT_ADC_LEVEL_MAX)
		percent = 100;
	else if (adc <=	BATT_ADC_LEVEL_MIN)
		percent = 0;
	else
		// make sure the calculation will not overflow
		percent = (uint8) (((adc-BATT_ADC_LEVEL_MIN) * 100) / (BATT_ADC_LEVEL_MAX-BATT_ADC_LEVEL_MIN));

	return percent;
}

/*********************************************************************
 * @fn		bat_volt_check
 *
 * @brief		check battery level in CARDROID & beacon mode
 *
 * @param	none
 *
 * @return	none
 */
static void bat_volt_check(void)
{
	uint8 bat_prcnt;

	bat_prcnt= get_batt_percent();
	DevAdvertData[BAT_PERCENT_POS_IN_ADV_DATA] =bat_prcnt;

	// Do not update advertising data after voltage check, leave this task to power change(beacon) or mode change(CARDROID) task
	//GAP_UpdateAdvertisingData(UPLBS_TaskId, TRUE, sizeof(DevAdvertData), DevAdvertData);
	
#if ( defined CARDROID )
	if (bat_prcnt < BAT_LOW_VOLT_WARN_PERCENT)
	{
#if ( !defined IMAGE_VERSION_NUM) || ( IMAGE_VERSION_NUM == 0 )
		BlinkLed(HAL_LED_1, LOW_VOLT_WARN_BLINK_TIMES, VOLT_WARN_LED_ON_PERCENTAGE, LED_VOLT_WARN_BLINK_CYCLE);
#elif ( IMAGE_VERSION_NUM == 1 )
		BlinkLed(HAL_LED_2, LOW_VOLT_WARN_BLINK_TIMES, VOLT_WARN_LED_ON_PERCENTAGE, LED_VOLT_WARN_BLINK_CYCLE);
#endif
	}
#if ( IMAGE_VERSION_NUM == 1 )
	else if(ExtPwrFlg == TRUE && bat_prcnt > BAT_ALMOST_FULL_PERCENT)
	{
		HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
	}
	else if (ExtPwrFlg == TRUE)
	{
		BlinkLed(HAL_LED_2, BAT_VOLT_CHECK_SHORT_CYCLE/LED_VOLT_WARN_BLINK_CYCLE,\
				VOLT_WARN_LED_ON_PERCENTAGE,LED_VOLT_WARN_BLINK_CYCLE);
	}
#endif

#endif /* CARDROID */

	return;
}

/*********************************************************************
 * @fn		ble_start_advertising
 *
 * @brief		Start advertising
 *
 * @param	none
 *
 * @return	none
 */
static void ble_start_advertising(void)
{
	uint8 adv_enable = TRUE;

	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enable );
}

/*********************************************************************
 * @fn		ble_stop_advertising
 *
 * @brief		Stop advertising
 *
 * @param	none
 *
 * @return	none
 */
static void ble_stop_advertising(void)
{
	uint8 adv_enable = FALSE;
	
	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enable );
}

#if ( defined BLE_BEACON )
/*********************************************************************
 * @fn		send_power_change
 *
 * @brief	 	change send power when in beacon mode
 *
 * @param	none
 *
 * @return	none
 */
static void send_power_change(void)
{
	if (UPLBSBLEState != UPLBS_BLE_STATE_IDLE)
		return;
	
	if (--Txpw < 0)
		Txpw = HCI_EXT_TX_POWER_0_DBM;

	HCI_EXT_SetTxPowerCmd(Txpw);

	DevAdvertData[DEVICE_STATUS_POS_IN_ADV_DATA] = \
			MODIFY_POWER_LEVEL(DevAdvertData[DEVICE_STATUS_POS_IN_ADV_DATA],Txpw);
	GAP_UpdateAdvertisingData(UPLBS_TaskId, TRUE, sizeof(DevAdvertData), DevAdvertData);

//	PrintIntValue("\r\nSET:",Txpw, 10);

	return;
}
#endif	/* BLE_BEACON */
#endif /* CARDROID || BLE_BEACON */

#if ( defined CARDROID ) || (defined IO_DETECT ) || ( defined BLE_BASE )
/*********************************************************************
 * @fn		init_gap_central_role
 *
 * @brief	 	Init gap central role state.
 *
 * @param	none
 *
 * @return	none
 */
static void init_gap_central_role(void)
{
	uint8 scanRes = DEV_DEFAULT_SCAN_RESPONSE;
	
	GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
}

/*********************************************************************
 * @fn		init_gap_central_params
 *
 * @brief		Init scan duration interval & window.
 *
 * @param	none
 *
 * @return	none
 */
static void init_gap_central_params(void)
{
	GAP_SetParamValue( TGAP_GEN_DISC_SCAN_INT, DEV_DEFAULT_SCAN_DURATION/5*8 );
	GAP_SetParamValue( TGAP_GEN_DISC_SCAN_WIND, DEV_DEFAULT_SCAN_DURATION/5*8 );

	GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEV_DEFAULT_SCAN_DURATION );
}

/*********************************************************************
 * @fn			CentralEventProcessCB
 *
 * @brief	 Central event callback function.
 *
 * @param	 pEvent - pointer to event structure
 *
 * @return	none
 */
static void CentralEventProcessCB( gapCentralRoleEvent_t *pEvent )
{
	switch ( pEvent->gap.opcode )
	{
		case GAP_DEVICE_INIT_DONE_EVENT:	//Device init finish
		{
#if ( defined CARDROID )
			osal_start_timerEx( UPLBS_TaskId, START_BLE_SCAN_EVT,START_SCAN_BLINK_CYCLE*START_SCAN_BLINK_TIMES);
#elif ( defined IO_DETECT ) || ( defined BLE_BASE )		
			osal_set_event( UPLBS_TaskId, START_BLE_SCAN_EVT);
#endif	/* CARDROID */

			break;
		}
		case GAP_DEVICE_INFO_EVENT:	// Beacon found during scanning
		{
#if ( defined CARDROID )
			// if filtering device discovery results based on service UUID
			if ( filter_32bit_uuid_by_dev_type( BEACON_DEV_TYPE,\
					pEvent->deviceInfo.pEvtData,\
					pEvent->deviceInfo.dataLen ) == TRUE)
			{

				if (add_dev_to_scan_list(pEvent->deviceInfo) == TRUE)
					FoundBeacon = TRUE;
				else
					PrintString("\r\n[MAX]");
			}
#elif ( defined BLE_BASE ) || (defined IO_DETECT)

#if ( !defined SCAN_DEBUG ) && ( defined BLE_BASE )
			if (LBSBcnDetctCnts == 0)	//do not connect CARDROID when performing beacon detect

			{
				if ( filter_32bit_uuid_by_dev_type( CARDROID_DEV_TYPE,\
						pEvent->deviceInfo.pEvtData,\
						pEvent->deviceInfo.dataLen ) == TRUE )
				{

#if ( !defined SCAN_DEBUG )
					if (add_dev_to_conn_list(pEvent->deviceInfo) == TRUE)
					{
						UPLBSBLEState = UPLBS_BLE_STATE_CONNECTING;
					}
#else
					PrintString("\r\n CDD");
					PrintDevInfo(pEvent->deviceInfo);
#endif	/* !SCAN_DEBUG */

				}
			}
#endif	/* !SCAN_DEBUG */

#if ( !defined SCAN_DEBUG ) && ( defined BLE_BASE )
			else
#endif	/* !SCAN_DEBUG */
			{
				if ( filter_32bit_uuid_by_dev_type( BEACON_DEV_TYPE,\
						pEvent->deviceInfo.pEvtData,\
						pEvent->deviceInfo.dataLen ) == TRUE)
				{
#if ( !defined SCAN_DEBUG )
					add_dev_to_detect_list(pEvent->deviceInfo);
#else
					PrintString("\r\nBCN ");
					PrintDevInfo(pEvent->deviceInfo);
#endif	/* SCAN_DEBUG */
				}
			}
#endif	/* CARDROID */

			break;
		}
		case GAP_DEVICE_DISCOVERY_EVENT:	// Finish scan
		{
			// Continue scan
			if (UPLBSBLEState == UPLBS_BLE_STATE_SCANNING)
			{
				UPLBSBLEState = UPLBS_BLE_STATE_IDLE;
#if ( defined CARDROID )
				// In case GAP role changed while scanning
				if (RoleSwitching == TRUE)
				{
					osal_set_event(UPLBS_TaskId, SWITCH_GAP_ROLE_EVT);
					return;
				}
				
				ScanTime++;
				if (FoundBeacon == TRUE )
				{
					// Wait for led blink once
					osal_start_timerEx(UPLBS_TaskId, START_BLE_SCAN_EVT, LED_BLINK_ONCE_CYCLE);
					BlinkLed(HAL_LED_1, LED_BLINK_ONCE_TIMES, NORMAL_LED_ON_PERCENTAGE, LED_BLINK_ONCE_CYCLE);
				
					return;
				}
#elif ( defined BLE_BASE )
				// Device detect times auto decrease
				if (LBSBcnDetctCnts != 0)
				{
					if (ScanDevCount > 0)
					{
						LBSsend(BLE_DEV_SCAN_RSLT,(uint8 *)ScanDev, sizeof(ScanDev)/DEV_DEFAULT_SCAN_RESPONSE*ScanDevCount);
						ScanDevCount = 0;
						osal_memset(ScanDev,0,sizeof(ScanDev));
					}
					LBSBcnDetctCnts--;
				}
#endif	/* CARDROID */
				osal_set_event( UPLBS_TaskId, START_BLE_SCAN_EVT);
			}
#if ( defined IO_DETECT ) || ( defined BLE_BASE )
			// Prepare to connect CARDROID/CARD after scan stopped
			else if (UPLBSBLEState == UPLBS_BLE_STATE_CONNECTING)
			{
				uint8 max_idx=0,tmp_idx;

				max_idx=0;
				// find most strong cardroid to connect
				for (tmp_idx=max_idx+1;tmp_idx<ConnListCount;tmp_idx++)
					if (ConnList[tmp_idx].conn_rssi > ConnList[max_idx].conn_rssi)
						max_idx=tmp_idx;

				osal_memcpy(ConnDevAddr.cur_conn_addr,ConnList[max_idx].conn_addr,B_ADDR_LEN);
				
				GAPCentralRole_EstablishLink( DEV_DEFAULT_LINK_HIGH_DUTY_CYCLE,\
						DEV_DEFAULT_LINK_WHITE_LIST,ConnList[max_idx].conn_addr_type,\
						ConnList[max_idx].conn_addr);
				PrintString("\r\nC...");
				osal_start_timerEx(UPLBS_TaskId,AUTO_DISCONNECT_EVT,CONN_TIME_OUT);
			}
#endif	/* IO_DETECT || BLE_BASE */

		}
			break;

		case GAP_LINK_ESTABLISHED_EVENT:
		{
#if ( defined IO_DETECT ) || ( defined BLE_BASE )
			if ( pEvent->gap.hdr.status == SUCCESS )
			{
				PrintString("\r\nCON");
				UPLBSBLEState = UPLBS_BLE_STATE_CONNECTED;
				UPLBSCentralConnHandle = pEvent->linkCmpl.connectionHandle;
				
				// If service discovery not performed, initiate service discovery
				if ( UPLBSCharHdl == 0 )
					UPLBSGATTStartDiscService();
				else
					read_data_by_handle(UPLBSCentralConnHandle,UPLBSCharHdl, UPLBS_TaskId);
				
#if defined BLE_BASE
				if (osal_memcmp(ConnDevAddr.cur_conn_addr,ConnDevAddr.old_conn_addr, B_ADDR_LEN) == FALSE)
					clear_pos_data();
				
				if (TotLen == 0)
				{
					init_blob_recv();
					osal_memcpy(PosData, ConnDevAddr.cur_conn_addr,B_ADDR_LEN);
					TotLen = B_ADDR_LEN;
				}
				osal_start_timerEx(UPLBS_TaskId, PERFORM_PERODIC_EVT, AUTO_DISCONNECT_TIME/2);
#endif	/* BLE_BASE */
				osal_start_timerEx(UPLBS_TaskId, AUTO_DISCONNECT_EVT, AUTO_DISCONNECT_TIME);
			}
			else
			{
				UPLBSBLEState = UPLBS_BLE_STATE_IDLE;
				UPLBSCentralConnHandle = GAP_CONNHANDLE_INIT;

				PrintIntValue("\r\nCON F:", pEvent->gap.hdr.status,10);
#if ( defined BLE_BASE )
				clear_conn_list();
				// Continue scan
				osal_set_event(UPLBS_TaskId, START_BLE_SCAN_EVT);
#endif	/* BLE_BASE */

			}
#endif	/* IO_DETECT || BLE_BASE */

		}
			break;
		
		case GAP_LINK_TERMINATED_EVENT:
		{
#if ( defined IO_DETECT ) || ( defined BLE_BASE )
			UPLBSBLEState = UPLBS_BLE_STATE_IDLE;
			UPLBSCentralConnHandle = GAP_CONNHANDLE_INIT;

			UPLBSCharHdl = 0;
			PrintIntValue("\r\nDIS:",pEvent->linkTerminate.reason,10);

			osal_memcpy(ConnDevAddr.old_conn_addr,ConnDevAddr.cur_conn_addr,B_ADDR_LEN);
			clear_conn_list();	
#if ( defined BLE_BASE )
			osal_set_event(UPLBS_TaskId, START_BLE_SCAN_EVT);
#endif	/* BLE_BASE */
#endif	/* IO_DETECT || BLE_BASE */
		}
		break;

		case GAP_LINK_PARAM_UPDATE_EVENT:
			break;

		default:
			break;
	}
}

/*********************************************************************
 * @fn			start_ble_scan
 *
 * @brief	 Perform a periodic application task. This function gets called every 
 *		1 second as a result of the START_BLE_SCAN_EVT OSAL event.
 *
 * @param	 none
 *
 * @return	TRUE - start scan.
 */
static bool start_ble_scan( void )
{
#if ( defined IO_DETECT ) || ( defined BLE_BASE )
	if ( UPLBSBLEState == UPLBS_BLE_STATE_IDLE)
	{
		PrintIntValue("\r\n>",ScanCounts++, '+');

		UPLBSBLEState = UPLBS_BLE_STATE_SCANNING;
		// start scan
		GAPCentralRole_StartDiscovery( DEV_DEFAULT_DISC_MODE,DEV_DEFAULT_DISC_ACTIVE_SCAN,\
				DEV_DEFAULT_DISC_WHITE_LIST );
	}
	else
		return FALSE;
#else
	if (ScanTime >= SCAN_LAST_PERIOD)
	{
		// Finish scanning, slow blink
		BlinkLed (HAL_LED_1, FINISH_SCAN_BLINK_TIMES, NORMAL_LED_ON_PERCENTAGE, FINISH_SCAN_BLINK_CYCLE);

		if (DevInScanList> 0)
		{
			ScanListSent = FALSE;
			calc_avg_rssi();
			if (CarMovDir != CAR_DIR_OUT)
			{
				CarMovDir = CAR_DIR_OUT;
				UPLBSProfile_SetParameter(UPLBS_CHAR_PARKING_STATE,1,&CarMovDir);
			}
		}
		else
		{
			if (CarMovDir != CAR_DIR_IN)
			{
				CarMovDir = CAR_DIR_IN;
				UPLBSProfile_SetParameter(UPLBS_CHAR_PARKING_STATE,1,&CarMovDir);
			}
		}
		osal_set_event(UPLBS_TaskId, SWITCH_GAP_ROLE_EVT);
	}
	else if ( UPLBSBLEState == UPLBS_BLE_STATE_IDLE)
	{
#if ( !defined IMAGE_VERSION_NUM) || ( IMAGE_VERSION_NUM == 0 )
		// Use WiFi position results temporarily, when no beacon found
		if (ScanTime == TEMP_USE_WIFI_POS_TIME)
			if(DevInScanList== 0)
				PrintString("{NULL}");
#endif	/* !IMAGE_VERSION_NUM ||  IMAGE_VERSION_NUM = 0*/

		UPLBSBLEState = UPLBS_BLE_STATE_SCANNING;
		FoundBeacon = FALSE;

		PrintIntValue("\r\n>",ScanTime,10);
		
		GAPCentralRole_StartDiscovery( DEV_DEFAULT_DISC_MODE,DEV_DEFAULT_DISC_ACTIVE_SCAN,\
				DEV_DEFAULT_DISC_WHITE_LIST );
	}
#endif	/* IO_DETECT || BLE_BASE */

	return TRUE;
}


/*********************************************************************
 * @fn			filter_32bit_uuid_by_dev_type
 *
 * @brief	 Find a given UUID in an advertiser's service UUID list.
 *
 * @return	TRUE if service UUID found
 */
static bool filter_32bit_uuid_by_dev_type( uint16 dev_type, uint8 *pData, uint8 dataLen )
{
	uint8 adLen;
	uint8 adType;
	uint8 *pEnd;

	if (dataLen < FILTER_DATA_LEN+1)
		return FALSE;
	
	pEnd = pData + dataLen - 1;
	// While end of data not reached
	while ( pData < pEnd )
	{
		adLen = *pData++;	// Get length of next AD item
		if ( adLen > 0 )
		{
			adType = *pData;

			// If AD type is for 32-bit service UUID
			if ( adType == GAP_ADTYPE_32BIT_MORE || adType == GAP_ADTYPE_32BIT_COMPLETE )
			{
				pData++;
				adLen--;
				
				while ( adLen >= 2 && pData < pEnd )	// For each UUID in list
				{
					if ( pData[0] == LO_UINT16(dev_type) && pData[1] == HI_UINT16(dev_type))
						return TRUE;	// Match found		
					// Go to next
					pData += 2;
					adLen -= 2;
				}
				// Handle possible erroneous extra byte in UUID list
				if ( adLen == 1 )
					pData++;
			}
			else	// Go to next item
				pData += adLen;
		}
	}

	return FALSE;	// Match not found
}

#if (defined IO_DETECT ) || ( defined BLE_BASE )
/*********************************************************************
 * @fn		add_dev_to_conn_list
 *
 * @brief		add scan device information to connection list
 *
 * @param	devinfo - device information
 *
 * @return	TRUE-Add OK
 */
static bool add_dev_to_conn_list(gapDeviceInfoEvent_t devinfo)
{
	uint8 i=0;
#if ( defined BLE_BASE )
	if (devinfo.pEvtData[POSITION_STAT_POS_IN_ADV_DATA] == TRUE)
		return FALSE;
#endif	/* BLE_BASE */
	if (devinfo.eventType != GAP_ADTYPE_ADV_IND)
		return FALSE;

	for (i=0; i<ConnListCount; i++ )
	{
		if (osal_memcmp(devinfo.addr, ConnList[i].conn_addr, B_ADDR_LEN))
			return FALSE;
	}
	
	if (ConnListCount< DEV_DEFAULT_SCAN_RESPONSE)
	{
		osal_memcpy( ConnList[i].conn_addr, devinfo.addr, B_ADDR_LEN );
		ConnList[i].conn_addr_type = devinfo.addrType;
		ConnList[i].conn_rssi = devinfo.rssi;
		ConnList[i].conn_state = FALSE;
		ConnListCount++;
	}
	else
		return FALSE;

	return TRUE;
}

/*********************************************************************
 * @fn		add_dev_to_detect_list
 *
 * @brief		add scan device information to connection list
 *
 * @param	devinfo - device information
 *
 * @return	TRUE-Add OK; FALSE - list full
 */
static bool add_dev_to_detect_list(gapDeviceInfoEvent_t devinfo)
{
	uint8 i=0;
	
	// Add new device info to scan result list
	if (ScanDevCount< DEV_DEFAULT_SCAN_RESPONSE)
	{
		for (i=0; i<ScanDevCount; i++ )
		{
			if (osal_memcmp(devinfo.addr, ScanDev[i].scan_dev_addr, B_ADDR_LEN))
				return TRUE;
		}
		
		osal_memcpy( ScanDev[i].scan_dev_addr, devinfo.addr, B_ADDR_LEN );
		ScanDev[i].scan_dev_bat = devinfo.pEvtData[BAT_PERCENT_POS_IN_ADV_DATA];
		ScanDev[i].scan_dev_power = GET_POWER_LEVEL(devinfo.pEvtData[DEVICE_STATUS_POS_IN_ADV_DATA]);
		ScanDev[i].scan_dev_rssi= devinfo.rssi;
		ScanDevCount++;
	}
	else
		return FALSE;

	return TRUE;
}

/*********************************************************************
 * @fn		UPLBSProcessGATTMsg
 *
 * @brief	 	Process GATT messages
 *
 *@param	pMsg - message from GATT
 *
 * @return	none
 */
static void UPLBSProcessGATTMsg( gattMsgEvent_t *pMsg )
{	
	// In case a GATT message came after a connection has dropped, ignore the message
	if ( UPLBSBLEState != UPLBS_BLE_STATE_CONNECTED )
		return;
	
	switch ( pMsg->method )
	{
		case ATT_READ_RSP:
		{
#if ( defined IO_DETECT )
			// After a successful read, display the parking state
			uint8 valueRead = pMsg->msg.readRsp.value[0];

			PrintIntValue("\r\nParking state:", valueRead, 10);

			// Disconect
			UPLBSBLEState = UPLBS_BLE_STATE_DISCONNECTING;
			GAPCentralRole_TerminateLink( UPLBSCentralConnHandle );
#endif	/* IO_DETECT */
			break;
		}
		case ATT_WRITE_RSP:
		{
			// Write succesful
//			PrintString("\r\nWR OK");
			break;

		}
		case ATT_HANDLE_VALUE_IND:
		{
			break;
		}
		case ATT_HANDLE_VALUE_NOTI:
		{
#if ( defined BLE_BASE )
			if (BLE_noti_recv_data(pMsg->msg.handleValueNoti,PosData+B_ADDR_LEN,\
					&TotLen) == TRUE )
			/*UPLBS_pkt_fmt_t *recv_data;
			uint8 len;

			if (TotLen == 0)
			{
				osal_memcpy(PosData, ConnList.conn_addr,B_ADDR_LEN);
				TotLen += B_ADDR_LEN;
			}

			recv_data = (UPLBS_pkt_fmt_t *) pMsg->msg.handleValueNoti.value;
			len=recv_data->pkt_hdr.pkt_len;
			osal_memcpy(PosData+TotLen, recv_data->pkt_txt, len);
			TotLen +=len;
			
			// Finish recieving, change CARDROID mode
			if (recv_data->pkt_hdr.cur_pkt_num == recv_data->pkt_hdr.tot_pkt_cnt)*/
			{
				PosDataFinish = TRUE;
				
			}
#endif	/* BLE_BASE */
			break;
		}
		case ATT_FIND_BY_TYPE_VALUE_RSP:
		{
			if (UPLBSDiscState != UPLBS_BLE_DISC_STATE_SVC)
				break;
			
			if ( pMsg->hdr.status == SUCCESS )
			{
				// Service found after perform UPLBSGATTStartDiscService, store handles
				if (pMsg->msg.findByTypeValueRsp.numInfo > 0 )
				{
					UPLBSSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
					UPLBSSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
				}
			}
			else if (pMsg->hdr.status == bleProcedureComplete )
			{
				if ( UPLBSSvcStartHdl != 0 )
				{
					// Discover characteristic
					UPLBSDiscState = UPLBS_BLE_DISC_STATE_CHAR;

					attReadByTypeReq_t req;

					req.startHandle = UPLBSSvcStartHdl;
					req.endHandle = UPLBSSvcEndHdl;
					req.type.len = ATT_BT_UUID_SIZE;
#if ( defined IO_DETECT )
					req.type.uuid[0] = LO_UINT16(UPLBS_CHAR_PARKING_STATE_UUID);
					req.type.uuid[1] = HI_UINT16(UPLBS_CHAR_PARKING_STATE_UUID);
#elif ( defined BLE_BASE )
					req.type.uuid[0] = LO_UINT16(UPLBS_CHAR_RUN_STATE_UUID);
					req.type.uuid[1] = HI_UINT16(UPLBS_CHAR_RUN_STATE_UUID);
#endif	/* IO_DETECT */

					VOID GATT_ReadUsingCharUUID( UPLBSCentralConnHandle, &req, UPLBS_TaskId);
				}
			}
			else
				PrintIntValue("\r\nEXCP:",pMsg->hdr.status,16);
			
			break;
		}
		case ATT_READ_BY_TYPE_RSP:
		{
			if (UPLBSDiscState != UPLBS_BLE_DISC_STATE_CHAR)
				break;
			
			if ( pMsg->hdr.status == SUCCESS )
			{
				// Characteristic found after perform GATT_ReadUsingCharUUID, store handle
				if( pMsg->msg.readByTypeRsp.numPairs > 0 )
				{
					UPLBSCharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],\
							pMsg->msg.readByTypeRsp.dataList[1] );
				}
			}
			else if (pMsg->hdr.status == bleProcedureComplete )
			{
				UPLBSDiscState = UPLBS_BLE_DISC_STATE_IDLE;
#if ( defined IO_DETECT )
				// Begin read parking state
				read_data_by_handle(UPLBSCentralConnHandle,UPLBSCharHdl, UPLBS_TaskId);
#endif	/* IO_DETECT */
			}
			else
				PrintIntValue("\r\nEXCP:", pMsg->hdr.status,10);
			
			break;
		}
		case ATT_ERROR_RSP:
		{
			// Error process
			switch (pMsg->msg.errorRsp.reqOpcode)
			{
				case ATT_READ_REQ:
					PrintIntValue("\r\n!!!Read Error:",pMsg->msg.errorRsp.errCode,16);
					break;
				case ATT_WRITE_REQ:
					PrintIntValue("\r\n!!!Write Error:",pMsg->msg.errorRsp.errCode,16);
					break;
				case ATT_FIND_BY_TYPE_VALUE_REQ:
					PrintIntValue("\r\n!!!Find type Error:",pMsg->msg.errorRsp.errCode,16);
					break;
				case ATT_READ_BY_TYPE_REQ:
					PrintIntValue("\r\n!!!Read type Error:",pMsg->msg.errorRsp.errCode,16);
					break;
				default:
					PrintIntValue("\r\n!!!Error REQ:",pMsg->msg.errorRsp.reqOpcode,10);
					break;
			}
			break;
		}
		default:
			PrintIntValue("\r\nUntreated method:",pMsg->method,10);
			break;
	}
}

/*********************************************************************
 * @fn			UPLBSGATTStartDiscService
 *
 * @brief	 Start service discovery.
 *
 * @return	none
 */
static void UPLBSGATTStartDiscService( void )
{
	uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(UPLBS_SERV_UUID),
			HI_UINT16(UPLBS_SERV_UUID) };
	
	// Initialize cached handles
	UPLBSSvcStartHdl = UPLBSSvcEndHdl = UPLBSCharHdl = 0;
	UPLBSDiscState = UPLBS_BLE_DISC_STATE_SVC;
	
	// Discover service
	GATT_DiscPrimaryServiceByUUID( UPLBSCentralConnHandle,uuid, ATT_BT_UUID_SIZE,UPLBS_TaskId );
}

/*********************************************************************
 * @fn		clear_conn_list
 *
 * @brief	 	clear the device connect list
 *
 * @param	none
 * @return	none
 */
static void clear_conn_list(void)
{	
	ConnListCount = 0;
	osal_memset( (uint8 *)ConnList, 0, sizeof(ConnList));

	return;
}

/*********************************************************************
 * @fn		PrintDevInfo
 *
 * @brief		Serial print device information
 *
 * @param	devinfo - device information
 *
 * @return	none
 */
void PrintDevInfo(gapDeviceInfoEvent_t devinfo)
{
	PrintIntValue(ble_addr_to_str(devinfo.addr),devinfo.rssi,10);
//	PrintIntValue(" VN:",devinfo.pEvtData[PROTO_VERN_POS_IN_ADV_DATA],10);
//	PrintIntValue(" BP:",devinfo.pEvtData[BAT_PERCENT_POS_IN_ADV_DATA],10);
	PrintIntValue("% PL:",GET_POWER_LEVEL(devinfo.pEvtData[DEVICE_STATUS_POS_IN_ADV_DATA]),10);
//	PrintIntValue(" TP:",devinfo.pEvtData[BEACON_TYPE_POS_IN_ADV_DATA],16);
}

#if ( defined BLE_BASE )
/*********************************************************************
 * @fn		clear_pos_data
 *
 * @brief	 	clear the position data
 *
 * @param	none
 * @return	none
 */
static void clear_pos_data(void)
{	
	TotLen=0;
	osal_memset(PosData,0,sizeof(PosData));
	
	return;
}
#endif	/* BLE_BASE */

#endif	/* IO_DETECT || BLE_BASE */
#endif	/* CARDROID || IO_DETECT || BLE_BASE */

#if ( defined CARDROID )
/*********************************************************************
 * @fn		GAP_role_swtich
 *
 * @brief	 	Switch GAP role when CARDROID working
 *
 * @param	none
 *
 * @return	none
 */
static void GAP_role_swtich(void)
{
	RoleSwitching = TRUE;
	
	if (SysState == SYS_PERIPHERAL || SysState == SYS_SENT_OVER )
	{		
		// Do not change role until advertising stopped (GAPROLE_WAITING state) or disconnected
		if (UPLBSBLEState == UPLBS_BLE_STATE_ADVERTISING )
		{
			ble_stop_advertising();
			return;
		}
		else if (UPLBSBLEState == UPLBS_BLE_STATE_CONNECTED )
		{
			GAPRole_TerminateConnection();
			return;
		}
		clear_scan_list();
		
		SysState = SYS_CENTRAL;
		VOID GAPCentralRole_StartDevice( &UPLBS_CentralCBs );
		// Fast blink, begin scan
		BlinkLed (HAL_LED_1, START_SCAN_BLINK_TIMES, NORMAL_LED_ON_PERCENTAGE, START_SCAN_BLINK_CYCLE);

		PrintString("\r\nMC");
	}
	else
	{	
		// Stop scan if not finished
		if (UPLBSBLEState == UPLBS_BLE_STATE_SCANNING)
		{
			GAPCentralRole_CancelDiscovery();
			return;
		}
		osal_stop_timerEx(UPLBS_TaskId,START_BLE_SCAN_EVT);

		SysState = SYS_PERIPHERAL;
		
		// Reinit peripheral params
		DevAdvertData[POSITION_STAT_POS_IN_ADV_DATA] = ScanListSent;
		init_gap_peripheral_role();
		
		VOID GAPRole_StartDevice( &UPLBS_PeripheralCBs);

		PrintString("\r\nMP");
	}
	RoleSwitching = FALSE;
	
	return;
}

/*********************************************************************
 * @fn		add_dev_to_scan_list
 *
 * @brief		add scan device information to result list
 *
 * @param	none
 *
 * @return	TRUE-Add OK
 */
static bool add_dev_to_scan_list(gapDeviceInfoEvent_t devinfo)
{
	uint8 i,pwl=GET_POWER_LEVEL(devinfo.pEvtData[DEVICE_STATUS_POS_IN_ADV_DATA]);

	for (i=0; i<DevInScanList; i++ )
	{
		if (osal_memcmp(devinfo.addr, ScanList[i].baseaddr, B_ADDR_LEN))
		{	
			// Record scan result based on different power level
			ScanList[i].scantimes[pwl]++;
			RssiSum[i][pwl] += (int16)devinfo.rssi;
			
			return TRUE;
		}
	}

	// Add new device info to scan result list
	if (DevInScanList< MAX_SCAN_LIST_LEN)
	{
		ScanList[i].basetype = devinfo.pEvtData[BEACON_TYPE_POS_IN_ADV_DATA];
		osal_memcpy( ScanList[i].baseaddr, devinfo.addr, B_ADDR_LEN );
		ScanList[i].scantimes[pwl] = 1;
		RssiSum[i][pwl] = (int16)devinfo.rssi;
		DevInScanList++;
	}
	else
		return FALSE;

	return TRUE;
}

/*********************************************************************
 * @fn		calc_avg_rssi
 *
 * @brief	 	calculate average RSSI value of each beacon
 *
 * @param	none
 * @return	none
 */
static void calc_avg_rssi(void)
{
	uint8 i,j;
	
	for (i=0; i<DevInScanList; i++)
		for (j=0; j<POWER_LEVEL_COUNT; j++)
		{
			if (ScanList[i].scantimes[j] != 0)
				ScanList[i].rssival[j] = (int8)(RssiSum[i][j]/ScanList[i].scantimes[j]);
		}
}

/*********************************************************************
 * @fn		clear_scan_list
 *
 * @brief	 	clear the device discovery result list
 *
 * @param	none
 * @return	none
 */
static void clear_scan_list(void)
{
	ScanTime = 0;
	ScanListSent=TRUE;
	
	DevInScanList=0;
	osal_memset( (uint8 *)ScanList, 0, sizeof(ScanList));
	osal_memset((uint8 *)RssiSum, 0, sizeof(RssiSum));

	return;
}

/*********************************************************************
 * @fn		set_GAP_state
 *
 * @brief	 	Set GAP state in CARDROID mode
 *
 * @return	none
 */
void set_GAP_state(uint8 set_role)
{
	SysState = set_role;

	return;
}

 /*********************************************************************
 * @fn		get_GAP_state
 *
 * @brief		Get GAP state in CARDROID mode
 *
 * @return	GAP state
 */
uint8 get_GAP_state(void)
{
	return SysState;
}
#endif	/* CARDROID */

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

#if ( defined CARDROID )
	if (RxTmpPwrOn == TRUE)
		osal_stop_timerEx(UPLBS_TaskId,UART_WAKE_UP_EVT);
#endif

	switch(SET_OUTPUT_TYPE(type))
	{
		case BLE_DEV_ADDR:
		{
			LBSsend(BLE_DEV_ADDR,OwnAddress,sizeof(OwnAddress));
			break;
		}
		case BLE_DEV_VERSION:
		{
			UPLBS_vern_t send_vern_info;
			fill_vern_info(& send_vern_info);
			LBSsend(BLE_DEV_VERSION,(uint8 *)&send_vern_info, sizeof(send_vern_info));
			break;
		}
		case BLE_DEV_SET_PARAMS:
			//params_modify();
			break;
		case BLE_DEV_GET_PARAMS:
			//params_modify();
			break;
		case BLE_DEV_CAR_SCAN_DATA:
#if ( defined BLE_BASE )
#endif	/* BLE_BASE */
			break;
		case BLE_DEV_START_SCAN:
		{
#if ( defined BLE_BASE )
			if (length > 0)
			{
				if (LBSBcnDetctCnts == 0)
					LBSBcnDetctCnts= buffer[0];
				else
					cmd_status = CMD_NOT_READY;
			}
			else
				cmd_status = CMD_INVALID;
			LBSsend(BLE_DEV_START_SCAN, &cmd_status, 1);
#endif	/* BLE_BASE */
			break;
		}
		case BLE_DEV_SCAN_RSLT:
		{
			//params_modify();
			break;
		}
		case BLE_DEV_UPGRADE:
		{
#if ( defined OAD_UPGRADE )
#if	( defined POWER_SAVING )
			system_power_hold(UPLBS_TaskId);
#endif	/* POWER_SAVING */
			prepare_oad_upgrade();
#endif	/* OAD_UPGRADE */
			break;
		}
		case BLE_DEV_REBOOT:
		{
			HAL_SYSTEM_RESET();
			break;
		}
		default:
			break;
	}

#if ( defined CARDROID )
	if (RxTmpPwrOn == TRUE)
		osal_start_timerEx(UPLBS_TaskId,UART_WAKE_UP_EVT,3000);
#endif

	VOID cmd_status;
#endif	/* HAL_UART && HAL_UART == TRUE */
};



