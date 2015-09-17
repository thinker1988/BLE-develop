/**************************************************************************************************
	Filename:			 simpleBLECentral.c
	Revised:				$Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
	Revision:			 $Revision: 28 $

	Description:		This file contains the Simple BLE Central sample application 
									for use with the CC2540 Bluetooth Low Energy Protocol Stack.

	Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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
	PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#include "hal_led.h"
#include "hal_lcd.h"
#include "hal_i2c.h"
#include "hal_adc.h"

#include "BLECore.h"
#include "Com433.h"
#include "GMProc.h"
#include "Pktfmt.h"

#if ( defined USE_CC112X_RF )
#include "Cc112x.h"
#endif	// USE_CC112X_RF

#if ( !defined NOT_USE_BLE_STACK && defined ALLOW_BLE_ADV )
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "peripheral.h"
#endif


/*********************************************************************
 * MACROS
 */
#define CLAC_GM_TEMPR(HiBt,LoBt)			(((HiBt)<<8+(LoBt))>>7+GM_BASE_TEMPR)

/*********************************************************************
 * CONSTANTS
 */
#if ( !defined NOT_USE_BLE_STACK && defined ALLOW_BLE_ADV )
#define BLE_CORE_DEV_TYPE				0xC9C9

#define BLE_CORE_ADV_INTVL				8000		//(units of 625us, 8000*0.625

#define BLE_CORE_MIN_CON_INTVL			80

#define BLE_CORE_MAX_CON_INTVL			80

#define BLE_CORE_SLAVE_LATENCY			0

#define BLE_CORE_CON_TIMEOUT			50

#define BLE_CORE_UPDATE_FLAG			TRUE

#define BLE_CORE_UPDATE_PAUSE			1
#endif


//  Geomagnetic sensor ID
#define GM_ID_STR						"H43"

// Geomagnetic sensor read period
#define GM_READ_EVT_PERIOD 			5000		//1000-> 1s

// Heart beat
#define HEART_BEAT_EVT_PERIOD 		120000	//600s = 10min heart beat now 2min

//G-Sensor I2C address
#define GM_I2C_ADDR					0x1E

//G-Sensor register address
#define GM_CONFIG_A_REG				0x00
#define GM_CONFIG_B_REG				0x01
#define GM_MODE_REG					0x02
#define GM_X_AXIS_MSB_REG				0x03
#define GM_X_AXIS_LSB_REG				0x04
#define GM_Z_AXIS_MSB_REG				0x05
#define GM_Z_AXIS_LSB_REG				0x06
#define GM_Y_AXIS_MSB_REG				0x07
#define GM_Y_AXIS_LSB_REG				0x08
#define GM_STATUS_REG					0x09
#define GM_ID_A_REG					0x0A
#define GM_ID_B_REG					0x0B
#define GM_ID_C_REG					0x0C
#define GM_TEMPR_MSB_REG				0x31
#define GM_TEMPR_LSB_REG				0x32

//GM base temperature
#define GM_BASE_TEMPR 					25

// enable temperature sensor, 8 average 15Hzoutput rate
#define SET_GM_NORMAL				0xF0

// Gaim =5
#define SET_GM_GAIN					0xA0

// Single measurement
#define SET_GM_READ_ONCE			0x21

// Sleep mode
#define SET_GM_SLEEP_MODE			0x03



/******************************
	HW_VERN  0:	use Dupont line connect GM detector
	HW_VERN  1:	use TEN308 as RF
	HW_VERN  2:	use TI CC1121 as RF
*******************************/

/***************GM detector define********************/
#if ( !defined HW_VERN ) || ( HW_VERN == 0 )
#define GM_DRDY_INT_PINSEL 			P1SEL
#define GM_DRDY_INT_PINDIR			P1DIR
#define GM_DRDY_INT_PXIFG			P1IFG
#define GM_DRDY_INT_PXIF 			P1IF
#define GM_DRDY_INT_PXIEN 			P1IEN

#define GM_DRDY_INT_IE				BV(6)

#elif ( HW_VERN >= 1 )

#define GM_DRDY_INT_PINSEL 			P0SEL
#define GM_DRDY_INT_PINDIR			P0DIR
#define GM_DRDY_INT_PXIFG			P0IFG
#define GM_DRDY_INT_PXIF 			P0IF
#define GM_DRDY_INT_PXIEN 			P0IEN
#define GM_DRDY_INT_IE				BV(1)

// Hgswitch high:deep sleep; low:working
#define DEV_EN_INT_PINSEL 			P0SEL
#define DEV_EN_INT_PINDIR			P0DIR
#define DEV_EN_INT_PXIEN 			P0IEN
#define DEV_EN_INT_IE				BV(4)
#define DEV_EN_INT_PXIFG			P0IFG

#define DEV_EN_INT_PIN				P0_4

#endif
#define GM_DRDY_INT_ENABLE()		st( GM_DRDY_INT_PXIEN |= GM_DRDY_INT_IE; )
#define GM_DRDY_INT_DISABLE()		st( GM_DRDY_INT_PXIEN &= ~GM_DRDY_INT_IE; )

#define DEV_EN_INT_ENABLE()			st( DEV_EN_INT_PXIEN |= DEV_EN_INT_IE; )
#define DEV_EN_INT_DISABLE()		st( DEV_EN_INT_PXIEN &= ~DEV_EN_INT_IE; )


/***************TEN308 RF define********************/
#if ( !defined USE_CC112X_RF )
#define TEN308_DIOA_PINSEL				P0SEL
#define TEN308_DIOA_PINDIR				P0DIR
#define TEN308_DIOA_PININP				P0INP
#define TEN308_DIOA_GPIO				BV(6)
#define TEN308_DIOA_PIN					P0_6

#define TEN308_DIOB_PINSEL				P1SEL
#define TEN308_DIOB_PINDIR				P1DIR
#define TEN308_DIOB_PININP				P1INP
#define TEN308_DIOB_GPIO				BV(2)
#define TEN308_DIOB_PIN					P1_2
#endif	// !USE_CC112X_RF


// ADC expected value : (V*100/370)/1.25*2047
// MAX : 3.8~1682		MIN : 2.5~1106
#define BATT_ADC_VAL_MAX					1682
#define BATT_ADC_VAL_MIN					1106

#define BLE_MAC_ADDR_IN_FLASH		0x780E

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint32 hrtbt_timeout = HEART_BEAT_EVT_PERIOD;

// Task ID for internal task/event processing
uint8 BLECore_TaskId;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
#if ( ! defined USE_CC112X_RF )
extern uint8 rfsndbuf[];
extern uint8 rfsndlen;
#endif
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint32 next=1;

static uint32 randwait;

static bool SysWakeup = FALSE;

static bool synctm;

static i2cClock_t i2cclk = i2cClock_123KHZ;

static bool RFrxtx=FALSE; 

#if ( !defined NOT_USE_BLE_STACK && defined ALLOW_BLE_ADV )
// GAP - Advertisement data (max size = 31 bytes, though this is best kept short to conserve power while advertisting)
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

// GAP - SCAN RSP data (max size = 31 bytes)
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
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
#if ( !defined USE_CC112X_RF )
static void TEN_RF_Init(void);
static void TEN_RF_stop(void);
#endif	// !USE_CC112X_RF

static bool GM_dev_init(void);
static void GM_dev_stop(void);

static void prepare_gm_read(void);
static void read_gm_data(void);

static bool gm_read_reg(uint8 addr,uint8 * pBuf,uint8 nBytes);
static bool gm_write_reg(uint8 addr, uint8 *pBuf, uint8 nBytes);

#if 0
static void GM_DRDY_INT_Cfg(void);
#endif

static uint32 c_rand(void);
static void c_srand(uint32 seed);

static void ReadBLEMac(uint8 * mac_addr);

#if ( !defined NOT_USE_BLE_STACK && defined ALLOW_BLE_ADV )
static void init_gap_periph_role(void);
static void init_gap_periph_params(void);
static void BLECorePeriphNotiCB(gaprole_States_t newState);
#endif
static void SYS_WS_INT_Cfg(void);


/*********************************************************************
 * PROFILE CALLBACKS
 */
#if ( !defined NOT_USE_BLE_STACK && defined ALLOW_BLE_ADV )
static gapRolesCBs_t BLECore_PeriphCBs =
{
	BLECorePeriphNotiCB,	// Profile State Change Callbacks
	NULL							// When a valid RSSI is read from controller (not used by application)
};
#endif
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn			BLECore_Init
 *
 * @brief	 Initialization function for the Simple BLE Central App Task.
 *					This is called during initialization and should contain
 *					any application specific initialization (ie. hardware
 *					initialization/setup, table initialization, power up
 *					notification).
 *
 * @param	 task_id - the ID assigned by OSAL.	This ID should be
 *										used to send messages and set timers.
 *
 * @return	none
 */
void BLECore_Init( uint8 task_id )
{
	uint8 BleMac[B_ADDR_LEN];
	BLECore_TaskId = task_id;

#if (defined POWER_SAVING)
	powersave(BLECore_TaskId);
#endif

	//HalLedBlink( HAL_LED_ALL,5,50,1000);
	Com433_Init();

#if ( defined USE_CC112X_RF )
	initRFcfg();
#endif	// USE_CC112X_RF

	ReadBLEMac(BleMac);

	c_srand(BUILD_UINT32(BleMac[0],BleMac[1],BleMac[2],BleMac[3]));
	// c_rand:0.001~32s, then distribute in heart beat time
	randwait = c_rand()*(hrtbt_timeout/1000/33);
	
#if ( !defined NOT_USE_BLE_STACK && defined ALLOW_BLE_ADV )
	HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_23_DBM);

	init_gap_periph_role();
	init_gap_periph_params();
	VOID GAPRole_StartDevice( &BLECore_PeriphCBs );
#endif
	// Start working
	osal_set_event( BLECore_TaskId, BLE_CORE_START_EVT );

	return;
}

/*********************************************************************
 * @fn		BLECore_ProcessEvent
 *
 * @brief	 Simple BLE Central Application Task event processor.	This function
 *					is called to process all events for the task.	Events
 *					include timers, messages and any other user defined events.
 *
 * @param	 task_id	- The OSAL assigned task ID.
 * @param	 events - events to process.	This is a bit map and can
 *									 contain more than one event.
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

	if ( events & BLE_CORE_START_EVT )
	{
#if ( defined TEST_433 )
		//RFwakeup();
#if 1
		rx_test();
#else
		osal_start_reload_timer(task_id,HEART_BEAT_EVT,5000);
#endif
#else
		SYS_WS_INT_Cfg();
		if ( SysWakeup == FALSE )
		{
			powerhold(task_id);
			osal_stop_timerEx(task_id, READ_GM_DATA_EVT);
			osal_stop_timerEx(task_id, GM_DRDY_INT_INT_EVT);
			osal_stop_timerEx(task_id, HEART_BEAT_EVT);
			osal_stop_timerEx(task_id, CORE_PWR_SAVING_EVT);
#if ( defined USE_CC112X_RF )
			osal_stop_timerEx(task_id, RF_RXTX_RDY_EVT);
#else
			osal_stop_timerEx(task_id, TEN_TX_RDY_EVT);
#endif
			GM_dev_stop();
			RFsleep();
			powersave(task_id);
			/*SLEEPCMD |= 0x03; // PM3
			PCON = 0x01;*/
		}
		else
		{
/*			RFwakeup();
			osal_start_timerEx(BLECore_TaskId, READ_GM_DATA_EVT,100);*/
			initGMstate();
			settmsync();

			if (GM_dev_init() == TRUE)
			{
				osal_start_reload_timer(task_id,READ_GM_DATA_EVT,GM_READ_EVT_PERIOD);
				// random start first heart beat
				osal_start_timerEx(task_id, HEART_BEAT_EVT, randwait);
			}
			else
				Com433WriteStr(COM433_DEBUG_PORT,"\r\nGM init failed...");
		}
#endif

		return ( events ^ BLE_CORE_START_EVT );
	}

	if ( events & READ_GM_DATA_EVT)
	{
		powerhold(BLECore_TaskId);
		prepare_gm_read();
/*		Com433WriteStr(COM433_WORKING_PORT,"\r\nTEST...");
		Com433WriteStr(COM433_DEBUG_PORT,"\r\nTEST...");

		osal_start_timerEx(BLECore_TaskId, CORE_PWR_SAVING_EVT,500);*/

		return (events ^ READ_GM_DATA_EVT);
	}
	
	if ( events & GM_DRDY_INT_INT_EVT)
	{
		read_gm_data();

		return (events ^ GM_DRDY_INT_INT_EVT);
	}

	if (events & HEART_BEAT_EVT)
	{
#if ( defined TEST_433 )
		tx_test();
#else
		set_heart_beat();
		osal_start_timerEx(task_id, HEART_BEAT_EVT, hrtbt_timeout);
#endif

		return (events ^ HEART_BEAT_EVT);
	}

	if (events & CORE_PWR_SAVING_EVT)
	{
		powersave(BLECore_TaskId);
		if (RFrxtx == TRUE)
			RFsleep();

		return (events ^ CORE_PWR_SAVING_EVT);
	}

#if ( defined USE_CC112X_RF )	
	if (events & RF_RXTX_RDY_EVT)
	{
		RFmodechange();

		return (events ^ RF_RXTX_RDY_EVT);
	}
#else
	if (events & TEN_TX_RDY_EVT)
	{
		Com433Write(COM433_WORKING_PORT, rfsndbuf, rfsndlen);
		// leave 1000 ms recieve
		osal_start_timerEx(BLECore_TaskId, CORE_PWR_SAVING_EVT, IDLE_PWR_HOLD_PERIOD);

		return (events ^ TEN_TX_RDY_EVT);
	}
#endif	// USE_CC112X_RF

	return 0;
}


/*********************************************************************
 * @fn		powersave
 *
 * @brief		Set system power mode to PM3
 *
 * @param	none
 * @return	none
 */
void powersave(uint8 task_id)
{
#if ( defined POWER_SAVING )
	(void)osal_pwrmgr_task_state(task_id, PWRMGR_CONSERVE);
	osal_pwrmgr_device( PWRMGR_BATTERY );
#endif
	VOID task_id;
}

/*********************************************************************
 * @fn		powerhold
 *
 * @brief		Hold system power
 *
 * @param	none
 * @return	none
 */
void powerhold(uint8 task_id)
{
#if ( defined POWER_SAVING )
	osal_pwrmgr_device( PWRMGR_ALWAYS_ON);
	(void)osal_pwrmgr_task_state(task_id, PWRMGR_HOLD);
#endif
	VOID task_id;
}


void RFwakeup(void)
{
	RFrxtx = TRUE;
#if ( !defined USE_CC112X_RF )
	TEN_RF_Init();
#endif	// !USE_CC112X_RF
}

void RFsleep(void)
{
	RFrxtx = FALSE;
#if ( defined USE_CC112X_RF )
	RFentersleep();
#else
	TEN_RF_stop();
#endif	// USE_CC112X_RF
}

void settmsync(void)
{
	synctm = TRUE;
}

void cleartmsync(void)
{
	synctm = FALSE;
}

bool gettmsync(void)
{
	return synctm;
}

/*********************************************************************
 * @fn		CalcBatteryPercent
 *
 * @brief	get battery percent
 *
 * @param	none
 *
 * @return	battery percent
 */
uint8 CalcBatteryPercent(void)
{
	uint16 adc;
	uint8 percent;

	// Configure ADC and perform a read
	HalAdcSetReference( HAL_ADC_REF_125V );
	
	adc = HalAdcRead( HAL_ADC_CHN_AIN0,HAL_ADC_RESOLUTION_12);

	if (adc >= BATT_ADC_VAL_MAX)
		percent = 100;
	else if (adc <=	BATT_ADC_VAL_MIN)
		percent = 0;
	else
		// make sure the calculation will not overflow
		percent = (uint8) (((adc-BATT_ADC_VAL_MIN) * 100) / (BATT_ADC_VAL_MAX-BATT_ADC_VAL_MIN));

	return percent;
}

int8 GetGDETmpr(void)
{
	uint8 tmprval[2] = {0};
	int16 fulltmprval = 0;
	int8 tmpr;
	
	gm_read_reg(GM_TEMPR_MSB_REG, tmprval, sizeof(tmprval));
	
	fulltmprval = BUILD_UINT16(tmprval[1], tmprval[0]);
	tmpr = (int8)(fulltmprval/128) + GM_BASE_TEMPR;

	return tmpr;
}

#if ( !defined USE_CC112X_RF )
void prepare_TEN_send(void)
{
	osal_start_timerEx(BLECore_TaskId,TEN_TX_RDY_EVT,250);
}
#endif

void Com433Handle(uint8 port,uint8 *pBuffer, uint16 length)
{
	if (port == COM433_WORKING_PORT)
	{
#if ( !defined USE_CC112X_RF && defined TEN_DEBUG_MODE )
		Com433Write(COM433_DEBUG_PORT, pBuffer, length);
#else
		// no func will be called in CC112X RF
		gmspktform(pBuffer,length);
#endif	// !USE_CC112X_RF && TEN_DEBUG_MODE
	}
	else if (port == COM433_DEBUG_PORT)
	{
#if ( !defined USE_CC112X_RF )
		Com433Write(COM433_WORKING_PORT, pBuffer, length);
#endif	// !USE_CC112X_RF
	}

	return;
}

/*********************************************************************
 * PRIVATE FUNCTIONS
 */

#if ( !defined USE_CC112X_RF )
static void TEN_RF_Init(void)
{
	TEN308_DIOA_PINSEL &= (uint8) ~TEN308_DIOA_GPIO;
	TEN308_DIOA_PINDIR |= (uint8) TEN308_DIOA_GPIO;
	TEN308_DIOA_PIN = 0;

	TEN308_DIOB_PINSEL &= (uint8) ~(TEN308_DIOB_GPIO);
	TEN308_DIOB_PINDIR |= (uint8) (TEN308_DIOB_GPIO);
	TEN308_DIOB_PIN = 0;
}

static void TEN_RF_stop(void)
{
	TEN308_DIOA_PINSEL &= (uint8) ~(TEN308_DIOA_GPIO);
	TEN308_DIOA_PINDIR &= (uint8) ~(TEN308_DIOA_GPIO);
	//TEN308_DIOA_PINDIR |= (uint8) (TEN308_DIOA_GPIO);
	TEN308_DIOA_PININP |= (uint8)(TEN308_DIOA_GPIO);
	//TEN308_DIOA_PIN = 1;

	TEN308_DIOB_PINSEL &= (uint8) ~(TEN308_DIOB_GPIO);
	TEN308_DIOB_PINDIR &= (uint8) ~(TEN308_DIOB_GPIO);
	//TEN308_DIOB_PINDIR |= (uint8) (TEN308_DIOB_GPIO);
	TEN308_DIOB_PININP |= (uint8) (TEN308_DIOB_GPIO);
	//TEN308_DIOB_PIN = 1;
}
#endif	// !USE_CC112X_RF


/*********************************************************************
 * @fn		GM_dev_init
 *
 * @brief	GM device init
 *
 * @param	none
 * @return	TRUE - init OK; FALSE - failed
 */
static bool GM_dev_init(void)
{
	uint8 id_str[3];
	uint8 gm_cfg = SET_GM_NORMAL;
	uint8 gm_gain = SET_GM_GAIN;
	
	HalI2CInit(GM_I2C_ADDR, i2cclk);

	gm_read_reg(GM_ID_A_REG,id_str,sizeof(id_str));

	if (osal_memcmp(id_str,GM_ID_STR,sizeof(id_str)) == FALSE)
		return FALSE;

//	Com433WriteStr(COM433_DEBUG_PORT,"\r\nGM:");
//	Com433Write(COM433_DEBUG_PORT,id_str,sizeof(id_str));
	
	gm_write_reg(GM_CONFIG_A_REG,&gm_cfg,1);
	gm_write_reg(GM_CONFIG_B_REG,&gm_gain,1);

#if 0
	GM_DRDY_INT_Cfg();
#endif

	return TRUE;
}

/*********************************************************************
 * @fn		GM_dev_stop
 *
 * @brief	GM device stop
 *
 * @param	none
 * @return	none
 */
static void GM_dev_stop(void)
{
	uint8 sleep_mode = SET_GM_SLEEP_MODE;

	HalI2CInit(GM_I2C_ADDR, i2cclk);
	gm_write_reg(GM_MODE_REG, &sleep_mode, 1);
}

static void prepare_gm_read(void)
{
	uint8 singl_read = SET_GM_READ_ONCE;

	// reinit I2C after power saving
	HalI2CInit(GM_I2C_ADDR, i2cclk);
	gm_write_reg(GM_MODE_REG, &singl_read, 1);

	osal_start_timerEx(BLECore_TaskId, GM_DRDY_INT_INT_EVT, 100);
}

/*********************************************************************
 * @fn			read_gm_data
 *
 * @brief	 read i2c
 *
 * @param	 none
 * @return	none
 */
static void read_gm_data(void)
{
	uint8 tmp_data[6]={0};
	uint16 len=0;
	int16 xval,yval,zval;

	len = gm_read_reg(GM_X_AXIS_MSB_REG, tmp_data, sizeof(tmp_data));
	
	if (len <= 0)
	{
		Com433WriteStr(COM433_DEBUG_PORT,"\r\nI2C read failed!!!");
		osal_stop_timerEx(BLECore_TaskId, READ_GM_DATA_EVT);
	}
	else
	{
		xval = BUILD_UINT16(tmp_data[1],tmp_data[0]);
		yval = BUILD_UINT16(tmp_data[5],tmp_data[4]);
		zval = BUILD_UINT16(tmp_data[3],tmp_data[2]);

		gm_data_proc(xval,yval,zval);
	}
	// RF have no data send, sleep 100 for serial output;
	if (RFrxtx == FALSE)
	{
#if ( defined ALLOW_DEBUG_OUTPUT )
		osal_start_timerEx(BLECore_TaskId, CORE_PWR_SAVING_EVT,100);
#else
		osal_set_event(BLECore_TaskId, CORE_PWR_SAVING_EVT);
#endif
	}

	return;
}


/**************************************************************************************************
* @fn					gm_write_reg
* @brief			 This function implements the I2C protocol to write to a sensor. he sensor must
*							be selected before this routine is called.
*
* @param			 addr - which register to write
* @param			 pBuf - pointer to buffer containing data to be written
* @param			 nBytes - number of bytes to write
*
* @return			TRUE if successful write
*/
static bool gm_write_reg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
	uint8 i;
	uint8 buffer[24];

	/* Copy address and data to local buffer for burst write */
	buffer[0] = addr;
	for (i = 0; i < nBytes; i++)
		buffer[1+i] = pBuf[i];
	nBytes++;

	/* Send address and data */
	i = HalI2CWrite(nBytes, buffer);

	return (i == nBytes);
}

/**************************************************************************************************
 * @fn					gm_read_reg
 *
 * @brief			 This function implements the I2C protocol to read from a sensor. The sensor must
 *							be selected before this routine is called.
 *
 * @param			 addr - which register to read
 * @param			 pBuf - pointer to buffer to place data
 * @param			 nBytes - numbver of bytes to read
 *
 * @return			TRUE if the required number of bytes are reveived
 */
static bool gm_read_reg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
	uint8 i = 0;

	/* Send address we're reading from */
	if (HalI2CWrite(1,&addr) == 1)
	{
		/* Now read data */
		i = HalI2CRead(nBytes,pBuf);
	}

	return i == nBytes;
}

#if 0
static void GM_DRDY_INT_Cfg(void)
{
#if ( !defined HW_VERN ) || ( HW_VERN == 0 )
	SET_P1_INT_DISABLE();
#else
	SET_P0_INT_DISABLE();
#endif

	GM_DRDY_INT_DISABLE();
	
	//Confige CC2541 pin
	GM_DRDY_INT_PINSEL &= (uint8) ~GM_DRDY_INT_IE;
	GM_DRDY_INT_PINDIR &= (uint8) ~GM_DRDY_INT_IE;

	GM_DRDY_INT_ENABLE();

#if ( !defined HW_VERN ) || ( HW_VERN == 0 )
	SET_P1_INT_DISABLE();
#else
	SET_P0_INT_ENABLE();
#endif
}
#endif

static void SYS_WS_INT_Cfg(void)
{
	SET_P0_INT_DISABLE();
	DEV_EN_INT_DISABLE();
	
	DEV_EN_INT_PINSEL &= (uint8) ~DEV_EN_INT_IE;
	DEV_EN_INT_PINDIR &= (uint8) ~DEV_EN_INT_IE;

	// Low to deep sleep
	if (DEV_EN_INT_PIN == 0)
	{
		SysWakeup = FALSE;
		// Rising edge detect
		SET_P0_INT_RISING_EDGE();
	}
	else
	{
		SysWakeup = TRUE;
		SET_P0_INT_FALLING_EDGE();
	}

	DEV_EN_INT_ENABLE();
	SET_P0_INT_ENABLE();
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

static void c_srand(uint32 seed)
{
	next=seed;
}


/*********************************************************************
 * @fn		ReadBLEMac
 *
 * @brief	Read own mac address in flash. Use *(unsigned char *) to read address
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

#if ( !defined NOT_USE_BLE_STACK && defined ALLOW_BLE_ADV )
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
#endif

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
		osal_set_event(BLECore_TaskId, BLE_CORE_START_EVT);
#endif

	// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
	GM_DRDY_INT_PXIFG = 0;
	GM_DRDY_INT_PXIF = 0;

	CLEAR_SLEEP_MODE();
	
	HAL_EXIT_ISR();
}

/*********************************************************************
*********************************************************************/
