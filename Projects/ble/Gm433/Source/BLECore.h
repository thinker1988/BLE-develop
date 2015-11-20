/********************************************************************
	Filename:	BLECore.h
	Revised:		Date: 2015-08-12
	Revision:	1.0 

*********************************************************************/

#ifndef BLECORE_H
#define BLECORE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// BLECore Task Events. Bit maps.
#define BLE_SYS_WORKING_EVT			0x0001
#define RF_DATA_PROC_EVT			0x0002
#define GM_DATA_PROC_EVT			0x0004
#define HG_SWITCH_EVT				0x0008
/*#define CORE_PWR_SAVING_EVT			0x0010
#define RF_RXTX_RDY_EVT				0x0020

#if ( !defined USE_CC112X_RF )
#define RF_WORK_EVT				0x0040
#endif	// USE_CC112X_RF
*/

// Self test period
#define SELF_TEST_PERIOD			1000UL

// Geomagnetic sensor read period
#define GM_READ_EVT_PERIOD 			5000UL		//1000-> 1s

// Idle hold period after RF send
#define IDLE_PWR_HOLD_PERIOD		1000UL

// No operation wait period
#define NO_OPERATION_WAIT_PERIOD	6000UL

// Hgswitch wait shake
#define SWITCH_WAIT_SHAKE_PERIOD	500UL

// Seconds in a minute & miliseconds in a second
#define SEC_IN_MIN					60UL
#define MILSEC_IN_SEC				1000UL
#define MILSEC_IN_MIN				(SEC_IN_MIN*MILSEC_IN_SEC)

// Minutes in a hour and hours in a day
#define MIN_IN_HOUR					60UL
#define HOUR_IN_DAY					24UL

// Max of c_rand results : 0.001~32s
#define MAX_RANDOM_SECONDS			33UL


/*********************************************************************
 * MACROS
 */
//config P2.0 P2.3 P2.4 P0 P1 input mode, 3 state
#define BC_INIT_IO_INPUT_3STATE()	st( P0SEL &= 0; P0DIR &= 0; P0INP |= 0xFF; \
											P1SEL &= 0; P1DIR &= 0; P1INP |= 0xFF; \
											P2INP |= BV(0)|BV(3)|BV(4);)
														
// Pin 0 Int enable/disable
#define SET_P0_INT_ENABLE()		st( IEN1 |=	BV(5); )
#define SET_P0_INT_DISABLE()	st( IEN1 &=	~(BV(5)); )

// Pin 1 Int enable/disable
#define SET_P1_INT_ENABLE()		st( IEN2 |=	BV(4); )
#define SET_P1_INT_DISABLE()	st( IEN2 &=	~(BV(4)); )

#define SET_P0_INT_FALLING_EDGE() 	st( PICTL |= BV(0); )
#define SET_P0_INT_RISING_EDGE() 	st( PICTL &= ~(BV(0)); )

#define SET_P1_INT_FALLING_EDGE() 	st( PICTL |= BV(1); )
#define SET_P1_INT_RISING_EDGE() 	st( PICTL &= ~(BV(1)); )


/*********************************************************************
 * TYPEDEFS
 */

// System working state
typedef enum
{
	SYS_BOOTUP,		// First boot
	SYS_WORKING,	// RF send
	SYS_SLEEPING,	// Sleeping
	SYS_DORMANT,	// Domancy
	SYS_SETUP,	// Setup
	SYS_UPGRADE	// Upgrade
}sysstate_t;


// System working state
typedef enum
{
	WS_INT_DISABLE,	// Disable int
	WS_INT_DETECT,	// Int type first detect
	WS_INT_CONFIRM,	// Int type confirm
	WS_INT_ENABLE,	// Enable int
}wsintstate_t;

/*********************************************************************
 * FUNCTIONS
 */

// Task Initialization for the BLE Application
extern void BLECore_Init( uint8 task_id );

// Task Event Processor for the BLE Application
extern uint16 BLECore_ProcessEvent( uint8 task_id, uint16 events );

extern void sys_working(uint8 task_id, sysstate_t newDevstate);
extern void SYS_WS_INT_Cfg(uint8 task_id, wsintstate_t curintst);

extern void SetSysState(sysstate_t newDevstate);
extern sysstate_t GetSysState(void);

extern void SetIntState(wsintstate_t newintstate);
extern wsintstate_t GetIntState(void);

extern void PowerSave(uint8 task_id);
extern void PowerHold(uint8 task_id);

extern uint8 CalcBatteryPercent(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BLECORE_H */
