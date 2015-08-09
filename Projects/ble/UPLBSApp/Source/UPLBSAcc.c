/**************************************************************************************************
	Filename:			BMA223.c
	Revised:			$Date: 2014-11-17 (Mon, 17 Nov 2014) $
	Revision:			$Revision: 1 $
	Author:			 LAN Chen


	Description:		This file contains the BLE GPIO and I2C device application 
					for use with the CC2541 Bluetooth Low Energy Protocol Stack.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "hal_i2c.h"
#include "hal_mcu.h"
#include "hal_types.h"
#include "OSAL.h"
#include "OSAL_timers.h"
#include "OSAL_PwrMgr.h"


#include "UPLBSAcc.h"
#include "UPLBSApp.h"
#include "UPLBSProfile.h"

#include "UPLBSSerial.h"
/*********************************************************************
 * MACROS
 */
#if ( defined CARDROID )
#define SYS_GPIO_P0_INT_ENABLE()	 	st( IEN1 |=	BV(5); )
#define SYS_GPIO_P0_INT_DISABLE()		st( IEN1 &=	~(BV(5)); )

#define PD_INT_ENABLE()			st( PD_INT_IEN |=	PD_INT_IE_GPIO_PINS; )
#define PD_INT_DISABLE()			st( PD_INT_IEN &= ~PD_INT_IE_GPIO_PINS; )

#define RX_WKP_INT_ENABLE()			st( RX_WKP_INT_IEN |=	RX_WKP_INT_IE_GPIO_PINS; )
#define RX_WKP_INT_DISABLE()			st( RX_WKP_INT_IEN &= ~RX_WKP_INT_IE_GPIO_PINS; )


#define SYS_GPIO_P1_INT_ENABLE()	 	st( IEN2 |=	BV(4); )
#define SYS_GPIO_P1_INT_DISABLE()		st( IEN2 &=	~(BV(4)); )

#define ACC_INT_ENABLE()			st( ACC_INT_IEN |=	ACC_INT_IE_GPIO_PINS; )
#define ACC_INT_DISABLE()			st( ACC_INT_IEN &= ~ACC_INT_IE_GPIO_PINS; )
#endif	/* CARDROID */

/*********************************************************************
 * CONSTANTS
 */
#if ( defined CARDROID ) || (defined IO_DETECT )
// Wi-Fi device power pin (Hardware designed)
#define WIFI_CTRL_GPIO_PINS			BV(0)
#define WIFI_DEV_POWER_PIN			P1_0
#endif	/* CARDROID || IO_DETECT */

#if ( defined CARDROID )
/***************CC2541 interrupt define********************/
// Port Interrupt Control
#define PORT_INT_CTRL					PICTL

// Power detect GPIO interrupt config
#define PD_INT_PINSEL 					P0SEL
#define PD_INT_PINDIR					P0DIR
#define PD_INT_IFG						P0IFG
#define PD_INT_IF 						P0IF
#define PD_INT_IEN 					P0IEN

#define PD_INT_IE_GPIO_PINS			BV(6)

// UART Rx wake up interrupt config
#define RX_WKP_INT_PINSEL 			P0SEL
#define RX_WKP_INT_PINDIR				P0DIR
#define RX_WKP_INT_IFG					P0IFG
#define RX_WKP_INT_IF 					P0IF
#define RX_WKP_INT_IEN 				P0IEN

#define RX_WKP_INT_IE_GPIO_PINS		BV(2)


// Accelerometer interrupt config
#define ACC_INT_PINSEL 				P1SEL
#define ACC_INT_PINDIR					P1DIR
#define ACC_INT_IFG					P1IFG
#define ACC_INT_IF 					P1IF
#define ACC_INT_IEN 					P1IEN
#if ( defined CARD ) || ( !defined IMAGE_VERSION_NUM ) || ( IMAGE_VERSION_NUM == 0 )
#define ACC_INT_IE_GPIO_PINS			BV(3)
#else
#define ACC_INT_IE_GPIO_PINS			BV(2)
#endif	/* CARD || !IMAGE_VERSION_NUM || IMAGE_VERSION_NUM == 0*/

// movement check period in ms <default 30000>
#define MOVING_CHECK_CYCLE			30000

// Movement interrupt counts produced by Accelerometer during movement check period <default 3>
#define MAX_MOVING_TEST_COUNTS		3

enum
{
	DEV_REST_TEST=0,
	DEV_MOVING_TEST,	
};
#endif	/* CARDROID */


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
#if ( defined CARDROID )
// I2C bus clock
static i2cClock_t i2cclk = i2cClock_533KHZ;

// If accelerometer is working
static uint8 AccState = TRUE;

// Task to process int
static uint8 Int_Process_TaskId;

// Motion/no-motion test state
static uint8 Dev_motion_state=DEV_REST_TEST;

// Moving counts in motion test
static uint8 Moving_int_counts=0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void acc_soft_reset(void);

static void acc_enter_deep_sleep_mode(void);
static void acc_enter_low_power_mode(void);
static void acc_leave_low_power_mode(void);

static bool dev_moving_test(void);

static void set_acc_int_config(void);
static void acc_nomo_int_config(void);
static void acc_motion_int_config(void);

static bool HalI2CReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes);
static bool HalI2CWriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes);

static void acc_int_process(void);
static void sys_disable_i2c_int(void);
static void sys_enable_i2c_int(void);
static void Hal_I2C_Int_Cfg(void);

#endif	/* CARDROID */


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn		system_power_saving
 *
 * @brief		Set system power mode to PM3
 *
 * @param	none
 * @return	none
 */
void system_power_saving(uint8 task_id)
{
	(void)osal_pwrmgr_task_state(task_id, PWRMGR_CONSERVE);
	osal_pwrmgr_device( PWRMGR_BATTERY );
}

/*********************************************************************
 * @fn		system_power_hold
 *
 * @brief		Hold system power
 *
 * @param	none
 * @return	none
 */
void system_power_hold(uint8 task_id)
{
	osal_pwrmgr_device( PWRMGR_ALWAYS_ON);
	(void)osal_pwrmgr_task_state(task_id, PWRMGR_HOLD);
}

#if ( defined CARDROID ) || (defined IO_DETECT )
/*********************************************************************
 * @fn		UPLBS_WiFi_Init
 *
 * @brief		Set WiFi device GPIO pin and default power it off
 *
 * @param	none
 * @return	none
 */
void UPLBS_WiFi_Init(void)
{
	P1SEL &= (uint8) ~WIFI_CTRL_GPIO_PINS;
	P1DIR |= (uint8) WIFI_CTRL_GPIO_PINS;

	wifi_dev_power_off();
}

/*********************************************************************
 * @fn		wifi_dev_power_on
 *
 * @brief		Set WiFi power on
 *
 * @param	none
 * @return	none
 */
void wifi_dev_power_on(void)
{
	WIFI_DEV_POWER_PIN = 1;
}

/*********************************************************************
 * @fn		wifi_dev_power_off
 *
 * @brief		Set WiFi power off
 *
 * @param	none
 * @return	none
 */
void wifi_dev_power_off(void)
{
	WIFI_DEV_POWER_PIN = 0;
}
#endif	/* CARDROID || IO_DETECT */

#if ( defined CARDROID )
/*********************************************************************
 * @fn		UPLBS_Acc_Init
 *
 * @brief		Init accelerometer I2C interface and perform device soft reset
 *
 * @param	none
 * @return	none
 */
void UPLBS_Acc_Init(void)
{
	HalI2CInit(ACC_I2C_ADDR, i2cclk);

	acc_soft_reset();
}

/*********************************************************************
 * @fn		acc_check
 *
 * @brief		Test if accelerometer is working
 *
 * @param	none
 * @return	TRUE working
 */
bool acc_check(void)
{
	uint8 chip_id = 0;

	HalI2CReadReg(ACC_CHIP_ID_REG, &chip_id, 1);

	if (chip_id == ACC_CHIP_ID)
		set_acc_int_config();
	else
		AccState = FALSE;
	
	return 	AccState;
}

/*********************************************************************
 * @fn		acc_soft_reset
 *
 * @brief		Perform accelerometer soft reset, clear all states and register value
 *
 * @param	none
 * @return	none
 */
static void acc_soft_reset(void)
{	
	uint8 data = ACC_EN_SOFT_RESET_VALUE;

	HalI2CWriteReg(ACC_RESET_REG, &data, 1);

	return;
}


/*********************************************************************
 * @fn		get_env_tmpr
 *
 * @brief		Get environment temperature
 *
 * @param	none
 * @return	temperature
 */
int8 get_env_tmpr(void)
{
	int8 tmprval;

	if (AccState == FALSE)
		return INVALID_TEMPR;

	HalI2CReadReg(ACC_TEMPERATURE_REG, (uint8 *)&tmprval, 1);
	
	tmprval = (int8)((float)tmprval*ACC_TEMPR_RESOL + ACC_BASE_TEMPR) ;

	return tmprval;
}

/*********************************************************************
 * @fn		acc_enter_deep_sleep_mode
 *
 * @brief		Accelerometer enter deep sleep, perform soft reset to return nomal state
 *
 * @param	none
 * @return	none
 */
static void acc_enter_deep_sleep_mode(void)
{
	uint8 pm=ACC_DEEP_SLEEP;

	HalI2CWriteReg(ACC_MODE_CTRL_REG,&pm,1);
}

/*********************************************************************
 * @fn		acc_enter_low_power_mode
 *
 * @brief		Enter low power mode 1, use event(interrupt) driven mode
 *
 * @param	none
 * @return	none
 */
static void acc_enter_low_power_mode(void)
{
	uint8 pm=ACC_LOW_POWER_MODE;

	HalI2CWriteReg(ACC_MODE_CTRL_REG,&pm,1);

	return;
}

/*********************************************************************
 * @fn		acc_leave_low_power_mode
 *
 * @brief		Leave low power mode by clear mode contrl register value
 *
 * @param	none
 * @return	none
 */
static void acc_leave_low_power_mode(void)
{
	uint8 pm=0;

	HalI2CWriteReg(ACC_MODE_CTRL_REG,&pm,1);

	return;
}

/*********************************************************************
 * @fn		detect_motion_state
 *
 * @brief		Detect motion state when get accelerometer interrupt
 *
 * @param	none
 * @return	none
 */
void detect_motion_state(void)
{
	uint8 role = get_GAP_state();
	
	if (Dev_motion_state == DEV_MOVING_TEST)
	{
		PrintString("\r\nm");
		if (dev_moving_test() == FALSE)
			return;
		// Do not change GAP role when acting moving test in peripheral mode(detecting CAR leaving)
		else if (role == SYS_CENTRAL )
			osal_start_timerEx(Int_Process_TaskId, SWITCH_GAP_ROLE_EVT,100);
	}
	else
	{
		PrintString("\r\nr");
		osal_start_timerEx(Int_Process_TaskId, SWITCH_GAP_ROLE_EVT, 100);
	}

	// Change motion test state
	Dev_motion_state = (Dev_motion_state==DEV_MOVING_TEST?DEV_REST_TEST: DEV_MOVING_TEST);
	set_acc_int_config();

	return;
}

/*********************************************************************
 * @fn		restore_moving_check
 *
 * @brief		Test if device is moving, calculate motion interrupt counts during MOVING_CHECK_CYCLE
 *
 * @param	none
 *
 * @return	none
 */
void restore_moving_check(void)
{
	Moving_int_counts = 0;
}

/*********************************************************************
 * @fn		dev_moving_test
 *
 * @brief		Test if device is moving, calculate motion interrupt counts during MOVING_CHECK_CYCLE
 *
 * @param	none
 * @return	TRUE if device is moving( int counts >=  MAX_MOVING_TEST_COUNTS)
 */
static bool dev_moving_test(void)
{
	if (Moving_int_counts == 0)
		osal_start_timerEx( Int_Process_TaskId, RESTORE_MOVING_CHECK_EVT, MOVING_CHECK_CYCLE);
	Moving_int_counts++;
	
	if (Moving_int_counts >= MAX_MOVING_TEST_COUNTS)
	{
		// Stop unfinished timer
		osal_stop_timerEx(Int_Process_TaskId, RESTORE_MOVING_CHECK_EVT);
		Moving_int_counts=0;
		
		return TRUE;
	}
	
	return FALSE;
}


/*********************************************************************
 * @fn		set_acc_int_config
 *
 * @brief		Set accelerometer interrupt map
 *
 * @param	none
 * @return	none
 */
static void set_acc_int_config(void)
{
	// Reinitialize I2C bus, in case I2C enter power saving
	HalI2CInit(ACC_I2C_ADDR, i2cclk);
	
	if (Dev_motion_state == DEV_REST_TEST)
	{
		 acc_leave_low_power_mode();
		 acc_nomo_int_config();
	}
	else
	{
		acc_motion_int_config();
		acc_enter_low_power_mode();
	}
	
	return;
}


/*********************************************************************
 * @fn		acc_nomo_int_config
 *
 * @brief		Set accelerometer interrupt map to 3-axis no-motion interrupt
 *
 * @param	none
 * @return	none
 */
static void acc_nomo_int_config(void)
{
	uint8 clr_int = 0;
	uint8 latch_time = ACC_INT_LATCH;
	uint8 slo_nomo_dur=SLOPE_NOMO_TEST_DURN<<2;
	uint8 slo_nomo_thresh=SLOPE_NOMO_TEST_THRES;
	uint8 slo_nomo_mot_int=ACC_SLO_NO_MOT_SEL|ACC_SLO_NO_MOT_X|ACC_SLO_NO_MOT_Y|ACC_SLO_NO_MOT_Z;
	uint8 slo_nomo_int_map=ACC_SLO_NO_INT_MAP;

	sys_disable_i2c_int();
	
	HalI2CWriteReg(ACC_INT1_PAD_SEL_REG,&clr_int,1);
	HalI2CWriteReg(ACC_INT_CTRL_REG,&latch_time,1);
	HalI2CWriteReg(ACC_SLOPE_DURN_REG,&slo_nomo_dur,1);
	HalI2CWriteReg(ACC_SLO_NO_MOT_THRES_REG,&slo_nomo_thresh,1);
	HalI2CWriteReg(ACC_INT_SLO_NO_MOT_REG,&slo_nomo_mot_int,1);
	HalI2CWriteReg(ACC_INT1_PAD_SEL_REG,&slo_nomo_int_map,1);

	Hal_I2C_Int_Cfg();
	sys_enable_i2c_int();
	
	return;
}

/*********************************************************************
 * @fn		acc_motion_int_config
 *
 * @brief		Set accelerometer interrupt map to 3-axis motion interrupt
 *
 * @param	none
 * @return	none
 */
static void acc_motion_int_config(void)
{
	uint8 clr_int = 0;
	uint8 latch_time = ACC_INT_LATCH;
	uint8 slo_motion_dur = SLOPE_COUNT_IN_MOTION_TEST;
	uint8 slo_motion_thresh=SLOPE_MOTION_TEST_THRES;
	uint8 slo_motion_xyz=ACC_SLO_NO_MOT_X|ACC_SLO_NO_MOT_Y|ACC_SLO_NO_MOT_Z;
	uint8 slo_motion_int_map=ACC_SLO_MOT_INT_MAP;

	sys_disable_i2c_int();

	HalI2CWriteReg(ACC_INT1_PAD_SEL_REG,&clr_int,1);
	HalI2CWriteReg(ACC_INT_CTRL_REG,&latch_time,1);
	HalI2CWriteReg(ACC_SLOPE_DURN_REG,&slo_motion_dur,1);
	HalI2CWriteReg(ACC_SLOPE_THRES_REG,&slo_motion_thresh,1);
	HalI2CWriteReg(ACC_INT_ENABLE1_REG,&slo_motion_xyz,1);
	HalI2CWriteReg(ACC_INT1_PAD_SEL_REG,&slo_motion_int_map,1);

	Hal_I2C_Int_Cfg();	
	sys_enable_i2c_int();

	return;
}

/***********************************************************************
 * @fn		HalI2CReadReg
 *
 * @brief		This function implements the I2C protocol to read from a sensor. The sensor must
 *				be selected before this routine is called.
 *
 * @param	addr - which register to read
 * @param	pBuf - pointer to buffer to place data
 * @param	nBytes - numbver of bytes to read
 *
 * @return	TRUE if the required number of bytes are reveived
 */
static bool HalI2CReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
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

/*********************************************************************
* @fn		HalI2CWriteReg
* @brief		This function implements the I2C protocol to write to a sensor. The sensor must
*				be selected before this routine is called.
*
* @param	addr - which register to write
* @param	pBuf - pointer to buffer containing data to be written
* @param	nBytes - number of bytes to write
*
* @return		TRUE if successful write
*/
static bool HalI2CWriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
	uint8 i;
	uint8 i2c_buf[24];
	uint8 *p = i2c_buf;

	/* Copy address and data to local buffer for burst write */
	*p++ = addr;
	for (i = 0; i < nBytes; i++)
	{
		*p++ = *pBuf++;
	}
	nBytes++;

	/* Send address and data */
	i = HalI2CWrite(nBytes, i2c_buf);

	return (i == nBytes);
}

/*********************************************************************
 * @fn		set_Int_Process_TaskId
 *
 * @brief		Set interrupt process task id
 *
 * @param	task_id - set task id
 * @return	none
 */
void set_Int_Process_TaskId(uint8 task_id)
{
	Int_Process_TaskId=task_id;
}

/*********************************************************************
 * @fn		acc_int_process
 *
 * @brief		Interrupt process function
 *
 * @param	none
 * @return	none
 */
static void acc_int_process(void)
{
	osal_start_timerEx( Int_Process_TaskId, INT_PROCESS_EVT,10);
}

/*********************************************************************
 * @fn		sys_disable_i2c_int
 *
 * @brief		Disable i2c interrupt
 *
 * @param	none
 * @return	none
 */
static void sys_disable_i2c_int(void)
{
	ACC_INT_DISABLE();
	SYS_GPIO_P1_INT_DISABLE();
}

/*********************************************************************
 * @fn		sys_enable_i2c_int
 *
 * @brief		Enable i2c interrupt
 *
 * @param	none
 * @return	none
 */
static void sys_enable_i2c_int(void)
{
	/* Enable P1 interrupts */
	ACC_INT_ENABLE();
	SYS_GPIO_P1_INT_ENABLE();
}

/*********************************************************************
 * @fn		Hal_I2C_Int_Cfg
 *
 * @brief		I2C interrupt config
 *
 * @param	none
 * @return	none
 */
static void Hal_I2C_Int_Cfg(void)
{
	PORT_INT_CTRL &= ~(BV(1));

	//Config CC2541 pin1
	ACC_INT_PINSEL &= ~ACC_INT_IE_GPIO_PINS;
	ACC_INT_PINDIR &= ~ACC_INT_IE_GPIO_PINS;

	// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
	ACC_INT_IFG = 0;
	ACC_INT_IF = 0;
}

/*********************************************************************
 * @fn		Hal_PD_Int_Cfg
 *
 * @brief		Power detect interrupt config
 *
 * @param	none
 *
 * @return	none
 */
void Hal_PD_Int_Cfg(uint8 pwr_on_flg)
{
	PD_INT_DISABLE();
	SYS_GPIO_P0_INT_DISABLE();

//	PrintIntValue("\r\n",pwr_on_flg,10);
	
	// Rising edge detect when no extra power
	if ( pwr_on_flg == FALSE )
		PORT_INT_CTRL &= ~(BV(0));
	else
		PORT_INT_CTRL |= (BV(0));

	//Config CC2541 pin1
	PD_INT_PINSEL &= ~PD_INT_IE_GPIO_PINS;
	PD_INT_PINDIR &= ~PD_INT_IE_GPIO_PINS;

	// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
	PD_INT_IFG = 0;
	PD_INT_IF = 0;

	PD_INT_ENABLE();
	SYS_GPIO_P0_INT_ENABLE();

	return;
}


void uart_wakeup_Int_cfg(void)
{
	RX_WKP_INT_DISABLE();
	SYS_GPIO_P0_INT_DISABLE();

//	PICTL &= ~(BV(0));

	//Config CC2541 pin0 p0.2(rx) in
	RX_WKP_INT_PINSEL &= ~RX_WKP_INT_IE_GPIO_PINS;
	RX_WKP_INT_PINDIR &= ~RX_WKP_INT_IE_GPIO_PINS;

	// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
	RX_WKP_INT_IFG = 0;
	RX_WKP_INT_IF = 0;

	/* Enable P0 interrupts */
	RX_WKP_INT_ENABLE();
	SYS_GPIO_P0_INT_ENABLE();
}

void uart_resume_cfg(void)
{
	/* disable uart rx interrupts */
	RX_WKP_INT_DISABLE();
	
	//Config CC2541 pin0 p0.2(rx) input
	RX_WKP_INT_PINSEL &= ~RX_WKP_INT_IE_GPIO_PINS;
	RX_WKP_INT_PINDIR &= ~RX_WKP_INT_IE_GPIO_PINS;
}

/*********************************************************************
 * @fn		HAL_ISR_FUNCTION
 *
 * @brief		Interrupt service routine function
 *
 * @param	unkown
 * @return	none
 */
HAL_ISR_FUNCTION(Pwr_Det_Isr, P0INT_VECTOR)
{
	HAL_ENTER_ISR();

	// Confirm accelerometer interrupt
	if ((PD_INT_IFG & PD_INT_IE_GPIO_PINS) && (PD_INT_IEN & PD_INT_IE_GPIO_PINS))
	{
		osal_set_event( Int_Process_TaskId, CHARGE_DETECT_EVT);
		// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
		PD_INT_IFG = 0;
		PD_INT_IF = 0;
	}

	// Confirm accelerometer interrupt
	if ((RX_WKP_INT_IFG & RX_WKP_INT_IE_GPIO_PINS) && (RX_WKP_INT_IEN & RX_WKP_INT_IE_GPIO_PINS))
	{
		osal_set_event( Int_Process_TaskId, UART_WAKE_UP_EVT);
		// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
		RX_WKP_INT_IFG = 0;
		RX_WKP_INT_IF = 0;
	}

	CLEAR_SLEEP_MODE();
	
	HAL_EXIT_ISR();
}

HAL_ISR_FUNCTION(DEV_I2C_Isr, P1INT_VECTOR)
{
	HAL_ENTER_ISR();

	// Confirm accelerometer interrupt
	if ((ACC_INT_IFG & ACC_INT_IE_GPIO_PINS) && (ACC_INT_IEN & ACC_INT_IE_GPIO_PINS))
		acc_int_process();

	// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
	ACC_INT_IFG = 0;
	ACC_INT_IF = 0;

	CLEAR_SLEEP_MODE();
	
	HAL_EXIT_ISR();
}
#endif	/* CARDROID */

