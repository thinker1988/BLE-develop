/**************************************************************************//**
		@file			 UPLBSAcc.h

		@brief			Header file for accelerometer BMA223. @Note This header file
								does not include all register addresses for the BMA223.

******************************************************************************/
#ifndef UPLBSACC_H
#define UPLBSACC_H


/******************************************************************************
 * INCLUDES
 */

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CONSTANTS
 */

//G-Sensor I2C address
#define ACC_I2C_ADDR							0x18

//G-Sensor register address
#define ACC_CHIP_ID_REG						0x00
#define ACC_VERSION_REG						0x01
#define ACC_X_AXIS_LSB_REG					0x02
#define ACC_X_AXIS_MSB_REG					0x03
#define ACC_Y_AXIS_LSB_REG					0x04
#define ACC_Y_AXIS_MSB_REG					0x05
#define ACC_Z_AXIS_LSB_REG					0x06
#define ACC_Z_AXIS_MSB_REG					0x07
#define ACC_TEMPERATURE_REG					0x08
#define ACC_STATUS1_REG						0x09
#define ACC_STATUS2_REG						0x0A
#define ACC_STATUS_TAP_SLOPE_REG				0x0B
#define ACC_STATUS_ORIENT_HIGH_REG			0x0C
#define ACC_STATUS_FIFO_REG					0x0E
#define ACC_RANGE_SEL_REG						0x0F
#define ACC_BW_SEL_REG							0x10
#define ACC_MODE_CTRL_REG						0x11
#define ACC_LOW_NOISE_CTRL_REG				0x12
#define ACC_DATA_CTRL_REG						0x13
#define ACC_RESET_REG							0x14
#define ACC_INT_ENABLE1_REG					0x16
#define ACC_INT_ENABLE2_REG					0x17
#define ACC_INT_SLO_NO_MOT_REG				0x18
#define ACC_INT1_PAD_SEL_REG					0x19
#define ACC_INT_DATA_SEL_REG					0x1A
#define ACC_INT2_PAD_SEL_REG					0x1B
#define ACC_INT_SRC_REG						0x1E
#define ACC_INT_SET_REG						0x20
#define ACC_INT_CTRL_REG						0x21
#define ACC_LOW_DURN_REG						0x22
#define ACC_LOW_THRES_REG						0x23
#define ACC_LOW_HIGH_HYST_REG					0x24
#define ACC_HIGH_DURN_REG						0x25
#define ACC_HIGH_THRES_REG					0x26
#define ACC_SLOPE_DURN_REG					0x27
#define ACC_SLOPE_THRES_REG					0x28
#define ACC_SLO_NO_MOT_THRES_REG				0x29
#define ACC_TAP_PARAM_REG						0x2A
#define ACC_TAP_THRES_REG						0x2B
#define ACC_ORIENT_PARAM_REG					0x2C
#define ACC_THETA_BLOCK_REG					0x2D
#define ACC_THETA_FLAT_REG					0x2E
#define ACC_FLAT_HOLD_TIME_REG				0x2F
#define ACC_FIFO_WML_TRIG						0x30
#define ACC_SELF_TEST_REG						0x32
#define ACC_EEPROM_CTRL_REG					0x33
#define ACC_SERIAL_CTRL_REG					0x34
#define ACC_EXTMODE_CTRL_REG					0x35
#define ACC_OFFSET_CTRL_REG					0x36
#define ACC_OFFSET_PARAMS_REG					0x37
#define ACC_OFFSET_X_AXIS_REG					0x38
#define ACC_OFFSET_Y_AXIS_REG					0x39
#define ACC_OFFSET_Z_AXIS_REG					0x3A
#define ACC_GP0_REG							0x3B
#define ACC_GP1_REG							0x3C
#define ACC_FIFO_MODE_REG						0x3E
#define ACC_FIFO_DATA_OUTPUT_REG				0x3F

// Acc chip id
#define ACC_CHIP_ID							0xF8

// Base temperature and resolution
#define ACC_BASE_TEMPR							23
#define ACC_TEMPR_RESOL						0.5

// Invalid termperature
#define INVALID_TEMPR							0x7F

// Soft reset wait
#define ACC_SOFT_RESET_DELAY					10

/****************Register define***************************/
#define ACC_EN_SOFT_RESET_VALUE				0xB6


/************Low Power Select*************
	bit		mode	
|	7	|    suspend	|
|	6	|   low power	|
|	5	|  deep sleep	|
|	4	| --(bit 3)--	|
|	3	|      sleep	|
|	2	|    duration	|
|	1	| --(bit 0)--	|
|	0	|    reserved	|
***************************************/
#define ACC_LOW_POWER_MODE					BV(6)
#define ACC_DEEP_SLEEP							BV(5)

// Motion /no-motion detect threshold <default 3 and 1>
#define SLOPE_NOMO_TEST_THRES 				3
#define SLOPE_MOTION_TEST_THRES				1

// No-motion detect duration	< default 10s >
/**********Duration Specification**********
	reg value	---> duration (s)	 interval(s)
	0~15		1~16		1
	16~21		40~80		8
	22~31		not specified
	32~63		88~336		8
**************************************/
#define SLOPE_NOMO_TEST_DURN					10

// Consecutive data upon threshold will generate motion interrupt
#define SLOPE_COUNT_IN_MOTION_TEST			3

// Accelerometer interrupt latch duration
/***********Latch Specification***********
   reg value	----> Latched status
	0			not-latched		
	1			250ms
	2			500ms
	3~4~5~6		1~2~4~8s
	7			latched
	8			not-latched
	9			250us
	10~11		500~1000us
	12~13~14	12.5~25~50ms
	15			latched
**************************************/
#define ACC_INT_LATCH							3

//  Accelerometer interrupt map bits
#define ACC_SLO_NO_INT_MAP 					BV(3)
#define ACC_SLO_MOT_INT_MAP 					BV(2)

// Accelerometer interrupt map axis bits
#define ACC_SLO_NO_MOT_SEL 					BV(3)
#define ACC_SLO_NO_MOT_X 						BV(0)
#define ACC_SLO_NO_MOT_Y 						BV(1)
#define ACC_SLO_NO_MOT_Z 						BV(2)

// Clear interrupt latch time
#define CLEAR_LATCH_INT						BV(7)


/******************************************************************************
 * FUNCTION PROTOTYPES
 */
void system_power_saving(uint8 task_id);
void system_power_hold(uint8 task_id);

#if ( defined CARDROID ) || (defined IO_DETECT )
void UPLBS_WiFi_Init(void);
void wifi_dev_power_on(void);
void wifi_dev_power_off(void);
#endif	/* CARDROID || IO_DETECT */

#if ( defined CARDROID )
void UPLBS_Acc_Init(void);

bool acc_check(void);

int8 get_env_tmpr(void);

void detect_motion_state(void);

void restore_moving_check(void);

void set_Int_Process_TaskId(uint8 task_id);

void Hal_PD_Int_Cfg(uint8 pwr_on_flg);

void uart_wakeup_Int_cfg(void);

void uart_resume_cfg(void);

#endif	/* CARDROID */

#endif

