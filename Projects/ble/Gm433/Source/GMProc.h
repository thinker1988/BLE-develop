#ifndef GM_PROCESS_H
#define GM_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif
/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"

/*********************************************************************
 * CONSTANTS
 */

// GM device parameters
//  Geomagnetic sensor ID
#define GM_ID_STR					"H43"

//G-Sensor I2C address
#define GM_I2C_ADDR					0x1E

//G-Sensor register address
#define GM_CONFIG_A_REG				0x00
#define GM_CONFIG_B_REG				0x01
#define GM_MODE_REG					0x02
#define GM_X_AXIS_MSB_REG			0x03
#define GM_X_AXIS_LSB_REG			0x04
#define GM_Z_AXIS_MSB_REG			0x05
#define GM_Z_AXIS_LSB_REG			0x06
#define GM_Y_AXIS_MSB_REG			0x07
#define GM_Y_AXIS_LSB_REG			0x08
#define GM_STATUS_REG				0x09
#define GM_ID_A_REG					0x0A
#define GM_ID_B_REG					0x0B
#define GM_ID_C_REG					0x0C
#define GM_TEMPR_MSB_REG			0x31
#define GM_TEMPR_LSB_REG			0x32

// enable temperature sensor, 8 average 15Hzoutput rate
#define SET_GM_NORMAL				0xF0

// Gaim =5
#define SET_GM_GAIN					0xA0

// Single measurement
#define SET_GM_READ_ONCE			0x21

// Sleep mode ( idle mode)
#define SET_GM_SLEEP_MODE			0x03

// GM base temperature and invalid temperature
#define GM_BASE_TEMPR 				25
#define GM_INVALID_TEMPR			127

// GM read once wait data ready period in ms, see HMC
#define GM_SNSR_MEASURE_PERIOD		6

// Wait GM sensor data strady
#define GM_SNSR_WAIT_STEADY_CNT		3

/**********Hardware version define**********/
#if ( !defined HW_VERN ) || ( HW_VERN == 0 )

// HW_VERN 0: use Dupont line connect GM detector
#define GM_DRDY_INT_PINSEL 			P1SEL
#define GM_DRDY_INT_PINDIR			P1DIR

#define GM_DRDY_INT_PXIFG			P1IFG
#define GM_DRDY_INT_PXIF 			P1IF
#define GM_DRDY_INT_PXIEN 			P1IEN

#define GM_DRDY_INT_IE				BV(6)
#define GM_DRDY_INT_PIN				P1_6


#elif ( HW_VERN >= 1 )

// HW_VERN 1: use TEN308 as RF
// HW_VERN 2: use TI CC1121 as RF
#define GM_DRDY_INT_PINSEL 			P0SEL
#define GM_DRDY_INT_PINDIR			P0DIR

#define GM_DRDY_INT_PXIFG			P0IFG
#define GM_DRDY_INT_PXIF 			P0IF
#define GM_DRDY_INT_PXIEN 			P0IEN

#define GM_DRDY_INT_IE				BV(1)
#define GM_DRDY_INT_PIN				P0_1


// Hgswitch high:deep sleep; low:working
#define DEV_EN_INT_PINSEL 			P0SEL
#define DEV_EN_INT_PINDIR			P0DIR
#define DEV_EN_INT_PININP			P0INP

#define DEV_EN_INT_PXIFG			P0IFG
#define DEV_EN_INT_PXIF				P0IF
#define DEV_EN_INT_PXIEN 			P0IEN

#define DEV_EN_INT_IE				BV(4)
#define DEV_EN_INT_PIN				P0_4

#endif	// HW_VERN

// GM data ready interupt define
#define GM_DRDY_INT_ENABLE()		st( GM_DRDY_INT_PXIEN |= GM_DRDY_INT_IE; )
#define GM_DRDY_INT_DISABLE()		st( GM_DRDY_INT_PXIEN &= ~GM_DRDY_INT_IE; )

#define DEV_EN_INT_ENABLE()			st( DEV_EN_INT_PXIEN |= DEV_EN_INT_IE; )
#define DEV_EN_INT_DISABLE()		st( DEV_EN_INT_PXIEN &= ~DEV_EN_INT_IE; )


/*********************************************************************
 * MACROS
 */
// Calculate GM temperature base on HMC
#define CLAC_GM_TEMPR(HiBt,LoBt)		(((HiBt)<<8+(LoBt))>>7+GM_BASE_TEMPR)

// Claculate absolute value
#define CALC_ABS(x)						((x)>0?(x):-(x))


#define NO_MSG_SEND(sndtyp)			(sndtyp == 0)

#define SET_SEND_BIT(sndtyp, sndbit)	(sndtyp |= BV(sndbit))
#define CLR_SEND_BIT(sndtyp, sndbit)	(sndtyp &= ~ BV(sndbit))

/*********************************************************************
 * TYPEDEFS
 */

typedef enum gmsensor
{
	GMSnTest = 0,
	GMSnReq,
	GMSnRead,
	GMSnSleep,
	GMSnErr,
}gmsensor_t;


typedef enum detectstatus
{
	NO_CAR_DETECTED=0,
	CAR_DETECTED_OK,
	BENCH_CALIBRATING,
	ABNORMAL_DETECTION,
	ERROR_DETECTION
}detectstatus_t;


// Bit define of send type
// Highest priority: data change; middle : heart beat; lowest: time sync
typedef enum sndtypbit
{
	SND_BIT_DAT_CHNG=0,
	SND_BIT_HRT_BT,
	SND_BIT_TM_SYNC
}sndtypbit_t;

/******************************************************************************
 * PROTPTYPES
 */
extern void SetGMSnState(gmsensor_t ngmsnst);
extern gmsensor_t GetGMSnState(void);

extern void GM_working(uint8 task_id, gmsensor_t ngmsnst);

extern void SetGMState(detectstatus_t nwst);
extern detectstatus_t GetGMState(void);

extern void ResetBenchmark(uint8 *bnchmk, uint8 len);

extern void SendSyncTMReq(void);
extern void FormHrtbtData(uint8 *hrtbtdata,int16 tmpX, int16 tmpY, int16 tmpZ);

extern void set_time_sync(void);
extern void set_heart_beat(void);
extern void set_data_change(void);

extern void ClearSyncTMReq(void);
extern void ClearHeartBeat(void);
extern void ClearDataResend(void);

extern void ReadGMParam(uint8 * rdbuf);
extern bool SetGMParam(uint8 hrtbtmin, uint8 dtval, uint8 alg, uint8 status);
extern void ReadGDEBench(uint8* pVal);

extern bool InitBenchmk(detectstatus_t cardetectts,int16 xVal,int16 yVal, int16 zVal);
extern uint8 GetEmpBenchCnt(void);
extern void GetDevPowerPrcnt(void);


#ifdef  __cplusplus
}
#endif

#endif
