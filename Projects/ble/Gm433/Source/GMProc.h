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

//GM base temperature and invalid temperature
#define GM_BASE_TEMPR 				25
#define GM_INVALID_TEMPR			127

// GM read once wait data ready period
#define GM_READ_ONCE_PERIOD			150

/**********Hardware version define**********/
#if ( !defined HW_VERN ) || ( HW_VERN == 0 )

// HW_VERN 0: use Dupont line connect GM detector
#define GM_DRDY_INT_PINSEL 			P1SEL
#define GM_DRDY_INT_PINDIR			P1DIR
#define GM_DRDY_INT_PXIFG			P1IFG
#define GM_DRDY_INT_PXIF 			P1IF
#define GM_DRDY_INT_PXIEN 			P1IEN

#define GM_DRDY_INT_IE				BV(6)

#elif ( HW_VERN >= 1 )

// HW_VERN 1: use TEN308 as RF
// HW_VERN 2: use TI CC1121 as RF
#define GM_DRDY_INT_PINSEL 			P0SEL
#define GM_DRDY_INT_PINDIR			P0DIR
#define GM_DRDY_INT_PXIFG			P0IFG
#define GM_DRDY_INT_PXIF 			P0IF
#define GM_DRDY_INT_PXIEN 			P0IEN
#define GM_DRDY_INT_IE				BV(1)

// Hgswitch high:deep sleep; low:working
#define DEV_EN_INT_PINSEL 			P0SEL
#define DEV_EN_INT_PINDIR			P0DIR
#define DEV_EN_INT_PININP			P0INP

#define DEV_EN_INT_PXIEN 			P0IEN
#define DEV_EN_INT_IE				BV(4)
#define DEV_EN_INT_PXIFG			P0IFG

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
#define CLAC_GM_TEMPR(HiBt,LoBt)			(((HiBt)<<8+(LoBt))>>7+GM_BASE_TEMPR)

// Claculate absolute value
#define CALC_ABS(x)	((x)>0?(x):-(x))


/*********************************************************************
 * TYPEDEFS
 */

typedef enum gmsensor
{
	GMSnTest = 0,
	GMSnSet,
	GMSnReq,
	GMSnRead,
	GMSnSleep,
	GMSnErr,
}gmsensor_t;


typedef enum gmstatus
{
	GMNoCar=0,
	GMGetCar,
	GMUnknow,
	GMFirstBoot,
	GMError
}gmstatus_t;

typedef enum sendtype
{
	SEND_NOTHG,
	SEND_HRTBY,
	SEND_CHNG,
	SEND_SYNC,
}sendtype_t;
/******************************************************************************
 * PROTPTYPES
 */
extern void SetGMSnState(gmsensor_t ngmsnst);
extern gmsensor_t GetGMSnState(void);

extern void GM_working(uint8 task_id, gmsensor_t ngmsnst);

extern void SetGMState(gmstatus_t nwst);
extern gmstatus_t GetGMState(void);

extern void SendSyncTMReq(void);
extern void SendGDEData(int16 tmpX, int16 tmpY, int16 tmpZ);

extern void ClearSyncTMReq(void);
extern void ClearDataResend(void);

extern void ReadGMParam(uint8 * rdbuf);
extern bool SetGMParam(uint8 hrtbtmin, uint8 dtval, uint8 alg, uint8 status);

#ifdef  __cplusplus
}
#endif

#endif
