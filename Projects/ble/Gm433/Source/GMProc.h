#ifndef GM_PROCESS_H
#define GM_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bcomdef.h"


#define CALC_ABS(x)	((x)>0?(x):-(x))

typedef enum gmstatus
{
	GMNoCar=0,
	GMGetCar,
	GMUnknow,
	GMFirstBoot
}gmstatus_t;

typedef enum sendtype
{
	SEND_NOTHG,
	SEND_HRTBY,
	SEND_CHNG,
	SEND_SYNC
}sendtype_t;

void setGMstate(gmstatus_t nwst);
gmstatus_t getGMstate(void);

void initGMstate(void);

void set_heart_beat(void);
void set_data_change(void);
void clear_send(void);

void gm_data_proc(int16 tmpX,int16 tmpY,int16 tmpZ);
void send_gde_data(uint8 prcnt, uint8 tmpr,int16 tmpX, int16 tmpY, int16 tmpZ);
void stopresend(uint8 *data, uint8 len);


#ifdef  __cplusplus
}
#endif

#endif
