#include "osal_snv.h"

#include "BLECore.h"
#include "Com433.h"
#include "GMProc.h"
#include "Pktfmt.h"


#define IN_ARRAY_BOUND(a,bnd)	((bnd)>(a)?(a):((a)-(bnd)))

// NV id (>0x80 available)
#define GM_STATE_NV_ID		0xA0

#define GM_PAST_XVAL_ID		0xA1
#define GM_PAST_YVAL_ID		0xA2
#define GM_PAST_ZVAL_ID		0xA3


// Threshold
#define GM_GET_CAR_THRHD	50

#define GM_NOISE_THRHD		10

// Length to count average benchmark
#define BENCH_AVG_LEN		10

// Length to count weight benchmark
#define BENCH_WEIGHT_LEN	5

// Resend times
#define MAX_RESEND_TIMES	3

// GM data send type
sendtype_t sndtyp = SEND_NOTHG;

// Resend times (max:3)
static uint8 rsndcnt;

// Current and previous GM state
static gmstatus_t gmst;
static gmstatus_t prevgmst;

// GM car detect threshold
uint8 carthrhld = GM_GET_CAR_THRHD;
uint8 noisethrhld = GM_GET_CAR_THRHD;

// Read counts when no car detect, save data in temporary benchmark array every 120s
static uint8 readcnt;

// Basic benchmark
int16 Xbenchmk,Ybenchmk,Zbenchmk;

// Temporary benchmark set and counts
int16 tmpXbench[BENCH_AVG_LEN],tmpYbench[BENCH_AVG_LEN],tmpZbench[BENCH_AVG_LEN];
uint8 tmpbenchcnt;

// Car detect counts, 
static uint8 dtctcnt;

// Car detect Benchmark
int16 Xdtctval,Ydtctval,Zdtctval;

// Temporary detect benchmark
int16 tmpXdtctval[BENCH_WEIGHT_LEN],tmpYdtctval[BENCH_WEIGHT_LEN],tmpZdtctval[BENCH_WEIGHT_LEN];
uint8 tmpdtctcnt;


static bool initbnchmk(gmstatus_t gmstts,int16 xVal,int16 yVal, int16 zVal);

static void storeGMstate(void);
static gmstatus_t ChangeGMstate(gmstatus_t curstate);

static void normrgltGMbenchmk(int16 xVal,int16 yVal, int16 zVal);
static void wghtrgltGMCbenchmk(int16 xVal, int16 yVal, int16 zVal);

static int16 calc_weight(int16 * buf, uint8 len, uint8 bound);
static int16 calc_avrg(int16 *buf, uint8 n);



void setGMstate(gmstatus_t nwst)
{
	gmst=nwst;
}

gmstatus_t getGMstate(void)
{
	return gmst;
}


void initGMstate(void)
{
	int16 tmpx,tmpy,tmpz;
	bool initflag = FALSE;
	
	if(osal_snv_read(GM_STATE_NV_ID, sizeof(prevgmst),&prevgmst)!=NV_OPER_FAILED)
	{
		if (prevgmst!=GMNoCar && prevgmst!=GMGetCar && prevgmst!=GMUnknow)
			prevgmst = GMFirstBoot;
		else
		{
			if (osal_snv_read(GM_PAST_XVAL_ID, sizeof(tmpx),&tmpx)!=NV_OPER_FAILED)
				if (osal_snv_read(GM_PAST_YVAL_ID, sizeof(tmpy),&tmpy)!=NV_OPER_FAILED)
					if (osal_snv_read(GM_PAST_ZVAL_ID, sizeof(tmpz),&tmpz)!=NV_OPER_FAILED)
						initflag = initbnchmk(prevgmst,tmpx,tmpy,tmpz);
			if (initflag == FALSE)
				prevgmst = GMFirstBoot;
		}
	}
	else
	{
		// First start up
		//Com433WriteStr(COM433_DEBUG_PORT,"\r\n!!!NV read failed!!!");
		prevgmst = GMFirstBoot;
	}
	gmst = GMFirstBoot;
	readcnt = 0;
}


void set_heart_beat(void)
{
	// send heart beat when nothing wait sent
	if (sndtyp == SEND_NOTHG)
		sndtyp = SEND_HRTBY;
}


void set_data_change(void)
{
	sndtyp = SEND_CHNG;
}


void clear_send(void)
{
	rsndcnt = 0;
	sndtyp = SEND_NOTHG;
}


void gm_data_proc(int16 tmpX, int16 tmpY, int16 tmpZ)
{	
	switch(gmst)
	{
		case GMNoCar:
			if (CALC_ABS(tmpZ-Zbenchmk)>carthrhld)
			{
				gmst = ChangeGMstate(gmst);
				// Reinit detect benchmark every time
				initbnchmk(gmst, tmpX, tmpY, tmpZ);
			}
			/*else if (CALC_ABS(tmpZ-Zbenchmk)>noisethrhld)
			{
				prevgmst = GMNoCar;
				gmst = GMUnknow;
			}*/
			else
				normrgltGMbenchmk(tmpX,tmpY,tmpZ);
			
			break;
		case GMGetCar:
			if (CALC_ABS(tmpZ-Zdtctval)>carthrhld && CALC_ABS(tmpZ-Zbenchmk)<carthrhld)
			{
				gmst = ChangeGMstate(gmst);
				normrgltGMbenchmk(tmpX,tmpY,tmpZ);
			}
			/*else if (CALC_ABS(tmpZ-Zbenchmk)>carthrhld && CALC_ABS(tmpZ-Zbenchmk)>noisethrhld)
			{
				prevgmst = GMGetCar;
				gmst = GMUnknow;
			}*/
			break;
		case GMUnknow:
			/*if (prevgmst == GMNoCar)
			{
				if (CALC_ABS(tmpZ-Zbenchmk)>carthrhld)
					gmst = ChangeGMstate(prevgmst);
			}
			else if (prevgmst == GMGetCar )
			{
				if (CALC_ABS(tmpZ-Zdtctval)>carthrhld && CALC_ABS(tmpZ-Zbenchmk)<=noisethrhld )
					gmst = ChangeGMstate(prevgmst);
				
			}*/
			break;
		case GMFirstBoot:
			// Discard first data and set second data to no car benchmark
			if ( ++readcnt == 2 )
			{
				initbnchmk(gmst,tmpX,tmpY,tmpZ);
				set_heart_beat();
			}
			break;
	}
//	storeGMstate();
	
	if (sndtyp != SEND_NOTHG)
	{
		uint8 prcnt;
		int8 gdetmpr;

		RFwakeup();
		
		prcnt = CalcBatteryPercent();
		gdetmpr = GetGDETmpr();
		send_gde_data(prcnt,gdetmpr,tmpX,tmpY,tmpZ);
	}
	else
	{
		if (gettmsync() == TRUE)
		{
			syncUTCtimereq();
		}
	}
}


void send_gde_data(uint8 prcnt, uint8 tmpr,int16 tmpX, int16 tmpY, int16 tmpZ)
{
	uint8 hrtbtdata[GMS_PKT_PLD_MAX_LEN];
	
	hrtbtdata[HRT_BT_BATT_POS]=prcnt;
	hrtbtdata[HRT_BT_TMPR_POS]=tmpr;
	
	hrtbtdata[HRT_BT_XVAL_POS]=HI_UINT16(tmpX);
	hrtbtdata[HRT_BT_XVAL_POS+1]=LO_UINT16(tmpX);

	hrtbtdata[HRT_BT_YVAL_POS]=HI_UINT16(tmpY);
	hrtbtdata[HRT_BT_YVAL_POS+1]=LO_UINT16(tmpY);

	hrtbtdata[HRT_BT_ZVAL_POS]=HI_UINT16(tmpZ);
	hrtbtdata[HRT_BT_ZVAL_POS+1]=LO_UINT16(tmpZ);
	
	hrtbtdata[HRT_BT_STAT_POS]=gmst;

	if (sndtyp == SEND_HRTBY)
	{
		rfdataform(GDE_ST_HRTBEAT_REQ,hrtbtdata,GDE_HRTBT_LEN);
		clear_send();
	}
	else if (sndtyp == SEND_CHNG)
	{
		rfdataform(GDE_ST_CARINFO_REQ,hrtbtdata,GDE_HRTBT_LEN);
		if (rsndcnt++ > MAX_RESEND_TIMES)
			clear_send();
	}
}


void stopresend(uint8 *data, uint8 len)
{
	VOID data;
	VOID len;

	clear_send();
}

static bool initbnchmk(gmstatus_t gmstts,int16 xVal,int16 yVal, int16 zVal)
{
	switch (gmstts)
	{
		case GMFirstBoot:	// Assume no car
		case GMNoCar:
			Xbenchmk = tmpXbench[0] = xVal;
			Ybenchmk = tmpYbench[0] = yVal;
			Zbenchmk = tmpZbench[0] = zVal;

			tmpbenchcnt = 1;
			readcnt = 0;
			PrintGMvalue(COM433_DEBUG_PORT, "\r\nNCInit", Xbenchmk, Ybenchmk, Zbenchmk);

			gmst = GMNoCar;
			break;
		case GMGetCar:
			Xdtctval = tmpXdtctval[0] = xVal;
			Ydtctval = tmpYdtctval[0] = yVal;
			Zdtctval = tmpZdtctval[0] = zVal;

			tmpdtctcnt = 1;
			dtctcnt = 0;
			PrintGMvalue(COM433_DEBUG_PORT, "\r\nDTInit", Xdtctval, Ydtctval, Zdtctval);
			
			gmst = GMGetCar;
			break;
		case GMUnknow:
			break;
		default:
			return FALSE;
	}

	return TRUE;
}

static void storeGMstate(void)
{
	 if(osal_snv_write(GM_STATE_NV_ID, sizeof(gmst),&gmst)==NV_OPER_FAILED)
	 	Com433WriteStr(COM433_DEBUG_PORT,"\r\n!!!NV Failed!!!");
}


static gmstatus_t ChangeGMstate(gmstatus_t curstate)
{
	set_data_change();
	Com433WriteStr(COM433_DEBUG_PORT,"\r\nCCC");
	return (curstate==GMNoCar? GMGetCar:GMNoCar);
}


static void normrgltGMbenchmk(int16 xVal,int16 yVal, int16 zVal)
{	
	// adjust benchmark ervery 5*120=10min
	if ( ++readcnt < 120 )
	{
		PrintGMvalue(COM433_DEBUG_PORT, "\r\nR:", xVal, yVal, zVal);
		return;
	}
	
	readcnt = 0;
	tmpbenchcnt++;

	if ( tmpbenchcnt > BENCH_AVG_LEN )
	{
		if (tmpbenchcnt > 2*BENCH_AVG_LEN)
			tmpbenchcnt = BENCH_AVG_LEN+1;
		
		tmpXbench[tmpbenchcnt-BENCH_AVG_LEN-1] = xVal;
		tmpYbench[tmpbenchcnt-BENCH_AVG_LEN-1] = yVal;
		tmpZbench[tmpbenchcnt-BENCH_AVG_LEN-1] = zVal;

		Xbenchmk = calc_avrg(tmpXbench,BENCH_AVG_LEN);
		Ybenchmk = calc_avrg(tmpYbench,BENCH_AVG_LEN);
		Zbenchmk = calc_avrg(tmpZbench,BENCH_AVG_LEN);
	}
	else
	{
		tmpXbench[tmpbenchcnt-1]=xVal;
		tmpYbench[tmpbenchcnt-1]=yVal;
		tmpZbench[tmpbenchcnt-1]=zVal;

		Xbenchmk = calc_avrg(tmpXbench,tmpbenchcnt);
		Ybenchmk = calc_avrg(tmpYbench,tmpbenchcnt);
		Zbenchmk = calc_avrg(tmpZbench,tmpbenchcnt);	
	}

	PrintGMvalue(COM433_DEBUG_PORT, "\r\nNC bench:", Xbenchmk, Ybenchmk, Zbenchmk);
}


static void wghtrgltGMCbenchmk(int16 xVal,int16 yVal, int16 zVal)
{
	// adjust benchmark ervery 10s
	if ( ++dtctcnt < 2 )
		return;

	dtctcnt = 0;
	tmpdtctcnt++;

	if ( tmpdtctcnt > BENCH_WEIGHT_LEN )
	{
		if (tmpdtctcnt > 2*BENCH_WEIGHT_LEN)
			tmpdtctcnt = BENCH_WEIGHT_LEN+1;
		
		tmpXdtctval[tmpdtctcnt-BENCH_WEIGHT_LEN-1] = xVal;
		tmpYdtctval[tmpdtctcnt-BENCH_WEIGHT_LEN-1] = yVal;
		tmpZdtctval[tmpdtctcnt-BENCH_WEIGHT_LEN-1] = zVal;

		Xdtctval = calc_weight(tmpXdtctval,tmpdtctcnt,BENCH_WEIGHT_LEN);
		Ydtctval = calc_weight(tmpXdtctval,tmpdtctcnt,BENCH_WEIGHT_LEN);
		Zdtctval = calc_weight(tmpXdtctval,tmpdtctcnt,BENCH_WEIGHT_LEN);
	}
	else
	{
		tmpXdtctval[tmpdtctcnt-1]=xVal;
		tmpYdtctval[tmpdtctcnt-1]=yVal;
		tmpZdtctval[tmpdtctcnt-1]=zVal;

		Xdtctval = calc_weight(tmpXdtctval,tmpdtctcnt,tmpdtctcnt);
		Ydtctval = calc_weight(tmpXdtctval,tmpdtctcnt,tmpdtctcnt);
		Zdtctval = calc_weight(tmpXdtctval,tmpdtctcnt,tmpdtctcnt);	
	}

	PrintGMvalue(COM433_DEBUG_PORT, "\r\nDT bench:", Xdtctval, Ydtctval, Zdtctval);
}


static int16 calc_avrg(int16 *buf, uint8 n)
{
	uint8 i;
	int32 sum=0;
	
	for (i=0;i<n;i++)
		sum+=buf[i];
	
	return sum/n;	
}

static int16 calc_weight(int16 *buf, uint8 len, uint8 bound)
{
	uint8 i;
	int32 sum=0,nsum=0;
	
	for (i=0;i<bound;i++)
	{
		if (len > bound)
			sum += (i+1)*buf[IN_ARRAY_BOUND(len%bound+i,bound)];
		else
			sum += (i+1)*buf[i];
		nsum += i+1;
	}

	return sum/nsum;	
}

