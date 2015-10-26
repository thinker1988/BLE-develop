/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "osal_snv.h"
#include "OSAL_Timers.h"

#include "hal_i2c.h"
#include "hal_adc.h"

#include "BLECore.h"
#include "Com433.h"
#include "GMProc.h"
#include "Pktfmt.h"
#include "Cc112x.h"

/*********************************************************************
 * MACROS
 */
// define weight value in this program
#define GET_WEIGHT(a)			(a)

/*********************************************************************
 * CONSTANTS
 */
// NV id (>0x80 available)
#define GM_STATE_NV_ID			0xA0

#define GM_PAST_XVAL_ID			0xA1
#define GM_PAST_YVAL_ID			0xA2
#define GM_PAST_ZVAL_ID			0xA3

// Length to count average benchmark
#define BENCH_AVG_LEN			10

// Length to count weight benchmark
#define BENCH_WEIGHT_LEN		5

// Data change packet resend times
#define MAX_RESEND_TIMES		3

// Time synchronization packet resend times
#define MAX_TM_SYNC_TIMES		5

// Benchmark adjust times, *READ_PEROID, default every 30min
#define ADJ_BENCHMK_TIMES		360

// Max times to change abnormal status, default 5min
#define MAX_ABNORMAL_TIMES		60

// Impossible value, should be caused by some iron materials, not car
#define GM_ERROR_VALUE			640000

// GM almost no change
#define NO_CHANGE_THRSHLD		10

/**********Battery power adc detect define**********/
// ADC expected value : (V*100/370)/1.25*2047
		
#define BATT_ADC_VAL_MAX		1682	// MAX : 3.8~1682
#define BATT_ADC_VAL_MIN		1106	// MIN : 2.5~1106

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Heart beat in minutes, default 10 min
uint8 hrtbt_inmin = 10;

// GM car detect threshold
uint32 sqrthrhldarr[] = {800,1000,1200};

// default use level 2, i.e. 1000
uint8 detectlevel = 2;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
#if ( !defined USE_CC112X_RF )
extern uint8 rfsndbuf[];
extern uint8 rfsndlen;
#endif	// ! USE_CC112X_RF

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// I2C clock
static i2cClock_t gm_i2cclk = i2cClock_123KHZ;

// GM sensor status
static gmsensor_t gmsnst = GMSnTest;

// GM data send type
static sendtype_t sndtyp = SEND_NOTHG;

// Current GM state
static gmstatus_t gmst = GMFirstBoot;
// Pevious GM state, only used in first boot
//static gmstatus_t prevgmst;

// Synchronize time flag (<=MAX_TM_SYNC_TIMES)
static bool tmsyncflag = FALSE;

// Resend times (<=MAX_RESEND_TIMES)
static uint8 rsndautostop;

// TM sync resend times
static uint8 syncautostop;

// Read GM data counts, increase every GM data read.
static uint16 readcnt = 0;

// Time synchronization counts, increase every heart beat times
static uint16 tmsynccnt = 0;

// Empty detect counts, adjust benchmark
static uint16 empcnt = 0;

// Unknow status detect counts
static uint16 unkwncnt = 0;


// Basic benchmark
static int16 Xbenchmk,Ybenchmk,Zbenchmk;
// Temporary benchmark set and counts
static int16 tmpXbench[BENCH_AVG_LEN],tmpYbench[BENCH_AVG_LEN],tmpZbench[BENCH_AVG_LEN];
static uint8 tmpbenchcnt;


// Car detect Benchmark
static int16 Xdtctval,Ydtctval,Zdtctval;
// Temporary detect benchmark
static int16 tmpXdtctval[BENCH_WEIGHT_LEN],tmpYdtctval[BENCH_WEIGHT_LEN],tmpZdtctval[BENCH_WEIGHT_LEN];
static uint8 tmpdtctcnt;

// GDE temperature
static int8 gdetmpr;

// GDE battery power remain percentage
static uint8 btprcnt = 100;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void InitGMState(void);

static bool GM_dev_init(void);
static void GM_dev_stop(void);

static void GM_dev_preread(void);
static void GM_dev_read(uint8 task_id);
static void GM_dev_proc(int16 tmpX,int16 tmpY,int16 tmpZ);
static void GM_send_data(uint8 task_id, int16 tmpX, int16 tmpY, int16 tmpZ);

static void SendXYZVal(uint8 task_id, int16 tmpX, int16 tmpY, int16 tmpZ);

static void set_time_sync(void);
static void set_heart_beat(void);
static void set_data_change(void);

static void GetDevPowerPrcnt(void);
static int8 GM_dev_get_tmpr(void);

static bool GM_read_reg(uint8 addr,uint8 * pBuf,uint8 nBytes);
static bool GM_write_reg(uint8 addr, uint8 *pBuf, uint8 nBytes);


static gmstatus_t CheckCarIn(int16 tmpX, int16 tmpY, int16 tmpZ);
static gmstatus_t CheckCarOut(int16 tmpX, int16 tmpY, int16 tmpZ);


static void ModifyBenchmk(gmstatus_t newstts);

static bool InitBenchmk(gmstatus_t gmstts,int16 xVal,int16 yVal, int16 zVal);

static void NormRgltEmpBenchmk(int16 xVal,int16 yVal, int16 zVal);

static void WghtRgltOcpBenchmk(int16 xVal, int16 yVal, int16 zVal);


static int16 CalcWeight(int16 * buf, uint8 len, uint8 bound);
static int16 CalcAvrg(int16 *buf, uint8 n);

#if 0
static void GM_DRDY_INT_Cfg(void);
static void storeGMstate(void);
#endif


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void SetGMSnState(gmsensor_t ngmsnst)
{
	gmsnst = ngmsnst;
}

gmsensor_t GetGMSnState(void)
{
	return gmsnst;
}

void GM_working(uint8 task_id, gmsensor_t ngmsnst)
{
	gmsnst = ngmsnst;
	switch (gmsnst)
	{
		case GMSnTest:
		{
			if (GM_dev_init() == TRUE)
			{
				InitGMState();
				InitDevID();
				// Prepare read once
				gmsnst = GMSnReq;
			}
			else
			{
				gmsnst = GMSnErr;
			}
			osal_start_timerEx(task_id,GM_DATA_PROC_EVT,WAIT_RF_WORK_PERIOD);
			break;
		}
		case GMSnReq:
			PowerHold(task_id);
			GM_dev_preread();
			gmsnst = GMSnRead;
			osal_start_timerEx(task_id, GM_DATA_PROC_EVT, GM_READ_ONCE_PERIOD);
			break;
		case GMSnRead:
			gmsnst = GMSnReq;
			GM_dev_read(task_id);
			break;
		case GMSnSleep:
			GM_dev_stop();
			break;
		case GMSnErr:
			Com433WriteStr(COM433_DEBUG_PORT,"\r\nGM init failed...");
			SetRFstate(RF_SLEEP);
			SetSysState(SYS_DORMANT);
			break;
		default:
			break;
	}
}

void SetGMState(gmstatus_t nwst)
{
	gmst=nwst;
}

gmstatus_t GetGMState(void)
{
	return gmst;
}

void SendSyncTMReq(void)
{
	uint8 tmsync=TRUE;

	RFDataForm(GDE_ST_TMSYN_REQ,&tmsync,sizeof(tmsync));

#if ( !defined GM_TEST_COMM )
	if (++syncautostop > MAX_TM_SYNC_TIMES)
		ClearSyncTMReq();
#else	// GM_TEST_COMM
	Com433WriteStr(COM433_DEBUG_PORT,"\r\nSend sync...\t");
#endif	// !GM_TEST_COMM
}


/*********************************************************************
 * @fn		SendGDEData
 *
 * @brief	Send GM read data, includes battery percentage,temperature(real time),
 *			X/Y/Z axis value and status.
 *
 * @param	tmpX - X axis read value.
 * @param	tmpY - Y axis read value.
 * @param	tmpZ - Z axis read value.
 *
 * @return	none
 */
void SendGDEData(int16 tmpX, int16 tmpY, int16 tmpZ)
{
	uint8 hrtbtdata[GMS_PKT_PLD_MAX_LEN];
	
	hrtbtdata[HRT_BT_BATT_POS]=btprcnt;
	hrtbtdata[HRT_BT_TMPR_POS]=gdetmpr;
	
	hrtbtdata[HRT_BT_XVAL_POS]=HI_UINT16(tmpX);
	hrtbtdata[HRT_BT_XVAL_POS+1]=LO_UINT16(tmpX);

	hrtbtdata[HRT_BT_YVAL_POS]=HI_UINT16(tmpY);
	hrtbtdata[HRT_BT_YVAL_POS+1]=LO_UINT16(tmpY);

	hrtbtdata[HRT_BT_ZVAL_POS]=HI_UINT16(tmpZ);
	hrtbtdata[HRT_BT_ZVAL_POS+1]=LO_UINT16(tmpZ);
	
	hrtbtdata[HRT_BT_STAT_POS]=gmst;

	//if ( gmst!=GMNoCar && gmst!=GMGetCar)
	//	return;

	if (sndtyp == SEND_HRTBY)
	{
		RFDataForm(GDE_ST_HRTBEAT_REQ,hrtbtdata,GDE_HRTBT_LEN);
		ClearDataResend();
	}
	else if (sndtyp == SEND_CHNG)
	{
		RFDataForm(GDE_ST_CARINFO_REQ,hrtbtdata,GDE_HRTBT_LEN);
		if (rsndautostop++ > MAX_RESEND_TIMES)
			ClearDataResend();
	}
}

void ClearSyncTMReq(void)
{
	tmsynccnt = 0;
	syncautostop = 0;
	tmsyncflag = FALSE;
	sndtyp =SEND_NOTHG;
}

/*********************************************************************
 * @fn		ClearDataResend
 *
 * @brief	Clear all send message.
 *
 * @param	none
 *
 * @return	none
 */
void ClearDataResend(void)
{
	rsndautostop = 0;
	sndtyp = SEND_NOTHG;
}

/*********************************************************************
 * @fn		readGMparam
 *
 * @brief	Read GDE GM parameters.
 *
 * @param	rdbuf -params save buffer
 *
 * @return	none
 */
void ReadGMParam(uint8 *rdbuf)
{
	rdbuf[ST_GM_HB_FREQ_POS] = hrtbt_inmin;
	rdbuf[ST_GM_DTCT_VAL_POS] = detectlevel;
	rdbuf[ST_GM_DTCT_ALG_POS] = 0;
	rdbuf[ST_GM_STATUS_POS] = gmst;
}

/*********************************************************************
 * @fn		setGMparam
 *
 * @brief	Set GDE GM parameters.
 *
 * @param	hrtbtmin -heart beat time in minutes.
 * @param	dtval -detect value level.
 * @param	alg -not used.
 * @param	status - lot status.
 *
 * @return	TRUE - set OK
 */
bool SetGMParam(uint8 hrtbtmin, uint8 dtval, uint8 alg, uint8 status)
{
	VOID alg;

	if ( hrtbtmin==0 || dtval==0 || dtval>sizeof(sqrthrhldarr))
		return FALSE;
	
	hrtbt_inmin = hrtbtmin;
	detectlevel = dtval;
	ModifyBenchmk((gmstatus_t)status);

	return TRUE;
}


/*********************************************************************
 * PRIVATE FUNCTIONS
 */

/*********************************************************************
 * @fn		GM_dev_init
 *
 * @brief	GM device init
 *
 * @param	none.
 *
 * @return	TRUE - init OK; FALSE - failed
 */
static bool GM_dev_init(void)
{
	uint8 id_str[3];
	uint8 gm_cfg = SET_GM_NORMAL;
	uint8 gm_gain = SET_GM_GAIN;
	
	HalI2CInit(GM_I2C_ADDR, gm_i2cclk);

	// Check GM sensor ID
	GM_read_reg(GM_ID_A_REG,id_str,sizeof(id_str));
	if (osal_memcmp(id_str,GM_ID_STR,sizeof(id_str)) == FALSE)
		return FALSE;

	// Setup GM sensor	
	GM_write_reg(GM_CONFIG_A_REG,&gm_cfg,1);
	GM_write_reg(GM_CONFIG_B_REG,&gm_gain,1);

#if 0
	GM_DRDY_INT_Cfg();
#endif

	return TRUE;
}


void InitGMState(void)
{
	// Clear resend times
	rsndautostop = 0;
	syncautostop = 0;
	// Clear read times and time sync times
	readcnt = 0;
	tmsynccnt = 0;
	// Clear empty and unknown status times
	empcnt = 0;
	unkwncnt = 0;
	// Clear benchmark times
	tmpbenchcnt = 0;
	
	gmst = GMFirstBoot;
	sndtyp =SEND_NOTHG;
/*
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
						initflag = InitBenchmk(prevgmst,tmpx,tmpy,tmpz);
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
	if (gmst != GMGetCar && gmst != GMNoCar)
		gmst = prevgmst;*/
}

/*********************************************************************
 * @fn		GM_dev_stop
 *
 * @brief	GM device stop
 *
 * @param	none
 *
 * @return	none
 */
static void GM_dev_stop(void)
{
	uint8 sleep_mode = SET_GM_SLEEP_MODE;

	HalI2CInit(GM_I2C_ADDR, gm_i2cclk);
	GM_write_reg(GM_MODE_REG, &sleep_mode, 1);
}

static void GM_dev_preread(void)
{
	uint8 singl_read = SET_GM_READ_ONCE;

	// reinit I2C after power saving
	HalI2CInit(GM_I2C_ADDR, gm_i2cclk);
	GM_write_reg(GM_MODE_REG, &singl_read, 1);
}

/*********************************************************************
 * @fn		GM_dev_read
 *
 * @brief	read i2c
 *
 * @param	none
 *
 * @return	none
 */
static void GM_dev_read(uint8 task_id)
{
	uint8 tmp_data[6]={0};
	int16 xval,yval,zval;
	
	if (GM_read_reg(GM_X_AXIS_MSB_REG, tmp_data, sizeof(tmp_data)) == TRUE)
	{
		xval = BUILD_UINT16(tmp_data[1],tmp_data[0]);
		yval = BUILD_UINT16(tmp_data[5],tmp_data[4]);
		zval = BUILD_UINT16(tmp_data[3],tmp_data[2]);

		// Advertised self test data (first data)
		if (GetSysState() == SYS_BOOTUP)
		{
			SendXYZVal(task_id,xval,yval,zval);
			SetIntState(WS_INT_DETECT);
			osal_start_timerEx(task_id, HG_SWITCH_EVT, SELF_TEST_PERIOD);
		}
		else
		{
#if ( !defined GM_TEST_COMM )
			osal_start_timerEx(task_id,GM_DATA_PROC_EVT,GM_READ_EVT_PERIOD-GM_READ_ONCE_PERIOD);
			GM_dev_proc(xval,yval,zval);
#else
			//osal_start_timerEx(task_id,GM_DATA_PROC_EVT,2000-GM_READ_ONCE_PERIOD);
			sndtyp = SEND_HRTBY;
#endif	// !GM_TEST_COMM

			GM_send_data(task_id,xval,yval,zval);

#if ( !defined ALLOW_DEBUG_OUTPUT )
			osal_set_event(task_id, BLE_SYS_WORKING_EVT);
#else
			osal_start_timerEx(task_id, BLE_SYS_WORKING_EVT, WAIT_SERIAL_OUTPUT_PERIOD);
#endif	// !ALLOW_DEBUG_OUTPUT
		}
	}

	return;
}


/*********************************************************************
 * @fn		GM_dev_proc
 *
 * @brief	GM read data process, includes status check, data process.
 *
 * @param	tmpX - X axis read value.
 * @param	tmpY - Y axis read value.
 * @param	tmpZ - Z axis read value.
 *
 * @return	none
 */
static void GM_dev_proc(int16 tmpX, int16 tmpY, int16 tmpZ)
{
	gmstatus_t tmpgmst;

	switch(gmst)
	{
		case GMNoCar:
		{
			tmpgmst = CheckCarIn(tmpX,tmpY,tmpZ);
			if (tmpgmst == GMGetCar)
			{
				set_data_change();
				// Reinit detect benchmark every time
				InitBenchmk(GMGetCar,tmpX, tmpY, tmpZ);

			}
			break;
		}
		case GMGetCar:
		{
			tmpgmst = CheckCarOut(tmpX,tmpY,tmpZ);
			if ( tmpgmst == GMNoCar)
			{
				gmst = tmpgmst;
				set_data_change();
				//NormRgltEmpBenchmk(tmpX,tmpY,tmpZ);
			}
			else if ( tmpgmst == GMError )
			{
				// Send error data
				set_heart_beat();
				PrintGMvalue(COM433_DEBUG_PORT, "\r\nOCP ERR:",tmpX,tmpY,tmpZ);
			}
			else
				PrintGMvalue(COM433_DEBUG_PORT, "\r\nDR:",tmpX,tmpY,tmpZ);
			break;
		}
		case GMFirstBoot:
		{
			InitBenchmk(gmst,tmpX,tmpY,tmpZ);
			GetDevPowerPrcnt();
			set_heart_beat();
			tmsyncflag = TRUE;
			break;
		}
		case GMError:
		default:
			break;
	}

}

/*********************************************************************
 * @fn		GM_send_data
 *
 * @brief	Send GM read data, include time sync, heart beat, change data.
 *
 * @param	task_id - Task ID.
 * @param	tmpX - X axis read value.
 * @param	tmpY - Y axis read value.
 * @param	tmpZ - Z axis read value.
 *
 * @return	none
 */
static void GM_send_data(uint8 task_id, int16 tmpX, int16 tmpY, int16 tmpZ)
{
	// Adjust benchmark ervery heart beat timing
	if (++readcnt >= (MILSEC_IN_MIN/GM_READ_EVT_PERIOD)*hrtbt_inmin)
	{
		readcnt = 0;
		tmsynccnt++;
		set_heart_beat();
	}

	// Perform time synchronization every 24h
	if (tmsynccnt >= (HOUR_IN_DAY*MIN_IN_HOUR)/hrtbt_inmin)
	{
		tmsynccnt = 0;
		tmsyncflag = TRUE;
		GetDevPowerPrcnt();
	}

	set_time_sync();

	if (sndtyp != SEND_NOTHG)
	{
		SetSysState(SYS_WORKING);
		switch (sndtyp)
		{
			case SEND_HRTBY:
				gdetmpr = GM_dev_get_tmpr();
				if (gdetmpr == GM_INVALID_TEMPR)
					SetGMSnState(GMSnErr);
				// Do not break
			case SEND_CHNG:
				SendGDEData(tmpX,tmpY,tmpZ);
				break;
			case SEND_SYNC:
				SendSyncTMReq();
				break;
			default:
				break;
		}
	}

	return;
}

/*********************************************************************
 * @fn		SendXYZVal
 *
 * @brief	Send X/Y/Z first read value from HMC.
 *
 * @param	tmpX - X axis read value.
 * @param	tmpY - Y axis read value.
 * @param	tmpZ - Z axis read value.
 *
 * @return	none
 */
static void SendXYZVal(uint8 task_id, int16 tmpX, int16 tmpY, int16 tmpZ)
{
#if ( defined USE_CC112X_RF )
	uint8 buf[GMS_PKT_MAX_LEN], len;

	IntConvertString(buf, tmpX);
	len = osal_strlen((char *)buf);
	buf[len++] = ' ';
	IntConvertString(buf+len, tmpY);
	len = osal_strlen((char *)buf);
	buf[len++] = ' ';
	IntConvertString(buf+len, tmpZ);
	len = osal_strlen((char *)buf);

	SetRFstate(RF_SEND);
	TxData(buf,len);
#else	// !USE_CC112X_RF
	IntConvertString(rfsndbuf, tmpX);
	rfsndlen = osal_strlen((char *)rfsndbuf);
	rfsndbuf[rfsndlen++] = ' ';
	IntConvertString(rfsndbuf+rfsndlen, tmpY);
	rfsndlen = osal_strlen((char *)rfsndbuf);
	rfsndbuf[rfsndlen++] = ' ';
	IntConvertString(rfsndbuf+rfsndlen, tmpZ);
	rfsndlen = osal_strlen((char *)rfsndbuf);

	RF_working(task_id, GetRFstate());
#endif	// USE_CC112X_RF
}

/*********************************************************************
 * @fn		GM_dev_get_tmpr
 *
 * @brief	Get temperature from HMC.
 *
 * @param	none
 *
 * @return	temperature
 */
static int8 GM_dev_get_tmpr(void)
{
	uint8 tmprval[2] = {0};
	int16 fulltmprval = 0;
	int8 tmpr;
	
	if (GM_read_reg(GM_TEMPR_MSB_REG, tmprval, sizeof(tmprval)) == FALSE)
		return GM_INVALID_TEMPR;
	
	fulltmprval = (int16)BUILD_UINT16(tmprval[1], tmprval[0]);
	tmpr = (int8)(fulltmprval/128) + GM_BASE_TEMPR;

	return tmpr;
}

/*********************************************************************
 * @fn		GetDevPowerPrcnt
 *
 * @brief	Get power percentage of device.
 *
 * @param	none
 *
 * @return	temperature
 */
static void GetDevPowerPrcnt()
{
	uint8 tmpprcnt;

	tmpprcnt = CalcBatteryPercent();

	// Use old percentage if new>old
	btprcnt = (tmpprcnt>btprcnt? btprcnt: tmpprcnt);
}

/*********************************************************************
 * @fn		set_time_sync
 *
 * @brief	Send time synchronization request.
 *
 * @param	none
 *
 * @return	none
 */
static void set_time_sync(void)
{
	// Send time synchronization when nothing wait sent, lowest priority
	if (sndtyp == SEND_NOTHG)
		if (tmsyncflag == TRUE)
			sndtyp = SEND_SYNC;
}


/*********************************************************************
 * @fn		set_heart_beat
 *
 * @brief	Send heart beat data next GM read process.
 *
 * @param	none
 *
 * @return	none
 */
static void set_heart_beat(void)
{
	// send heart beat when nothing wait sent
	if (sndtyp == SEND_NOTHG)
		sndtyp = SEND_HRTBY;
}

/*********************************************************************
 * @fn		set_data_change
 *
 * @brief	Send lot status change data next GM read process, have highest priority.
 *
 * @param	none
 *
 * @return	none
 */
static void set_data_change(void)
{
	sndtyp = SEND_CHNG;
}

/*********************************************************************
* @fn		GM_write_reg
*
* @brief		This function implements the I2C protocol to write to a sensor. he sensor must
*			be selected before this routine is called.
*
* @param	addr - which register to write
* @param	pBuf - pointer to buffer containing data to be written
* @param	nBytes - number of bytes to write
*
* @return	TRUE if successful write
*/
static bool GM_write_reg(uint8 addr, uint8 *pBuf, uint8 nBytes)
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

/*********************************************************************

 * @fn		GM_read_reg
 *
 * @brief	This function implements the I2C protocol to read from a sensor. The sensor must
 *			be selected before this routine is called.
 *
 * @param	addr - which register to read
 * @param	pBuf - pointer to buffer to place data
 * @param	nBytes - numbver of bytes to read
 *
 * @return			TRUE if the required number of bytes are reveived
 */
static bool GM_read_reg(uint8 addr, uint8 *pBuf, uint8 nBytes)
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
 * @fn		CheckCarIn
 *
 * @brief	Check lot status.
 *
 * @param	tmpX - X axis read value.
 * @param	tmpY - Y axis read value.
 * @param	tmpZ - Z axis read value.
 *
 * @return	none
 */
static gmstatus_t CheckCarIn(int16 tmpX, int16 tmpY, int16 tmpZ)
{
	uint32 xdev,ydev,zdev,devsqrsum;

	// Prepare set benchmark
	if (tmpbenchcnt == 0)
		return GMNoCar;

	xdev = CALC_ABS(tmpX-Xbenchmk);
	ydev = CALC_ABS(tmpY-Ybenchmk);
	zdev = CALC_ABS(tmpZ-Zbenchmk);

	// X/Y/Z almost no change
	if (xdev+ydev+zdev < NO_CHANGE_THRSHLD)
		return GMNoCar;

	devsqrsum = xdev*xdev+ydev*ydev+zdev*zdev;

	// First process long time abnormal value
	if (devsqrsum >= sqrthrhldarr[detectlevel-1]/2 && devsqrsum < sqrthrhldarr[detectlevel-1] )
	{
		if (++unkwncnt > MAX_ABNORMAL_TIMES)
		{
			unkwncnt = 0;

			return GMGetCar;
		}
		else
		{
			PrintGMvalue(COM433_DEBUG_PORT, "\r\nABN:",tmpX,tmpY,tmpZ);

			return GMNoCar;
		}
	}
	else
	{
		unkwncnt = 0;
	}

	// Dev^2 belongs to normal car in range
	if (devsqrsum >= sqrthrhldarr[detectlevel-1] && devsqrsum < GM_ERROR_VALUE)
	{
		return GMGetCar;
	}
	// Almost no change
	else if (devsqrsum < sqrthrhldarr[detectlevel-1]/4)
	{
		empcnt++;
		// Continuously adjust benchmark at first 10 times and fill the array
		if ( tmpbenchcnt < BENCH_AVG_LEN)
			NormRgltEmpBenchmk(tmpX,tmpY,tmpZ);
		// Ajust benchmark envery 5min
		else if (empcnt >= ADJ_BENCHMK_TIMES)
			NormRgltEmpBenchmk(tmpX,tmpY,tmpZ);
		else
			PrintGMvalue(COM433_DEBUG_PORT, "\r\nR:",tmpX,tmpY,tmpZ);
	}
	// A little change
	else if (devsqrsum >= sqrthrhldarr[detectlevel-1]/4 && devsqrsum < sqrthrhldarr[detectlevel-1]/2 )
	{
		PrintGMvalue(COM433_DEBUG_PORT, "\r\nR:",tmpX,tmpY,tmpZ);
	}
	// Error value detect
	else
	{
		// Send error data
		set_heart_beat();
		PrintGMvalue(COM433_DEBUG_PORT, "\r\nEMP ERR:",tmpX,tmpY,tmpZ);
	}

	return GMNoCar;
}

/*********************************************************************
 * @fn		CheckCarOut
 *
 * @brief	Check lot status.
 *
 * @param	tmpX - X axis read value.
 * @param	tmpY - Y axis read value.
 * @param	tmpZ - Z axis read value.
 *
 * @return	none
 */
static gmstatus_t CheckCarOut(int16 tmpX, int16 tmpY, int16 tmpZ)
{
	uint32 xdev,ydev,zdev,devsqrsum;

	// normal check
	if ( tmpbenchcnt != 0 )
	{
		xdev = CALC_ABS(tmpX-Xbenchmk);
		ydev = CALC_ABS(tmpY-Ybenchmk);
		zdev = CALC_ABS(tmpZ-Zbenchmk);

		devsqrsum = xdev*xdev+ydev*ydev+zdev*zdev;

		// Change state when lower than 1/4 threshold, use hysteresis method
		if (devsqrsum < sqrthrhldarr[detectlevel-1]/4)
			return GMNoCar;
		else if (devsqrsum >= GM_ERROR_VALUE)
			return GMError;
		else
			return GMGetCar;
	}
	// in case gmstatus modified by GTE, no benchmark
	else
	{
		xdev = CALC_ABS(tmpX-Xdtctval);
		ydev = CALC_ABS(tmpY-Ydtctval);
		zdev = CALC_ABS(tmpZ-Zdtctval);

		devsqrsum = xdev*xdev+ydev*ydev+zdev*zdev;

		// Use 1/2 threshold
		if ( devsqrsum >= sqrthrhldarr[detectlevel-1]/2 )
			return GMNoCar;
		else
			return GMGetCar;
	}
}


/*********************************************************************
 * @fn		ModifyBenchmk
 *
 * @brief	Modify benchmark by GM status.
 *
 * @param	newstts -new GM status
 *
 * @return	none
 */
static void ModifyBenchmk(gmstatus_t newstts)
{
	if (gmst == newstts)
		return;

	if (newstts > GMGetCar)
		return;

	set_data_change();
	
	// No Car--> Get Car
	if (newstts == GMGetCar)
	{
		InitBenchmk(newstts,Xbenchmk,Ybenchmk,Zbenchmk);
		tmpbenchcnt = 0;	// previous benchmark invalid, when perform CheckCarOut
		
	}
	// Get Car-->No Car
	else
		InitBenchmk(newstts,Xdtctval,Ydtctval,Zdtctval);

	// Clear resend times
	rsndautostop = 0;
	syncautostop = 0;
	
	empcnt = 0;
	unkwncnt = 0;
}

/*********************************************************************
 * @fn		InitBenchmk
 *
 * @brief	Initiate GM benchmark based on different GM & lot status.
 *
 * @param	gmstts - current GM & lot status.
 * @param	xVal - current X axis value.
 * @param	yVal - current Y axis value.
 * @param	zVal - current Z axis value.
 *
 * @return	TRUE - init OK
 */
static bool InitBenchmk(gmstatus_t gmstts,int16 xVal,int16 yVal, int16 zVal)
{
	switch (gmstts)
	{
		case GMFirstBoot:	// Assume no car
		case GMNoCar:
			// clear previous benchmark
			tmpbenchcnt = 0;
			NormRgltEmpBenchmk(xVal,yVal,zVal);
			gmst = GMNoCar;
			break;
		case GMGetCar:
			WghtRgltOcpBenchmk(xVal,yVal,zVal);
			gmst = GMGetCar;
			break;
		default:
			return FALSE;
	}

	return TRUE;
}

/*********************************************************************
 * @fn		NormRgltEmpBenchmk
 *
 * @brief	Regulate GM detect benchmark by arithmetic average.
 *
 * @param	xVal - current X axis value.
 * @param	yVal - current Y axis value.
 * @param	zVal - current Z axis value.
 *
 * @return	none
 */
static void NormRgltEmpBenchmk(int16 xVal,int16 yVal, int16 zVal)
{
	empcnt = 0;
	tmpbenchcnt++;

	if ( tmpbenchcnt > BENCH_AVG_LEN )
	{
		// Keep tmpbenchcnt always > 10
		if (tmpbenchcnt > 2*BENCH_AVG_LEN)
			tmpbenchcnt = BENCH_AVG_LEN+1;
		
		tmpXbench[tmpbenchcnt-BENCH_AVG_LEN-1] = xVal;
		tmpYbench[tmpbenchcnt-BENCH_AVG_LEN-1] = yVal;
		tmpZbench[tmpbenchcnt-BENCH_AVG_LEN-1] = zVal;

		Xbenchmk = CalcAvrg(tmpXbench,BENCH_AVG_LEN);
		Ybenchmk = CalcAvrg(tmpYbench,BENCH_AVG_LEN);
		Zbenchmk = CalcAvrg(tmpZbench,BENCH_AVG_LEN);
	}
	else
	{
		tmpXbench[tmpbenchcnt-1]=xVal;
		tmpYbench[tmpbenchcnt-1]=yVal;
		tmpZbench[tmpbenchcnt-1]=zVal;

		Xbenchmk = CalcAvrg(tmpXbench,tmpbenchcnt);
		Ybenchmk = CalcAvrg(tmpYbench,tmpbenchcnt);
		Zbenchmk = CalcAvrg(tmpZbench,tmpbenchcnt);	
	}

	PrintGMvalue(COM433_DEBUG_PORT, "\r\nNC bench:", Xbenchmk, Ybenchmk, Zbenchmk);
}

/*********************************************************************
 * @fn		CalcAvrg
 *
 * @brief	Calculate arithmetic average of int16 buffer.
 *
 * @param	buf - int16 buffer
 * @param	n - counts in buffer
 *
 * @return	arithmetic average
 */
static int16 CalcAvrg(int16 *buf, uint8 n)
{
	uint8 i;
	int32 sum=0;
	
	for (i=0;i<n;i++)
		sum+=buf[i];
	
	return sum/n;	
}

/*********************************************************************
 * @fn		WghtRgltOcpBenchmk
 *
 * @brief	Regulate GM car detect benchmark by weight average.
 *
 * @param	xVal - current X axis value.
 * @param	yVal - current Y axis value.
 * @param	zVal - current Z axis value.
 *
 * @return	none
 */
static void WghtRgltOcpBenchmk(int16 xVal,int16 yVal, int16 zVal)
{
#if 1
	tmpdtctcnt = 1;
#else
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

		Xdtctval = CalcWeight(tmpXdtctval,tmpdtctcnt,BENCH_WEIGHT_LEN);
		Ydtctval = CalcWeight(tmpXdtctval,tmpdtctcnt,BENCH_WEIGHT_LEN);
		Zdtctval = CalcWeight(tmpXdtctval,tmpdtctcnt,BENCH_WEIGHT_LEN);
	}
	else
#endif
	{
		tmpXdtctval[tmpdtctcnt-1]=xVal;
		tmpYdtctval[tmpdtctcnt-1]=yVal;
		tmpZdtctval[tmpdtctcnt-1]=zVal;

		Xdtctval = CalcWeight(tmpXdtctval,tmpdtctcnt,tmpdtctcnt);
		Ydtctval = CalcWeight(tmpYdtctval,tmpdtctcnt,tmpdtctcnt);
		Zdtctval = CalcWeight(tmpZdtctval,tmpdtctcnt,tmpdtctcnt);	
	}

	PrintGMvalue(COM433_DEBUG_PORT, "\r\nDT bench:", Xdtctval, Ydtctval, Zdtctval);
}


/*********************************************************************
 * @fn		CalcWeight
 *
 * @brief	Calculate weight average of int16 buffer. High weight index calculate by length automatically.
 *
 * @param	buf - int16 buffer
 * @param	len - current data recieve(current highest weight index)
 * @param	bound - array bound
 *
 * @return	arithmetic average
 */
static int16 CalcWeight(int16 *buf, uint8 len, uint8 bound)
{
	uint8 i,weight;
	int32 sum=0,nweight=0;
	
	for (i=0;i<bound;i++)
	{
		weight = GET_WEIGHT(i+1);
		// restrict index in bound
		sum += weight*buf[(i+len)%bound];
		nweight += weight;
	}

	return sum/nweight;	
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
static uint8 CalcBatteryPercent(void)
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


/*
static void storeGMstate(void)
{
	 if(osal_snv_write(GM_STATE_NV_ID, sizeof(gmst),&gmst)==NV_OPER_FAILED)
	 	Com433WriteStr(COM433_DEBUG_PORT,"\r\n!!!NV Failed!!!");
}
*/

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

