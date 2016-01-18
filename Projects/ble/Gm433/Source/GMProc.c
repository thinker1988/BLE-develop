/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "OSAL_Timers.h"

#include "hal_i2c.h"
#include "hal_adc.h"

#include "BLECore.h"
#include "Com433.h"
#include "GMProc.h"
#include "Pktfmt.h"
#include "RFProc.h"

#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
#include "CoreUpgrade.h"
#endif	// GM_IMAGE_A || GM_IMAGE_B

/*********************************************************************
 * MACROS
 */
// define weight value in this program
#define GET_WEIGHT(a)			(a)

/*********************************************************************
 * CONSTANTS
 */
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

// GM car detect threshold & default use level 1, i.e. 1000
uint32 sqrthrhldarr[] = {800,1000,1200};
uint8 detectlevel = 1;

// GDE benchmark fix flag
bool benchfixflg = FALSE;

// Read GM data tick counts, increase every GM data read.
uint16 readcnt = 0;

#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
uint16 ordrcnt = 0;
#endif	// GM_IMAGE_A || GM_IMAGE_B

/*********************************************************************
 * EXTERNAL VARIABLES
 */

// Basic benchmark
int16 Xbenchmk,Ybenchmk,Zbenchmk;

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
static uint8 sndtyp = 0;

// Current GM state
static detectstatus_t cardetect = BENCH_CALIBRATING;

// Lot status change resend times (<=MAX_RESEND_TIMES)
static uint8 chngautostop;

// Time synchronization counts, increase every heart beat times
static uint16 tmsynccnt = 0;
// Time synchronization data resend times
static uint8 syncautostop;

// Adjust benchmark count in empty status
static uint16 bnchadjcnt = 0;

// Unknow status detect counts
static uint16 unkwncnt = 0;

// Temporary benchmark set and counts
static int16 tmpXbench[BENCH_AVG_LEN],tmpYbench[BENCH_AVG_LEN],tmpZbench[BENCH_AVG_LEN];
static uint8 tmpbenchcnt = 0;

// Car detect Benchmark
static int16 Xdtctval,Ydtctval,Zdtctval;
// Temporary detect benchmark
static int16 tmpXdtctval[BENCH_WEIGHT_LEN],tmpYdtctval[BENCH_WEIGHT_LEN],tmpZdtctval[BENCH_WEIGHT_LEN];
static uint8 tmpdtctcnt = 0;

// GDE temperature
static int8 gdetmpr = 0;

// GDE battery power remain percentage
static uint8 btprcnt = 100;

// Empty status reversed flag, means no empty benchmark
static bool emprvsflg = FALSE;

// Finish first boot read GM value flag
static bool finfistbootflg = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bool GM_dev_init(void);
static void GM_DRDY_INT_Cfg(void);

static void InitGMState(void);
static void ReadPrevGMSettings(void);

static void GM_dev_stop(void);

static void GM_dev_preread(void);

static void GM_dev_read(uint8 task_id);
static void GM_dev_proc(int16 tmpX,int16 tmpY,int16 tmpZ);

static void ReprepareSync(void);

static void GM_read_tick_rnd_set(void);
static void GM_read_tick_rst(void);
static void GM_read_tick_update(void);
static void GM_send_data(uint8 task_id, int16 tmpX, int16 tmpY, int16 tmpZ);

static void SendXYZVal(uint8 task_id, int16 tmpX, int16 tmpY, int16 tmpZ);

static int8 GM_dev_get_tmpr(void);
static uint8 CalcBatteryPercent(void);

static bool GM_read_reg(uint8 addr,uint8 * pBuf,uint8 nBytes);
static bool GM_write_reg(uint8 addr, uint8 *pBuf, uint8 nBytes);


static detectstatus_t CheckCarIn(int16 tmpX, int16 tmpY, int16 tmpZ);
static void ProcEmpData(detectstatus_t curdtct, int16 tmpX, int16 tmpY, int16 tmpZ);
static detectstatus_t CheckCarOut(int16 tmpX, int16 tmpY, int16 tmpZ);
static void ProcOcpData(detectstatus_t curdtct, int16 tmpX, int16 tmpY, int16 tmpZ);


static void ModifyBenchmk(detectstatus_t newstts);

static void NormRgltEmpBenchmk(int16 xVal,int16 yVal, int16 zVal);
static int16 CalcAvrg(int16 *buf, uint8 n);

static void WghtRgltOcpBenchmk(int16 xVal, int16 yVal, int16 zVal);
static int16 CalcWeight(int16 * buf, uint8 len, uint8 bound);

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
			GM_dev_init();
			InitGMState();
			InitCommDevID();
			gmsnst = GMSnReq;	// Prepare read once
			osal_start_timerEx(task_id,GM_DATA_PROC_EVT,WAIT_RF_WORK_PERIOD);
			break;
		case GMSnReq:
			PowerHold(task_id);
			GM_dev_preread();
			gmsnst = GMSnRead;
			osal_start_timerEx(task_id, GM_DATA_PROC_EVT, GM_SNSR_MEASURE_PERIOD);
			/*while(! GM_DRDY_INT_PIN);
			osal_set_event(task_id, GM_DATA_PROC_EVT);*/
			break;
		case GMSnRead:
			gmsnst = GMSnReq;
			GM_dev_read(task_id);
			break;
		case GMSnSleep:
			GM_dev_stop();
			break;
		default:
			break;
	}
}

void SetGMState(detectstatus_t nwst)
{
	cardetect=nwst;
}

detectstatus_t GetGMState(void)
{
	return cardetect;
}



void ResetBenchmark(uint8 *bnchmk, uint8 len)
{
	int16 tmpX,tmpY,tmpZ;
	
	if (len != EVLEN_GDE_BENCH_INFO)
		return;

	tmpX = BUILD_UINT16(bnchmk[GDE_X_L_BCHMRK_POS], bnchmk[GDE_X_H_BCHMRK_POS]);
	tmpY = BUILD_UINT16(bnchmk[GDE_Y_L_BCHMRK_POS], bnchmk[GDE_Y_H_BCHMRK_POS]);
	tmpZ = BUILD_UINT16(bnchmk[GDE_Z_L_BCHMRK_POS], bnchmk[GDE_Z_H_BCHMRK_POS]);

	if (tmpX==0 && tmpY==0 && tmpZ==0)
		InitGMState();
}


void SendSyncTMReq(void)
{
	uint8 tmsync=TRUE;

	RFDataForm(GDE_SUBTYPE_TMSYN_REQ,&tmsync,sizeof(tmsync));

#if ( !defined GM_TEST_COMM )
	if (syncautostop++ > MAX_TM_SYNC_TIMES)
	{
		ClearSyncTMReq();
		ReprepareSync();
	}
#else	// GM_TEST_COMM
	Com433WriteStr(COM433_DEBUG_PORT,"\r\nSend sync...\t");
#endif	// !GM_TEST_COMM
}


/*********************************************************************
 * @fn		FormHrtbtData
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
void FormHrtbtData(uint8* hrtbtdata, int16 tmpX, int16 tmpY, int16 tmpZ)
{
	hrtbtdata[HRT_BT_BATT_POS]=btprcnt;
	hrtbtdata[HRT_BT_TMPR_POS]=gdetmpr;
	
	hrtbtdata[HRT_BT_XVAL_H_POS]=HI_UINT16(tmpX);
	hrtbtdata[HRT_BT_XVAL_L_POS]=LO_UINT16(tmpX);

	hrtbtdata[HRT_BT_YVAL_H_POS]=HI_UINT16(tmpY);
	hrtbtdata[HRT_BT_YVAL_L_POS]=LO_UINT16(tmpY);

	hrtbtdata[HRT_BT_ZVAL_H_POS]=HI_UINT16(tmpZ);
	hrtbtdata[HRT_BT_ZVAL_L_POS]=LO_UINT16(tmpZ);
	
	hrtbtdata[HRT_BT_STAT_POS]=(cardetect==CAR_DETECTED_OK? CAR_DETECTED_OK: NO_CAR_DETECTED);

	//if ( cardetect!=NO_CAR_DETECTED && cardetect!=CAR_DETECTED_OK)
	//	return;
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
void set_time_sync(void)
{
	SET_SEND_BIT(sndtyp, SND_BIT_TM_SYNC);
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
void set_heart_beat(void)
{
	SET_SEND_BIT(sndtyp, SND_BIT_HRT_BT);
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
void set_data_change(void)
{
	chngautostop = 0;	// restart sen count
	SET_SEND_BIT(sndtyp, SND_BIT_DAT_CHNG);
}

void ClearSyncTMReq(void)
{
	tmsynccnt = 0;
	syncautostop = 0;
	CLR_SEND_BIT(sndtyp, SND_BIT_TM_SYNC);
}


void ClearHeartBeat(void)
{
	CLR_SEND_BIT(sndtyp, SND_BIT_HRT_BT);
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
	chngautostop = 0;
	CLR_SEND_BIT(sndtyp, SND_BIT_DAT_CHNG);
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
	rdbuf[ST_GM_DTCT_SENS_POS] = detectlevel;
	rdbuf[ST_GM_BENCH_ALG_POS] = benchfixflg;
	rdbuf[ST_GM_STATUS_POS] = (cardetect==CAR_DETECTED_OK? CAR_DETECTED_OK: NO_CAR_DETECTED);
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
	hrtbt_inmin = (hrtbtmin==0? hrtbt_inmin: hrtbtmin);
	detectlevel = (dtval>=sizeof(sqrthrhldarr)? detectlevel: dtval);
	benchfixflg = (alg==FALSE? FALSE: TRUE);
	ModifyBenchmk((detectstatus_t)status);

	return TRUE;
}

void ReadGDEBench(uint8* pVal)
{
	if (GetEmpBenchCnt() == 0)
	{
		osal_memset(pVal, 0, EVLEN_GDE_BENCH_INFO);
		return;
	}
	pVal[GDE_X_H_BCHMRK_POS] = HI_UINT16(Xbenchmk);
	pVal[GDE_X_L_BCHMRK_POS] = LO_UINT16(Xbenchmk);
	pVal[GDE_Y_H_BCHMRK_POS] = HI_UINT16(Ybenchmk);
	pVal[GDE_Y_L_BCHMRK_POS] = LO_UINT16(Ybenchmk);
	pVal[GDE_Z_H_BCHMRK_POS] = HI_UINT16(Zbenchmk);
	pVal[GDE_Z_L_BCHMRK_POS] = LO_UINT16(Zbenchmk);
}


/*********************************************************************
 * @fn		InitBenchmk
 *
 * @brief	Initiate GM benchmark based on different GM & lot status.
 *
 * @param	cardetectts - current GM & lot status.
 * @param	xVal - current X axis value.
 * @param	yVal - current Y axis value.
 * @param	zVal - current Z axis value.
 *
 * @return	TRUE - init OK
 */
bool InitBenchmk(detectstatus_t cardetectts,int16 xVal,int16 yVal, int16 zVal)
{
	switch (cardetectts)
	{
		case BENCH_CALIBRATING:
			// clear previous benchmark, Assume no car
			tmpbenchcnt = 0;
			NormRgltEmpBenchmk(xVal,yVal,zVal);
			cardetect = NO_CAR_DETECTED;
			break;
		case NO_CAR_DETECTED:
			NormRgltEmpBenchmk(xVal,yVal,zVal);
			cardetect = NO_CAR_DETECTED;
			break;
		case CAR_DETECTED_OK:
			tmpdtctcnt = 0;
			WghtRgltOcpBenchmk(xVal,yVal,zVal);
			cardetect = CAR_DETECTED_OK;
			break;
		default:
			return FALSE;
	}

	return TRUE;
}

uint8 GetEmpBenchCnt(void)
{
	return tmpbenchcnt;
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
void GetDevPowerPrcnt()
{
	uint8 tmpprcnt;

	tmpprcnt = CalcBatteryPercent();

	// Use old percentage if new>old
	btprcnt = (tmpprcnt>btprcnt? btprcnt: tmpprcnt);
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

	GM_DRDY_INT_Cfg();

	return TRUE;
}

static void GM_DRDY_INT_Cfg(void)
{
	GM_DRDY_INT_PINSEL &= (uint8) ~GM_DRDY_INT_IE;
	GM_DRDY_INT_PINDIR &= (uint8) ~GM_DRDY_INT_IE;
#if 0

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

#endif
}

/*********************************************************************
 * @fn		InitGMState
 *
 * @brief	Init GM system state, including read count and cardetect state, etc..
 *
 * @param	none
 *
 * @return	none
 */

static void InitGMState(void)
{
	// Clear resend times and time synchronization
	ClearDataResend();
	ClearSyncTMReq();

#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
	// Clear read times if not preparing upgrade
	if (GetPrepUpgdState() == FALSE)
#endif	// GM_IMAGE_A || GM_IMAGE_B
		GM_read_tick_rnd_set();

	// Clear adjust and unknown status times
	bnchadjcnt = 0;
	unkwncnt = 0;
	
	// Clear benchmark times
	tmpbenchcnt = 0;
	cardetect = BENCH_CALIBRATING;
	btprcnt = 100;

#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
	ReadPrevGMSettings();
#endif	// GM_IMAGE_A || GM_IMAGE_B

	// Begin time sync and heart beat every time
	set_time_sync();
	set_heart_beat();
	GetDevPowerPrcnt();
}

static void ReadPrevGMSettings(void)
{
	int16 prevXbench,prevYbench,prevZbench;
	uint8 prevstatus;

	if (finfistbootflg == FALSE)
	{
		finfistbootflg = TRUE;

		if (ReadPrevBenchVal(&prevXbench,&prevYbench,&prevZbench) == TRUE)
			InitBenchmk(BENCH_CALIBRATING,prevXbench,prevYbench,prevZbench);

		if (ReadPrevDtctStatus(&prevstatus) == TRUE)
		{
			if (prevstatus!=NO_CAR_DETECTED && prevstatus!=CAR_DETECTED_OK)
				prevstatus = BENCH_CALIBRATING;
			Com433WriteInt(COM433_DEBUG_PORT, "\r\nPV:", prevstatus, 10);
			SetGMState((detectstatus_t)prevstatus);
		}
	}
	return;
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
	int16 xval=0,yval=0,zval=0;
	
	if (GM_read_reg(GM_X_AXIS_MSB_REG, tmp_data, sizeof(tmp_data)) == TRUE)
	{
		xval = BUILD_UINT16(tmp_data[1],tmp_data[0]);
		yval = BUILD_UINT16(tmp_data[5],tmp_data[4]);
		zval = BUILD_UINT16(tmp_data[3],tmp_data[2]);
	}

	// Advertised self test data (first data)
	if (GetSysState() == SYS_BOOTUP)
	{
		SendXYZVal(task_id,xval,yval,zval);
		SetIntState(WS_INT_DETECT);
		osal_start_timerEx(task_id, HG_SWITCH_EVT, SELF_TEST_PERIOD);
	}
	else
	{
		osal_start_timerEx(task_id,GM_DATA_PROC_EVT,GM_READ_EVT_PERIOD-GM_SNSR_MEASURE_PERIOD);
		GM_dev_proc(xval,yval,zval);
		GM_read_tick_update();
		GM_send_data(task_id,xval,yval,zval);

#if ( !defined ALLOW_DEBUG_OUTPUT )
		osal_set_event(task_id, BLE_SYS_WORKING_EVT);
#else
		osal_start_timerEx(task_id, BLE_SYS_WORKING_EVT, WAIT_SERIAL_OUTPUT_PERIOD);
#endif	// !ALLOW_DEBUG_OUTPUT
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
#if ( !defined GM_TEST_COMM )
	detectstatus_t tmpcardetect;

	switch(cardetect)
	{
		case NO_CAR_DETECTED:
		{
			tmpcardetect = CheckCarIn(tmpX,tmpY,tmpZ);
			ProcEmpData(tmpcardetect,tmpX,tmpY,tmpZ);
			break;
		}
		case CAR_DETECTED_OK:
		{
			tmpcardetect = CheckCarOut(tmpX,tmpY,tmpZ);
			ProcOcpData(tmpcardetect,tmpX,tmpY,tmpZ);
			break;
		}
		case BENCH_CALIBRATING:
		{
			if (bnchadjcnt++ < GM_SNSR_WAIT_STEADY_CNT)
			{
				PrintGMvalue(COM433_DEBUG_PORT, "\r\nR:",tmpX,tmpY,tmpZ);
				set_heart_beat();	// keep heart beat during wait steady
			}
			else
				InitBenchmk(cardetect,tmpX,tmpY,tmpZ);
			break;
		}
		case ABNORMAL_DETECTION:
		default:
			break;
	}
#else	//  GM_TEST_COMM
	set_time_sync();
#endif	// !GM_TEST_COMM

}

static void ReprepareSync(void)
{
	tmsynccnt = c_rand()*MIN_IN_HOUR*HOUR_IN_DAY/MAX_RANDOM_SECONDS/hrtbt_inmin/MILSEC_IN_SEC;
}

static void GM_read_tick_rnd_set(void)
{
	readcnt = c_rand()*SEC_IN_MIN/MAX_RANDOM_SECONDS*hrtbt_inmin/GM_READ_EVT_PERIOD;
}

static void GM_read_tick_rst(void)
{
	readcnt = 0;
}

static void GM_read_tick_update(void)
{
	uint16 hrtbt_tick_cnt;

	hrtbt_tick_cnt = ((MILSEC_IN_MIN/GM_READ_EVT_PERIOD)*hrtbt_inmin);
	// Calculate heart beat timing
	if (++readcnt >= hrtbt_tick_cnt)
	{
		GM_read_tick_rst();
		tmsynccnt++;
		set_heart_beat();
	}

	// Perform time synchronization every 24h
	if (tmsynccnt >= (HOUR_IN_DAY*MIN_IN_HOUR)/hrtbt_inmin)
	{
		tmsynccnt = 0;
		set_time_sync();
		GetDevPowerPrcnt();
	}

#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
	if (GetPrepUpgdState()== TRUE)
	{
		if (tmsynccnt >= ordrcnt/hrtbt_tick_cnt)
			if (readcnt >= ordrcnt%hrtbt_tick_cnt)
				SetSysState(SYS_UPGRADE);	//  system state will change
	}
#endif	// GM_IMAGE_A || GM_IMAGE_B
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
	uint8 i,hrtbtdata[EVLEN_GDE_HRTBT];

	if (NO_MSG_SEND(sndtyp) || GetSysState()==SYS_SETUP || GetSysState()==SYS_UPGRADE)
		return;

	SetSysState(SYS_WORKING);
	PowerHold(task_id);

	for (i=0;i<sizeof(sndtyp)*8;i++)
	{
		if ((sndtyp>>i) & 0x01 != 0)
			break;
	}

	switch(i)
	{
		case SND_BIT_DAT_CHNG:
			FormHrtbtData(hrtbtdata,tmpX,tmpY,tmpZ);
			RFDataForm(GDE_SUBTYPE_CARINFO_REQ,hrtbtdata,EVLEN_GDE_HRTBT);
			if (chngautostop++ > MAX_RESEND_TIMES)
				ClearDataResend();
			break;
		case SND_BIT_HRT_BT:
			// Get temperature
			gdetmpr = GM_dev_get_tmpr();
			FormHrtbtData(hrtbtdata,tmpX,tmpY,tmpZ);
			RFDataForm(GDE_SUBTYPE_HRTBEAT_REQ,hrtbtdata,EVLEN_GDE_HRTBT);
			ClearHeartBeat();
			break;
		case SND_BIT_TM_SYNC:
			SendSyncTMReq();
			break;
		default:
			break;
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
	uint8 buf[GMS_PKT_MAX_LEN] = {0}, len;

	IntConvertString(buf, tmpX);
	len = osal_strlen((char *)buf);
	buf[len++] = ' ';
	IntConvertString(buf+len, tmpY);
	len = osal_strlen((char *)buf);
	buf[len++] = ' ';
	IntConvertString(buf+len, tmpZ);
	len = osal_strlen((char *)buf);

	RFDataSend(buf,len);
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
static detectstatus_t CheckCarIn(int16 tmpX, int16 tmpY, int16 tmpZ)
{
	uint32 xdev,ydev,zdev,devsqrsum;

	if (tmpbenchcnt == 0)	// if there is no benchmark
		return BENCH_CALIBRATING;

	xdev = CALC_ABS(tmpX-Xbenchmk);
	ydev = CALC_ABS(tmpY-Ybenchmk);
	zdev = CALC_ABS(tmpZ-Zbenchmk);

	// X/Y/Z almost no change
	if (xdev+ydev+zdev < NO_CHANGE_THRSHLD)
		return NO_CAR_DETECTED;

	devsqrsum = xdev*xdev+ydev*ydev+zdev*zdev;

	// First process long time abnormal value
	if (devsqrsum >= sqrthrhldarr[detectlevel]/4 && devsqrsum < sqrthrhldarr[detectlevel] )
		return ABNORMAL_DETECTION;
	else if (devsqrsum >= sqrthrhldarr[detectlevel] && devsqrsum < GM_ERROR_VALUE)
		return CAR_DETECTED_OK;
	else if (devsqrsum < sqrthrhldarr[detectlevel]/4)
		return NO_CAR_DETECTED;
	else
		return ERROR_DETECTION;

}

static void ProcEmpData(detectstatus_t curdtct, int16 tmpX, int16 tmpY, int16 tmpZ)
{
	if (curdtct != ABNORMAL_DETECTION)
		unkwncnt = 0;

	switch(curdtct)
	{
		case CAR_DETECTED_OK:
			set_data_change();
			// Reinit detect benchmark every time
			InitBenchmk(CAR_DETECTED_OK,tmpX, tmpY, tmpZ);
			break;
		case NO_CAR_DETECTED:
		{
			bnchadjcnt++;
			// Continuously adjust benchmark at first 10 times and fill the array
			if ( tmpbenchcnt < BENCH_AVG_LEN)
				NormRgltEmpBenchmk(tmpX,tmpY,tmpZ);
			// Adjust benchmark at each ADJ_BENCHMK_TIMES
			else if (bnchadjcnt >= ADJ_BENCHMK_TIMES && benchfixflg == FALSE)
				NormRgltEmpBenchmk(tmpX,tmpY,tmpZ);
			else
				PrintGMvalue(COM433_DEBUG_PORT, "\r\nR:",tmpX,tmpY,tmpZ);
			break;
		}
		case BENCH_CALIBRATING:
			InitBenchmk(BENCH_CALIBRATING,tmpX,tmpY,tmpZ);
			break;
		case ABNORMAL_DETECTION:
			if (++unkwncnt > MAX_ABNORMAL_TIMES)
			{
				unkwncnt = 0;
				set_data_change();
				InitBenchmk(CAR_DETECTED_OK,tmpX, tmpY, tmpZ);
			}
			else
				PrintGMvalue(COM433_DEBUG_PORT, "\r\nABN:",tmpX,tmpY,tmpZ);
			break;
		case ERROR_DETECTION:
			// Send error data
			set_heart_beat();
			PrintGMvalue(COM433_DEBUG_PORT, "\r\nEMP ERR:",tmpX,tmpY,tmpZ);
			break;
		default:
			break;
	}
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
static detectstatus_t CheckCarOut(int16 tmpX, int16 tmpY, int16 tmpZ)
{
	uint32 xdev,ydev,zdev,devsqrsum;

	// Normal check base on empty benchmark
	if ( emprvsflg == FALSE )
	{
		xdev = CALC_ABS(tmpX-Xbenchmk);
		ydev = CALC_ABS(tmpY-Ybenchmk);
		zdev = CALC_ABS(tmpZ-Zbenchmk);

		devsqrsum = xdev*xdev+ydev*ydev+zdev*zdev;

		// Change state when lower than 1/4 threshold, use hysteresis method
		if (devsqrsum < sqrthrhldarr[detectlevel]/4)
			return NO_CAR_DETECTED;
		else if (devsqrsum >= GM_ERROR_VALUE)
			return ERROR_DETECTION;
		else
			return CAR_DETECTED_OK;
	}
	// In case car detect status modified by GTE, no benchmark
	else
	{
		xdev = CALC_ABS(tmpX-Xdtctval);
		ydev = CALC_ABS(tmpY-Ydtctval);
		zdev = CALC_ABS(tmpZ-Zdtctval);

		devsqrsum = xdev*xdev+ydev*ydev+zdev*zdev;

		// Use 1/2 threshold
		if ( devsqrsum >= sqrthrhldarr[detectlevel]/2 )
			return NO_CAR_DETECTED;
		else
			return CAR_DETECTED_OK;
	}
}

static void ProcOcpData(detectstatus_t curdtct, int16 tmpX, int16 tmpY, int16 tmpZ)
{
	// Use empty reverse flag not tmpbenchcnt, in case forming benchmark array will break the normal process.
	if (emprvsflg == FALSE)
	{
		switch(curdtct)
		{
			case NO_CAR_DETECTED:
				cardetect = curdtct;
				set_data_change();
				break;
			case ERROR_DETECTION:
				set_heart_beat();
				PrintGMvalue(COM433_DEBUG_PORT, "\r\nOCP ERR:",tmpX,tmpY,tmpZ);
				break;
			case CAR_DETECTED_OK:
				if ( tmpdtctcnt < BENCH_WEIGHT_LEN)
					WghtRgltOcpBenchmk(tmpX,tmpY,tmpZ);
				else
					PrintGMvalue(COM433_DEBUG_PORT, "\r\nDR:",tmpX,tmpY,tmpZ);
				break;
			default:
				break;
		}
	}
	else
	{
		switch(curdtct)
		{
			case NO_CAR_DETECTED:
				if ( tmpbenchcnt <= BENCH_AVG_LEN)	// form benchmark array
					NormRgltEmpBenchmk(tmpX,tmpY,tmpZ);
				else
				{
					emprvsflg = FALSE;
					cardetect = curdtct;
					set_data_change();
				}
				break;
			case CAR_DETECTED_OK:
				tmpbenchcnt = 0;	// clear benchmark array
				WghtRgltOcpBenchmk(tmpX,tmpY,tmpZ);
				break;
			default:
				break;
		}	
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
static void ModifyBenchmk(detectstatus_t newstts)
{
	if (cardetect == newstts)
		return;

	if (newstts > CAR_DETECTED_OK || cardetect > CAR_DETECTED_OK || tmpbenchcnt == 0)
		return;

	ClearDataResend();
	ClearSyncTMReq();
	
	set_data_change();
	
	// No Car--> Get Car
	if (newstts == CAR_DETECTED_OK)
	{
		// Change current empty benchmark to occupyed
		InitBenchmk(CAR_DETECTED_OK,Xbenchmk,Ybenchmk,Zbenchmk);
		// Previous benchmark invalid, when perform CheckCarOut
		emprvsflg = TRUE;
		tmpbenchcnt = 0;	
	}
	// Get Car-->No Car
	else
		InitBenchmk(BENCH_CALIBRATING,Xdtctval,Ydtctval,Zdtctval);

	bnchadjcnt = 0;
	unkwncnt = 0;
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
	bnchadjcnt = 0;
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
	tmpdtctcnt++;

	if ( tmpdtctcnt > BENCH_WEIGHT_LEN )
	{
		if (tmpdtctcnt > 2*BENCH_WEIGHT_LEN)
			tmpdtctcnt = BENCH_WEIGHT_LEN+1;
		
		tmpXdtctval[tmpdtctcnt-BENCH_WEIGHT_LEN-1] = xVal;
		tmpYdtctval[tmpdtctcnt-BENCH_WEIGHT_LEN-1] = yVal;
		tmpZdtctval[tmpdtctcnt-BENCH_WEIGHT_LEN-1] = zVal;

		Xdtctval = CalcWeight(tmpXdtctval,tmpdtctcnt,BENCH_WEIGHT_LEN);
		Ydtctval = CalcWeight(tmpYdtctval,tmpdtctcnt,BENCH_WEIGHT_LEN);
		Zdtctval = CalcWeight(tmpZdtctval,tmpdtctcnt,BENCH_WEIGHT_LEN);
	}
	else
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
