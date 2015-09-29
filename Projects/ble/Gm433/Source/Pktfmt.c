/*********************************************************************
 * INCLUDES
 */
#include "comdef.h"
#include "OnBoard.h"
#include "OSAL.h"
#include "OSAL_Clock.h"

#include "Com433.h"
#include "Pktfmt.h"
#include "GMProc.h"
#include "BLECore.h"

#include "Cc112x.h"

/*********************************************************************
 * MACROS
 */

#define GET_RF_SRC_ADDR(rfbuf)	\
	(BUILD_UINT16(rfbuf[GMS_SRC_ADDR_POS+1],rfbuf[GMS_SRC_ADDR_POS]))

#define GET_RF_DEST_ADDR(rfbuf)	\
	(BUILD_UINT16(rfbuf[GMS_DEST_ADDR_POS+1],rfbuf[GMS_DEST_ADDR_POS]))

/*********************************************************************
 * CONSTANTS
 */

#ifndef GDE_DEV_ID
#define GDE_DEV_ID		1
#endif	// GDE_DEV_ID

#ifndef GME_DEV_ID
#define GME_DEV_ID		1001
#endif	// GME_DEV_ID


#ifndef GTE_DEV_ID
#define GTE_DEV_ID		2001
#endif	//GTE_DEV_ID

// Default version 1.0
#ifndef VERSION_NUMBER
#define VERSION_NUMBER		0x0100
#endif	// VERSION_NUMBER


#define GDE_ADV_ID		999

#define GME_ADV_ID		1999

#define GTE_ADV_ID		2999

// Stop time synchronise after 5 times
#define TIME_SYNC_AUTO_STOP			5
/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
	PKT_GMS_ID,
	PKT_GMS_LEN,
	PKT_GMS_DATA
}pktgms_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint16 RFGDEID = GDE_DEV_ID;
uint16 RFGMEID = GME_DEV_ID;
uint16 RFGTEID = GTE_DEV_ID;

uint16 version = VERSION_NUMBER;

uint16 RFdevID;
uint16 RFdestID;

#if ( ! defined USE_CC112X_RF )
uint8 rfsndbuf[GMS_PKT_MAX_LEN] = {0};
uint8 rfsndlen;
#endif
/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 BLECore_TaskId;

// Device set frequency
extern uint8 RFstfrq;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static pktgms_t pktgmsst=PKT_GMS_ID;

static uint8 gmsrdpkt[GMS_PKT_MAX_LEN]={0};
static uint8 currdlen=0;
static uint8 totrdlen=0;

static uint8 syncautostop;

// Last time struct
static UTCTimeStruct lasttm;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 filltime(uint8 *buf);
static uint8 calcGMchksum(uint8* chkbuf, uint16 len);

static void syncUTCtimeresp(uint8 *data, uint8 len);
static void presetParamresp(uint8 *data, uint8 len);

static void readGDEparam(void);
static void readIDparam(uint8 * rdbuf);

static bool setGDEparam(uint8 * setdata, uint8 len);
static bool setIDparam(uint16 GDEaddr, uint16 GMEaddr, uint16 vern);


static rferr_t rfdatasend(uint8 *buf, uint8 len);

static void savelasttminfo(UTCTimeStruct tm);
static void chktmsync(UTCTimeStruct tm);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn		initDevID
 *
 * @brief	Initiate device ID and destination ID, avoid destination ID changed by GTE working.
 *
 * @param	none
 *
 * @return	none
 */
void initDevID(void)
{
	RFdevID = RFGDEID;
	RFdestID = RFGMEID;
}

void syncUTCtimereq(void)
{
	uint8 tmsync=TRUE;

	if (++syncautostop == TIME_SYNC_AUTO_STOP)
	{
		syncautostop = 0;
		cleartmsync();
	}
	else
		rfdataform(GDE_ST_TMSYN_REQ,&tmsync,sizeof(tmsync));
}

/*********************************************************************
 * @fn		gmspktform
 *
 * @brief	form seperate GM system packet into one
 *
 * @param	rawbuf - seperate data from serial port
 * @param	rawlen - seperate data length from serial port
 *
 * @return	none
 */
void gmspktform(uint8 *rawbuf, uint8 rawlen)
{
	uint8 i;

	for (i=0; i<rawlen; i++)
	{
		switch(pktgmsst)
		{
			case PKT_GMS_ID:
				if (rawbuf[i]==GME_SRC_ID || rawbuf[i]==GTE_SRC_ID)
				{
					pktgmsst=PKT_GMS_LEN;
					gmsrdpkt[currdlen++] = rawbuf[i];
				}
				break;
			case PKT_GMS_LEN:
				totrdlen= rawbuf[i];
				if (totrdlen >= GMS_PKT_MAX_LEN)
				{
					pktgmsst = PKT_GMS_ID;
					currdlen = 0;
				}
				else
				{
					gmsrdpkt[currdlen++] =totrdlen;
					pktgmsst=PKT_GMS_DATA;
				}
				break;
			case PKT_GMS_DATA:
				gmsrdpkt[currdlen++]= rawbuf[i];
				if (currdlen == totrdlen)
				{
					Com433WriteInt(COM433_DEBUG_PORT, "\r\nRF",rfdataparse(gmsrdpkt, currdlen),10);
					pktgmsst=PKT_GMS_ID;
					currdlen = 0;
				}
				break;
			default:
				break;
		}
	}
}


/*********************************************************************
 * @fn		rfdataparse
 *
 * @brief	parse whole GM system packet.
 *
 * @param	rfdata - packet buf
 * @param	len - packet buf length
 *
 * @return	packet parse result
 */
rferr_t rfdataparse(uint8 *rfdata,uint8 len)
{
	if (rfdata[GMS_ID_POS]!=GME_SRC_ID && rfdata[GMS_ID_POS]!=GTE_SRC_ID)
		return PKT_DENY;
	
	if (rfdata[GMS_TOT_LEN_POS] != len)
		return PKT_ERR;

	if (osal_memcmp(rfdata+GMS_RESERVE_POS, GMS_RESERVE_STR, GMS_RESERVE_SIZE) == FALSE)
		return PKT_DENY;
	
	if (GET_RF_DEST_ADDR(rfdata) != RFdevID && GET_RF_DEST_ADDR(rfdata) != GDE_ADV_ID)
		return BAD_ADDR;
		
	if (calcGMchksum(rfdata,len) != rfdata[GMS_CHK_SUM_POS])
		return CHKSUM_ERR;

#if (defined GDE_RELEASE)

	if (rfdata[GMS_ID_POS] == GME_SRC_ID)
	{
		switch(rfdata[GMS_SUB_TYPE_POS])
		{
			case GME_ST_HRTBEAT_ACK:
				// prepare upgrade?
				break;
			case GME_ST_CARINFO_ACK:
				// GME recieved carinfo successfully
				stopresend(rfdata+GMS_PKT_PAYLOAD_POS,rfdata[GMS_PKT_PAYLOAD_POS+EID_SIZE]);
				break;
			case GME_ST_TMSYN_ACK:
				// set time
				syncUTCtimeresp(rfdata+GMS_PKT_PAYLOAD_POS,rfdata[GMS_PKT_PAYLOAD_POS+EID_SIZE]);
				break;
			case GME_ST_UPGD_REQ:
				// upgrade process
				break;
			default:
				return SUB_TYPE_ERR;
		}
	}
	else if (rfdata[GMS_ID_POS] == GTE_SRC_ID)
	{
		switch(rfdata[GMS_SUB_TYPE_POS])
		{
			case GTE_ST_PARAM_REQ:
				presetParamresp(rfdata+GMS_PKT_PAYLOAD_POS,rfdata[GMS_PKT_PAYLOAD_POS+EID_SIZE]);
				break;
			case GTE_ST_PARAM_READ:
				readGDEparam();
				break;
			case GTE_ST_PARAM_SET:
				// set GDE params
				setGDEparam(rfdata+GMS_PKT_PAYLOAD_POS,rfdata[GMS_PKT_PAYLOAD_POS+EID_SIZE]);
				break;
			case GTE_ST_UPGD_REQ:
				// upgrade process
				break;
			default:
				return SUB_TYPE_ERR;
		}
	}
#elif ( defined GTE_RELEASE )


#endif	// GDE_RELEASE
	return RF_SUCCESS;
}

/*********************************************************************
 * @fn		rfdataform
 *
 * @brief	form whole GM system packet.
 *
 * @param	subtype - sub type of packet, fill element ID based on this
 * @param	data - packet content
 * @param	datalen - packet content length
 *
 * @return	packet parse result
 */
rferr_t rfdataform(uint8 subtype, uint8 *data, uint8 datalen)
{
	uint8 rfbuf[GMS_PKT_MAX_LEN]={0};
	uint8 curpldpos = GMS_PKT_PAYLOAD_POS;

	rfbuf[GMS_ID_POS]=GDE_SRC_ID;
	
	rfbuf[GMS_SUB_TYPE_POS]=subtype;

	osal_memcpy(rfbuf+GMS_RESERVE_POS, GMS_RESERVE_STR, GMS_RESERVE_SIZE);

	rfbuf[GMS_SRC_ADDR_POS] = HI_UINT16(RFdevID);
	rfbuf[GMS_SRC_ADDR_POS+1] = LO_UINT16(RFdevID);

	rfbuf[GMS_DEST_ADDR_POS] = HI_UINT16(RFdestID);	// need mutex?
	rfbuf[GMS_DEST_ADDR_POS+1] = LO_UINT16(RFdestID);

	rfbuf[GMS_VERSION_POS] = HI_UINT16(version);	// how to fill fist update
	rfbuf[GMS_VERSION_POS+1] = LO_UINT16(version);

	switch(subtype)
	{
		case GDE_ST_HRTBEAT_REQ:
			// heart beat packet format
			if (datalen != GDE_HRTBT_LEN)
				return DATA_ERR;
			
			curpldpos += filltime(rfbuf+GMS_PKT_PAYLOAD_POS);
			rfbuf[curpldpos] = EID_GDE_HRTBT;
			break;
		case GDE_ST_CARINFO_REQ:
			// car info request
			if (datalen != GDE_CAR_INFO_LEN)
				return DATA_ERR;
			curpldpos = GMS_PKT_PAYLOAD_POS + filltime(rfbuf+GMS_PKT_PAYLOAD_POS);
			rfbuf[curpldpos] = EID_GDE_CAR_INFO;
			break;
		case GDE_ST_TMSYN_REQ:
			// time sync
			if (datalen != GDE_TMSYN_LEN)
				return DATA_ERR;
			rfbuf[curpldpos] = EID_GDE_TMSYN;
			break;
		case GDE_ST_UPGD_REQ_ACK:
		case GDE_ST_UPGD_ACK:
			// upgrade ack
			break;
		case GDE_ST_T_REQ_ACK:
			// setup mode request ack
			if (datalen != GDE_PREREQ_LEN)
				return DATA_ERR;
			rfbuf[curpldpos] = EID_GDE_SET_RESP;
			break;
		case GDE_ST_T_READ_ACK:
			// read GDE cmd ack
			if (datalen != GDE_READ_ACK_LEN)
				return DATA_ERR;
			rfbuf[curpldpos] = EID_GDE_READ_ACK;
			break;
		case GDE_ST_T_SET_ACK:
			// set GDE cmd ack
			if (datalen != GDE_SET_ACK_LEN)
				return DATA_ERR;
			rfbuf[curpldpos] = EID_GDE_SET_ACK;
			break;
		default:
			return SUB_TYPE_ERR;
	}
	rfbuf[curpldpos+EID_SIZE] = datalen;
	osal_memcpy(rfbuf+curpldpos+ELM_HDR_SIZE, data, datalen);
	
	rfbuf[GMS_TOT_LEN_POS]=curpldpos+ELM_HDR_SIZE+datalen;

	rfbuf[GMS_CHK_SUM_POS]=calcGMchksum(rfbuf, rfbuf[GMS_TOT_LEN_POS]);

	return rfdatasend(rfbuf,rfbuf[GMS_TOT_LEN_POS]);
}


/*********************************************************************
 * PRIVATE FUNCTIONS
 */

/*********************************************************************
 * @fn		filltime
 *
 * @brief	Fill time information into buffer.
 *
 * @param	buf - time buffer.
 *
 * @return	time buffer length, include EID and Elen
 */
static uint8 filltime(uint8 *buf)
{
	UTCTime curtm = osal_getClock();
	UTCTimeStruct curtmst;

	buf[0] = EID_UTCL;
	buf[EID_SIZE] = UTCL_EVAL_LEN;
	osal_ConvertUTCTime(&curtmst, curtm);

	buf[ELM_HDR_SIZE]=curtmst.hour;
	buf[ELM_HDR_SIZE+1]=curtmst.minutes;
	buf[ELM_HDR_SIZE+2]=curtmst.seconds;
	buf[ELM_HDR_SIZE+3]=curtmst.day+1;	
	buf[ELM_HDR_SIZE+4]=curtmst.month+1;
	buf[ELM_HDR_SIZE+5]=curtmst.year - 2000;

	// Check time sync status
	chktmsync(curtmst);

	return (ELM_HDR_SIZE + UTCL_EVAL_LEN);
}

/*********************************************************************
 * @fn		calcGMchksum
 *
 * @brief	Calculate GM packet check sum, remain check sum position.
 *
 * @param	chkbuf - check buffer.
 * @param	len - check buffer length.
 *
 * @return	check sum value
 */
static uint8 calcGMchksum(uint8* chkbuf, uint16 len)
{
	uint8 i,sum=0;

	for(i=0;i<len;i++)
	{
		if (i == GMS_CHK_SUM_POS)
			continue;
		sum ^= chkbuf[i];
	}
	return sum;
}

/*********************************************************************
 * @fn		syncUTCtimeresp
 *
 * @brief	Synchronize time based on element data
 *
 * @param	data - sync time element data buf
 * @param	len - sync time element data length
 *
 * @return	none
 */
static void syncUTCtimeresp(uint8 *data, uint8 len)
{
	uint8 *pVal = data+ELM_HDR_SIZE;

	if (data[0]!=EID_GME_TMSYN_ACK || len!=GME_TMSYN_ACK_LEN)
		return;

	// time overflow
	if (data[ELM_HDR_SIZE+UTCL_YEAR_POS] > 136)
		return;

	UTCTimeStruct settmst;
	
	settmst.hour = pVal[UTCL_HOUR_POS];
	settmst.minutes = pVal[UTCL_MINTS_POS];
	settmst.seconds = pVal[UTCL_SECND_POS];
	settmst.day = pVal[UTCL_DAY_POS]-1;
	settmst.month = pVal[UTCL_MONTH_POS]-1;
	settmst.year = 2000+pVal[UTCL_YEAR_POS];
	
	osal_setClock(osal_ConvertUTCSecs(&settmst));

	savelasttminfo(settmst);

	cleartmsync();
}

/*********************************************************************
 * @fn		presetParamresp
 *
 * @brief	Synchronize time based on element data
 *
 * @param	data - pre-setup params element data buf
 * @param	len - pre-setup params element data length
 *
 * @return	none
 */
static void presetParamresp(uint8 *data, uint8 len)
{
	uint8 stfreq;

	if ( getsysstate() == SYS_SETUP)
		return;

	if ( data[0] != EID_GTE_SET_REQ || len != GDE_PREREQ_LEN )
		return;

	// Change dest ID to GTE ID
	RFdestID = RFGTEID;
	
	stfreq = data[ELM_HDR_SIZE];
	// Use device own setup frequency if set value is 0 else change setup frequency
	RFstfrq = (stfreq==0? RFstfrq: stfreq);

	rfdataform(GDE_ST_T_REQ_ACK, &RFstfrq, sizeof(RFstfrq));
	// change system state after data send
	setsysstate(SYS_WAITING);
}


/*********************************************************************
 * @fn		readGDEparam
 *
 * @brief	read GDE parameters.
 *
 * @param	none
 *
 * @return	none
 */
static void readGDEparam(void)
{
	if ( getsysstate() != SYS_SETUP)
		return;
	
	uint8 rddata[GDE_READ_ACK_LEN];

	readRFparam(rddata);
	readGMparam(rddata);
	readIDparam(rddata);

	rfdataform(GDE_ST_T_READ_ACK, rddata, sizeof(rddata));
}

/*********************************************************************
 * @fn		readIDparam
 *
 * @brief	Read GDE ID parameters.
 *
 * @param	rdbuf -params save buffer
 *
 * @return	none
 */
static void readIDparam(uint8 *rdbuf)
{
	rdbuf[ST_GDE_ADDR_H_POS] = HI_UINT16(RFGDEID);
	rdbuf[ST_GDE_ADDR_L_POS] = LO_UINT16(RFGDEID);

	rdbuf[ST_GME_ADDR_H_POS] = HI_UINT16(RFGMEID);
	rdbuf[ST_GME_ADDR_L_POS] = LO_UINT16(RFGMEID);

	rdbuf[ST_VERSION_H_POS] = HI_UINT16(version);
	rdbuf[ST_VERSION_L_POS] = LO_UINT16(version);
}

/*********************************************************************
 * @fn		setGDEparam
 *
 * @brief	Set GDE parameters.
 *
 * @param	setdata - setup buf
 * @param	len - setup buf length
 *
 * @return	setup result
 */
static bool setGDEparam(uint8 *setdata, uint8 len)
{
	uint8 *pVal = setdata+ELM_HDR_SIZE;
	bool flag = FALSE;
	
	if (setdata[0]!=EID_GTE_SET || len!=GTE_SET_LEN || getsysstate() != SYS_SETUP)
		return flag;

	flag = setRFparam(pVal[ST_RF_FREQ_POS],pVal[ST_RF_ST_FREQ_POS],pVal[ST_RF_UPGD_FREQ_POS],\
			pVal[ST_RF_BAUD_POS],pVal[ST_RF_PWR_LVL_POS]);
	flag &= setGMparam(pVal[ST_GM_HB_FREQ_POS],pVal[ST_GM_DTCT_VAL_POS],pVal[ST_GM_DTCT_ALG_POS],\
			pVal[ST_GM_STATUS_POS]);
	flag &= setIDparam(BUILD_UINT16(pVal[ST_GDE_ADDR_L_POS],pVal[ST_GDE_ADDR_H_POS]),\
			BUILD_UINT16(pVal[ST_GME_ADDR_L_POS],pVal[ST_GME_ADDR_H_POS]),\
			BUILD_UINT16(pVal[ST_VERSION_L_POS],pVal[ST_VERSION_H_POS]));

	rfdataform(GDE_ST_T_SET_ACK, &flag, sizeof(flag));

	return flag;
}

static bool setIDparam(uint16 GDEaddr, uint16 GMEaddr, uint16 vern)
{
	if (GDEaddr>GDE_ADV_ID || GMEaddr>GME_ADV_ID)
		return FALSE;

	RFGDEID = GDEaddr;
	RFGMEID = GMEaddr;
	version = vern;

	return TRUE;
}


static rferr_t rfdatasend(uint8 *buf, uint8 len)
{
	RFwakeup();

#if ( defined USE_CC112X_RF )
	txdata(buf,len);
#else
	//Copy send data to buffer and wait 250ms for TEN308 RF wake up
	osal_memcpy(rfsndbuf,buf,len);
	rfsndlen = len;
	osal_start_timerEx(BLECore_TaskId,RF_RXTX_RDY_EVT,250);
#endif	// USE_CC112X_RF

	return RF_SUCCESS;
}


static void savelasttminfo(UTCTimeStruct tm)
{
	lasttm.seconds = tm.seconds;
	lasttm.minutes = tm.minutes;
	lasttm.hour = tm.hour;
	lasttm.day = tm.day;
	lasttm.month = tm.month;
	lasttm.year = tm.year;
}

static void chktmsync(UTCTimeStruct tm)
{
	if (gettmsync() == TRUE)
		return;
	if (tm.day - lasttm.day > 1)
		settmsync();
}
