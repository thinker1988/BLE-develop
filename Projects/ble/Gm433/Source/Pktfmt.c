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

#include "RFProc.h"

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
#define GME_DEV_ID		8001
#endif	// GME_DEV_ID


#ifndef GTE_DEV_ID
#define GTE_DEV_ID		9001
#endif	//GTE_DEV_ID

// Default version 1.0
#ifndef VERSION_NUMBER
#define VERSION_NUMBER		0x0100
#endif	// VERSION_NUMBER


#define GDE_ADV_ID		7999

#define GME_ADV_ID		8999

#define GTE_ADV_ID		9999


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

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 BLECore_TaskId;

extern int16 Xbenchmk;
extern int16 Ybenchmk;
extern int16 Zbenchmk;


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

/*
// Last time struct
static UTCTimeStruct lasttm;*/
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 FillElmInfo(uint8 *buf, uint8 eid, uint8 evallen, uint8* evalbuf);
static uint8 CalcGMChksum(uint8* chkbuf, uint16 len);

static void SyncTMResp(uint8 *data, uint8 len);
static void PresetParamResp(uint8 *data, uint8 len);

static void ReadGDEParam(void);
static void ReadIDParam(uint8 * rdbuf);

static bool SetGDEParam(uint8 * setdata, uint8 len);
static bool SetIDParam(uint16 GDEaddr, uint16 GMEaddr);

/*static void SaveLastTMInfo(UTCTimeStruct tm);*/

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
void InitDevID(void)
{
	RFdevID = RFGDEID;
	RFdestID = RFGMEID;
}

/*********************************************************************
 * @fn		GMSPktForm
 *
 * @brief	form seperate GM system packet into one
 *
 * @param	rawbuf - seperate data from serial port
 * @param	rawlen - seperate data length from serial port
 *
 * @return	none
 */
void GMSPktForm(uint8 *rawbuf, uint8 rawlen)
{
	uint8 i,tmp;

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
#if ( defined GME_WORKING )
					tmp = RFDataSend(gmsrdpkt, currdlen);
#else	// !GME_WORKING
					tmp = RFDataParse(gmsrdpkt, currdlen);
#endif	// GME_WORKING

#if ( defined ALLOW_DEBUG_OUTPUT )
					Com433WriteInt(COM433_DEBUG_PORT, "\r\nRF",tmp,10);
#else	// !ALLOW_DEBUG_OUTPUT
					VOID tmp;
#endif	// ALLOW_DEBUG_OUTPUT

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
 * @fn		RFDataParse
 *
 * @brief	parse whole GM system packet.
 *
 * @param	rfdata - packet buf
 * @param	len - packet buf length
 *
 * @return	packet parse result
 */
rferr_t RFDataParse(uint8 *rfdata,uint8 len)
{
	if (rfdata[GMS_ID_POS]!=GME_SRC_ID && rfdata[GMS_ID_POS]!=GTE_SRC_ID)
		return PKT_DENY;
	
	if (rfdata[GMS_TOT_LEN_POS] != len)
		return PKT_ERR;

	if (osal_memcmp(rfdata+GMS_RESERVE_POS, GMS_RESERVE_STR, GMS_RESERVE_SIZE) == FALSE)
		return PKT_DENY;

	if (GET_RF_DEST_ADDR(rfdata) != RFdevID && GET_RF_DEST_ADDR(rfdata) != GDE_ADV_ID)
		return BAD_ADDR;

	if (CalcGMChksum(rfdata,len) != rfdata[GMS_CHK_SUM_POS])
		return CHKSUM_ERR;

	if (rfdata[GMS_ID_POS] == GME_SRC_ID)
	{
		switch(rfdata[GMS_SUB_TYPE_POS])
		{
			case GME_SUBTYPE_HRTBEAT_ACK:
				// prepare upgrade?
				break;
			case GME_SUBTYPE_CARINFO_ACK:
				// GME recieved carinfo successfully
				ClearDataResend();
#if ( defined ALLOW_DEBUG_OUTPUT )
				Com433WriteStr(COM433_DEBUG_PORT, "\r\nSEND OK!");
#endif
				break;
			case GME_SUBTYPE_TMSYN_ACK:
				// set time
				SyncTMResp(rfdata+GMS_PKT_PAYLOAD_POS,rfdata[GMS_PKT_PAYLOAD_POS+EID_SIZE]);
				break;
			case GME_SUBTYPE_UPGD_PKT:
				// upgrade process
				break;
			case GME_SUBTYPE_RST_BENCH_PKT:
				break;
			default:
				return SUB_TYPE_ERR;
		}
	}
	else if (rfdata[GMS_ID_POS] == GTE_SRC_ID)
	{
		switch(rfdata[GMS_SUB_TYPE_POS])
		{
			case GTE_SUBTYPE_PRESET_REQ:
				PresetParamResp(rfdata+GMS_PKT_PAYLOAD_POS,rfdata[GMS_PKT_PAYLOAD_POS+EID_SIZE]);
				break;
			case GTE_SUBTYPE_PARAM_READ:
				ReadGDEParam();
				break;
			case GTE_SUBTYPE_PARAM_SET:
				// set GDE params
				SetGDEParam(rfdata+GMS_PKT_PAYLOAD_POS,rfdata[GMS_PKT_PAYLOAD_POS+EID_SIZE]);
				break;
			case GTE_SUBTYPE_UPGD_REQ:
				// upgrade process
				break;
			case GTE_SUBTYPE_UPGD_PKT:
				// upgrade process
				break;
			case GTE_SUBTYPE_RST_BENCH_PKT:
				// upgrade process
				break;
			default:
				return SUB_TYPE_ERR;
		}
	}

	return RF_SUCCESS;
}

/*********************************************************************
 * @fn		RFDataForm
 *
 * @brief	form whole GM system packet.
 *
 * @param	subtype - sub type of packet, fill element ID based on this
 * @param	data - packet content
 * @param	datalen - packet content length
 *
 * @return	packet parse result
 */
rferr_t RFDataForm(uint8 subtype, uint8 *data, uint8 datalen)
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
		case GDE_SUBTYPE_HRTBEAT_REQ:
			// Heart beat packet format
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_LOC_TM, GDE_LOC_TM_LEN, NULL);
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_HRTBT, datalen, data);
			break;
		case GDE_SUBTYPE_CARINFO_REQ:
			// Car info request
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_LOC_TM, GDE_LOC_TM_LEN, NULL);
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_HRTBT, datalen, data);
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_BENCH_INFO, GDE_BENCH_INFO_LEN, NULL);
			break;
		case GDE_SUBTYPE_TMSYN_REQ:
			// Time synchronization request
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_TMSYN, datalen, data);
			break;
		case GDE_SUBTYPE_UPGD_REQ_ACK:
			// Order upgrade response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_RF_FREQ, datalen, data);
			break;
		case GDE_SUBTYPE_UPGD_ACK:
			// Response after uprade finish
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_INFO_ACK, datalen, data);
			break;
		case GDE_SUBTYPE_T_PRESET_ACK:
			// Preset response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_RF_FREQ, datalen, data);
			break;
		case GDE_SUBTYPE_T_READ_ACK:
			// Read parameters response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_PARAMS, datalen, data);
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_BENCH_INFO, GDE_BENCH_INFO_LEN, NULL);
			break;
		case GDE_SUBTYPE_T_SET_ACK:
			// Set GDE parameters response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_INFO_ACK, datalen, data);
			break;
		case GDE_SUBTYPE_T_UPGD_REQ_ACK:
			// Set GDE parameters response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_RF_FREQ, datalen, data);
			break;
		case GDE_SUBTYPE_T_UPGD_ACK:
			// Set GDE parameters response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_INFO_ACK, datalen, data);
			break;
		default:
			return SUB_TYPE_ERR;
	}
	
	rfbuf[GMS_TOT_LEN_POS] = curpldpos;
	rfbuf[GMS_CHK_SUM_POS] = CalcGMChksum(rfbuf, rfbuf[GMS_TOT_LEN_POS]);

	return RFDataSend(rfbuf,rfbuf[GMS_TOT_LEN_POS]);
}

/*********************************************************************
 * PRIVATE FUNCTIONS
 */

/*********************************************************************
 * @fn		FillElmInfo
 *
 * @brief	Fill time information into buffer.
 *
 * @param	buf - payload buffer.
 *
 * @return	time buffer length, include EID and Elen
 */
static uint8 FillElmInfo(uint8 *buf, uint8 eid, uint8 evallen, uint8* evalbuf)
{
	uint8* pVal = buf+ELM_HDR_SIZE;

	switch(eid)
	{
		case EID_GDE_LOC_TM:
		{
			if (evallen != GDE_LOC_TM_LEN)
				return 0;

			UTCTime curtm = osal_getClock();
			UTCTimeStruct curtmst;

			VOID evalbuf;
			osal_ConvertUTCTime(&curtmst, curtm);
			pVal[UTCL_HOUR_POS]=curtmst.hour;
			pVal[UTCL_MINTS_POS]=curtmst.minutes;
			pVal[UTCL_SECND_POS]=curtmst.seconds;
			pVal[UTCL_DAY_POS]=curtmst.day+1;
			pVal[UTCL_MONTH_POS]=curtmst.month+1;
			pVal[UTCL_YEAR_POS]=curtmst.year - 2000;
			break;
		}
		case EID_GDE_HRTBT:
		{
			if (evallen != GDE_HRTBT_LEN)
				return 0;
			
			osal_memcpy(pVal, evalbuf, evallen);
			break;
		}
		case EID_GDE_BENCH_INFO:
		{
			if (evallen != GDE_BENCH_INFO_LEN)
				return 0;

			VOID evalbuf;
			pVal[GDE_X_H_BCHMRK_POS] = HI_UINT16(Xbenchmk);
			pVal[GDE_X_L_BCHMRK_POS] = LO_UINT16(Xbenchmk);
			pVal[GDE_Y_H_BCHMRK_POS] = HI_UINT16(Ybenchmk);
			pVal[GDE_Y_L_BCHMRK_POS] = LO_UINT16(Ybenchmk);
			pVal[GDE_Z_H_BCHMRK_POS] = HI_UINT16(Zbenchmk);
			pVal[GDE_Z_L_BCHMRK_POS] = LO_UINT16(Zbenchmk);
			break;
		}
		case EID_GMS_INFO_ACK:
		{
			if (evallen != GMS_INFO_ACK_LEN)
				return 0;

			osal_memcpy(pVal, evalbuf, evallen);
			break;
		}
		case EID_GDE_PARAMS:
		{
			if (evallen != GDE_BENCH_INFO_LEN)
				return 0;

			VOID evalbuf;
			pVal[GDE_X_H_BCHMRK_POS] = HI_UINT16(Xbenchmk);
			pVal[GDE_X_L_BCHMRK_POS] = LO_UINT16(Xbenchmk);
			pVal[GDE_Y_H_BCHMRK_POS] = HI_UINT16(Ybenchmk);
			pVal[GDE_Y_L_BCHMRK_POS] = LO_UINT16(Ybenchmk);
			pVal[GDE_Z_H_BCHMRK_POS] = HI_UINT16(Zbenchmk);
			pVal[GDE_Z_L_BCHMRK_POS] = LO_UINT16(Zbenchmk);
			break;
		}
		case EID_GMS_RF_FREQ:
		{
			if (evallen != GMS_RF_FREQ_LEN)
				return 0;

			osal_memcpy(pVal, evalbuf, evallen);
			break;
		}
		case EID_GDE_TMSYN:
		{
			if (evallen != GDE_TMSYN_LEN)
				return 0;

			osal_memcpy(pVal, evalbuf, evallen);
			break;
		}
		case EID_GME_NT_TM:
		case EID_GMS_FW_INFO:
		case EID_GTE_READ:
		case EID_GMS_UPGD:
		default:
			return 0;
	}
	buf[0] = eid;
	buf[EID_SIZE] = evallen;
	
	return evallen+ELM_HDR_SIZE;
}

/*********************************************************************
 * @fn		CalcGMChksum
 *
 * @brief	Calculate GM packet check sum, remain check sum position.
 *
 * @param	chkbuf - check buffer.
 * @param	len - check buffer length.
 *
 * @return	check sum value
 */
static uint8 CalcGMChksum(uint8* chkbuf, uint16 len)
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
 * @fn		SyncTMResp
 *
 * @brief	Synchronize time based on element data
 *
 * @param	data - sync time element data buf
 * @param	len - sync time element data length
 *
 * @return	none
 */
static void SyncTMResp(uint8 *data, uint8 len)
{
	uint8 *pVal = data+ELM_HDR_SIZE;

	if (data[0]!=EID_GME_NT_TM || len!=GME_NT_TM_LEN)
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

#if ( !defined GM_TEST_COMM )
	ClearSyncTMReq();
#else	// GM_TEST_COMM
	Com433WriteInt(COM433_DEBUG_PORT," ",settmst.hour,10);
	Com433WriteInt(COM433_DEBUG_PORT,":",settmst.minutes,10);
	Com433WriteInt(COM433_DEBUG_PORT,":",settmst.seconds,10);
#endif	// !GM_TEST_COMM


}

/*********************************************************************
 * @fn		PresetParamResp
 *
 * @brief	Synchronize time based on element data
 *
 * @param	data - pre-setup params element data buf
 * @param	len - pre-setup params element data length
 *
 * @return	none
 */
static void PresetParamResp(uint8 *data, uint8 len)
{
	uint8 stfreq;

	// Ignore this command when already in setup mode
	if ( GetSysState() == SYS_SETUP)
		return;

	if ( data[0] != EID_GMS_RF_FREQ || len != GMS_RF_FREQ_LEN )
		return;

	// Change dest ID to GTE ID
	RFdestID = RFGTEID;
	
	stfreq = data[ELM_HDR_SIZE];
	// Use device own setup frequency if set value is 0 else change setup frequency
	RFstfrq = (stfreq==0? RFstfrq: stfreq);

	RFDataForm(GDE_SUBTYPE_T_PRESET_ACK, &RFstfrq, sizeof(RFstfrq));
	//RF_working(BLECore_TaskId, GetRFstate());

	// State will change after working state over
	SetSysState(SYS_SETUP);
}


/*********************************************************************
 * @fn		ReadGDEParam
 *
 * @brief	read GDE parameters.
 *
 * @param	none
 *
 * @return	none
 */
static void ReadGDEParam(void)
{
	uint8 rddata[GDE_PARAMS_LEN];

	if ( GetSysState() != SYS_SETUP)
		return;

	osal_start_timerEx(BLECore_TaskId, BLE_SYS_WORKING_EVT, NO_OPERATION_WAIT_PERIOD);

	ReadRFParam(rddata);
	ReadGMParam(rddata);
	ReadIDParam(rddata);

	RFDataForm(GDE_SUBTYPE_T_READ_ACK, rddata, sizeof(rddata));
	//RF_working(BLECore_TaskId, GetRFstate());
}

/*********************************************************************
 * @fn		ReadIDParam
 *
 * @brief	Read GDE ID parameters.
 *
 * @param	rdbuf -params save buffer
 *
 * @return	none
 */
static void ReadIDParam(uint8 *rdbuf)
{
	rdbuf[ST_GDE_ADDR_H_POS] = HI_UINT16(RFGDEID);
	rdbuf[ST_GDE_ADDR_L_POS] = LO_UINT16(RFGDEID);

	rdbuf[ST_GME_ADDR_H_POS] = HI_UINT16(RFGMEID);
	rdbuf[ST_GME_ADDR_L_POS] = LO_UINT16(RFGMEID);
}

/*********************************************************************
 * @fn		SetGDEParam
 *
 * @brief	Set GDE parameters.
 *
 * @param	setdata - setup buf
 * @param	len - setup buf length
 *
 * @return	setup result
 */
static bool SetGDEParam(uint8 *setdata, uint8 len)
{
	uint8 *pVal = setdata+ELM_HDR_SIZE;
	bool flag = FALSE;
	
	if (setdata[0]!=EID_GDE_PARAMS || len!=GDE_PARAMS_LEN || GetSysState() != SYS_SETUP)
		return flag;

	osal_start_timerEx(BLECore_TaskId, BLE_SYS_WORKING_EVT, NO_OPERATION_WAIT_PERIOD);

	flag = SetRFParam(pVal[ST_RF_WK_FREQ_POS],pVal[ST_RF_ST_FREQ_POS],pVal[ST_RF_UPGD_FREQ_POS],\
			pVal[ST_RF_AIR_BAUD_POS],pVal[ST_RF_PWR_LVL_POS]);
	flag &= SetGMParam(pVal[ST_GM_HB_FREQ_POS],pVal[ST_GM_DTCT_SENS_POS],pVal[ST_GM_BENCH_ALG_POS],\
			pVal[ST_GM_STATUS_POS]);
	flag &= SetIDParam(BUILD_UINT16(pVal[ST_GDE_ADDR_L_POS],pVal[ST_GDE_ADDR_H_POS]),\
			BUILD_UINT16(pVal[ST_GME_ADDR_L_POS],pVal[ST_GME_ADDR_H_POS]));

	RFDataForm(GDE_SUBTYPE_T_SET_ACK, &flag, sizeof(flag));
	//RF_working(BLECore_TaskId, GetRFstate());

	return flag;
}

static bool SetIDParam(uint16 GDEaddr, uint16 GMEaddr)
{
	RFGDEID = ((GDEaddr>=GDE_ADV_ID || GDEaddr==0)? RFGDEID: GDEaddr);
	RFGMEID = ((GMEaddr<= GDE_ADV_ID || GMEaddr>=GME_ADV_ID)? RFGMEID: GMEaddr);

	return TRUE;
}
