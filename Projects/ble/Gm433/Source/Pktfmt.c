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
#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
#include "CoreUpgrade.h"
#endif	// GM_IMAGE_A || GM_IMAGE_B

/*********************************************************************
 * MACROS
 */


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


/*********************************************************************
 * TYPEDEFS
 */

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


// Device setup frequency and upgrade frequency
extern uint8 RFstfrq;
extern uint8 RFupgfrq;

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

static rfpkterr_t ParseElmInfo(uint8 subtype,uint8 *pldbuf,  uint8 pldlen);
static uint8 FillElmInfo(uint8 *buf, uint8 eid, uint8 evallen, uint8* evalbuf);

static void ProcAckMsg(uint8 subtype,uint8 *ackbuf,  uint8 acklen);
static void SetRfFreq(uint8 subtype,uint8 *rffrqbuf,  uint8 rffrqlen);


static uint8 CalcGMChksum(uint8* chkbuf, uint16 len);

static void SyncTMResp(uint8 *data, uint8 len);

static void ReadGDEParam(uint8* readreq, uint8 len);
static void ReadIDParam(uint8 * rdbuf);

static bool SetGDEParam(uint8 * setdata, uint8 len);

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
void InitCommDevID(void)
{
	RFdevID = RFGDEID;
	RFdestID = RFGMEID;
}

bool SetIDParam(uint16 GDEaddr, uint16 GMEaddr)
{
	RFGDEID = ((GDEaddr>=GDE_ADV_ID || GDEaddr==0)? RFGDEID: GDEaddr);
	RFGMEID = ((GMEaddr<= GDE_ADV_ID || GMEaddr>=GME_ADV_ID)? RFGMEID: GMEaddr);

	return TRUE;
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
rfpkterr_t RFDataParse(uint8 *rfdata,uint8 len)
{
	uint8 pldlen = len-GMS_PKT_HDR_SIZE;

	if (rfdata[GMS_ID_POS]!=GME_SRC_ID && rfdata[GMS_ID_POS]!=GTE_SRC_ID)
		return RF_NOT_GMS;
	
	if (rfdata[GMS_TOT_LEN_POS] != len)
		return RF_PKTLEN_ERR;

	if (osal_memcmp(rfdata+GMS_RESERVE_POS, GMS_RESERVE_STR, GMS_RESERVE_SIZE) == FALSE)
		return RF_NOT_GMS;

	if (GET_RF_DEST_ADDR(rfdata) != RFdevID && GET_RF_DEST_ADDR(rfdata) != GDE_ADV_ID)
		return RF_ADDR_DENY;

	if (CalcGMChksum(rfdata,len) != rfdata[GMS_CHK_SUM_POS])
		return RF_CHKSUM_ERR;

	if (rfdata[GMS_ID_POS] == GME_SRC_ID)
	{
		switch(rfdata[GMS_SUB_TYPE_POS])
		{
			case GME_SUBTYPE_ORDER_UPGD_REQ:
				if (pldlen==GME_SUBTYPE_ORDER_UPGD_REQ_PL_LEN)
					break;
			case GME_SUBTYPE_CARINFO_RESP:
				if (pldlen==GME_SUBTYPE_CARINFO_RESP_PL_LEN)
					break;
			case GME_SUBTYPE_TMSYN_RESP:
				if (pldlen==GME_SUBTYPE_TMSYN_RESP_PL_LEN)
					break;
			case GME_SUBTYPE_UPGD_PKT:
				if (pldlen==GME_SUBTYPE_UPGD_PKT_PL_LEN && GetSysState()== SYS_UPGRADE)
					break;
			case GME_SUBTYPE_RST_BENCH_PKT:
				if (pldlen==GME_SUBTYPE_RST_BENCH_PKT_PL_LEN)
					break;

				return RF_PLD_ERR;
			default:
				return RF_SUBTYPE_UNK;
		}
	}
	else if (rfdata[GMS_ID_POS] == GTE_SRC_ID)
	{
		switch(rfdata[GMS_SUB_TYPE_POS])
		{
			case GTE_SUBTYPE_PRESET_REQ:
				if (len==GMS_PKT_HDR_SIZE+GTE_SUBTYPE_PRESET_REQ_PL_LEN && GetSysState()!= SYS_SETUP \
						&& GetSysState()!= SYS_UPGRADE)	// Ignore this command when already in setup or upgrade mode
				{
					RFGTEID = GET_RF_SRC_ADDR(rfdata);
					break;
				}
			case GTE_SUBTYPE_PARAM_READ_REQ:
				if (pldlen==GTE_SUBTYPE_PARAM_READ_REQ_PL_LEN && GetSysState()== SYS_SETUP)
					break;
			case GTE_SUBTYPE_PARAM_SET_REQ:
				if (pldlen==GTE_SUBTYPE_PARAM_SET_REQ_PL_LEN && GetSysState()== SYS_SETUP)
					break;
			case GTE_SUBTYPE_ORDER_UPGD_REQ:
				if (pldlen==GTE_SUBTYPE_ORDER_UPGD_REQ_PL_LEN && GetSysState()== SYS_SETUP)
					break;
				// Prepare upgrade
			case GTE_SUBTYPE_UPGD_PKT:
				if (pldlen==GTE_SUBTYPE_UPGD_PKT_PL_LEN && GetSysState()== SYS_UPGRADE)
					break;
				// Upgrade process
			case GTE_SUBTYPE_RST_BENCH_PKT:
				if (pldlen==GTE_SUBTYPE_RST_BENCH_PKT_PL_LEN && GetSysState()== SYS_SETUP)
					break;
				
				return RF_PLD_ERR;
			default:
				return RF_SUBTYPE_UNK;
		}
	}
	// Delay auto change to normal working
	if (GetSysState() == SYS_SETUP)
		osal_start_timerEx(BLECore_TaskId, BLE_SYS_WORKING_EVT, GTE_NO_OPR_WAIT_PERIOD);
	else if (GetSysState() == SYS_UPGRADE)
		osal_start_timerEx(BLECore_TaskId, BLE_SYS_WORKING_EVT, UPGD_RF_WAIT_PERIOD);

	return ParseElmInfo(rfdata[GMS_SUB_TYPE_POS], rfdata+GMS_PKT_HDR_SIZE, pldlen);
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
rfpkterr_t RFDataForm(uint8 subtype, uint8 *data, uint8 datalen)
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
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_LOC_TM, EVLEN_GDE_LOC_TM, NULL);
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_HRTBT, datalen, data);
			break;
		case GDE_SUBTYPE_CARINFO_REQ:
			// Car info request
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_LOC_TM, EVLEN_GDE_LOC_TM, NULL);
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_HRTBT, datalen, data);
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_BENCH_INFO, EVLEN_GDE_BENCH_INFO, NULL);
			break;
		case GDE_SUBTYPE_TMSYN_REQ:
			// Time synchronization request
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_TMSYN, datalen, data);
			break;
		case GDE_SUBTYPE_ORDER_RESP:
		case GDE_SUBTYPE_T_ORDER_RESP:
			// Order upgrade response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_INFO_ACK, datalen, data);
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_RF_FREQ, sizeof(RFupgfrq), &RFupgfrq);
			break;

		case GDE_SUBTYPE_T_PRESET_RESP:
			// Preset response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_RF_FREQ, datalen, data);
			break;
		case GDE_SUBTYPE_T_READ_RESP:
			// Read parameters response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_PARAMS, datalen, data);
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GDE_BENCH_INFO, EVLEN_GDE_BENCH_INFO, NULL);
			break;
		case GDE_SUBTYPE_T_SET_RESP:
			// Set GDE parameters response
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_INFO_ACK, datalen, data);
			break;
		case GDE_SUBTYPE_UPGD_ACK:
		case GDE_SUBTYPE_T_UPGD_ACK:
			// Response after uprade finish
			curpldpos += FillElmInfo(rfbuf+curpldpos, EID_GMS_INFO_ACK, datalen, data);
			break;
		default:
			return RF_SUBTYPE_UNK;
	}
	
	rfbuf[GMS_TOT_LEN_POS] = curpldpos;
	rfbuf[GMS_CHK_SUM_POS] = CalcGMChksum(rfbuf, rfbuf[GMS_TOT_LEN_POS]);

	return RFDataSend(rfbuf,rfbuf[GMS_TOT_LEN_POS]);
}

/*********************************************************************
 * PRIVATE FUNCTIONS
 */

/*********************************************************************
 * @fn		ParseElmInfo
 *
 * @brief	Parse time information into buffer.
 *
 * @param	pldbuf - payload buffer.
 *
 * @return	time buffer length, include EID and Elen
 */
static rfpkterr_t ParseElmInfo(uint8 subtype,uint8 *pldbuf,  uint8 pldlen)
{
	uint8 elmpos = 0;

	while(elmpos<pldlen)
	{
		switch(pldbuf[elmpos])
		{
			case EID_GDE_LOC_TM:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_LOC_TM)
				{
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_LOC_TM;
					break;
				}
			case EID_GDE_HRTBT:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_HRTBT)
				{
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_HRTBT;
					break;
				}
			case EID_GDE_BENCH_INFO:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_BENCH_INFO)
				{
					// Reset benchmark
					ResetBenchmark(pldbuf+elmpos+ELM_HDR_SIZE,EVLEN_GDE_BENCH_INFO);
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_BENCH_INFO;
					break;
				}
			case EID_GMS_INFO_ACK:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GMS_INFO_ACK)
				{
					ProcAckMsg(subtype,pldbuf+elmpos+ELM_HDR_SIZE,EVLEN_GMS_INFO_ACK);
					elmpos += ELM_HDR_SIZE+EVLEN_GMS_INFO_ACK;
					break;
				}
			case EID_GDE_PARAMS:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_PARAMS)
				{
					// Set GDE params
					SetGDEParam(pldbuf+elmpos+ELM_HDR_SIZE,EVLEN_GDE_PARAMS);
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_PARAMS;
					break;
				}
			case EID_GMS_RF_FREQ:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GMS_RF_FREQ)
				{
					SetRfFreq(subtype,pldbuf+elmpos+ELM_HDR_SIZE,EVLEN_GMS_RF_FREQ);
					elmpos += ELM_HDR_SIZE+EVLEN_GMS_RF_FREQ;
					break;
				}
			case EID_GDE_TMSYN:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_TMSYN)
				{
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_TMSYN;
					break;
				}
			case EID_GME_NT_TM:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GME_NT_TM)
				{
					// Synchronizing time
					SyncTMResp(pldbuf+elmpos+ELM_HDR_SIZE,EVLEN_GME_NT_TM);
					elmpos += ELM_HDR_SIZE+EVLEN_GME_NT_TM;
					break;
				}
			case EID_GMS_FW_INFO:
#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GMS_FW_INFO)
				{
					// Prepare upgrade
					PrepareUpgrade(subtype,pldbuf+elmpos+ELM_HDR_SIZE,EVLEN_GMS_FW_INFO);
					elmpos += ELM_HDR_SIZE+EVLEN_GMS_FW_INFO;
					break;
				}
#endif	// GM_IMAGE_A || GM_IMAGE_B

			case EID_GTE_READ:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GTE_READ)
				{
					// Reset benchmark
					ReadGDEParam(pldbuf+elmpos+ELM_HDR_SIZE,EVLEN_GTE_READ);
					elmpos += ELM_HDR_SIZE+EVLEN_GTE_READ;
					break;
				}
			case EID_GMS_UPGD:
#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GMS_UPGD)
				{
					// Upgrade data process
					RFOadImgBlockWrite(subtype,pldbuf+elmpos+ELM_HDR_SIZE,EVLEN_GMS_UPGD);
					elmpos += ELM_HDR_SIZE+EVLEN_GMS_UPGD;
					break;
				}
#endif	// GM_IMAGE_A || GM_IMAGE_B

				return RF_PLD_ERR;
			default:
				return RF_EID_UNK;
		}
	}
	return RF_SUCCESS;
}

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
			if (evallen != EVLEN_GDE_LOC_TM)
				return 0;

			UTCTimeStruct curtmst;

			VOID evalbuf;
			osal_ConvertUTCTime(&curtmst, osal_getClock());
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
			if (evallen != EVLEN_GDE_HRTBT)
				return 0;
			
			osal_memcpy(pVal, evalbuf, evallen);
			break;
		}
		case EID_GDE_BENCH_INFO:
		{
			if (evallen != EVLEN_GDE_BENCH_INFO)
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
			if (evallen != EVLEN_GMS_INFO_ACK)
				return 0;

			osal_memcpy(pVal, evalbuf, evallen);
			break;
		}
		case EID_GDE_PARAMS:
		{
			if (evallen != EVLEN_GDE_BENCH_INFO)
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
			if (evallen != EVLEN_GMS_RF_FREQ)
				return 0;

			osal_memcpy(pVal, evalbuf, evallen);
			break;
		}
		case EID_GDE_TMSYN:
		{
			if (evallen != EVLEN_GDE_TMSYN)
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
	buf[EVAL_LEN_POS] = evallen;
	
	return evallen+ELM_HDR_SIZE;
}

static void ProcAckMsg(uint8 subtype,uint8 *ackbuf,  uint8 acklen)
{
	switch(subtype)
	{
		case GME_SUBTYPE_CARINFO_RESP:
			// GME recieved carinfo successfully
			ClearDataResend();
#if ( defined ALLOW_DEBUG_OUTPUT )
			Com433WriteStr(COM433_DEBUG_PORT, "\r\nSEND OK!");
#endif
			break;
		default:
			break;
	}
	return ;
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

static void SetRfFreq(uint8 subtype,uint8 *rffrqbuf,  uint8 rffrqlen)
{
	uint8 tmpfreq = 0;// Use value 0 to use default frequency setup
	if ( rffrqlen != EVLEN_GMS_RF_FREQ )
		return;

	if (CHECK_FREQ_VALID(rffrqbuf[0]) == TRUE)
		tmpfreq = rffrqbuf[0];

	switch(subtype)
	{
		case GTE_SUBTYPE_PRESET_REQ:
			// Change dest ID to GTE ID, prepare change into setup mode
			RFdestID = RFGTEID;
			RFstfrq = (tmpfreq==0? RFstfrq: tmpfreq);
			RFDataForm(GDE_SUBTYPE_T_PRESET_RESP, &RFstfrq, sizeof(RFstfrq));
			//RF_working(BLECore_TaskId, GetRFstate());

			SetSysState(SYS_SETUP);// State will change after working state over
			break;
		case GTE_SUBTYPE_ORDER_UPGD_REQ:
		case GME_SUBTYPE_ORDER_UPGD_REQ:
			RFupgfrq = (tmpfreq==0? RFupgfrq: tmpfreq);
			break;
		default:
			return;
	}
	return;
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
 * @param	data - sync time element value buf
 * @param	len - sync time element data length
 *
 * @return	none
 */
static void SyncTMResp(uint8 *data, uint8 len)
{
	if (len!=EVLEN_GME_NT_TM)
		return;

	// time overflow
	if (data[ELM_HDR_SIZE+UTCL_YEAR_POS] > 136)
		return;

	UTCTimeStruct settmst;
	
	settmst.hour = data[UTCL_HOUR_POS];
	settmst.minutes = data[UTCL_MINTS_POS];
	settmst.seconds = data[UTCL_SECND_POS];
	settmst.day = data[UTCL_DAY_POS]-1;
	settmst.month = data[UTCL_MONTH_POS]-1;
	settmst.year = 2000+data[UTCL_YEAR_POS];
	
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
 * @fn		ReadGDEParam
 *
 * @brief	read GDE parameters.
 *
 * @param	none
 *
 * @return	none
 */
static void ReadGDEParam(uint8* readreq, uint8 len)
{
	uint8 rddata[EVLEN_GDE_PARAMS];

	if (len != EVLEN_GTE_READ)
		return;

	ReadRFParam(rddata);
	ReadGMParam(rddata);
	ReadIDParam(rddata);

	RFDataForm(GDE_SUBTYPE_T_READ_RESP, rddata, sizeof(rddata));
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
	bool flag = FALSE;
	
	if (len!=EVLEN_GDE_PARAMS || GetSysState() != SYS_SETUP)
		return flag;

	flag = SetRFParam(setdata[ST_RF_WK_FREQ_POS],setdata[ST_RF_ST_FREQ_POS],setdata[ST_RF_UPGD_FREQ_POS],\
			setdata[ST_RF_AIR_BAUD_POS],setdata[ST_RF_PWR_LVL_POS]);
	flag &= SetGMParam(setdata[ST_GM_HB_FREQ_POS],setdata[ST_GM_DTCT_SENS_POS],setdata[ST_GM_BENCH_ALG_POS],\
			setdata[ST_GM_STATUS_POS]);
	flag &= SetIDParam(BUILD_UINT16(setdata[ST_GDE_ADDR_L_POS],setdata[ST_GDE_ADDR_H_POS]),\
			BUILD_UINT16(setdata[ST_GME_ADDR_L_POS],setdata[ST_GME_ADDR_H_POS]));

	RFDataForm(GDE_SUBTYPE_T_SET_RESP, &flag, sizeof(flag));
	//RF_working(BLECore_TaskId, GetRFstate());

	return flag;
}
