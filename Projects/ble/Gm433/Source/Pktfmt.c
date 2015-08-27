#include "comdef.h"
#include "OnBoard.h"
#include "OSAL.h"
#include "OSAL_Clock.h"

#include "Com433.h"
#include "Pktfmt.h"
#include "GMProc.h"
#include "BLECore.h"

#if ( defined USE_CC112X_RF )
#include "Cc112x.h"
#endif

#define GET_RF_SRC_ADDR(rfbuf)	\
	(BUILD_UINT16(rfbuf[GMS_SRC_ADDR_POS+1],rfbuf[GMS_SRC_ADDR_POS]))

#define GET_RF_DEST_ADDR(rfbuf)	\
	(BUILD_UINT16(rfbuf[GMS_DEST_ADDR_POS+1],rfbuf[GMS_DEST_ADDR_POS]))


// Wait 1 [ms]
#define SYS_WAIT_1MS()      {for(unsigned short i=0;i<32000;i++)asm("NOP"); }

// Wait t [ms]
#define SYS_WAIT_MS(t)                      \
    do{                                 \
        for(uint8 i = 0; i<t; i++)      \
            SYS_WAIT_1MS();                 \
    }while(0)



#ifndef GDE_DEV_ID
#define GDE_DEV_ID		1
#endif

#ifndef GME_DEV_ID
#define GME_DEV_ID		1001
#endif

// Default version 1.0
#ifndef VERSION_NUMBER
#define VERSION_NUMBER		0x0100
#endif

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


uint16 RFdevID = GDE_DEV_ID;
uint16 RFdestID = GME_DEV_ID;
uint16 version = VERSION_NUMBER;

#if ( ! defined USE_CC112X_RF )
uint8 rfsndbuf[GMS_PKT_MAX_LEN] = {0};
uint8 rfsndlen;
#endif

static pktgms_t pktgmsst=PKT_GMS_ID;

static uint8 gmsrdpkt[GMS_PKT_MAX_LEN]={0};
static uint8 currdlen=0;
static uint8 totrdlen=0;

static uint8 syncautostop;

static UTCTimeStruct lasttm;

static void syncUTCtimeresp(uint8 *data, uint8 len);
static uint8 filltime(uint8 *buf);

static uint8 calcGMchksum(uint8* chkbuf, uint16 len);

static rferr_t rfdatasend(uint8 *buf, uint8 len);

static void savelasttminfo(UTCTimeStruct tm);
static void chktmsync(UTCTimeStruct tm);


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
 * @brief		
 *
 * @param	
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


rferr_t rfdataparse(uint8 *rfdata,uint8 len)
{
	if (rfdata[GMS_ID_POS]!=GME_SRC_ID && rfdata[GMS_ID_POS]!=GTE_SRC_ID)
		return PKT_DENY;
	
	if (rfdata[GMS_TOT_LEN_POS] != len)
		return PKT_ERR;

	if (osal_memcmp(rfdata+GMS_RESERVE_POS, GMS_RESERVE_STR, GMS_RESERVE_SIZE) == FALSE)
		return PKT_DENY;
	
	if (GET_RF_DEST_ADDR(rfdata) != RFdevID)
		return BAD_ADDR;
		
	if (calcGMchksum(rfdata,len) != rfdata[GMS_CHK_SUM_POS])
		return CHKSUM_ERR;

	RFdestID = GET_RF_SRC_ADDR(rfdata);

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
			case GTE_ST_PARAM_SET:
				// set GDE params
				break;
			case GTE_ST_UPGD_REQ:
				// upgrade process
				break;
			default:
				return SUB_TYPE_ERR;
		}
	}

	return RF_SUCCESS;
}

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
		case GDE_ST_M_UPGD_ACK:
		case GDE_ST_T_UPGD_ACK:
			// upgrade ack
			break;
		case GDE_ST_T_SET_ACK:
			// set cmd ack
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


/*******************************************
	return total length of time element, 1+1+6
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

	chktmsync(curtmst);

	return (ELM_HDR_SIZE + UTCL_EVAL_LEN);
}


/*******************************
	hhmmss ddmmyy
*/
static void syncUTCtimeresp(uint8 *data, uint8 len)
{
	if (data[0]!=EID_GME_TMSYN_ACK || len!=GME_TMSYN_ACK_LEN)
		return;

	// time overflow
	if (data[ELM_HDR_SIZE+5] > 136)
		return;

	UTCTimeStruct settmst;
	
	settmst.hour = data[ELM_HDR_SIZE];
	settmst.minutes = data[ELM_HDR_SIZE+1];
	settmst.seconds = data[ELM_HDR_SIZE+2];
	settmst.day = data[ELM_HDR_SIZE+3]-1;
	settmst.month = data[ELM_HDR_SIZE+4]-1;
	settmst.year = 2000+data[ELM_HDR_SIZE+5];
	
	osal_setClock(osal_ConvertUTCSecs(&settmst));

	savelasttminfo(settmst);

	cleartmsync();
}


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

static rferr_t rfdatasend(uint8 *buf, uint8 len)
{
	RFwakeup();

#if ( defined USE_CC112X_RF )
	txdata(buf,len);
#else
	osal_memcpy(rfsndbuf,buf,len);
	rfsndlen = len;
	prepare_TEN_send();
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
