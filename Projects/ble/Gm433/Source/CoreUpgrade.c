/********************************************************************
	Filename:	CoreUpgrade.c
	Revised:		Date: 2015-11-22
	Revision:	1.0 

*********************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "hal_aes.h"
#include "hal_crc.h"
#include "hal_flash.h"
#include "hal_types.h"

#include "OSAL.h"
#include "osal_snv.h"
#include "OSAL_Clock.h"

#include "Com433.h"
#include "BLECore.h"
#include "GMProc.h"
#include "Pktfmt.h"
#include "CoreUpgrade.h"
#include "RFProc.h"

/*********************************************************************
 * CONSTANTS
 */
// App NV id (>0x80 available, see Bcomdef.h: BLE_NVID_XXXX)
// Save previous GDE ID, GME ID
#define GMS_NV_GDE_ID_ID		0xA0
#define GMS_NV_GME_ID_ID		0xA1

//Save previous benchmark count and car detect state
#define GMS_NV_DT_STATE_ID		0xA2
#define GMS_NV_BENCH_CNT_ID		0xA3

// Benchmark of X/Y/Z, in case upgrade when car detected
#define GMS_NV_GDE_X_BENCH_ID	0xA4
#define GMS_NV_GDE_Y_BENCH_ID	0xA5
#define GMS_NV_GDE_Z_BENCH_ID	0xA6

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern uint8 BLECore_TaskId;
extern uint16 RFGDEID,RFGMEID,version;
extern uint16 readcnt,ordrcnt;
extern int16 Xbenchmk,Ybenchmk,Zbenchmk;

/*********************************************************************
 * LOCAL VARIABLES
 */
// Read previous GM status flag
static bool finfistbootflg = FALSE;

static bool prepupgrd = FALSE;
static bool upgdfin = FALSE;
static msgerrcd_t upgdret=MSG_SUCCESS;
static uint8 upgdsrcgte=FALSE;

static uint16 oadodrvrn, oadodrfwcrc, oadodrtotblk, oadcurblknum = 0;
static uint32 oadodrfwlen;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static bool ReadPrevGMstatus(int16* tmpxbm, int16* tmpybm, int16* tmpzbm);
static uint8 GetValidFirmwareCount(void);

static void set_fw_upgd_tick_alarm(uint8 alrmhh, uint8 alrmmm, uint8 alrmss);
static uint16 RFOadCrcCalc(void);

static uint8 RFOadCheckDL(void);



uint8 StoreSetting(uint8 NvId)
{
	uint8 ret = SUCCESS;
	
	switch(NvId)
	{
		case GMS_NV_GDE_ID_ID:
			ret |= osal_snv_write(NvId, sizeof(RFGDEID),&RFGDEID);
			break;
		case GMS_NV_GME_ID_ID:
			ret |= osal_snv_write(NvId, sizeof(RFGMEID),&RFGMEID);
			break;
		case GMS_NV_DT_STATE_ID:
		{
			uint8 cdst = GetGMSnState();
			ret |= osal_snv_write(NvId, sizeof(cdst),&cdst);
			break;
		}
		case GMS_NV_BENCH_CNT_ID:
		{
			uint8 tmpcnt = GetEmpBenchCnt();
			ret |= osal_snv_write(NvId, sizeof(tmpcnt),&tmpcnt);
			break;
		}
		case GMS_NV_GDE_X_BENCH_ID:
			ret |= osal_snv_write(NvId, sizeof(Xbenchmk),&Xbenchmk);
			break;
		case GMS_NV_GDE_Y_BENCH_ID:
			ret |= osal_snv_write(NvId, sizeof(Ybenchmk),&Ybenchmk);
			break;
		case GMS_NV_GDE_Z_BENCH_ID:
			ret |= osal_snv_write(NvId, sizeof(Zbenchmk),&Zbenchmk);
			break;
		default:
			break;
	}

	return ret;
}


void ReadSysSetting(void)
{
	uint8 ret = SUCCESS;
	uint16 prevgdeid,prevgmeid;

	ret |= osal_snv_read(GMS_NV_GDE_ID_ID, sizeof(prevgdeid),&prevgdeid);
	ret |= osal_snv_read(GMS_NV_GME_ID_ID, sizeof(prevgmeid),&prevgmeid);

	// No parameter saved yet
	if (ret != SUCCESS)
	{
		SetDevID(GDE_DEV_ID, GME_DEV_ID, 0);
		StoreSetting(GMS_NV_GDE_ID_ID);
		StoreSetting(GMS_NV_GME_ID_ID);
	}
	else
	{
		SetDevID(prevgdeid, prevgmeid, 0);
	}
}

void ReadGMSetting(void)
{
	int16 prevXbench,prevYbench,prevZbench;

	if (finfistbootflg == FALSE)
	{
		finfistbootflg = TRUE;
		if (ReadPrevGMstatus(&prevXbench,&prevYbench,&prevZbench) == TRUE)
		{
			InitBenchmk(BENCH_CALIBRATING,prevXbench,prevYbench,prevZbench);
			SetGMState(CAR_DETECTED_OK);
		}
	}
	return;
}


void PrepareUpgrade(uint8 subtype, uint8* upgpkt, uint8 len)
{
	if (len != EVLEN_GMS_FW_INFO || (subtype!=GME_SUBTYPE_ORDER_UPGD_REQ && subtype!=GTE_SUBTYPE_ORDER_UPGD_REQ))
		return;

#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
	oadodrvrn = BUILD_UINT16(upgpkt[NEW_FW_VERN_NUM_L_POS],upgpkt[NEW_FW_VERN_NUM_H_POS]);
	oadodrfwlen = BUILD_UINT32(upgpkt[NEW_FW_TOT_LEN_LL_POS], upgpkt[NEW_FW_TOT_LEN_LH_POS],\
			upgpkt[NEW_FW_TOT_LEN_HL_POS], upgpkt[NEW_FW_TOT_LEN_HH_POS]);
	oadodrfwcrc = BUILD_UINT16(upgpkt[NEW_FW_CRC_L_POS],upgpkt[NEW_FW_CRC_H_POS]);
	oadodrtotblk = BUILD_UINT16(upgpkt[NEW_FW_TOT_BLK_L_POS],upgpkt[NEW_FW_TOT_BLK_H_POS]);

	if (RF_OAD_IMG_ID(oadodrvrn) == RF_OAD_IMG_ID(version) )
	{
		upgdret = IMG_TYPE_ERR;

#if ( defined ALLOW_DEBUG_OUTPUT )
		Com433WriteStr(COM433_DEBUG_PORT, "\r\nError image type.");
#endif	// ALLOW_DEBUG_OUTPUT
	}
	else if (oadodrfwlen != ((uint32)oadodrtotblk*RF_OAD_BLOCK_SIZE))
	{
		upgdret = IMG_SIZE_ERR;

#if ( defined ALLOW_DEBUG_OUTPUT )
		Com433WriteInt(COM433_DEBUG_PORT, "\r\nError firmware len:", oadodrtotblk, 10);
#endif	// ALLOW_DEBUG_OUTPUT	
	}
	else
	{
		upgdret = MSG_SUCCESS;
		prepupgrd = TRUE;
	}

	if (subtype == GME_SUBTYPE_ORDER_UPGD_REQ)
	{
		upgdsrcgte = FALSE;
		RFDataForm(GDE_SUBTYPE_ORDER_RESP, (uint8 *)&upgdret, sizeof(uint8));

		if (prepupgrd == TRUE)
		{
			set_fw_upgd_tick_alarm(upgpkt[NEW_FW_UPGD_HOUR_POS],upgpkt[NEW_FW_UPGD_MINTS_POS],\
					upgpkt[NEW_FW_UPGD_SECND_POS]);
#if ( defined ALLOW_DEBUG_OUTPUT )
			Com433WriteInt(COM433_DEBUG_PORT, "\r\nOrder left read count:",ordrcnt,10);
#endif	// ALLOW_DEBUG_OUTPUT
		}
	}
	else if (subtype == GTE_SUBTYPE_ORDER_UPGD_REQ)
	{
		upgdsrcgte = TRUE;
		RFDataForm(GDE_SUBTYPE_T_ORDER_RESP, (uint8 *)&upgdret, sizeof(uint8));

#if ( defined ALLOW_DEBUG_OUTPUT )
			Com433WriteStr(COM433_DEBUG_PORT, "\r\nGTE upgrade begin");
#endif	// ALLOW_DEBUG_OUTPUT

		if (prepupgrd == TRUE)
			ChngSysState(SYS_UPGRADE, IDLE_PWR_HOLD_PERIOD);

	}
#endif
}


void EraseFirmwareInfo(void)
{
	uint16 clrcrc[2]={ 0x0000, 0xFFFF };//{0};

	if (GetValidFirmwareCount() == 2)
	{
		//HalFlashWrite(RF_OAD_IMG_ORG_PAGE, (uint8 *)clrcrc, 1);
		HalFlashErase(RF_OAD_IMG_ORG_PAGE);
		ASM_NOP;
		HAL_SYSTEM_RESET();
	}

	return;
}


static bool ReadPrevGMstatus(int16* tmpxbm, int16* tmpybm, int16* tmpzbm)
{
	uint8 ret = SUCCESS,prevcnt;
	detectstatus_t prevdtst;

	ret = osal_snv_read(GMS_NV_DT_STATE_ID, sizeof(prevdtst),&prevdtst);
	ret |= osal_snv_read(GMS_NV_BENCH_CNT_ID, sizeof(prevcnt),&prevcnt);

	// Only set benchmark when it's CAR_DETECTED status and have benchmark last working
	if (ret == SUCCESS && prevdtst == CAR_DETECTED_OK && prevcnt !=0)
	{
		ret |= osal_snv_read(GMS_NV_GDE_X_BENCH_ID, sizeof(int16),tmpxbm);
		ret |= osal_snv_read(GMS_NV_GDE_Y_BENCH_ID, sizeof(int16),tmpybm);
		ret |= osal_snv_read(GMS_NV_GDE_Z_BENCH_ID, sizeof(int16),tmpzbm);
		if (ret == SUCCESS)
			return TRUE;
	}

	return FALSE;
}

static uint8 GetValidFirmwareCount(void)
{
	uint16 destcrc[2];

	HalFlashRead(RF_OAD_IMG_DST_PAGE, RF_OAD_IMG_CRC_OSET, (uint8 *)destcrc, HAL_FLASH_WORD_SIZE);

	if (destcrc[0]==destcrc[1] && destcrc[0]!=0x0000 && destcrc[0]!=0xFFFF)
		return 2;
	else
		return 1;
}

static void set_fw_upgd_tick_alarm(uint8 alrmhh, uint8 alrmmm, uint8 alrmss)
{
	UTCTimeStruct curtmst;
	uint32 totscnd;

	// Prepare upgrade right now if 00:00:00
	if (alrmhh==0 && alrmmm==0 && alrmss==0)
	{
		ordrcnt = 0;
		return;
	}

	osal_ConvertUTCTime(&curtmst, osal_getClock());
	// tommorow
	if (alrmhh < curtmst.hour)
		alrmhh += 24;
	else if (alrmhh==curtmst.hour && alrmmm<curtmst.minutes && (curtmst.minutes-alrmmm>1) )
		alrmhh += 24;	// Give up when current time passed setting upgrade time>1mins
	else if (alrmhh==curtmst.hour && alrmmm<curtmst.minutes)
		alrmmm += 1;	// Prepare upgrade if passed less than 1 minutes in case clock drift
	else if (alrmhh==curtmst.hour && alrmmm==curtmst.minutes && alrmss < curtmst.seconds)
		alrmmm += 1;	// Prepare upgrade if upgrade time and current time in same minutes

	totscnd = (uint32)(alrmhh-curtmst.hour)*MIN_IN_HOUR*SEC_IN_MIN+(int32)(alrmmm-curtmst.minutes)*SEC_IN_MIN\
			+(int32)(alrmss-curtmst.seconds);

	ordrcnt = totscnd/(GM_READ_EVT_PERIOD/MILSEC_IN_SEC)+readcnt;
}


void SetPrepUpgdState(bool state)
{
	prepupgrd = state;
}

bool GetPrepUpgdState(void)
{
	return prepupgrd;
}

void ReportUpgdState(void)
{
	uint8 subtype;

	// Always check upgrade state no mater it success
	if ( upgdret == MSG_SUCCESS)
	{
		if(RFOadCheckDL())
		{
			upgdfin = TRUE;
			upgdret = MSG_SUCCESS;
			Com433WriteStr(COM433_DEBUG_PORT,"\r\nUpgrade OK\r\n");
		}
		else
		{
			upgdret = IMG_CRC_FAIL;
			Com433WriteStr(COM433_DEBUG_PORT,"\r\nUpgrade CRC failed\r\n");
		}
	}

	subtype = (upgdsrcgte==FALSE? GDE_SUBTYPE_UPGD_ACK: GDE_SUBTYPE_T_UPGD_ACK);
	RFDataForm(subtype, (uint8*)&upgdret, sizeof(uint8));

#if ( !defined USE_TEN308_RF)
	while (GetRTxRdyFlg() == FALSE);
#endif

	if (upgdfin == TRUE)
	{
		HAL_SYSTEM_RESET();
	}
}

/*********************************************************************
 * @fn		RFOadImgBlockWrite
 *
 * @brief	 Process the Image Block Write.
 *
 * @param	 subtype - send to GDE or GTE subtype
 * @param	 pValue - pointer to data to be written
 *
 * @return	status
 */
void RFOadImgBlockWrite(uint8 subtype, uint8 *pValue, uint8 len)
{
	uint16 blkNum = BUILD_UINT16( pValue[NEW_FW_CUR_BLK_L_POS], pValue[NEW_FW_CUR_BLK_H_POS] );

	Com433WriteInt(COM433_DEBUG_PORT, "\r\nN:",blkNum,10);

	// make sure this is the image we're expecting
	if ( blkNum == 0 && oadcurblknum == 0 )
	{
		uint16 fwcrc;
		oad_img_hdr_t ImgHdr;

		osal_memcpy(&fwcrc, pValue+RF_OAD_BLOCK_BEG_POS, sizeof(uint16));
		osal_memcpy((uint8 *)&ImgHdr, pValue+RF_OAD_BLOCK_BEG_POS+RF_OAD_IMG_HDR_OSET,sizeof(oad_img_hdr_t));

		if ( oadodrvrn != ImgHdr.ver )
			upgdret = IMG_TYPE_ERR;

		if ( oadcurblknum != blkNum  || oadodrfwcrc != fwcrc || oadodrtotblk != \
				ImgHdr.len/(RF_OAD_BLOCK_SIZE/HAL_FLASH_WORD_SIZE))
			upgdret = IMG_SIZE_ERR;

		if (upgdret != MSG_SUCCESS)
			ChngSysState(SYS_UPGRADE, 0);
	}

	if (oadcurblknum == blkNum)
	{
		uint16 addr = oadcurblknum * (RF_OAD_BLOCK_SIZE / HAL_FLASH_WORD_SIZE) + \
								(RF_OAD_IMG_DST_PAGE * RF_OAD_FLASH_PAGE_MULT);
		oadcurblknum++;	
#if ( defined GM_IMAGE_B )
		// Skip the Image-B area which lies between the lower & upper Image-A parts, when upgrading image A.
		if (addr >= (RF_OAD_IMG_B_PAGE * RF_OAD_FLASH_PAGE_MULT))
			addr += RF_OAD_IMG_B_AREA * RF_OAD_FLASH_PAGE_MULT;
#endif	// GM_IMAGE_B


#if ( defined GM_IMAGE_A ) || ( defined GM_IMAGE_B )
		if ((addr % RF_OAD_FLASH_PAGE_MULT) == 0)	// Erase next flash page
			HalFlashErase(addr / RF_OAD_FLASH_PAGE_MULT);

		HalFlashWrite(addr, pValue+RF_OAD_BLOCK_BEG_POS, (RF_OAD_BLOCK_SIZE/ HAL_FLASH_WORD_SIZE));
#endif	// GM_IMAGE_A || GM_IMAGE_B

		if (oadcurblknum == oadodrtotblk)	// If the OAD Image is complete.
			ChngSysState(SYS_UPGRADE, IDLE_PWR_HOLD_PERIOD+c_rand()*SEC_IN_MIN/MAX_RANDOM_SECONDS);	
	}
	else if (blkNum>oadcurblknum)
	{
		Com433WriteInt(COM433_DEBUG_PORT, "\r\nLOSE:",oadcurblknum,10);
		ChngSysState(SYS_UPGRADE, 0);
		return;
	}

	return;
}


/**************************************************************************************************
 * @fn			RFOadCrcCalc
 *
 * @brief		 Run the CRC16 Polynomial calculation over the DL image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return		The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 RFOadCrcCalc(void)
{
	uint8 pageEnd = oadodrtotblk / RF_OAD_BLOCKS_PER_PAGE;
	uint16 osetEnd = (oadodrtotblk - (pageEnd * RF_OAD_BLOCKS_PER_PAGE)) * RF_OAD_BLOCK_SIZE;

#if ( defined GM_IMAGE_B )
	pageEnd += RF_OAD_IMG_DST_PAGE + RF_OAD_IMG_B_AREA;
#else
	pageEnd += RF_OAD_IMG_DST_PAGE;
#endif	// GM_IMAGE_B

	HalCRCInit(0x0000);	// Seed thd CRC calculation with zero.

	for (uint8 page = RF_OAD_IMG_DST_PAGE; ; page++)
	{
#if ( defined GM_IMAGE_B )
		// Skip the Image-B area which lies between the lower & upper Image-A parts.
		if (page == RF_OAD_IMG_B_PAGE)
		{
			page += RF_OAD_IMG_B_AREA;
		}
#endif	// GM_IMAGE_B

		for (uint16 oset = 0; oset < HAL_FLASH_PAGE_SIZE; oset += HAL_FLASH_WORD_SIZE)
		{
			if ((page == RF_OAD_IMG_DST_PAGE) && (oset == RF_OAD_IMG_CRC_OSET))
			{
				continue;	// Skip the CRC and shadow.
			}
			else if ((page == pageEnd) && (oset == osetEnd))
			{
				return HalCRCCalc();
			}
			else
			{
				uint8 buf[HAL_FLASH_WORD_SIZE];
				HalFlashRead(page, oset, buf, HAL_FLASH_WORD_SIZE);

				for (uint8 idx = 0; idx < HAL_FLASH_WORD_SIZE; idx++)
				{
					HalCRCExec(buf[idx]);
				}
			}
		}
	}
}

/**************************************************************************************************
 * @fn			RFOadCheckDL
 *
 * @brief		 Check validity of the downlRFOaded image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return		TRUE or FALSE for image valid.
 **************************************************************************************************
 */
static uint8 RFOadCheckDL(void)
{
	uint16 crc[2];
	uint16 addr = RF_OAD_IMG_DST_PAGE * RF_OAD_FLASH_PAGE_MULT + RF_OAD_IMG_CRC_OSET / HAL_FLASH_WORD_SIZE;

	HalFlashRead(RF_OAD_IMG_DST_PAGE, RF_OAD_IMG_CRC_OSET, (uint8 *)crc, sizeof(crc));

	if ((crc[0] == 0xFFFF) || (crc[0] == 0x0000))
	{
		return FALSE;
	}

	// Enable it in crc area
	if (crc[1] == 0xFFFF)
	{
		crc[1] = RFOadCrcCalc();
		crc[0] = 0xFFFF;
		HalFlashWrite(addr, (uint8 *)crc, 1);
		HalFlashRead(RF_OAD_IMG_DST_PAGE, RF_OAD_IMG_CRC_OSET, (uint8 *)crc, sizeof(crc));
	}

	return (crc[0] == crc[1]);
}

/*********************************************************************
*********************************************************************/

