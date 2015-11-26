/*********************************************************************
 * INCLUDES
 */

#include "RFProc.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Default RF frequency params: work 433 set 433 upgrade 470
uint8 RFwkfrq = 0x01,RFstfrq = 0x01,RFupgfrq = 0x02;

// Default RF air baud rate: 9600
uint8 RFairbaud = 0x05;

// Default RF air baud rate : 20dBm
uint8 RFpwr = 0x08;  // Max level 

/*********************************************************************
 * EXTERNAL VARIABLES
 */

extern uint8 BLECore_TaskId;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static bool RFworkingflag=FALSE; 


static uint8 rfsndbuf[GMS_PKT_MAX_LEN] = {0};
static uint8 rfsndlen;

// Set RF working params command buffer and length, separate with data send buffer
static uint8 stcmdbuf[32];
static uint8 stcmdlen;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SetTENModule(void);

static void TENModuleWakeup(void);
static void TENModuleSleep(void);

static void TENRFWakeup(void);
static void TENRFSleep(void);

static uint8 FromTENCmd(bool wrDir, uint8 *wrbuf, uint8 freq);


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void RF_working(uint8 task_id, rfstate_t newrfstate)
{
	uint32 lasttmout;
	
	SetRFstate(newrfstate);

	// Wait untill last RF work finish in case of reentry
	lasttmout = osal_get_timeoutEx(task_id, RF_DATA_PROC_EVT);
	if (lasttmout != 0)
	{
		osal_start_timerEx(task_id, RF_DATA_PROC_EVT, lasttmout);
		return;
	}

	switch (newrfstate)
	{
		case RF_PRESET:
		{
			TENModuleWakeup();
			SetRFstate(RF_BEG_SET);
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT, WAIT_RF_PRESET_PERIOD);
			break;
		}
		case RF_BEG_SET:
		{
			SetTENModule();
			SetRFstate(RF_WAIT_RESET);
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT, WAIT_TEN_CMD_PERIOD);

			break;
		}
		case RF_WAIT_RESET:
		{
			TENModuleSleep();
			SetRFstate(RF_WAKEUP);
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT, WAIT_TEN_STOP_PERIOD);
			break;
		}
		case RF_WAKEUP:
		{
			TENModuleWakeup();
			SetRFstate(RF_WORK);
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT, WAIT_TEN_RF_RDY_PERIOD);
			break;
		}
		case RF_WORK:
		{
			if ( rfsndlen > 0 )
			{
				Com433Write(COM433_WORKING_PORT, rfsndbuf, rfsndlen);
				rfsndlen = 0;
			}
			break;
		}
		case RF_SLEEP:
			TENModuleSleep();
			break;
		default:
			break;
	}

	return;
}

rfpkterr_t RFDataSend(uint8 *buf, uint8 len)
{
	//Copy send data to buffer and wait 250ms for TEN308 RF wake up
	osal_memcpy(rfsndbuf,buf,len);
	rfsndlen = len;
	RF_working(BLECore_TaskId, RF_WAKEUP);

	return RF_SUCCESS;
}


/*********************************************************************
 * @fn		Com433Handle
 *
 * @brief	COM port call back function
 *
 * @param	port - serial port.
 * @param	pBuffer - read buffer.
 * @param	length - read buffer length.
 *
 * @return	none.
 */
void Com433Handle(uint8 port,uint8 *pBuffer, uint16 length)
{
	// No func will be called in CC112X RF from working port (recieve data by interrupt)
	if (port == COM433_WORKING_PORT)
	{
#if ( defined TEN_DEBUG_MODE )
		// Print serial data directly in debug mode
		Com433Write(COM433_DEBUG_PORT, pBuffer, length);
#else	// !TEN_DEBUG_MODE
		GMSPktForm(pBuffer,length);
#endif	// TEN_DEBUG_MODE
	}
	else if (port == COM433_DEBUG_PORT)
	{
#if ( defined TEN_DEBUG_MODE )
		Com433Write(COM433_WORKING_PORT, pBuffer, length);
#endif	// TEN_DEBUG_MODE
	}

	return;
}

/*********************************************************************
* Private functions
*/

static void SetTENModule(void)
{
	uint8 wkfreq;

	wkfreq = GetCurFreq(GetSysState());
	stcmdlen = FromTENCmd(TRUE, stcmdbuf, wkfreq);
	if ( stcmdlen > 0 )
	{
		Com433Write(COM433_WORKING_PORT, stcmdbuf, stcmdlen);
		stcmdlen = 0;
	}
}

static void TENModuleSleep(void)
{
	if (RFworkingflag == FALSE)
		return;

	RFworkingflag = FALSE;
	TENRFSleep();
}

static void TENModuleWakeup(void)
{
	if (RFworkingflag == TRUE)
		return;

	RFworkingflag = TRUE;

	TENRFWakeup();
}


static void TENRFWakeup(void)
{
	TEN308_DIOA_PINSEL &= (uint8) ~TEN308_DIOA_GPIO;
	TEN308_DIOA_PINDIR |= (uint8) TEN308_DIOA_GPIO;
	TEN308_DIOA_PIN = 0;

	TEN308_DIOB_PINSEL &= (uint8) ~(TEN308_DIOB_GPIO);
	TEN308_DIOB_PINDIR |= (uint8) (TEN308_DIOB_GPIO);
	TEN308_DIOB_PIN = 0;
}

static void TENRFSleep(void)
{
	TEN308_DIOA_PINSEL &= (uint8) ~(TEN308_DIOA_GPIO);
	TEN308_DIOA_PINDIR &= (uint8) ~(TEN308_DIOA_GPIO);
	//TEN308_DIOA_PINDIR |= (uint8) (TEN308_DIOA_GPIO);
	TEN308_DIOA_PININP |= (uint8)(TEN308_DIOA_GPIO);
	//TEN308_DIOA_PIN = 1;

	TEN308_DIOB_PINSEL &= (uint8) ~(TEN308_DIOB_GPIO);
	TEN308_DIOB_PINDIR &= (uint8) ~(TEN308_DIOB_GPIO);
	//TEN308_DIOB_PINDIR |= (uint8) (TEN308_DIOB_GPIO);
	TEN308_DIOB_PININP |= (uint8) (TEN308_DIOB_GPIO);
	//TEN308_DIOB_PIN = 1;
}

/*********************************************************************
 * @fn		FromTENCmd
 *
 * @brief	Form TEN308 serial command.
 *
 * @param	wrDir - Write/read direction, TRUE to write, false to read.
 * @param	wrbuf - Write/read buffer.
 * @param	freq - set frequency(only used in write mode).
 *
 * @return	Length of cmd buf.
 */
static uint8 FromTENCmd(bool wrDir, uint8 *wrbuf, uint8 freq)
{
	uint8 header[]={0xFF,0x56,0xAE,0x35,0xA9,0x55};
	uint8 curlen = sizeof(header);

	osal_memcpy(wrbuf, header, curlen);

	if (wrDir == TRUE)
	{
		wrbuf[curlen++] = 0x90;
		wrbuf[curlen++] = tenRFfreq[freq][0];
		wrbuf[curlen++] = tenRFfreq[freq][1];
		wrbuf[curlen++] = tenRFfreq[freq][2];
		wrbuf[curlen++] = tenRFairbaud[RFairbaud-1];
		wrbuf[curlen++] = tenRFpwr[RFpwr-1];
		wrbuf[curlen++] = 0x03;	// serial baud 9600
		wrbuf[curlen++] = 0x00;
		wrbuf[curlen++] = 0x05;	// wakeup time 50ms
	}
	else
	{
		VOID freq;
		wrbuf[curlen++] = 0xF0;
	}

	return curlen;
}

