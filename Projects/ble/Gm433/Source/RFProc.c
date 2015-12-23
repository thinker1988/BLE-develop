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

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 BLECore_TaskId;

extern uint8 RFwkfrq, RFstfrq, RFupgfrq, RFairbaud, RFpwr;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// RF init state
static rfstate_t rfst=RF_PRESET;

#if ( defined HAL_SPI_MASTER )
// Receive or send data ready flag, controled by interrupt
static bool rtxrdyflag=FALSE; 
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void SetRFstate(rfstate_t newrfstate)
{
	rfst = newrfstate;
}

rfstate_t GetRFstate(void)
{
	return rfst;
}

/*********************************************************************
 * @fn		ReadRFParam
 *
 * @brief	Read GDE RF parameters.
 *
 * @param	rdbuf -params save buffer
 *
 * @return	none
 */
void ReadRFParam(uint8 * rdbuf)
{
	rdbuf[ST_RF_WK_FREQ_POS] = RFwkfrq;
	rdbuf[ST_RF_ST_FREQ_POS] = RFstfrq;
	rdbuf[ST_RF_UPGD_FREQ_POS] = RFupgfrq;
	rdbuf[ST_RF_AIR_BAUD_POS] = RFairbaud;
	rdbuf[ST_RF_PWR_LVL_POS] = RFpwr;
}


/*********************************************************************
 * @fn		SetRFParam
 *
 * @brief	Set GDE RF parameters.
 *
 * @param	wkfreq - working frequency
 * @param	stfreq - setup frequency
 * @param	upgdfreq - upgrade frequency
 * @param	baud - air baud rate
 * @param	pwlvl - send power level
 *
 * @return	TRUE - setup result OK
 */
bool SetRFParam(uint8 wkfreq, uint8 setfreq, uint8 upgdfreq, uint8 baud, uint8 pwlvl)
{
	if (CHECK_FREQ_VALID(wkfreq) && CHECK_FREQ_VALID(setfreq) && CHECK_FREQ_VALID(upgdfreq))
	{
		if ( CHECK_BAUD_VALID(baud))
		{
			if (CHECK_PWR_VALID(pwlvl))
			{
				RFwkfrq = wkfreq;
				RFstfrq = setfreq;
				RFupgfrq = upgdfreq;
				RFairbaud = baud;
				RFpwr = pwlvl;

				return TRUE;
			}
		}
	}

	return FALSE;
}

/*********************************************************************
 * @fn		GetCurFreq
 *
 * @brief	Get appropriate RF frequency of current state
 *
 * @param	state - current state of device.
 *
 * @return	none.
 */
uint8 GetCurFreq(sysstate_t state)
{
	uint8 freqsel = 0;
	
	switch (state)
	{
		case SYS_BOOTUP:
			freqsel = 0;	// always use default (option 0) for bootup
			break;
		case SYS_WORKING:
			freqsel = RFwkfrq-1;
			break;
		case SYS_SETUP:
			freqsel = RFstfrq-1;
			break;
		case SYS_UPGRADE:
			freqsel = RFupgfrq-1;
			break;
		case SYS_SLEEPING:
		case SYS_DORMANT:
		default:
			break;
	}
	
	return freqsel;
}

#if ( defined HAL_SPI_MASTER )

void trxSingleTX(uint8 data)
{
	uint8 rc = data;
	Com433Write(COM433_WORKING_PORT, &rc, sizeof(rc));
}

uint8 trxSingleRX(void)
{
	uint8 rc=0;
//	while(Hal_UART_RxBufLen(COM433_WORKING_PORT) == 0);
	Com433Read(COM433_WORKING_PORT, &rc, sizeof(rc));
	return rc;
}

void SetRTxRdyFlg(bool flag)
{
	rtxrdyflag = flag;
}

bool GetRTxRdyFlg()
{
	return rtxrdyflag;
}

HAL_ISR_FUNCTION(RF_RTX_RDY_Isr, P1INT_VECTOR)
{
	HAL_ENTER_ISR();

	if (RF_SYNC_INT_IEN & RF_SYNC_INT_IE)
	{
		rtxrdyflag = TRUE;
		osal_set_event(BLECore_TaskId, RF_DATA_PROC_EVT);
	}

	// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
	RF_SYNC_INT_IFG = 0;
	RF_SYNC_INT_IF = 0;

	CLEAR_SLEEP_MODE();
	
	HAL_EXIT_ISR();
}
#endif	// HAL_SPI_MASTER

