/*********************************************************************
 * INCLUDES
 */
#include "RFProc.h"

#if ( defined USE_SX1278_RF )
#include "Sx1278.h"
#elif ( defined USE_CC112X_RF )
#include "Cc112x.h"
#elif ( defined USE_TEN308_RF )
#include "Ten308.h"
#else
	#error "Not defined RF!"
#endif


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
extern uint8 RFwkfrq, RFstfrq, RFupgfrq, RFairbaud, RFpwr;
 ;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// RF init state
static rfstate_t rfst=RF_PRESET;

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
	rdbuf[ST_RF_FREQ_POS] = RFwkfrq;
	rdbuf[ST_RF_ST_FREQ_POS] = RFstfrq;
	rdbuf[ST_RF_UPGD_FREQ_POS] = RFupgfrq;
	rdbuf[ST_RF_BAUD_POS] = RFairbaud;
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
			freqsel = 0;	// always use 428 for bootup
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
#endif	// HAL_SPI_MASTER

