/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "Com433.h"
#include "Cc112x.h"
#include "BLECore.h"
#include "Pktfmt.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define PKTLEN				64
#define RX_FIFO_ERROR		0x11

/**********TEN308 RF GPIO define**********/
#if ( !defined USE_CC112X_RF )
#define TEN308_DIOA_PINSEL			P0SEL
#define TEN308_DIOA_PINDIR			P0DIR
#define TEN308_DIOA_PININP			P0INP
#define TEN308_DIOA_GPIO			BV(6)
#define TEN308_DIOA_PIN				P0_6

#define TEN308_DIOB_PINSEL			P1SEL
#define TEN308_DIOB_PINDIR			P1DIR
#define TEN308_DIOB_PININP			P1INP
#define TEN308_DIOB_GPIO			BV(2)
#define TEN308_DIOB_PIN				P1_2
#endif	// !USE_CC112X_RF


//modified by lan 15.8.3, control by app
#define SPI_CSN_PXDIR				P1DIR
#define SPI_CSN_PIN					P1_4
#define SPI_CSN_BIT					BV(4)

#define SPI_MISO_PIN				P1_7

#define TRXEN_SPI_END()				st(asm("NOP"); SPI_CSN_PIN = 1;)
#define TRXEN_SPI_BEGIN()			st(SPI_CSN_PIN = 0; asm("NOP");)

// CC112X GPIO2 INT -> P1.3
#define RF_SYNC_INT_PINSEL			P1SEL
#define RF_SYNC_INT_PINDIR			P1DIR
#define RF_SYNC_INT_IFG				P1IFG
#define RF_SYNC_INT_IF				P1IF
#define RF_SYNC_INT_IEN				P1IEN

#define RF_SYNC_INT_IE				BV(3)

#define RF_SYNC_INT_ENABLE()		st(RF_SYNC_INT_IEN |= RF_SYNC_INT_IE;)
#define RF_SYNC_INT_DISABLE()		st(RF_SYNC_INT_IEN &= ~RF_SYNC_INT_IE;)

#define WAIT_SO_STABLE()			st(while(SPI_MISO_PIN == 1);)

enum RFworkingmode{
	RF_IDLE_M,
	RF_TX_M,
	RF_RX_M
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Default RF frequency params
// TEN308: work 433 set 433 upgrade 470
// CC1120: work 428 set 433 upgrade 470

// Default RF air baud rate
// CC1120: 1200
// TEN308 : 9600
#if ( defined USE_CC112X_RF )
uint8 RFwkfrq = 0x01,RFstfrq = 0x07,RFupgfrq = 0x0C;
uint8 RFairbaud = 0x02;
#else
uint8 RFwkfrq = 0x01,RFstfrq = 0x01,RFupgfrq = 0x02;
uint8 RFairbaud = 0x05;
#endif	// USE_CC112X_RF


/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 BLECore_TaskId;

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
#if ( defined USE_CC112X_RF )

// Receive or send data ready flag, controled by interrupt
static bool rtxrdyflag=FALSE; 

#else	// !USE_CC112X_RF

static bool RFworkingflag=FALSE; 

// Set RF working params command buffer and length, separate with data send buffer
static uint8 stcmdbuf[32];
static uint8 stcmdlen;

#endif	// USE_CC112X_RF


// Default RF air baud rate
// TEN308: 20dBm
// CC1120: 15dBm
static uint8 RFpwr = 0x08;	// Max level


// TEN & CC112X RF state
static rfstate_t rfst=RF_PRESET;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void InitRFCfg(sysstate_t state);

static void registerConfig(sysstate_t state);

static void RFsleep(void);

#if ( defined USE_CC112X_RF )
static void RxData(void);

static void CC112XRFSleep(void);
/* basic set of access functions */
static rfStatus_t CC112XSpiReadReg(uint16 addr, uint8 *data, uint8 len);
static rfStatus_t CC112XSpiWriteReg(uint16 addr, uint8 *data, uint8 len);

static rfStatus_t CC112XSpiWriteTxFifo(uint8 *pWriteData, uint8 len);
static rfStatus_t CC112XSpiReadRxFifo(uint8 *pReadData, uint8 len);

static rfStatus_t CC112XGetTxStatus(void);
static rfStatus_t CC112XGetRxStatus(void);
static int8 Read8BitRssi(void);

static void trxSyncIntCfg(void);

static rfStatus_t trxSpiCmdStrobe(uint8 cmd);

static rfStatus_t trx8BitRegAccess(uint8 accessType, uint8 addrByte, uint8 *pData, uint16 len);
static rfStatus_t trx16BitRegAccess(uint8 accessType, uint8 extAddr, uint8 regAddr, uint8 *pData, uint8 len);
static void trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);

static void trxSingleTX(uint8 data);
static uint8 trxSingleRX(void);

static void manualCalibration(void);

#else	// !USE_CC112X_RF

static void RFwakeup(void);

static void TENRFWakeup(void);
static void TENRFSleep(void);

static uint8 FromTENCmd(bool wrDir, uint8 *wrbuf, uint8 freq);


#endif	// USE_CC112X_RF
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

void RF_working(uint8 task_id, rfstate_t newrfstate)
{
	rfst = newrfstate;

#if ( !defined USE_CC112X_RF )
	uint32 lasttmout;

	// Wait untill last RF work finish in case of reentry
	lasttmout = osal_get_timeoutEx(task_id, RF_DATA_PROC_EVT);
	if (lasttmout != 0)
	{
		osal_start_timerEx(task_id, RF_DATA_PROC_EVT, lasttmout);
		return;
	}
#endif	// !USE_CC112X_RF

	switch (rfst)
	{
		case RF_PRESET:
		{
#if ( defined USE_CC112X_RF )
			trxSpiCmdStrobe(CC112X_SRES);	// Reset radio
#else	// !USE_CC112X_RF
			RFwakeup();
#endif	// USE_CC112X_RF

			rfst = RF_BEG_SET;
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT,WAIT_RF_START_PERIOD);
			break;
		}
		case RF_BEG_SET:
		{
			InitRFCfg(GetSysState());

#if ( defined USE_CC112X_RF )
			rfst = RF_RECV;	// Enter receive mode after set
			trxSpiCmdStrobe(CC112X_SRX);
#else	// !USE_CC112X_RF
			if ( stcmdlen > 0 )
			{
				Com433Write(COM433_WORKING_PORT, stcmdbuf, stcmdlen);
				stcmdlen = 0;
			}
			rfst = RF_WAIT_RESET;
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT, WAIT_TEN_CMD_PERIOD);
#endif	// USE_CC112X_RF

			break;
		}
#if ( defined USE_CC112X_RF )
		case RF_SEND:	// This state changed by interrupt
		{
			if (rtxrdyflag == TRUE)	// In case SYS_WORKING reentry
			{
				rtxrdyflag = FALSE;
				rfst = RF_RECV;
				trxSpiCmdStrobe(CC112X_SRX);
			}
			break;
		}
		case RF_RECV:
		{
			if (rtxrdyflag == TRUE)	// In case SYS_WORKING reentry
			{
				rtxrdyflag = FALSE;
				RxData();
			}
			break;
		}
#else	// !USE_CC112X_RF
		case RF_WAIT_RESET:
		{
			RFsleep();
			rfst = RF_WAKEUP;
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT, WAIT_TEN_STOP_PERIOD);
			break;
		}
		case RF_WAKEUP:
		{
			RFwakeup();
			rfst = RF_WORK;
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
#endif	// USE_CC112X_RF

		case RF_SLEEP:
			RFsleep();
			break;
		default:
			break;
	}

	return;
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


#if ( defined USE_CC112X_RF )

/*********************************************************************
 * @fn		TxData
 *
 * @brief	RF send data.
 *
 * @param	txbuf - RF send buf.
 * @param	len - send length.
 *
 * @return	none
 */
void TxData(uint8 *txbuf, uint8 len)
{
	uint8 txBuffer[GMS_PKT_MAX_LEN] = {0};

	txBuffer[0] = len+1;
	txBuffer[1] = 0x01;	// address
	osal_memcpy(txBuffer+2, txbuf, len);

	// Write packet to TX FIFO
	CC112XSpiWriteTxFifo(txBuffer, len+2);
	// Strobe TX to send packet
	trxSpiCmdStrobe(CC112X_STX);
}

#endif	// USE_CC112X_RF


/*********************************************************************
* Private functions
*/

static void InitRFCfg(sysstate_t state)
{
	registerConfig(state);

#if ( defined USE_CC112X_RF )
	uint8 rfvern;
	
	CC112XSpiReadReg(CC112X_PARTNUMBER, &rfvern, 1);
	//Com433WriteInt(COM433_DEBUG_PORT, "\r\nDEV", rfvern, 16);

	// Chip ID is not CC1120
	if ( rfvern != 0x48 )
		return;

	CC112XSpiReadReg(CC112X_PARTVERSION, &rfvern, 1);
	//Com433WriteInt(COM433_DEBUG_PORT, "\r\nVERN", rfvern, 16);

	// CC1120 version is 0x21, need calibration
	if ( rfvern == 0x21 )
		manualCalibration();
	else
		trxSpiCmdStrobe(CC112X_SCAL);
	
	trxSyncIntCfg();

#endif	// USE_CC112X_RF
}

/*********************************************************************
 * @fn		registerConfig
 *
 * @brief	write config register config based on current state
 *
 * @param	state - current state of device.
 *
 * @return	none.
 */
static void registerConfig(sysstate_t state)
{
	uint8 freqsel;

	switch (state)
	{
		case SYS_BOOTUP:
			freqsel = 0;	// always use 433 for bootup
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
			RFsleep();
			return;
	}

#if ( defined USE_CC112X_RF )
	uint8 writeByte;
	uint16 i;

	// Write registers to radio
	for( i=0;i<(sizeof(rfRadioCfgCm)/sizeof(registerSetting_t));i++)
	{
		writeByte = rfRadioCfgCm[i].data;
		CC112XSpiWriteReg(rfRadioCfgCm[i].addr, &writeByte, 1);
	}

	for( i=0;i<(sizeof(rfFreqCfgSp)/sizeof(rfFreqCfgSp[0]));i++)
	{
		writeByte = rfFreqCfgSp[i][freqsel];
		CC112XSpiWriteReg(rfFreqCfgSp[i][12], &writeByte, 1);
	}

	for( i=0;i<(sizeof(rfRadioCfgSp)/sizeof(rfRadioCfgSp[0]));i++)
	{
		writeByte = rfRadioCfgSp[i][RFairbaud-1];
		CC112XSpiWriteReg(rfRadioCfgSp[i][10], &writeByte, 1);
	}

	for( i=0;i<(sizeof(rfPowerCfgSp)/sizeof(rfPowerCfgSp[0]));i++)
	{
		writeByte = rfPowerCfgSp[i][RFpwr-1];
		CC112XSpiWriteReg(rfPowerCfgSp[i][8], &writeByte, 1);
	}

#else

	stcmdlen = FromTENCmd(TRUE, stcmdbuf, freqsel);
#endif	// !defined USE_CC112X_RF
}


static void RFsleep(void)
{
#if ( defined USE_CC112X_RF )
	CC112XRFSleep();
#else
	if (RFworkingflag == FALSE)
		return;

	RFworkingflag = FALSE;
	TENRFSleep();
#endif	// USE_CC112X_RF
}

#if ( defined USE_CC112X_RF )

/*********************************************************************
 * @fn		RxData
 *
 * @brief	RF recieve data.
 *
 * @param	none
 *
 * @return	none
 */
static void RxData(void)
{
	uint8 rxBuffer[GMS_PKT_MAX_LEN] = {0};
	uint8 rxBytes;
	uint8 marcState;
	
	CC112XSpiReadReg(CC112X_NUM_RXBYTES, &rxBytes, 1);
	// Check that we have bytes in FIFO
	if(rxBytes != 0)
	{
		// Read MARCSTATE to check for RX FIFO error
		CC112XSpiReadReg(CC112X_MARCSTATE, &marcState, 1);

		// Mask out MARCSTATE bits and check if we have a RX FIFO error
		if((marcState & 0x1F) == RX_FIFO_ERROR)
		{
			Com433WriteStr(COM433_DEBUG_PORT, "\r\nERR");
			// Flush RX FIFO
			trxSpiCmdStrobe(CC112X_SFRX);
		}
		else
		{
			// Read n bytes from RX FIFO
			CC112XSpiReadRxFifo(rxBuffer, rxBytes);
			//Com433WriteInt(COM433_DEBUG_PORT, "\r\nR:", rxBytes,10);
			if(rxBuffer[rxBytes-1] & 0x80)
			{
				Com433WriteInt(COM433_DEBUG_PORT," R:",Read8BitRssi(),10);
				//Com433WriteInt(COM433_DEBUG_PORT," R2:",rxBuffer[rxBytes-2],10);
				//rxBuffer[0]=' ';
				//Com433Write(COM433_DEBUG_PORT, rxBuffer, rxBytes-2);
				GMSPktForm(rxBuffer+2, rxBytes-2-2);
			}
			else
				Com433WriteStr(COM433_DEBUG_PORT,"CRC Fail");
		}
	}
	trxSpiCmdStrobe(CC112X_SRX);
}


/*********************************************************************
 * @fn		CC112XRFSleep
 *
 * @brief	CC1120 enter sleep
 *
 * @param	none.
 *
 * @return	none.
 */
static void CC112XRFSleep(void)
{
	trxSpiCmdStrobe(CC112X_SIDLE);
	trxSpiCmdStrobe(CC112X_SPWD);
}

/******************************************************************************
 * @fn					CC112XSpiReadReg
 *
 * @brief			 Read value(s) from config/status/extended radio register(s).
 *							If len	= 1: Reads a single register
 *							if len != 1: Reads len register values in burst mode 
 *
 * input parameters
 *
 * @param			 addr	 - address of first register to read
 * @param			 *pData - pointer to data array where read bytes are saved
 * @param			 len	 - number of bytes to read
 *
 * output parameters
 *
 * @return			rfStatus_t
 */
rfStatus_t CC112XSpiReadReg(uint16 addr, uint8 *pData, uint8 len)
{
	uint8 tempExt	= (uint8)(addr>>8);
	uint8 tempAddr = (uint8)(addr & 0x00FF);
	uint8 rc;
	
	/* Checking if this is a FIFO access -> returns chip not ready	*/
	if((CC112X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0)) return STATUS_CHIP_RDYn_BM;
	
	/* Decide what register space is accessed */
	if(!tempExt)
	{
		rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempAddr,pData,len);
	}
	else if (tempExt == 0x2F)
	{
		rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempExt,tempAddr,pData,len);
	}
	return (rc);
}

/******************************************************************************
 * @fn					CC112XSpiWriteReg
 *
 * @brief			 Write value(s) to config/status/extended radio register(s).
 *							If len	= 1: Writes a single register
 *							if len	> 1: Writes len register values in burst mode 
 *
 * input parameters
 *
 * @param			 addr	 - address of first register to write
 * @param			 *pData - pointer to data array that holds bytes to be written
 * @param			 len		- number of bytes to write
 *
 * output parameters
 *
 * @return			rfStatus_t
 */
rfStatus_t CC112XSpiWriteReg(uint16 addr, uint8 *pData, uint8 len)
{
	uint8 tempExt	= (uint8)(addr>>8);
	uint8 tempAddr = (uint8)(addr & 0x00FF);
	uint8 rc;
	
	/* Checking if this is a FIFO access - returns chip not ready */
	if((CC112X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0))
			return STATUS_CHIP_RDYn_BM;
			
	/* Decide what register space is accessed */	
	if(!tempExt)
	{
		rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempAddr,pData,len);
	}
	else if (tempExt == 0x2F)
	{
		rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempExt,tempAddr,pData,len);
	}
	return (rc);
}

/*******************************************************************************
 * @fn					CC112XSpiWriteTxFifo
 *
 * @brief			 Write pData to radio transmit FIFO.
 *
 * input parameters
 *
 * @param			 *pData - pointer to data array that is written to TX FIFO
 * @param			 len		- Length of data array to be written
 *
 * output parameters
 *
 * @return			rfStatus_t
 */
rfStatus_t CC112XSpiWriteTxFifo(uint8 *pData, uint8 len)
{
	uint8 rc;
	rc = trx8BitRegAccess(0x00,CC112X_BURST_TXFIFO, pData, len);
	return (rc);
}

/*******************************************************************************
 * @fn			CC112XSpiReadRxFifo
 *
 * @brief		Reads RX FIFO values to pData array
 *
 * input parameters
 *
 * @param		*pData - pointer to data array where RX FIFO bytes are saved
 * @param		len		- number of bytes to read from the RX FIFO
 *
 * output parameters
 *
 * @return		rfStatus_t
 */
rfStatus_t CC112XSpiReadRxFifo(uint8 * pData, uint8 len)
{
	uint8 rc;
	rc = trx8BitRegAccess(0x00,CC112X_BURST_RXFIFO, pData, len);
	return (rc);
}

/******************************************************************************
 * @fn			CC112XGetTxStatus(void)
 *					
 * @brief	 	This function transmits a No Operation Strobe (SNOP) to get the 
 *				status of the radio and the number of free bytes in the TX FIFO.
 *					
 *				Status byte:
 *					
 *				---------------------------------------------------------------------------
 *				|					|						|																								 |
 *				| CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
 *				|					|						|																								 |
 *				---------------------------------------------------------------------------
 *
 *
 * input parameters
 *
 * @param	 none
 *
 * output parameters
 *				 
 * @return	rfStatus_t 
 *
 */
rfStatus_t CC112XGetTxStatus(void)
{
	return(trxSpiCmdStrobe(CC112X_SNOP));
}

/******************************************************************************
 *
 *	@fn			 CC112XGetRxStatus(void)
 *
 *	@brief	 
 *				This function transmits a No Operation Strobe (SNOP) with the 
 *				read bit set to get the status of the radio and the number of 
 *				available bytes in the RXFIFO.
 *						
 *				Status byte:
 *						
 *				--------------------------------------------------------------------------------
 *				|					|						|																											|
 *				| CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO |
 *				|					|						|																											|
 *				--------------------------------------------------------------------------------
 *
 *
 * input parameters
 *
 * @param		none
 *
 * output parameters
 *				 
 * @return		rfStatus_t 
 *
 */
rfStatus_t CC112XGetRxStatus(void)
{
	return(trxSpiCmdStrobe(CC112X_SNOP | RADIO_READ_ACCESS));
}


static int8 Read8BitRssi(void)
{
	uint8 rssi2compl,rssiValid;
	uint8 rssiOffset = 102;
	int8 rssiConverted;

	// Read RSSI_VALID from RSSI0
	CC112XSpiReadReg(CC112X_RSSI0, &rssiValid, 1);
	// Check if the RSSI_VALID flag is set
	if(rssiValid & 0x01)
	{
		// Read RSSI from MSB register
		CC112XSpiReadReg(CC112X_RSSI1, &rssi2compl, 1);
		rssiConverted = (int8)rssi2compl - rssiOffset;
		return rssiConverted;
	}

	// return 0 since new value is not valid
	return 0; 
}

static void trxSyncIntCfg(void)
{
	SET_P1_INT_DISABLE();
	RF_SYNC_INT_DISABLE();

	// Falling edge of P1	
	SET_P1_INT_FALLING_EDGE();
	
	RF_SYNC_INT_PINSEL &= (uint8) ~ RF_SYNC_INT_IE;
	RF_SYNC_INT_PINDIR &= (uint8) ~ RF_SYNC_INT_IE;

	RF_SYNC_INT_IFG = 0;
	RF_SYNC_INT_IF = 0;

	RF_SYNC_INT_ENABLE();
	// P1 Int enable
	SET_P1_INT_ENABLE();
}


/*******************************************************************************
 * @fn					trxSpiCmdStrobe
 *
 * @brief			 Send command strobe to the radio. Returns status byte read
 *							during transfer of command strobe. Validation of provided
 *							is not done. Function assumes chip is ready.
 *
 * input parameters
 *
 * @param			 cmd - command strobe
 *
 * output parameters
 *
 * @return			status byte
 */
static rfStatus_t trxSpiCmdStrobe(uint8 cmd)
{
	uint8 rc=0;
	TRXEN_SPI_BEGIN();
	WAIT_SO_STABLE();
	trxSingleTX(cmd);
	
	rc = trxSingleRX();
	TRXEN_SPI_END();
	
	return (rc);
}

/*******************************************************************************
 * @fn					trx8BitRegAccess
 *
 * @brief			 This function performs a read or write from/to a 8bit register
 *							address space. The function handles burst and single read/write
 *							as specfied in addrByte. Function assumes that chip is ready.
 *
 * input parameters
 *
 * @param			 accessType - Specifies if this is a read or write and if it's
 *													 a single or burst access. Bitmask made up of
 *													 RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *													 RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param			 addrByte - address byte of register.
 * @param			 pData		- data array
 * @param			 len			- Length of array to be read(TX)/written(RX)
 *
 * output parameters
 *
 * @return			chip status
 */
static rfStatus_t trx8BitRegAccess(uint8 accessType, uint8 addrByte, uint8 *pData, uint16 len)
{
	uint8 readValue;

	/* Pull CS_N low and wait for SO to go low before communication starts */
	TRXEN_SPI_BEGIN();
	WAIT_SO_STABLE();
	/* send register address byte */
	trxSingleTX(accessType|addrByte);
	/* Storing chip status */
	readValue = trxSingleRX();

	trxReadWriteBurstSingle(accessType|addrByte,pData,len);
	TRXEN_SPI_END();

	/* return the status byte value */
	return(readValue);
}

/******************************************************************************
 * @fn					trx16BitRegAccess
 *
 * @brief			 This function performs a read or write in the extended adress
 *							space of CC112X.
 *
 * input parameters
 *
 * @param			 accessType - Specifies if this is a read or write and if it's
 *													 a single or burst access. Bitmask made up of
 *													 RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *													 RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param			 extAddr - Extended register space address = 0x2F.
 * @param			 regAddr - Register address in the extended address space.
 * @param			 *pData	- Pointer to data array for communication
 * @param			 len		 - Length of bytes to be read/written from/to radio
 *
 * output parameters
 *
 * @return			rfStatus_t
 */
static rfStatus_t trx16BitRegAccess(uint8 accessType, uint8 extAddr, uint8 regAddr, uint8 *pData, uint8 len)
{
	uint8 readValue;
	
	TRXEN_SPI_BEGIN();
	WAIT_SO_STABLE();
	/* send extended address byte with access type bits set */
	trxSingleTX(accessType|extAddr);
	/* Storing chip status */
	readValue = trxSingleRX();

	trxSingleTX(regAddr);
	/* Communicate len number of bytes */
//	readValue = trxSingleRX();
	trxReadWriteBurstSingle(accessType|extAddr,pData,len);
	TRXEN_SPI_END();

	/* return the status byte value */
	return(readValue);
}


/*******************************************************************************
 * @fn					trxReadWriteBurstSingle
 *
 * @brief			 When the address byte is sent to the SPI slave, the next byte
 *							communicated is the data to be written or read. The address
 *							byte that holds information about read/write -and single/
 *							burst-access is provided to this function.
 *
 *							Depending on these two bits this function will write len bytes to
 *							the radio in burst mode or read len bytes from the radio in burst
 *							mode if the burst bit is set. If the burst bit is not set, only
 *							one data byte is communicated.
 *
 *							NOTE: This function is used in the following way:
 *
 *							TRXEM_SPI_BEGIN();
 *							while(TRXEM_PORT_IN & TRXEM_SPI_MISO_PIN);
 *							...[Depending on type of register access]
 *							trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);
 *							TRXEM_SPI_END();
 *
 * input parameters
 *
 * @param			 none
 *
 * output parameters
 *
 * @return			void
 */
static void trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len)
{
	uint16 i;
	/* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
	if(addr&RADIO_READ_ACCESS)
	{
		if(addr&RADIO_BURST_ACCESS)
		{
			for (i = 0; i < len; i++)
			{
				trxSingleTX(0);			/* Possible to combining read and write as one access type */
				*pData = trxSingleRX();	/* Store pData from last pData RX */
				pData++;
			}
		}
		else
		{
			trxSingleTX(0);
			*pData = trxSingleRX();
		}
	}
	else
	{
		if(addr&RADIO_BURST_ACCESS)
		{
			/* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
			for (i = 0; i < len; i++)
			{
				trxSingleTX(*pData);
				pData++;
			}
		}
		else
		{
			trxSingleTX(*pData);
		}
	}
	return;
}

static void trxSingleTX(uint8 data)
{
	uint8 rc = data;
	Com433Write(COM433_WORKING_PORT, &rc, sizeof(rc));
}

static uint8 trxSingleRX(void)
{
	uint8 rc=0;
//	while(Hal_UART_RxBufLen(COM433_WORKING_PORT) == 0);
	Com433Read(COM433_WORKING_PORT, &rc, sizeof(rc));
	return rc;
}


/*******************************************************************************
*   @fn		manualCalibration
*
*   @brief	Calibrates radio according to CC112x errata
*
*   @param	none
*
*   @return	none
*/
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
static void manualCalibration(void)
{
	uint8 original_fs_cal2;
	uint8 calResults_for_vcdac_start_high[3];
	uint8 calResults_for_vcdac_start_mid[3];
	uint8 marcstate;
	uint8 writeByte;

	// 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	CC112XSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

	// 2) Start with high VCDAC (original VCDAC_START + 2):
	CC112XSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
	writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
	CC112XSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

	// 3) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	trxSpiCmdStrobe(CC112X_SCAL);

	do
	{
		CC112XSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
	} while (marcstate != 0x41);

	// 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
	//	high VCDAC_START value
	CC112XSpiReadReg(CC112X_FS_VCO2,
					 &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
	CC112XSpiReadReg(CC112X_FS_VCO4,
					 &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
	CC112XSpiReadReg(CC112X_FS_CHP,
					 &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

	// 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	CC112XSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

	// 6) Continue with mid VCDAC (original VCDAC_START):
	writeByte = original_fs_cal2;
	CC112XSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

	// 7) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	trxSpiCmdStrobe(CC112X_SCAL);

	do
	{
		CC112XSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
	} while (marcstate != 0x41);

	// 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
	//	with mid VCDAC_START value
	CC112XSpiReadReg(CC112X_FS_VCO2,
					 &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
	CC112XSpiReadReg(CC112X_FS_VCO4,
					 &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
	CC112XSpiReadReg(CC112X_FS_CHP,
					 &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

	// 9) Write back highest FS_VCO2 and corresponding FS_VCO
	//	and FS_CHP result
	if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
		calResults_for_vcdac_start_mid[FS_VCO2_INDEX])
	{
		writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
		CC112XSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
		CC112XSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
		CC112XSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
	}
	else
	{
		writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
		CC112XSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
		CC112XSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
		CC112XSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
	}
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

#else	// !USE_CC112X_RF

static void RFwakeup(void)
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

#endif	// USE_CC112X_RF
