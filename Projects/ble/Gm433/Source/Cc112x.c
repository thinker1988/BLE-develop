#include "OSAL.h"
#include "Com433.h"
#include "Cc112x.h"
#include "BLECore.h"
#include "Pktfmt.h"

#define PKTLEN				64
#define RX_FIFO_ERROR		0x11

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


extern uint8 BLECore_TaskId;


static uint8 RFmode = RF_IDLE_M;

static bool RFrxtxRdy;

static void trxSyncIntCfg(void);

static void registerConfig(void);

static rfStatus_t trxSpiCmdStrobe(uint8 cmd);

static rfStatus_t trx8BitRegAccess(uint8 accessType, uint8 addrByte, uint8 *pData, uint16 len);
static rfStatus_t trx16BitRegAccess(uint8 accessType, uint8 extAddr, uint8 regAddr, uint8 *pData, uint8 len);
static void trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);

static int8 Read8BitRssi(void);

static void trxSingleTX(uint8 data);
static uint8 trxSingleRX(void);

static void manualCalibration(void);


void initRFcfg(void)
{
	uint8 rfvern;
	
	registerConfig();
	cc112xSpiReadReg(CC112X_PARTNUMBER, &rfvern, 1);
	Com433WriteInt(COM433_DEBUG_PORT, "\r\nDEV", rfvern, 16);
	cc112xSpiReadReg(CC112X_PARTVERSION, &rfvern, 1);
	Com433WriteInt(COM433_DEBUG_PORT, "\r\nVERN", rfvern, 16);

	if ( rfvern == 0x21 )
		manualCalibration();
	else
		trxSpiCmdStrobe(CC112X_SCAL);
	
	trxSyncIntCfg();

	// enter low power mode
	//RFentersleep();
//	Com433WriteStr(COM433_DEBUG_PORT,"\r\nOK!");
}

void RFentersleep(void)
{
	RFmode = RF_IDLE_M;
	trxSpiCmdStrobe(CC112X_SIDLE);
}

void RFmodechange(void)
{
	if ( RFmode == RF_TX_M )
	{
//		RFentersleep();
		// begin to RX after TX over
		RFmode = RF_RX_M;
		trxSpiCmdStrobe(CC112X_SRX);

		// leave 1000 ms recieve
		osal_start_timerEx(BLECore_TaskId, CORE_PWR_SAVING_EVT, IDLE_PWR_HOLD_PERIOD);
	}
	else if ( RFmode == RF_RX_M )
		rxdata();
}
void txdata(uint8 *txbuf, uint8 len)
{
#if 1
	uint8 txBuffer[GMS_PKT_MAX_LEN] = {0};

	txBuffer[0] = len;
	osal_memcpy(txBuffer+1, txbuf, len);
	
	RFmode = RF_TX_M;
	// Write packet to TX FIFO
	cc112xSpiWriteTxFifo(txBuffer, len+1);
	// Strobe TX to send packet
	trxSpiCmdStrobe(CC112X_STX);
#else
	tx_test();
#endif
}

void rxdata(void)
{
	uint8 rxBuffer[GMS_PKT_MAX_LEN] = {0};
	uint8 rxBytes;
	uint8 marcState;
	
	cc112xSpiReadReg(CC112X_NUM_RXBYTES, &rxBytes, 1);
	// Check that we have bytes in FIFO
	if(rxBytes != 0)
	{
		// Read MARCSTATE to check for RX FIFO error
		cc112xSpiReadReg(CC112X_MARCSTATE, &marcState, 1);

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
			cc112xSpiReadRxFifo(rxBuffer, rxBytes);
			Com433WriteInt(COM433_DEBUG_PORT, "\r\nR:", rxBytes,10);
			if(rxBuffer[rxBytes - 1] & 0x80)
			{
				rxBuffer[0]=' ';
				Com433Write(COM433_DEBUG_PORT, rxBuffer, rxBytes-2);
			}
			else
				Com433WriteStr(COM433_DEBUG_PORT,"CRC Fail");
		}
	}
	trxSpiCmdStrobe(CC112X_SRX);
}

void tx_test(void)
{
#if ( defined USE_CC112X_RF )
	uint8 txBuffer[PKTLEN+1] = {0};

	createPacket(txBuffer);
	// Write packet to TX FIFO
	cc112xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));
	Com433WriteInt(COM433_DEBUG_PORT,"\r\nT:",PKTLEN+1,10);
	Com433WriteInt(COM433_DEBUG_PORT," V:",txBuffer[1]-'0',10);
	// Strobe TX to send packet
	trxSpiCmdStrobe(CC112X_STX);

	while(RFrxtxRdy == FALSE);
	RFrxtxRdy = FALSE;
#else
	uint8 txBuffer[PKTLEN+1]="123456789012345678901234567890123456789012345678901234567890";
	Com433WriteStr(COM433_WORKING_PORT, txBuffer);
#endif
}

void rx_test(void)
{
	uint8 rxBuffer[128] = {0};
	uint8 rxBytes;
	uint8 marcState;
	
	trxSpiCmdStrobe(CC112X_SRX);

	while(1)
	{
		if (RFrxtxRdy == FALSE)
			continue;
	
		// Read number of bytes in RX FIFO
		cc112xSpiReadReg(CC112X_NUM_RXBYTES, &rxBytes, 1);
		// Check that we have bytes in FIFO
		if(rxBytes != 0)
		{
			// Read MARCSTATE to check for RX FIFO error
			cc112xSpiReadReg(CC112X_MARCSTATE, &marcState, 1);

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
				cc112xSpiReadRxFifo(rxBuffer, rxBytes);
				Com433WriteInt(COM433_DEBUG_PORT, "\r\nR:", rxBytes,10);
				if(rxBuffer[rxBytes - 1] & 0x80)
				{
					Com433WriteInt(COM433_DEBUG_PORT," ",Read8BitRssi(),10);
//					rxBuffer[0]=' ';
					Com433WriteStr(COM433_DEBUG_PORT," ");
					Com433Write(COM433_DEBUG_PORT, rxBuffer, rxBytes-2);			
				}
				else
					Com433WriteStr(COM433_DEBUG_PORT,"CRC Fail");
				// Check CRC ok (CRC_OK: bit7 in second status byte)
				// This assumes status bytes are appended in RX_FIFO
				// (PKT_CFG1.APPEND_STATUS = 1)
				// If CRC is disabled the CRC_OK field will read 1
			
				//if(rxBuffer[rxBytes - 1] & 0x80)
				// Update packet counter
				//Com433Write(COM433_DEBUG_PORT, rxBuffer, rxBytes-1);
			}
		}

		RFrxtxRdy = FALSE;
		trxSpiCmdStrobe(CC112X_SRX);
	}
}
#if ( defined TEST_433 )
static void createPacket(uint8 txBuffer[]);

static void createPacket(uint8 txBuffer[])
{
	txBuffer[0] = PKTLEN; // Length byte
	static uint8 val='0';
	
	// Fill rest of buffer with random bytes
	for(uint8 i = 1; i < (PKTLEN + 1); i++)
		txBuffer[i] = val;
	if (++val > '9')
		val= '0';
}
#endif

void trxSyncIntCfg(void)
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

static void registerConfig(void)
{
	uint8 writeByte;
	uint16 i;
	
	// Reset radio
	trxSpiCmdStrobe(CC112X_SRES);

	// Write registers to radio
#if defined CC1120
	for( i=0;i<(sizeof(rfRadioCfgCm)/sizeof(registerSetting_t));i++)
	{
		writeByte = rfRadioCfgCm[i].data;
		cc112xSpiWriteReg(rfRadioCfgCm[i].addr, &writeByte, 1);
	}

	for( i=0;i<(sizeof(rfRadioCfgSp)/sizeof(rfRadioCfgSp[0]));i++)
	{
		writeByte = rfRadioCfgSp[i][0];
		cc112xSpiWriteReg(rfRadioCfgSp[i][10], &writeByte, 1);
	}
#else
#if 1
	for( i=0;i<(sizeof(preferredSettings434)/sizeof(registerSetting_t));i++)
	{
		writeByte = preferredSettings434[i].data;
		cc112xSpiWriteReg(preferredSettings434[i].addr, &writeByte, 1);
	}
#else
	for( i=0;i<(sizeof(preferredSettings470)/sizeof(registerSetting_t));i++)
	{
		writeByte = preferredSettings470[i].data;
		cc112xSpiWriteReg(preferredSettings470[i].addr, &writeByte, 1);
	}
#if 0
	for ( i=0;i<(sizeof(br_1200_cfg)/sizeof(registerSetting_t));i++)
	{
		writeByte = br_1200_cfg[i].data;
		cc112xSpiWriteReg(br_1200_cfg[i].addr, &writeByte, 1);
	}
#else
	for ( i=0;i<(sizeof(br_9600_cfg)/sizeof(registerSetting_t));i++)
	{
		writeByte = br_9600_cfg[i].data;
		cc112xSpiWriteReg(br_9600_cfg[i].addr, &writeByte, 1);
	}
#endif
#endif

#endif
}



/******************************************************************************
 * @fn					cc112xSpiReadReg
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
rfStatus_t cc112xSpiReadReg(uint16 addr, uint8 *pData, uint8 len)
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
 * @fn					cc112xSpiWriteReg
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
rfStatus_t cc112xSpiWriteReg(uint16 addr, uint8 *pData, uint8 len)
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
 * @fn					cc112xSpiWriteTxFifo
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
rfStatus_t cc112xSpiWriteTxFifo(uint8 *pData, uint8 len)
{
	uint8 rc;
	rc = trx8BitRegAccess(0x00,CC112X_BURST_TXFIFO, pData, len);
	return (rc);
}

/*******************************************************************************
 * @fn			cc112xSpiReadRxFifo
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
rfStatus_t cc112xSpiReadRxFifo(uint8 * pData, uint8 len)
{
	uint8 rc;
	rc = trx8BitRegAccess(0x00,CC112X_BURST_RXFIFO, pData, len);
	return (rc);
}

/******************************************************************************
 * @fn			cc112xGetTxStatus(void)
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
rfStatus_t cc112xGetTxStatus(void)
{
	return(trxSpiCmdStrobe(CC112X_SNOP));
}

/******************************************************************************
 *
 *	@fn			 cc112xGetRxStatus(void)
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
rfStatus_t cc112xGetRxStatus(void)
{
	return(trxSpiCmdStrobe(CC112X_SNOP | RADIO_READ_ACCESS));
}


static int8 Read8BitRssi(void)
{
	uint8 rssi2compl,rssiValid;
	uint8 rssiOffset = 102;
	int8 rssiConverted;

	// Read RSSI_VALID from RSSI0
	cc112xSpiReadReg(CC112X_RSSI0, &rssiValid, 1);
	// Check if the RSSI_VALID flag is set
	if(rssiValid & 0x01)
	{
		// Read RSSI from MSB register
		cc112xSpiReadReg(CC112X_RSSI1, &rssi2compl, 1);
		rssiConverted = (int8)rssi2compl - rssiOffset;
		return rssiConverted;
	}

	// return 0 since new value is not valid
	return 0; 
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
	cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

	// 2) Start with high VCDAC (original VCDAC_START + 2):
	cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
	writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
	cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

	// 3) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	trxSpiCmdStrobe(CC112X_SCAL);

	do
	{
		cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
	} while (marcstate != 0x41);

	// 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
	//	high VCDAC_START value
	cc112xSpiReadReg(CC112X_FS_VCO2,
					 &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
	cc112xSpiReadReg(CC112X_FS_VCO4,
					 &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
	cc112xSpiReadReg(CC112X_FS_CHP,
					 &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

	// 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

	// 6) Continue with mid VCDAC (original VCDAC_START):
	writeByte = original_fs_cal2;
	cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

	// 7) Calibrate and wait for calibration to be done
	//   (radio back in IDLE state)
	trxSpiCmdStrobe(CC112X_SCAL);

	do
	{
		cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
	} while (marcstate != 0x41);

	// 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
	//	with mid VCDAC_START value
	cc112xSpiReadReg(CC112X_FS_VCO2,
					 &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
	cc112xSpiReadReg(CC112X_FS_VCO4,
					 &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
	cc112xSpiReadReg(CC112X_FS_CHP,
					 &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

	// 9) Write back highest FS_VCO2 and corresponding FS_VCO
	//	and FS_CHP result
	if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
		calResults_for_vcdac_start_mid[FS_VCO2_INDEX])
	{
		writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
		cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
		cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
		cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
	}
	else
	{
		writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
		cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
		cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
		cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
	}
}

#if ( defined USE_CC112X_RF )
HAL_ISR_FUNCTION(RF_RTX_RDY_Isr, P1INT_VECTOR)
{
	HAL_ENTER_ISR();

	if (RF_SYNC_INT_IEN & RF_SYNC_INT_IE)
	{
#if ( defined TEST_433 )
		RFrxtxRdy = TRUE;
#else
		osal_set_event(BLECore_TaskId,RF_RXTX_RDY_EVT);
#endif	// TEST_433
	}

	// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
	RF_SYNC_INT_IFG = 0;
	RF_SYNC_INT_IF = 0;

	CLEAR_SLEEP_MODE();
	
	HAL_EXIT_ISR();
}
#endif	// USE_CC112X_RF