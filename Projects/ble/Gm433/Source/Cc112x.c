#include "Com433.h"
#include "Cc112x.h"

#define PKTLEN				10
#define RX_FIFO_ERROR		0x11

//modified by lan 15.8.3, control by app
#define SPI_CSN_PXDIR				P1DIR
#define SPI_CSN_PIN					P1_4
#define SPI_CSN_BIT					BV(4)

#define TRXEN_SPI_END()				(SPI_CSN_PIN = 1)
#define TRXEN_SPI_BEGIN()			(SPI_CSN_PIN = 0)

// CC112X GPIO2 INT -> P1.3
#define RF_SYNC_INT_PINSEL			P1SEL
#define RF_SYNC_INT_PINDIR			P1DIR
#define RF_SYNC_INT_IFG				P1IFG
#define RF_SYNC_INT_IF				P1IF
#define RF_SYNC_INT_IEN				P1IEN

#define RF_SYNC_INT_PIN				BV(3)

#define SET_P1_INT_FALLING_EDGE() 	st(PICTL |= BV(1))

#define ENABLE_P1_INT()			st(IEN2 |=	BV(4))


enum RFworkingmode{
	RF_IDLE_M,
	RF_TX_M,
	RF_RX_M
};

static bool RFrxtxRdy;

static uint8 RFmode = RF_IDLE_M;

static void trxSyncIntCfg(void);

static void registerConfig(void);

static rfStatus_t trxSpiCmdStrobe(uint8 cmd);

static rfStatus_t trx8BitRegAccess(uint8 accessType, uint8 addrByte, uint8 *pData, uint16 len);
static rfStatus_t trx16BitRegAccess(uint8 accessType, uint8 extAddr, uint8 regAddr, uint8 *pData, uint8 len);
static void trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);

static void createPacket(uint8 txBuffer[]);

static void trxSingleTX(uint8 data);
static uint8 trxSingleRX(void);

static void trxBurstTX(uint8 *data, uint8 len);
static void trxBurstRX(uint8 *data, uint8 len);

static void manualCalibration(void);


void initRFcfg(void)
{	
	registerConfig();
	manualCalibration();
	
	trxSyncIntCfg();

	// enter low power mode
	trxSpiCmdStrobe(CC112X_SIDLE);
//	Com433WriteStr(COM433_DEBUG_PORT,"\r\nOK!");
}


void txdata(uint8 *txbuf, uint8 len)
{
	RFmode = RF_TX_M;
	// Write packet to TX FIFO
	cc112xSpiWriteTxFifo(txbuf, len);
	// Strobe TX to send packet
	trxSpiCmdStrobe(CC112X_STX);
	while(RFrxtxRdy == FALSE);


	RFmode = RF_RX_M;
	trxSpiCmdStrobe(CC112X_SRX);
	// wait data receive
	halSleep();
}

void rxdata(void)
{
	
}

void tx_test(void)
{
	uint8 txBuffer[PKTLEN+1] = {0};

	createPacket(txBuffer);
	// Write packet to TX FIFO
	cc112xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));
	Com433WriteInt(COM433_DEBUG_PORT,"\r\nT:",PKTLEN+1,10);
	Com433WriteInt(COM433_DEBUG_PORT," V:",txBuffer[1]-'0',10);
	// Strobe TX to send packet
	trxSpiCmdStrobe(CC112X_STX);
}

void rx_test(void)
{
	uint8 rxBuffer[128] = {0};
	uint8 rxBytes;
	uint8 marcState;
	
	trxSpiCmdStrobe(CC112X_SRX);

	while(1){

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
				rxBuffer[0]=' ';
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

void trxSyncIntCfg(void)
{
	// Falling edge of P1
	SET_P1_INT_FALLING_EDGE();
	
	RF_SYNC_INT_PINSEL &= (uint8) ~ RF_SYNC_INT_PIN;
	RF_SYNC_INT_PINDIR &= (uint8) ~ RF_SYNC_INT_PIN;

	RF_SYNC_INT_IFG = 0;
	RF_SYNC_INT_IF = 0;

	RF_SYNC_INT_IEN |= RF_SYNC_INT_PIN;

	// P1 Int enable
	ENABLE_P1_INT();
}

static void registerConfig(void)
{
	uint8 writeByte;
	uint16 i;
	
	// Reset radio
	trxSpiCmdStrobe(CC112X_SRES);

	// Write registers to radio
	for( i=0;i<(sizeof(preferredSettings470)/sizeof(registerSetting_t));i++)
	{
		writeByte = preferredSettings470[i].data;
		cc112xSpiWriteReg(preferredSettings470[i].addr, &writeByte, 1);
	}

	for ( i=0;i<(sizeof(br_9600_cfg)/sizeof(registerSetting_t));i++)
	{
		writeByte = br_9600_cfg[i].data;
		cc112xSpiWriteReg(br_9600_cfg[i].addr, &writeByte, 1);
	}
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
#if 1
			for (i = 0; i < len; i++)
			{
				trxSingleTX(0);			/* Possible to combining read and write as one access type */
				*pData = trxSingleRX();	/* Store pData from last pData RX */
				pData++;
			}
#else
			trxBurstRX(pData, len);
#endif
		}
		else
		{
#if 1
			trxSingleTX(0);
#endif
			*pData = trxSingleRX();
		}
	}
	else
	{
		if(addr&RADIO_BURST_ACCESS)
		{
#if 1
			/* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
			for (i = 0; i < len; i++)
			{
				trxSingleTX(*pData);
				pData++;
			}
#else
			trxBurstTX(pData, len);
#endif
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

static void trxBurstTX(uint8 *data, uint8 len)
{
	Com433Write(COM433_WORKING_PORT, data, len);
}

static void trxBurstRX(uint8 *data, uint8 len)
{
//	while(Hal_UART_RxBufLen(COM433_WORKING_PORT) == 0);
	Com433Read(COM433_WORKING_PORT, data, len);
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
	marcstate = trxSpiCmdStrobe(CC112X_SCAL);

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

HAL_ISR_FUNCTION(DEV_I2C_Isr, P1INT_VECTOR)
{
	HAL_ENTER_ISR();

	// Confirm accelerometer interrupt
	if (RF_SYNC_INT_IEN & RF_SYNC_INT_PIN)
		RFrxtxRdy = TRUE;

	// Clear the CPU interrupt flag for Port PxIFG has to be cleared before PxIF.
	RF_SYNC_INT_IFG = 0;
	RF_SYNC_INT_IF = 0;

	CLEAR_SLEEP_MODE();
	
	HAL_EXIT_ISR();
}
