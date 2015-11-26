/*********************************************************************
 * INCLUDES
 */
#include "RFProc.h"
#include "Sx1278.h"

#if ( ! defined RION_CODE )
#include "Sx1278-LoRa.h"
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
// Default RF frequency params: work 428 set 433 upgrade 470
uint8 RFwkfrq = 0x06,RFstfrq = 0x01,RFupgfrq = 0x0C;

// Default RF air baud rate: 9600
uint8 RFairbaud = 0x05;

// Default RF air baud rate : 20dBm
uint8 RFpwr = 0x08;  // Max level

/*********************************************************************
 * EXTERNAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

uint8_t SX1276Regs[SX1278_REG_SIZE];

static bool LoRaOn = false;
static bool LoRaOnState = false;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void RXTXAntSet(uint8 antst);
static void trxSyncIntCfg(void);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
#if ( defined RION_CODE )
#define gb_SF			1
#define gb_BW			8
#define CR				0x02
#define CRC				0x00  //CRC Enable

uint8 gtmp;


__root const uint16 RFM96FreqTbl[3] = {0x066C, 0x0740, 0x0800}; //{0x0674, 0x0799, 0x0899};//433MHz频率

__root const uint16 RFM96PowerTbl[4] =
{
	0x09FF,					 //20dbm
	0x09FC,					 //17dbm
	0x09F9,					 //14dbm
	0x09F6,					 //11dbm 
};

__root const uint8 RFM96SpreadFactorTbl[7] =
{
	6,7,8,9,10,11,12
};
 
 __root const uint8 RFM96LoRaBwTbl[10] =
 {// 0		  1 	2		3	   4	   5		 6		7		8	   9
 //7.8KHz,10.4KHz,15.6KHz,20.8KHz,31.2KHz,41.7KHz,62.5KHz,125KHz,250KHz,500KHz
   0,1,2,3,4,5,6,7,8,9
 };
 

 
 uint8	gb_RxData[32];										   //Receive data buffer
#endif

void RF_working(uint8 task_id, rfstate_t newrfstate)
{
	SetRFstate(newrfstate);
	
	switch (newrfstate)
	{
		case RF_PRESET:
		{
			SX1278Reset(FALSE);	// Reset radio
			SetRFstate(RF_WAIT_RESET);
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT, WAIT_RF_PRESET_PERIOD/2);
			break;
		}
		case RF_WAIT_RESET:
		{
			SX1278Reset(TRUE);	// Reset radio
			SetRFstate(RF_BEG_SET);
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT, WAIT_RF_PRESET_PERIOD/2);
			break;
		}
		case RF_BEG_SET:
		{
#if ( defined RION_CODE )	
			Com433WriteInt(COM433_DEBUG_PORT, "\r\nV:", SPIRead((uint8)(REG_LR_VERSION>>8)), 16);
			asm("NOP");

			RF_SYNC_INT_PINSEL &= (uint8) ~ RF_SYNC_INT_IE;
			RF_SYNC_INT_PINDIR &= (uint8) ~ RF_SYNC_INT_IE;

#if ( RX_MODE == 1 )
			RFM96_LoRaEntryRx();
			while(1)
			{RFM96_LoRaRxPacket();}
#else		
			RFM96_LoRaEntryTx();//进入发送模式
			Com433WriteStr(COM433_DEBUG_PORT,"\r\nWR");
			RFM96_LoRaTxPacket();//发送数据
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT,WAIT_RF_PRESET_PERIOD*10);
#endif
			break;
		}
#else

			osal_memset(SX1276Regs+1, 0x00, SX1278_REG_SIZE-1);
			SX1276LR = ( tSX1276LR* )SX1276Regs;

			SX1276SetLoRaOn( LoRaOn );
			// Initialize LoRa modem
			SX1276LoRaInit();
			trxSyncIntCfg();

			SetRFstate(RF_RECV);
			SX1276StartRx();
			SX1276Process();
			break;
		}
		case RF_SEND:	// This state changed by interrupt
		{
			if (GetRTxRdyFlg() == TRUE)	// In case SYS_WORKING reentry
			{
				SetRTxRdyFlg(FALSE);
				if (SX1276Process() == RF_TX_DONE)
				{
					SetRFstate(RF_RECV);
					SX1276StartRx();
					SX1276Process();
				}
			}
			break;
		}
		case RF_RECV:	// Leave recv state by timer
		{
			if (GetRTxRdyFlg() == TRUE)	// In case SYS_WORKING reentry
			{
				SetRTxRdyFlg(FALSE);
				if ( SX1276Process() == RF_RX_DONE)
				{
					uint8 gmdata[RF_BUFFER_SIZE+1];
					uint16 len;

					SX1276GetRxPacket(gmdata+1, &len);
					gmdata[0] = ' ';
					gmdata[len] = '\0';
					Com433WriteInt(COM433_DEBUG_PORT, "\r\nR:",len,10);
					Com433WriteStr(COM433_DEBUG_PORT, gmdata);
					GMSPktForm(gmdata,len);
				}
			}
			break;
		}
		case RF_SLEEP:
			RXTXAntSet(RFLR_OPMODE_SLEEP);
			SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
			break;
		default:
			break;
#endif

	}
	
	return;
}

rfpkterr_t RFDataSend(uint8 *buf, uint8 len)
{
#if ( !defined RION_CODE )
	SetRFstate(RF_SEND);
	SX1276SetTxPacket(buf, len);
	RXTXAntSet(RFLR_OPMODE_TRANSMITTER);
	SX1276Process();
#endif

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
	if (port == COM433_DEBUG_PORT)
	{
	}

	return;
}


void SX1278Reset(bool flg)
{
	if (flg)	// Resume
	{
		SX1278_RESET_PINSEL &= (uint8) ~(SX1278_RESET_GPIO);
		SX1278_RESET_PINDIR &= (uint8) ~(SX1278_RESET_GPIO);
		SX1278_RESET_PININP |= (uint8) (SX1278_RESET_GPIO);
		
		LoRaOn = TRUE;
//		SX1278_RESET_PINDIR |= (uint8) (SX1278_RESET_GPIO);
//		SX1278_RESET_PIN = 1;
	}
	else	// Reset
	{		
		LoRaOn = FALSE;
		LoRaOnState = FALSE;
		SX1278_RESET_PINSEL &= (uint8) ~(SX1278_RESET_GPIO);
		SX1278_RESET_PINDIR |= (uint8) (SX1278_RESET_GPIO);
		SX1278_RESET_PIN = 0;
	}
}
#if ( ! defined RION_CODE )
void SX1276SetLoRaOn( bool enable )
{
	if( LoRaOnState == enable )
	{
		return;
	}
	LoRaOnState = enable;
	LoRaOn = enable;

	if( LoRaOn == true )
	{
		SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
		
		SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;
		SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );
		
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
										// RxDone			   RxTimeout				   FhssChangeChannel		   CadDone
		SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
										// CadDetected		  ModeReady
		SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
		SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
		
		SX1276ReadBuffer( REG_LR_OPMODE, SX1276Regs + 1, SX1278_REG_SIZE - 1 );
	}
	else
	{
		SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
		
		SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
		SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );
		
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
		
		//SX1276ReadBuffer( REG_OPMODE, SX1276Regs + 1, SX1278_REG_SIZE - 1 );
	}
}

bool SX1276GetLoRaOn( void )
{
	return LoRaOn;
}

void SX1276SetOpMode( uint8_t opMode )
{
	if( LoRaOn == false )
	{
		//SX1276FskSetOpMode( opMode );
	}
	else
	{
		SX1276LoRaSetOpMode( opMode );
	}
}

uint8_t SX1276GetOpMode( void )
{
	if( LoRaOn == false )
	{
		return LoRaOn;//SX1276FskGetOpMode( );
	}
	else
	{
		return SX1276LoRaGetOpMode( );
	}
}

double SX1276ReadRssi( void )
{
	if( LoRaOn == false )
	{
		return LoRaOn;//SX1276FskReadRssi( );
	}
	else
	{
		return SX1276LoRaReadRssi( );
	}
}

uint8_t SX1276ReadRxGain( void )
{
	if( LoRaOn == false )
	{
		return LoRaOn;//SX1276FskReadRxGain( );
	}
	else
	{
		return SX1276LoRaReadRxGain( );
	}
}

uint8_t SX1276GetPacketRxGain( void )
{
	if( LoRaOn == false )
	{
		return LoRaOn;//SX1276FskGetPacketRxGain(  );
	}
	else
	{
		return SX1276LoRaGetPacketRxGain(  );
	}
}

int8_t SX1276GetPacketSnr( void )
{
	if( LoRaOn == false )
	{
		 while( 1 )
		 {
			 // Useless in FSK mode
			 // Block program here
		 }
	}
	else
	{
		return SX1276LoRaGetPacketSnr(  );
	}
}

double SX1276GetPacketRssi( void )
{
	if( LoRaOn == false )
	{
		return LoRaOn;//SX1276FskGetPacketRssi(  );
	}
	else
	{
		return SX1276LoRaGetPacketRssi( );
	}
}

uint32_t SX1276GetPacketAfc( void )
{
	if( LoRaOn == false )
	{
		return LoRaOn;//SX1276FskGetPacketAfc(  );
	}
	else
	{
		 while( 1 )
		 {
			 // Useless in LoRa mode
			 // Block program here
		 }
	}
}

void SX1276StartRx( void )
{
	if( LoRaOn == false )
	{
		//SX1276FskSetRFState( RF_STATE_RX_INIT );
	}
	else
	{
		SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
		RXTXAntSet(RFLR_OPMODE_RECEIVER);
	}
}

void SX1276GetRxPacket( void *buffer, uint16_t *size )
{
	if( LoRaOn == false )
	{
		//SX1276FskGetRxPacket( buffer, size );
	}
	else
	{
		SX1276LoRaGetRxPacket( buffer, size );
	}
}

void SX1276SetTxPacket( const void *buffer, uint16_t size )
{
	if( LoRaOn == false )
	{
		//SX1276FskSetTxPacket( buffer, size );
	}
	else
	{
		SX1276LoRaSetTxPacket( buffer, size );
	}
}

uint8_t SX1276GetRFState( void )
{
	if( LoRaOn == false )
	{
		return LoRaOn;//SX1276FskGetRFState( );
	}
	else
	{
		return SX1276LoRaGetRFState( );
	}
}

void SX1276SetRFState( uint8_t state )
{
	if( LoRaOn == false )
	{
		//SX1276FskSetRFState( state );
	}
	else
	{
		SX1276LoRaSetRFState( state );
	}
}

uint32_t SX1276Process( void )
{
	if( LoRaOn == false )
	{
		return LoRaOn;//SX1276FskProcess( );
	}
	else
	{
		return SX1276LoRaProcess( );
	}
}


void SX1276Write( uint8_t addr, uint8_t data )
{
	SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
	SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8 addr, uint8 *buffer, uint8 size )
{
	uint8 i;

	//NSS = 0;
	TRXEN_SPI_BEGIN();

	SpiInOut( addr | 0x80 );
	for( i = 0; i < size; i++ )
	{
		SpiInOut( buffer[i] );
	}

	//NSS = 1;
	TRXEN_SPI_END();
}

void SX1276ReadBuffer( uint8 addr, uint8 *buffer, uint8 size )
{
	uint8 i;

	//NSS = 0;
	TRXEN_SPI_BEGIN();

	SpiInOut( addr & 0x7F );
	for( i = 0; i < size; i++ )
	{
		buffer[i] = SpiInOut( 0 );
	}

	//NSS = 1;
	TRXEN_SPI_END();
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
	SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
	SX1276ReadBuffer( 0, buffer, size );
}
#else
void delayms(unsigned int t)
{
	unsigned int i;
	unsigned char j; 
	for(i=0;i<t;i++)
	for(j=0;j<120;j++);
}

/**********************************************************
**Name:	 SPIBurstRead
**Function: SPI burst read mode
**Input:	adr-----address for read
**		  ptr-----data buffer point for read
**		  length--how many bytes for read
**Output:   None
**********************************************************/
void SPIBurstRead(uint8 adr, uint8 *ptr, uint8 length)
{
 	 uint8 i;

	//NSS = 0;
	TRXEN_SPI_BEGIN();
	//WAIT_SO_STABLE();

	SpiInOut( adr & 0x7F );
	for( i = 0; i < length; i++ )
	{
		ptr[i] = SpiInOut( 0 );
	}

	//NSS = 1;
	TRXEN_SPI_END();
}

/**********************************************************
**Name:	 SPIBurstWrite
**Function: SPI burst write mode
**Input:	adr-----address for write
**		  ptr-----data buffer point for write
**		  length--how many bytes for write
**Output:   none
**********************************************************/
void BurstWrite(uint8 adr, uint8 *ptr, uint8 length)
{ 
	uint8 i;
	
	//NSS = 0;
	TRXEN_SPI_BEGIN();
	//WAIT_SO_STABLE();
	
	SpiInOut( adr | 0x80 );
	for( i = 0; i < length; i++ )
	{
		SpiInOut( ptr[i] );
	}

	//NSS = 1;
	TRXEN_SPI_END();
}

/**********************************************************
**Name:	 SPIRead
**Function: SPI Read CMD
**Input:	adr -> address for read
**Output:   None
**********************************************************/
uint8 SPIRead(uint8 adr)
{
	uint8 tmp=0;
  SPIBurstRead(adr,&tmp,1);
  return tmp;
}


/**********************************************************
**Name:	 SPIWrite
**Function: SPI Write CMD
**Input:	WrPara -> address & data
**Output:   None
**********************************************************/
void SPIWrite(uint16 WrPara)				
{
	WrPara |= 0x8000;
	
	//NSS = 0;
	TRXEN_SPI_BEGIN();
	//WAIT_SO_STABLE();
	asm("NOP");
	trxSingleTX((uint8)((WrPara&0xFF00)>>8));
	asm("NOP");
	trxSingleTX((uint8)(WrPara&0x00FF));

	//NSS = 1;
	TRXEN_SPI_END();
}


/**********************************************************
**Name:	 RFM96_Standby
**Function: Entry standby mode
**Input:	None
**Output:   None
**********************************************************/
void RFM96_Standby(void)
{
  SPIWrite(LR_RegOpMode+0x01+0x08);							  //Standby
}

/**********************************************************
**Name:	 RFM96_Sleep
**Function: Entry sleep mode
**Input:	None
**Output:   None
**********************************************************/
void RFM96_Sleep(void)
{
  SPIWrite(LR_RegOpMode+0x00+0x08);							  //Sleep
  asm("NOP");
  asm("NOP");
  asm("NOP");
  Com433WriteInt(COM433_DEBUG_PORT,"\r\nRD SL:",SPIRead((uint8)(LR_RegOpMode>>8)),16);
}

/*********************************************************/
//LoRa mode
/*********************************************************/
/**********************************************************
**Name:	 RFM96_EntryLoRa
**Function: Set RFM69 entry LoRa(LongRange) mode
**Input:	None
**Output:   None
**********************************************************/
void RFM96_EntryLoRa(void)
{
	SPIWrite(LR_RegOpMode+0x80+0x08);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	Com433WriteInt(COM433_DEBUG_PORT,"\r\nRD LR:",SPIRead((uint8)(LR_RegOpMode>>8)),16);
}

/**********************************************************
**Name:	 RFM96_LoRaClearIrq
**Function: Clear all irq
**Input:	None
**Output:   None
**********************************************************/
void RFM96_LoRaClearIrq(void)
{
  SPIWrite(LR_RegIrqFlags+0xFF);
}

/**********************************************************
**Name:	 RFM96_Config
**Function: RFM96 base config
**Input:	mode
**Output:   None
**********************************************************/
void RFM96_Config(uint8 mode)
{
  uint8 i; 
	
  /*RF_RST=0;
  for(i=100;i!=0;i--)									  //Delay
	asm("NOP"); 
  
  //RF_RST=1;
  
  for(i=250;i!=0;i--)									  //Delay
	asm("NOP");  
	*/

  RFM96_Sleep();										   //Change modem mode Must in Sleep mode 
  for(i=250;i!=0;i--)									  //Delay
	asm("NOP");  

	RFM96_EntryLoRa();  
	//SPIWrite(0x5904);   //?? Change digital regulator form 1.6V to 1.47V: see errata note
	
	for(i=0;i<3;i++)									   //setting frequency parameter
	{
	  SPIWrite(RFM96FreqTbl[i]);  
	}

	//setting base parameter 
	SPIWrite(RFM96PowerTbl[0]);			 //Setting output power parameter  
	
	SPIWrite(LR_RegOcp+0x0B);							  //RegOcp,Close Ocp
	SPIWrite(LR_RegLna+0x23);							  //RegLNA,High & LNA Enable
	
	if(RFM96SpreadFactorTbl[gb_SF]==6)		   //SFactor=6
	{
	  uint8 tmp;
	  SPIWrite(LR_RegModemConfig1+(RFM96LoRaBwTbl[gb_BW]<<4)+(CR<<1)+0x01);//Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
	  SPIWrite(LR_RegModemConfig2+(RFM96SpreadFactorTbl[gb_SF]<<4)+(CRC<<2)+0x03);
	  
	  tmp = SPIRead(0x31);
	  tmp &= 0xF8;
	  tmp |= 0x05;
	  SPIWrite(0x3100+tmp);
	  SPIWrite(0x3700+0x0C);
	} 
	else
	{
	  SPIWrite(LR_RegModemConfig1+(RFM96LoRaBwTbl[gb_BW]<<4)+(CR<<1)+0x00);//Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
	  SPIWrite(LR_RegModemConfig2+(RFM96SpreadFactorTbl[gb_SF]<<4)+(CRC<<2)+0x03);  //SFactor &  LNA gain set by the internal AGC loop 
	}
	SPIWrite(LR_RegSymbTimeoutLsb+0xFF);				   //RegSymbTimeoutLsb Timeout = 0x3FF(Max) 
	
	SPIWrite(LR_RegPreambleMsb + 0);					   //RegPreambleMsb 
	SPIWrite(LR_RegPreambleLsb + 8);					  //RegPreambleLsb 8+4=12byte Preamble
	
	SPIWrite(REG_LR_DIOMAPPING2+0x01);					 //RegDioMapping2 DIO5=00, DIO4=01
	SPIWrite(LR_RegModemConfig3+0x0c);
	RFM96_Standby();										 //Entry standby mode*/
}

/**********************************************************
**Name:	 RFM96_LoRaEntryRx
**Function: Entry Rx mode
**Input:	None
**Output:   None
**********************************************************/
void RFM96_LoRaEntryRx(void)
{
  uint8 addr; 
  RXTXAntSet(RFLR_OPMODE_RECEIVER);
  //RXTXAntSet(RFLR_OPMODE_TRANSMITTER);
  RFM96_Config(0);										 //setting base parameter
  
  SPIWrite(0x4D00+0x84);								   //Normal and Rx
  SPIWrite(LR_RegHopPeriod+0xFF);						  //RegHopPeriod NO FHSS
  SPIWrite(REG_LR_DIOMAPPING1+0x01);					   //DIO0=00, DIO1=00, DIO2=00, DIO3=01  DIO0=00--RXDONE
	  
  SPIWrite(LR_RegIrqFlagsMask+0x3F);					   //Open RxDone interrupt & Timeout
  RFM96_LoRaClearIrq();   
  
  SPIWrite(LR_RegPayloadLength+21);					   //RegPayloadLength  21byte(this register must difine when the data long of one byte in SF is 6)
	
  addr = SPIRead((uint8)(LR_RegFifoRxBaseAddr>>8));		   //Read RxBaseAddr
  SPIWrite(LR_RegFifoAddrPtr+addr);						//RxBaseAddr -> FiFoAddrPtr　 
  SPIWrite(LR_RegOpMode+0x0D);						//Continuous Rx Mode
  
}

/**********************************************************
**Name:	 RFM96_LoRaRxWaitStable
**Function: Determine whether the state of stable Rx 查询RX 状态
**Input:	none
**Output:   none
**********************************************************/
uint8 RFM96_LoRaRxWaitStable(void)
{ 
	uint8 tmp;
	tmp=SPIRead((uint8)(LR_RegModemStat>>8));
	return tmp;
}

/**********************************************************
**Name:	 RFM96_LoRaRxPacket
**Function: Receive data in LoRa mode
**Input:	None
**Output:   1- Success
			0- Fail
**********************************************************/
uint8 RFM96_LoRaRxPacket(void)
{
		uint8 i; 
		uint8 addr;
		uint8 packet_size;

		if(RF_IRQ_DIO0)// 收到一包LoRa 数据了RF_IRQ ---DIO0
			{
				for(i=0;i<32;i++) //清Buffer
					gb_RxData[i] = 0x00;

				addr = SPIRead((uint8)(LR_RegFifoRxCurrentaddr>>8));	  //last packet addr 数据包的最后地址(数据的尾地址)
				SPIWrite(LR_RegFifoAddrPtr+addr);					  //RxBaseAddr -> FiFoAddrPtr   
				asm("nop");

				if(RFM96SpreadFactorTbl[gb_SF]==6)		   //When SpreadFactor is six，will used Implicit Header mode(Excluding internal packet length)
					packet_size=21;
				else
					packet_size = SPIRead((uint8)(LR_RegRxNbBytes>>8));	 //Number for received bytes	

				gtmp= packet_size;
				Com433WriteInt(COM433_DEBUG_PORT,"\r\nGET:",gtmp,10);
				SPIBurstRead(0x00, gb_RxData, packet_size);

				RFM96_LoRaClearIrq();
				asm("nop");
				for(i=0;i<gtmp;i++)//验证数据 数据OK i==17
					{
						Com433WriteInt(COM433_DEBUG_PORT," ",gb_RxData[i],16);  
					}	
				/*if(i>=17)											  //Rx success
				{
					Com433WriteStr(COM433_DEBUG_PORT,gb_RxData);
					return(1);
				}
				else
					return(0);*/
			}
		else
				
		return(0);
}


/**********************************************************
**Name:	 RFM96_LoRaEntryTx
**Function: Entry Tx mode
**Input:	None
**Output:   None
**********************************************************/
void RFM96_LoRaEntryTx(void)
{
		uint8 addr;
				
		//RXTXAntSet(RFLR_OPMODE_RECEIVER);
		RXTXAntSet(RFLR_OPMODE_TRANSMITTER);
		RFM96_Config(0);										 //setting base parameter

		SPIWrite(0x4D00+0x87);								   //Tx for 20dBm
		SPIWrite(LR_RegHopPeriod);							   //RegHopPeriod NO FHSS
		SPIWrite(REG_LR_DIOMAPPING1+0x41);					   //DIO0=01, DIO1=00, DIO2=00, DIO3=01

		RFM96_LoRaClearIrq();
		SPIWrite(LR_RegIrqFlagsMask+0xF7);					   //Open TxDone interrupt
		SPIWrite(LR_RegPayloadLength+ 22);					   //RegPayloadLength  21byte

		addr = SPIRead((uint8)(LR_RegFifoTxBaseAddr>>8));		   //RegFiFoTxBaseAddr
		SPIWrite(LR_RegFifoAddrPtr+addr);						//RegFifoAddrPtr*/
}

/**********************************************************
**Name:	 RFM96_LoRaTxPacket
**Function: Send data in LoRa mode
**Input:	None
**Output:   1- Send over
**********************************************************/
uint8 RFM96_LoRaTxPacket(void)
{ 
		uint8 TxFlag=0;	
		//delayms(100);
		uint8 RFM96Data[32];
		osal_memset(RFM96Data,0,sizeof(RFM96Data));
		osal_memcpy(RFM96Data,"1234567890ABCDEFGHIJKA",22);
		RFM96Data[21]=0x0F;

		BurstWrite(0x00, (uint8 *)RFM96Data, 22);
		SPIWrite(LR_RegOpMode+0x03+0x08);					//Tx Mode	   
		Com433WriteStr(COM433_DEBUG_PORT,(uint8 *)RFM96Data);

		asm("nop");	

		gtmp= SPIRead((uint8)(LR_RegIrqFlags>>8));
		asm("nop");	
		gtmp= SPIRead((uint8)(LR_RegIrqFlags>>8));
		asm("nop");	
		gtmp= SPIRead((uint8)(LR_RegIrqFlags>>8));
		asm("nop");	

		while(!RF_IRQ_DIO0) ;					//Packet send over 发送完成了IRQ 变为H,平时L

		SPIRead((uint8)(LR_RegIrqFlags>>8));
		RFM96_LoRaClearIrq();								//Clear irq
		RFM96_Standby();									 //Entry Standby mode	  

		asm("nop");	
		return TxFlag;  
}
#endif

uint8 SpiInOut(uint8 outdata)
{
	asm("NOP");
	trxSingleTX(outdata);
	asm("NOP");
	return trxSingleRX();

}

static void RXTXAntSet(uint8 antst)
{
	switch(antst)
	{
		case RFLR_OPMODE_TRANSMITTER:
			SX1278_VA_WK_PINSEL &= (uint8) ~SX1278_VA_WK_GPIO;
			SX1278_VA_WK_PINDIR |= (uint8) SX1278_VA_WK_GPIO;
			SX1278_VA_WK_PIN = 1;

			SX1278_VB_ST_PINSEL &= (uint8) ~(SX1278_VB_ST_GPIO);
			SX1278_VB_ST_PINDIR |= (uint8) (SX1278_VB_ST_GPIO);
			SX1278_VB_ST_PIN = 0;
			break;
		case RFLR_OPMODE_RECEIVER:
			SX1278_VA_WK_PINSEL &= (uint8) ~SX1278_VA_WK_GPIO;
			SX1278_VA_WK_PINDIR |= (uint8) SX1278_VA_WK_GPIO;
			SX1278_VA_WK_PIN = 0;

			SX1278_VB_ST_PINSEL &= (uint8) ~(SX1278_VB_ST_GPIO);
			SX1278_VB_ST_PINDIR |= (uint8) (SX1278_VB_ST_GPIO);
			SX1278_VB_ST_PIN = 1;
			break;
		case RFLR_OPMODE_SLEEP:
		default:
			SX1278_VA_WK_PINSEL &= (uint8) ~SX1278_VA_WK_GPIO;
			SX1278_VA_WK_PINDIR &= (uint8) ~SX1278_VA_WK_GPIO;
			SX1278_VA_WK_PININP |= (uint8) SX1278_VA_WK_GPIO;
			
			SX1278_VB_ST_PINSEL &= (uint8) ~(SX1278_VB_ST_GPIO);
			SX1278_VB_ST_PINDIR &= (uint8) ~(SX1278_VB_ST_GPIO);
			SX1278_VB_ST_PININP |= (uint8) SX1278_VB_ST_GPIO;
			break;
		break;
	}
}

static void trxSyncIntCfg(void)
{
	SET_P1_INT_DISABLE();
	RF_SYNC_INT_DISABLE();

	SET_P1_INT_RISING_EDGE();
	
	RF_SYNC_INT_PINSEL &= (uint8) ~ RF_SYNC_INT_IE;
	RF_SYNC_INT_PINDIR &= (uint8) ~ RF_SYNC_INT_IE;

	RF_SYNC_INT_IFG = 0;
	RF_SYNC_INT_IF = 0;

	RF_SYNC_INT_ENABLE();
	// P1 Int enable
	SET_P1_INT_ENABLE();
}

