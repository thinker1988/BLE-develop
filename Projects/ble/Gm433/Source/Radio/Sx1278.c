/*********************************************************************
 * INCLUDES
 */
#include "RFProc.h"
#include "Sx1278.h"
#include "Sx1278-LoRa.h"
//#include ""
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


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void RF_working(uint8 task_id, rfstate_t newrfstate)
{
	SetRFstate(newrfstate);
	
	switch (newrfstate)
	{
		case RF_PRESET:
		{
			SX1278Reset(FALSE);	// Reset radio
			SetRFstate(RF_WAIT_RESET);
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT,WAIT_RF_START_PERIOD);
			break;
		}
		case RF_WAIT_RESET:
		{
			SX1278Reset(TRUE);	// Reset radio
			SetRFstate(RF_BEG_SET);
			osal_start_timerEx(task_id, RF_DATA_PROC_EVT,WAIT_RF_START_PERIOD);
			break;
		}
		case RF_BEG_SET:
		{
			LoRaOn = true;
			SX1276SetLoRaOn( LoRaOn );
			// Initialize LoRa modem
			SX1276LoRaInit();
			Com433WriteInt(COM433_DEBUG_PORT, "\r\nVern:", SX1276LR->RegVersion, 16);
			break;
		}
		case RF_SEND:	// This state changed by interrupt
		{
			
			break;
		}
		case RF_RECV:	// Leave recv state by timer
		{

			break;
		}
		case RF_SLEEP:

			break;
		default:
			break;
	}
	
	return;
}

rferr_t RFDataSend(uint8 *buf, uint8 len)
{
	VOID buf;
	VOID len;

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
	SX1278_RESET_PINSEL &= (uint8) ~(SX1278_RESET_GPIO);
	SX1278_RESET_PINDIR |= (uint8) ~(SX1278_RESET_GPIO);

	SX1278_RESET_PIN = flg;
}

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
	WAIT_SO_STABLE();

	trxSingleTX( addr | 0x80 );
	for( i = 0; i < size; i++ )
	{
		trxSingleTX( buffer[i] );
	}

	//NSS = 1;
	TRXEN_SPI_END();
}

void SX1276ReadBuffer( uint8 addr, uint8 *buffer, uint8 size )
{
	uint8 i;

	//NSS = 0;
	TRXEN_SPI_BEGIN();
	WAIT_SO_STABLE();

	trxSingleTX( addr & 0x7F );
	for( i = 0; i < size; i++ )
	{
		trxSingleTX( 0 );
		buffer[i] = trxSingleRX();
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

