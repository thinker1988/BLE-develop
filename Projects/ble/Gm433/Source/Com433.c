/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_lcd.h"
#include "Com433.h"

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
 
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
 
/*********************************************************************
 * LOCAL FUNCTIONS
 */
 
// Call back function of serial port
static void com433_CB(uint8 port, uint8 event);
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void Com433_Init(void)
{
	halUARTCfg_t uartConfig;

	// configure UART
	uartConfig.configured				= TRUE;
	uartConfig.baudRate					= COM433_UART_BR;
	uartConfig.flowControl				= COM433_UART_FC;
	uartConfig.flowControlThreshold		= COM433_UART_FC_THRESHOLD;
	uartConfig.rx.maxBufSize			= COM433_UART_RX_BUF_SIZE;
	uartConfig.tx.maxBufSize			= COM433_UART_TX_BUF_SIZE;
	uartConfig.idleTimeout				= COM433_UART_IDLE_TIMEOUT;
	uartConfig.intEnable				= COM433_UART_INT_ENABLE;
	uartConfig.callBackFunc				= com433_CB;

	(void)HalUARTOpen( COM433_WORKING_PORT, &uartConfig );
	(void)HalUARTOpen( COM433_DEBUG_PORT, &uartConfig );

	return;
}


void Com433Read(uint8 port, uint8 *pBuffer, uint16 length)
{
	HalUARTRead (port, pBuffer, length);
}

void Com433Write(uint8 port, uint8 *pBuffer, uint16 length)
{
	HalUARTWrite (port, pBuffer, length);
}

void Com433WriteStr(uint8 port, uint8 str[])
{
	Com433Write(port, str, osal_strlen((char*)str));
}

void Com433WriteInt(uint8 port, char *title, int16 value, uint8 format)
{  
	uint8 tmpLen;
	uint8 buf[COM433_UART_TX_BUF_SIZE]={0};
	uint32 err;
	
	tmpLen = (uint8)osal_strlen( (char*)title );
	osal_memcpy( buf, title, tmpLen );
	if (value < 0 && format == 10)
	{
		value = -value;
		buf[tmpLen++] = '-';
	}
	err = (uint32)(value);
	
	_ltoa( err, &buf[tmpLen], format);
	Com433WriteStr(port, buf);
}

void Com433WriteBoth(uint8 *pBuffer, uint16 length)
{
	HalUARTWrite (COM433_WORKING_PORT, pBuffer, length);
	HalUARTWrite (COM433_DEBUG_PORT, pBuffer, length);
}

void Com433WriteIntBoth( char *title, int16 value, uint8 format)
{
	Com433WriteInt(COM433_WORKING_PORT,title,value,format);
	Com433WriteInt(COM433_DEBUG_PORT,title,value,format);
}

void PrintGMvalue(uint8 port, char *pBuffer, int16 xval, int16 yval, int16 zval)
{
#if ( defined ALLOW_DEBUG_OUTPUT )
	Com433WriteInt(port, pBuffer, xval,10);
	Com433WriteInt(port, " ", yval,10);
	Com433WriteInt(port, " ", zval,10);
#else
	VOID port;
	VOID pBuffer;
	VOID xval;
	VOID yval;
	VOID zval;
#endif
}

/*********************************************************************
* PRIVATE FUNCTIONS
*/
static void com433_CB(uint8 port, uint8 event)
{
	uint8	pktBuffer[COM433_UART_RX_BUF_SIZE]={0};
	uint16 numBytes;

	if ((numBytes = Hal_UART_RxBufLen(port)) > 0 )
	{
		(void)HalUARTRead (port, pktBuffer, numBytes);
		Com433Handle(port,pktBuffer, numBytes);
	}
	VOID event;
}

