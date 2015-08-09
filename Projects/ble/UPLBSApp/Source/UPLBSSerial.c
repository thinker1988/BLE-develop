/**************************************************************************************************
	Filename:			 UPLBSSerial.c
	Revised:			$Date: 2015-02-02$
	Revision:			$Revision: 1 $
	Author:			 LAN Chen


	Description:		This file contains the UPLBS device Serial application.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_uart.h"
#include "UPLBSSerial.h"

#if (defined HAL_UART) && (HAL_UART == TRUE)
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Length of bd addr as a string
#define BLE_ADDR_STR_LEN				20

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
pkt_state_t pktSt = PKT_LBS_HEADER;

// Intact LBS buffer
uint8 pktBuffer[UPLBS_UART_RX_BUF_SIZE]={0};

// Current LBS packet position
uint8 pktCurLen=0;

// Target LBS packet data length
uint8 pktTotLen=0;

// Recieved data length
uint8 pktDataLen=0;

// BLE address string
static uint8 ble_str[BLE_ADDR_STR_LEN];

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void serialEventCB(uint8 port, uint8 event);
static void LBSform(uint8 *rawbuf, uint8 rawlen);
static void LBSparse(uint8 *parsebuf, uint8 parselen);
static uint8 calc_sum(uint8* chkbuf, uint16 len);
#endif	/* HAL_UART && HAL_UART==TRUE */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn		UPLBS_Serial_Init
 *
 * @brief		Serial init and open port
 *
 * @param	none
 *
 * @return	none
 */
void UPLBS_Serial_Init(void)
{
#if (defined HAL_UART) && (HAL_UART == TRUE)
	halUARTCfg_t uartConfig;

	// configure UART
	uartConfig.configured			= TRUE;
	uartConfig.baudRate				= UPLBS_UART_BR;
	uartConfig.flowControl			= UPLBS_UART_FC;
	uartConfig.flowControlThreshold = UPLBS_UART_FC_THRESHOLD;
	uartConfig.rx.maxBufSize		= UPLBS_UART_RX_BUF_SIZE;
	uartConfig.tx.maxBufSize		= UPLBS_UART_TX_BUF_SIZE;
	uartConfig.idleTimeout			= UPLBS_UART_IDLE_TIMEOUT;
	uartConfig.intEnable			= UPLBS_UART_INT_ENABLE;
	uartConfig.callBackFunc			= serialEventCB;

	// start UART
	// Note: Assumes no issue opening UART port.
	(void)HalUARTOpen( UPLBS_UART_PORT, &uartConfig );
#endif	/* HAL_UART && HAL_UART==TRUE */
	return;
}

/*********************************************************************
 * @fn		SendBinData
 *
 * @brief		Send bin data directly
 *
 * @param	buf - bin buf
 * @param	len - bin data length
 *
 * @return	none
 */
void SendBinData(uint8 *buf, uint8 len)
{
#if (defined HAL_UART) && (HAL_UART == TRUE)
	if (len > UPLBS_UART_TX_BUF_SIZE-1)
		return;

	HalUARTWrite (UPLBS_UART_PORT, buf, len);
#endif	/* HAL_UART && HAL_UART==TRUE */
}

/*********************************************************************
 * @fn		PrintString
 *
 * @brief		Serial print string
 *
 * @param	str - string
 *
 * @return	none
 */
void PrintString(uint8 *str)
{
#if (defined HAL_UART) && (HAL_UART == TRUE)
	int16 tmpLen = osal_strlen((char *)str);

	SendBinData(str, tmpLen);
#endif	/* HAL_UART && HAL_UART==TRUE */
}

/*********************************************************************
 * @fn		PrintIntValue
 *
 * @brief		Serial print integer value
 *
 * @param	title - string data
 * @param	value - integer value
 * @param	format - 10 or 16 or '+'(positive)
 *
 * @return	none
 */
void PrintIntValue(uint8 *title, int16 value, uint8 format)
{
#if (defined HAL_UART) && (HAL_UART == TRUE)
	int16 tmpLen;
	uint8 buf[UPLBS_UART_TX_BUF_SIZE];
	uint32 err;

	tmpLen = (uint8)osal_strlen((char *)title);
	if (tmpLen > UPLBS_UART_TX_BUF_SIZE/2)
		return;

	osal_memcpy( buf, title, tmpLen);
	
	if (format == 16)
	{
		buf[tmpLen++] = '0';
		buf[tmpLen++] = 'x';
		err = (uint32)(value);
	}
	else if ((format==10 && value<0) || (format == '-'))
	{
		format = 10;
		buf[tmpLen++] = '-';
		err = (uint32)(-value);
	}
	else
	{
		format = 10;
		err = (uint32)(value);
	}
	
	_ltoa( err, buf+tmpLen, format);
	PrintString(buf);
#endif	/* HAL_UART && HAL_UART==TRUE */
}

/*********************************************************************
 * @fn		LBSsend
 *
 * @brief		Serial print string
 *
 * @param	str - string
 *
 * @return	none
 */
void LBSsend(uint8 type,uint8 *sendbuf, uint8 sendlen)
{
#if (defined HAL_UART) && (HAL_UART == TRUE)
	uint8 buf[UPLBS_UART_TX_BUF_SIZE];

	osal_memcpy(buf,LBS_MARKER,LBS_MARKER_SIZE);
	buf[LBS_LENGTH_POS]=sendlen+LBS_TYPE_SIZE;
	buf[LBS_TYPE_POS]=SET_OUTPUT_TYPE(type);
	osal_memcpy(buf+LBS_PAYLOAD_POS,sendbuf,sendlen);
	buf[LBS_PAYLOAD_POS+sendlen]=calc_sum(buf+LBS_TYPE_POS,LBS_TYPE_SIZE+sendlen);

	SendBinData(buf,LBS_OVERHEAD_SIZE+LBS_TYPE_SIZE+sendlen);
#endif	/* HAL_UART && HAL_UART==TRUE */
}

#if (defined HAL_UART) && (HAL_UART == TRUE)
/*********************************************************************
 * @fn		serialEventCB
 *
 * @brief		Serial event call back
 *
 * @param	port - serial port
 * @param	event - serial event
 *
 * @return	none
 */
static void serialEventCB(uint8 port, uint8 event)
{
	if (event & HAL_UART_RX_TIMEOUT)
	{
		uint16 numBytes;
		
		if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 )
		{
			uint8 buf[UPLBS_UART_RX_BUF_SIZE]={0};
			(void)HalUARTRead (port, buf, numBytes);
			LBSform(buf, numBytes);
		}
	}
/*
	if (event & HAL_UART_TX_EMPTY)
	{
//		HalUARTWrite (UPLBS_UART_PORT,"O",1);
		TxBufEmp=TRUE;
	}
*/

	return;
}

/*********************************************************************
 * @fn		LBSform
 *
 * @brief		Serial print string
 *
 * @param	str - string
 *
 * @return	none
 */
static void LBSform(uint8 *rawbuf, uint8 rawlen)
{
	uint8 i;

	for (i=0; i<rawlen; i++)
	{
		switch(pktSt)
		{
			case PKT_LBS_HEADER:
				if (rawbuf[i] == LBS_MARKER[pktCurLen])
				{
					pktBuffer[pktCurLen++] = rawbuf[i];
					if (pktCurLen == LBS_MARKER_SIZE)
						pktSt=PKT_LBS_LEN;
				}
				else
					pktCurLen = 0;	
				break;
			case PKT_LBS_LEN:
				pktTotLen= rawbuf[i];
				pktBuffer[pktCurLen++] =pktTotLen;
				pktDataLen = 0;
				pktSt=PKT_LBS_DATA;
				break;
			case PKT_LBS_DATA:
				pktBuffer[pktCurLen++]= rawbuf[i];
				pktDataLen++;
				if (pktDataLen == pktTotLen)
					pktSt=PKT_LBS_SUM;
				break;
			case PKT_LBS_SUM:
				pktBuffer[pktCurLen++]= rawbuf[i];
				//SendBinData(pktBuffer, pktCurLen);
				LBSparse(pktBuffer, pktCurLen);
				pktCurLen = 0;
				pktSt=PKT_LBS_HEADER;
				break;
			default:
				break;
		}
	}
}


/*********************************************************************
 * @fn		LBSparse
 *
 * @brief		Serial print string
 *
 * @param	str - string
 *
 * @return	none
 */
static void LBSparse(uint8 *parsebuf, uint8 parselen)
{
	uint8 len;

	if (osal_memcmp(parsebuf,LBS_MARKER,LBS_MARKER_SIZE) == FALSE)
		return;
	
	len = parsebuf[LBS_LENGTH_POS];
	if (parselen != len+LBS_OVERHEAD_SIZE)
		return;
	if (calc_sum(parsebuf+LBS_TYPE_POS, len) != parsebuf[parselen-1])
		return;
	
	serial_data_proccess(parsebuf[LBS_TYPE_POS],parsebuf+LBS_PAYLOAD_POS,len-LBS_TYPE_SIZE);
}


/*********************************************************************
 * @fn		calc_sum
 *
 * @brief		Serial print string
 *
 * @param	str - string
 *
 * @return	none
 */
static uint8 calc_sum(uint8* chkbuf, uint16 len)
{
	uint8 i,sum=0;

	for(i=0;i<len;i++)
		sum ^= chkbuf[i];
	
	return sum;
}


/*********************************************************************
 * @fn		ble_addr_to_str
 *
 * @brief	 	Convert Bluetooth address to string
 *
 * @return	none
 */
uint8 *ble_addr_to_str( uint8 *pAddr )
{
	uint8 i;
	uint8 hex[] = "0123456789ABCDEF";
	uint8 *pStr = ble_str;

	// Start from end of addr
	pAddr += B_ADDR_LEN;
	for ( i=B_ADDR_LEN; i>0; i-- )
	{
		*pStr++ = hex[*--pAddr >> 4];
		*pStr++ = hex[*pAddr & 0x0F];
		*pStr++ = ':';
	}
	*(pStr-1)=0;

	return ble_str;
}
#endif	/* HAL_UART && HAL_UART==TRUE */

