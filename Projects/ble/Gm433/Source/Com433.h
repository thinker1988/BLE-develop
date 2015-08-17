#ifndef SERIAL_APP_H_
#define SERIAL_APP_H_

#include "hal_uart.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if ( !defined HW_VERN ) || ( HW_VERN == 0 )
#define COM433_WORKING_PORT					HAL_UART_PORT_0
#define COM433_DEBUG_PORT						HAL_UART_PORT_1
#elif ( HW_VERN == 1 )
#define COM433_WORKING_PORT					HAL_UART_PORT_1
#define COM433_DEBUG_PORT						HAL_UART_PORT_0
#endif

#define COM433_UART_FC							FALSE
#define COM433_UART_FC_THRESHOLD				48
#define COM433_UART_RX_BUF_SIZE				128
#define COM433_UART_TX_BUF_SIZE				128
#define COM433_UART_IDLE_TIMEOUT				6
#define COM433_UART_INT_ENABLE				TRUE

#if ( defined UART_HIGH_BAUD )
#define COM433_UART_BR							HAL_UART_BR_115200
#else
#define COM433_UART_BR							HAL_UART_BR_9600
#endif	// UART_HIGH_BAUD

// Serial Port Related
extern void Com433_Init(void);

extern void Com433Handle(uint8 port, uint8 *pBuffer, uint16 length);


extern void Com433Read(uint8 port, uint8 *pBuffer, uint16 length);
extern void Com433Write(uint8 port,uint8 *pBuffer, uint16 length);


extern void Com433WriteInt(uint8 port,char *title, int16 value, uint8 format);
extern void Com433WriteStr(uint8 port,uint8 str[]);


extern void Com433WriteBoth(uint8 *pBuffer, uint16 length);
extern void Com433WriteIntBoth( char *title, int16 value, uint8 format);

extern void PrintGMvalue(uint8 port, char * pBuffer, int16 xval, int16 yval, int16 zval);
#ifdef __cplusplus
}
#endif

#endif
