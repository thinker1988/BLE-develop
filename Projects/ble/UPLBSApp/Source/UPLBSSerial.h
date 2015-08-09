#ifndef UPLBS_SERIAL_H_
#define UPLBS_SERIAL_H_

#ifdef __cplusplus
extern "C"
{
#endif
	
/*********************************************************************
 * INCLUDES
 */
#include "UPLBSPacket.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define UPLBS_UART_PORT						HAL_UART_PORT_0
#define UPLBS_UART_FC							FALSE
#define UPLBS_UART_FC_THRESHOLD				48
#define UPLBS_UART_RX_BUF_SIZE				128
#define UPLBS_UART_TX_BUF_SIZE				128
#define UPLBS_UART_IDLE_TIMEOUT				6
#define UPLBS_UART_INT_ENABLE					TRUE
#define UPLBS_UART_BR							HAL_UART_BR_115200
#define UPLBS_UART_SEND_DELAY					30

/*********************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * FUNCTION PROTOTYPES
 */
void UPLBS_Serial_Init(void);

void SendBinData(uint8 * buf,uint8 len);
void PrintString(uint8 *str);
void PrintIntValue(uint8 *title, int16 value, uint8 format);

void LBSsend(uint8 type,uint8 *sendbuf, uint8 sendlen);

void serial_data_proccess(uint8 type,uint8 *buffer, uint16 length);

uint8 *ble_addr_to_str(uint8 * pAddr);


#ifdef __cplusplus
}
#endif

#endif	/* UPLBS_SERIAL_H_ */
