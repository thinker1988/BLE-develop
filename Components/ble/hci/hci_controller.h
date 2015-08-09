/*******************************************************************************
  Filename:       hci_controller.h
  Revised:        $Date: 2011-04-13 15:25:24 -0700 (Wed, 13 Apr 2011) $
  Revision:       $Revision: 25688 $

  Description:    This file contains the types, contants, external functions
                  etc. for the BLE Controller.

  Copyright 2009-2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

#ifndef HCI_CONTROLLER_H
#define HCI_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */
#include "hci.h"
#include "OSAL.h"
#include "hal_lcd.h"
#include "hal_uart.h"
#include "hci_c_data.h"
#include "hci_c_event.h"

extern uint8 hciTaskID;
extern uint8 hciCtrlCmdToken;
extern uint8 hciGapTaskID;
extern uint8 hciL2capTaskID;
extern uint8 hciSmpTaskID;

// Internal LCD Related Prototypes
extern void hciInitTransport( void );
extern void hciDisplayBDADDR( void );
extern void hciDisplayLCD( void );

/*******************************************************************************
 * MACROS
 */

#define HCI_ASSERT(condition)  HAL_ASSERT(condition)

/*******************************************************************************
 * CONSTANTS
 */

#define HCI_BDADDR_LEN                 6

/* UART port */
#define HCI_UART_PORT                  HAL_UART_PORT_0
#define HCI_UART_FC                    TRUE
#define HCI_UART_FC_THRESHOLD          48
#define HCI_UART_RX_BUF_SIZE           128
#define HCI_UART_TX_BUF_SIZE           128
#define HCI_UART_IDLE_TIMEOUT          6
#define HCI_UART_INT_ENABLE            TRUE

#if !defined ( HCI_UART_BR )
  #define HCI_UART_BR                  HAL_UART_BR_115200
#endif

/* Max buffers supported */
#define HCI_MAX_NUM_DATA_BUFFERS       0x04
#define HCI_MAX_NUM_CMD_BUFFERS        0x01

/* Maximum length for the data in DATA packet */
#define HCI_DATA_MAX_DATA_LENGTH       27

/*
  Minimum length for DATA packet is 1+2+2
  | Packet Type (1) | Handler(2) | Length(2) |
*/
#define HCI_DATA_MIN_LENGTH            0x05

/*
  Minimum length for EVENT packet is 1+1+1
  | Packet Type (1) | Event Code(1) | Length(1) |
*/
#define HCI_EVENT_MIN_LENGTH           0x03

/* Max connections */
#define HCI_MAX_NUM_CONNECTIONS        0x03

/* Serial Events */
#define HCI_CONTROLLER_TO_HOST_EVENT   0x01
#define HCI_HOST_TO_CTRL_CMD_EVENT     0x02
#define HCI_HOST_TO_CTRL_DATA_EVENT    0x04

/* Osal events */
#define HCI_TX_PROCESS_EVENT           0x1000
#define HCI_TEST_UART_SEND             0x2000

#define HCI_TX_DATA_ANY_CONNECTION     0xFF

// HCI Packet Types
#define HCI_CMD_PACKET                 0x01
#define HCI_ACL_DATA_PACKET            0x02
#define HCI_SCO_DATA_PACKET            0x03
#define HCI_EVENT_PACKET               0x04

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

#ifdef __cplusplus
}
#endif

#endif /* HCI_CONTROLLER_H */
