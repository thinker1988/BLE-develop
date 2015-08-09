/**************************************************************************************************
	Filename:			 simpleBLECentral.h
	Revised:				$Date: 2011-03-03 15:46:41 -0800 (Thu, 03 Mar 2011) $
	Revision:			 $Revision: 12 $

	Description:		This file contains the Simple BLE Central sample application
									definitions and prototypes.

	Copyright 2011 Texas Instruments Incorporated. All rights reserved.

	IMPORTANT: Your use of this Software is limited to those specific rights
	granted under the terms of a software license agreement between the user
	who downloaded the software, his/her employer (which must be your employer)
	and Texas Instruments Incorporated (the "License").	You may not use this
	Software unless you agree to abide by the terms of the License. The License
	limits your use, and you acknowledge, that the Software may not be modified,
	copied or distributed unless embedded on a Texas Instruments microcontroller
	or used solely and exclusively in conjunction with a Texas Instruments radio
	frequency transceiver, which is integrated into your product.	Other than for
	the foregoing purpose, you may not use, reproduce, copy, prepare derivative
	works of, modify, distribute, perform, display or sell this Software and/or
	its documentation for any purpose.

	YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
	PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/

#ifndef SIMPLEBLECENTRAL_H
#define SIMPLEBLECENTRAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Simple BLE Central Task Events
#define BLE_CORE_START_EVT					0x0001
#define READ_GM_DATA_EVT						0x0002
#define GM_DRDY_INT_INT_EVT					0x0004
#define HEART_BEAT_EVT					0x0008
#define CORE_PWR_SAVING_EVT				0x0010

/*********************************************************************
 * MACROS
 */
//config P2.0 P2.3 P2.4 P0 P1 input mode, 3 state
#define BC_INIT_IO_INPUT_3STATE()	st( P0SEL &= 0; P0DIR &= 0; P0INP |= 0xFF; \
											P1SEL &= 0; P1DIR &= 0; P1INP |= 0xFF; \
														P2INP |= BV(0)|BV(3)|BV(4);)
														

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void BLECore_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 BLECore_ProcessEvent( uint8 task_id, uint16 events );

extern void powersave(uint8 task_id);
extern void powerhold(uint8 task_id);

extern void RFwakeup(void);
extern void RFsleep(void);

extern void settmsync(void);
extern void cleartmsync(void);
extern bool gettmsync(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLECENTRAL_H */
