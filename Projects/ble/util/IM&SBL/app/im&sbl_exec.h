/**************************************************************************************************
	Filename:			 sbl_exec.h
	Revised:				$Date: 2012-08-08 18:03:58 -0700 (Wed, 08 Aug 2012) $
	Revision:			 $Revision: 31165 $

	Description:

	Serial Bootloader Executive.


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
	PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#ifndef SBL_EXEC_H
#define SBL_EXEC_H

/* ------------------------------------------------------------------------------------------------
 *																					Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_types.h"

/* ------------------------------------------------------------------------------------------------
 *																					Constants
 * ------------------------------------------------------------------------------------------------
 */

#define SBL_FORCE_BOOT						0xF8
#define SBL_FORCE_RUN						(SBL_FORCE_BOOT ^ 0xFF)


// FLASH 256k = 128 pages, includes 125~126 NV pages and 127 non erasable page, total available pages is 125 (0~124)
#define TOTAL_AVAIL_PAGE					125

#define BIM_IMG_A_PAGE						2
#define BIM_IMG_A_AREA						46

#define BIM_IMG_B_PAGE						8
#define BIM_IMG_B_AREA						(TOTAL_AVAIL_PAGE-BIM_IMG_A_PAGE-BIM_IMG_A_AREA)

#define BIM_CRC_OSET						0x00
#define BIM_HDR_OSET						0x00

#define IMG_B_BEG_FLASH_ADDR				(uint16)(BIM_IMG_B_PAGE*HAL_FLASH_PAGE_SIZE/HAL_FLASH_WORD_SIZE)

enum
{
	IMG_A=0,
	IMG_B,
	IMG_WAIT
};

/* ------------------------------------------------------------------------------------------------
 *																					Functions
 * ------------------------------------------------------------------------------------------------
 */

uint8 sbl_Poll(void);

//void load_img(uint8 img_choice);

#endif
/**************************************************************************************************
*/
