/**************************************************************************************************
  Filename:       simpleBLECentral_Main.c
  Revised:        $Date: 2011-02-24 15:48:00 -0800 (Thu, 24 Feb 2011) $
  Revision:       $Revision: 11 $

  Description:    This file contains the main and callback functions for
                  the Simple BLE Central sample application.


  Copyright 2011 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

#include "BLECore.h"

#if ( defined OAD_UPGRADE )
#include "hal_aes.h"


// OAD Image Header
typedef struct {
	uint16 crc1;		// CRC-shadow must be 0xFFFF.
	// User-defined Image Version Number - default logic uses simple a '!=' comparison to start an OAD.
	uint16 ver;
	uint16 len;			// Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
	uint8	uid[4];		// User-defined Image Identification bytes.
	uint8	res[4];		// Reserved space for future use.
} img_hdr_t;

// The AES Header must be encrypted and the Signature must include the Image Header.
typedef struct {
	uint8 signature[KEY_BLENGTH];	// The AES-128 CBC-MAC signature.
	uint8 nonce12[12];				// The 12-byte Nonce for calculating the signature.
	uint8 spare[4];
} aes_hdr_t;


// OAD image header 
#pragma location="IMAGE_HEADER"
const __code img_hdr_t _BLECORE_imgHdr = {
	0xFFFF,									// CRC-shadow must be 0xFFFF for everything else
	0x0000,	// 15-bit Version #, left-shifted 1; OR with Image-B/Not-A bit.
	BLECORE_IMG_PG_SIZE * FLASH_PAGE_IN_WORD,
	{ 'I', 'M', 'G', 'A' },			// User-Id
	{ 0xFF, 0xFF, 0xFF, 0xFF }			// Reserved
};
#pragma required=_BLECORE_imgHdr

#pragma location="AES_HEADER"
const __code aes_hdr_t _BLECORE_aesHdr = {
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
 { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B },	// Dummy Nonce
 { 0xFF, 0xFF, 0xFF, 0xFF }	 // Spare
};
#pragma required=_BLECORE_aesHdr
#endif	/* OAD_UPGRADE */

/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/

/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
int main(void)
{
  /* Initialize hardware */
  HAL_BOARD_INIT();

//  BC_INIT_IO_INPUT_3STATE();

  // Initialize board I/O
  InitBoard( OB_COLD );

  /* Initialze the HAL driver */
  HalDriverInit();

  /* Initialize NV system */
  osal_snv_init();
  
  /* Initialize LL */

  /* Initialize the operating system */
  osal_init_system();

  /* Enable interrupts */
  HAL_ENABLE_INTERRUPTS();

  // Final board initialization
  InitBoard( OB_READY );

  #if defined ( POWER_SAVING )
	 osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif

  /* Start OSAL */
  osal_start_system(); // No Return from here

  return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
