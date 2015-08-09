/**************************************************************************************************
  Filename:       UPLBS_Main.c
  Revised:        $Date: 2014-09-12 (Fri, 12 Sep 2014) $
  Revision:       $Revision: 1 $
  Author:         LAN Chen

  Description:    This file contains the main and callback functions for
                  the Simple BLE Dual Mode application.
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

#include "UPLBSApp.h"

#if ( defined OAD_UPGRADE )
#include "UPLBSUpgrade.h"
// OAD image header 
#pragma location="IMAGE_HEADER"
const __code img_hdr_t _UPLBS_imgHdr = {
	0xFFFF,									// CRC-shadow must be 0xFFFF for everything else
	SET_IMG_VER(IMAGE_VERSION_NUM),	// 15-bit Version #, left-shifted 1; OR with Image-B/Not-A bit.
	IMG_PG_SIZE * FLASH_PAGE_IN_WORD,
	IMAGE_USER_ID,						// User-Id
	{ 0xFF, 0xFF, 0xFF, 0xFF }			// Reserved
};
#pragma required=_UPLBS_imgHdr

#pragma location="AES_HEADER"
const __code aes_hdr_t _UPLBS_aesHdr = {
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
 { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B },	// Dummy Nonce
 { 0xFF, 0xFF, 0xFF, 0xFF }	 // Spare
};
#pragma required=_UPLBS_aesHdr
#endif	/* OAD_UPGRADE */

/*
// Flash lock bits
#pragma location="FLASH_LOCK_BITS"
const __code char _UPLBS_arry[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//__root const __code char _UPLBS_arry[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
#pragma required= _UPLBS_arry
*/

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
//	DEV_ALL_GPIO_INPUT_3STATE();

	/* Initialize hardware */
	HAL_BOARD_INIT();

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

	/* Start OSAL */
	osal_start_system(); // No Return from here

	return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
