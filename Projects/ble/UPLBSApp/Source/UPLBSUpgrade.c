/**************************************************************************************************
	Filename:			UPLBSUpgrade.c
	Revised:			$Date: 2015-02-02$
	Revision:			$Revision: 1 $
	Author:			 LAN Chen


	Description:		This file contains the UPLBS device upgrade application.
**************************************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "hal_flash.h"
#include "UPLBSUpgrade.h"

#include "UPLBSSerial.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define MAC_ADDR_BEG_IN_FLASH		0x780E

/*********************************************************************
 * TYPEDEFS
 */
#define XREG(addr)					((unsigned char volatile __xdata *) 0)[addr]

#define BigLittleSwap16(A)	((((uint16)(A) & 0xff00) >> 8) | (((uint16)(A) & 0x00ff) << 8))

#define BigLittleSwap32(A)  ((((uint32)(A) & 0xff000000) >> 24) | (((uint32)(A) & 0x00ff0000) >> 8)\
			| (((uint32)(A) & 0x0000ff00) << 8) | (((uint32)(A) & 0x000000ff) << 24))


/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 checkCPUendian(void);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn		ReadOwnMac
 *
 * @brief	 	Read own mac address in flash. Use *(unsigned char *) to read address
 *
 * @param	none
 *
 * @return	none
 */
void ReadOwnMac(uint8 *mac_addr)  
{
	uint8 i;
	for (i=0;i<B_ADDR_LEN; i++)
		mac_addr[i]=XREG(MAC_ADDR_BEG_IN_FLASH+i);
	
	return ;  
}


/*********************************************************************
 * @fn		checkCPUendian
 *
 * @brief	 	check CPU endian
 *
 * @param	none
 *
 * @return	TRUE - big endian
 */
static uint8 checkCPUendian(void)
{
	union{
		uint32 i;
		uint8 s[4];
	}c;

	c.i = 0x12345678;
	return (0x12 == c.s[0]);
}

/*********************************************************************
 * @fn		t_htonl
 *
 * @brief	 	the same as htonl
 *
 * @param	none
 *
 * @return	changed variable
 */
uint32 t_htonl(uint32 h)
{
	return checkCPUendian() ? h : BigLittleSwap32(h);
}

/*********************************************************************
 * @fn		t_ntohl
 *
 * @brief	 	the same as ntohl
 *
 * @param	none
 *
 * @return	changed variable
 */
uint32 t_ntohl(uint32 n)
{
return checkCPUendian() ? n : BigLittleSwap32(n);

}

/*********************************************************************
 * @fn		t_htons
 *
 * @brief	 	the same as htons
 *
 * @param	none
 *
 * @return	changed variable
 */

uint16 t_htons(uint16 h)
{
	return checkCPUendian() ? h : BigLittleSwap16(h);
}

/*********************************************************************
 * @fn		t_ntohs
 *
 * @brief	 	the same as ntohs
 *
 * @param	none
 *
 * @return	changed variable
 */

uint16 t_ntohs(uint16 n)
{
	return checkCPUendian() ? n : BigLittleSwap16(n);
}

#if ( defined OAD_UPGRADE )
/*********************************************************************
  * @fn 	 prepare_oad_upgrade
  *
  * @brief		 erase crc block and reset, bootloader will start image A
  *
  * @param	 none
  *
  * @return  none
  */
void prepare_oad_upgrade(void)
{
	//uint16 clr_crc[2]={0x0000,0xFFFF}; 
	// erase crc
	//HalFlashWrite(BL_IMG_B_BEG_FLASH_ADDR, (uint8 *)clr_crc, 1);
	HAL_SYSTEM_RESET();
}
#endif	/* OAD_UPGRADE */

