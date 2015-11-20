/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "hal_aes.h"
#include "hal_crc.h"
#include "hal_flash.h"
#include "hal_types.h"
#include "OSAL.h"

#include "Pktfmt.h"
#include "CoreUpgrade.h"

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * Profile Attributes - variables
 */

/*********************************************************************
 * Profile Attributes - Table
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint16 RFOadBlkNum = 0, RFOadBlkTot = 0xFFFF;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static bStatus_t RFOadImgBlockWrite( uint8 *pValue, uint8 len );

static uint16 RFOadCrcCalc(void);

static uint8 RFOadCheckDL(void);



/*********************************************************************
 * @fn		RFOadImgBlockWrite
 *
 * @brief	 Process the Image Block Write.
 *
 * @param	 pValue - pointer to data to be written
 * @param	 len - packet length
 *
 * @return	status
 */
static bStatus_t RFOadImgBlockWrite( uint8 *pValue, uint8 len )
{
	uint16 blkNum = BUILD_UINT16( pValue[NEW_FW_CUR_BLK_L_POS], pValue[NEW_FW_CUR_BLK_H_POS] );

	// make sure this is the image we're expecting
	if ( blkNum == 0 && RFOadBlkNum == 0 )
	{
		RFOadBlkTot = BUILD_UINT16( pValue[NEW_FW_TOT_BLK_L_POS], pValue[NEW_FW_TOT_BLK_H_POS] );
		
		oad_img_hdr_t ImgHdr;
		uint16 ver = BUILD_UINT16( pValue[6], pValue[7] );
		uint16 blkTot = BUILD_UINT16( pValue[8], pValue[9] ) / (RF_OAD_BLOCK_SIZE / HAL_FLASH_WORD_SIZE);

		HalFlashRead(RF_OAD_IMG_ORG_PAGE, RF_OAD_IMG_HDR_OSET, (uint8 *)&ImgHdr, sizeof(oad_img_hdr_t));

		if ( ( RFOadBlkNum != blkNum ) || ( RFOadBlkTot != blkTot ) || \
				( RF_OAD_IMG_ID( ImgHdr.ver ) == RF_OAD_IMG_ID( ver ) ) )
		{
			return ( ATT_ERR_WRITE_NOT_PERMITTED );
		}
	}

	if (RFOadBlkNum == blkNum)
	{
		uint16 addr = RFOadBlkNum * (RF_OAD_BLOCK_SIZE / HAL_FLASH_WORD_SIZE) + \
								(RF_OAD_IMG_DST_PAGE * RF_OAD_FLASH_PAGE_MULT);
		RFOadBlkNum++;

#if ( defined GM_IMAGE_B )
		// Skip the Image-B area which lies between the lower & upper Image-A parts, when upgrading image A.
		if (addr >= (RF_OAD_IMG_B_PAGE * RF_OAD_FLASH_PAGE_MULT))
		{
			addr += RF_OAD_IMG_B_AREA * RF_OAD_FLASH_PAGE_MULT;
		}
#endif	// GM_IMAGE_B

		if ((addr % RF_OAD_FLASH_PAGE_MULT) == 0)
		{
			HalFlashErase(addr / RF_OAD_FLASH_PAGE_MULT);
		}

		HalFlashWrite(addr, pValue+GMS_UPGD_LEN, (RF_OAD_BLOCK_SIZE/ HAL_FLASH_WORD_SIZE));
	}

	if (RFOadBlkNum == RFOadBlkTot)	// If the OAD Image is complete.
	{
		if (RFOadCheckDL())
		{
#if ( defined GM_IMAGE_B )
			// The BIM always checks for a valid Image-B before Image-A,
			// so Image-A never has to invalidate itself.
			uint16 crc[2] = { 0x0000, 0xFFFF };
			uint16 addr = RF_OAD_IMG_ORG_PAGE * RF_OAD_FLASH_PAGE_MULT + RF_OAD_IMG_CRC_OSET / HAL_FLASH_WORD_SIZE;
			HalFlashWrite(addr, (uint8 *)crc, 1);
#endif
			HAL_SYSTEM_RESET();
		}
	}

	return ( SUCCESS );
}


/**************************************************************************************************
 * @fn			RFOadCrcCalc
 *
 * @brief		 Run the CRC16 Polynomial calculation over the DL image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return		The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 RFOadCrcCalc(void)
{
	uint8 pageEnd = RFOadBlkTot / RF_OAD_BLOCKS_PER_PAGE;
	uint16 osetEnd = (RFOadBlkTot - (pageEnd * RF_OAD_BLOCKS_PER_PAGE)) * RF_OAD_BLOCK_SIZE;

#if ( defined GM_IMAGE_B )
	pageEnd += RF_OAD_IMG_DST_PAGE + RF_OAD_IMG_B_AREA;
#else
	pageEnd += RF_OAD_IMG_DST_PAGE;
#endif	// GM_IMAGE_B

	HalCRCInit(0x0000);	// Seed thd CRC calculation with zero.

	for (uint8 page = RF_OAD_IMG_DST_PAGE; ; page++)
	{
#if ( defined GM_IMAGE_B )
		// Skip the Image-B area which lies between the lower & upper Image-A parts.
		if (page == RF_OAD_IMG_B_PAGE)
		{
			page += RF_OAD_IMG_B_AREA;
		}
#endif	// GM_IMAGE_B

		for (uint16 oset = 0; oset < HAL_FLASH_PAGE_SIZE; oset += HAL_FLASH_WORD_SIZE)
		{
			if ((page == RF_OAD_IMG_DST_PAGE) && (oset == RF_OAD_IMG_CRC_OSET))
			{
				continue;	// Skip the CRC and shadow.
			}
			else if ((page == pageEnd) && (oset == osetEnd))
			{
				return HalCRCCalc();
			}
			else
			{
				uint8 buf[HAL_FLASH_WORD_SIZE];
				HalFlashRead(page, oset, buf, HAL_FLASH_WORD_SIZE);

				for (uint8 idx = 0; idx < HAL_FLASH_WORD_SIZE; idx++)
				{
					HalCRCExec(buf[idx]);
				}
			}
		}
	}
}

/**************************************************************************************************
 * @fn			RFOadCheckDL
 *
 * @brief		 Check validity of the downlRFOaded image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return		TRUE or FALSE for image valid.
 **************************************************************************************************
 */
static uint8 RFOadCheckDL(void)
{
	uint16 crc[2];

	HalFlashRead(RF_OAD_IMG_DST_PAGE, RF_OAD_IMG_CRC_OSET, (uint8 *)crc, sizeof(crc));

	if ((crc[0] == 0xFFFF) || (crc[0] == 0x0000))
	{
		return FALSE;
	}

	if (crc[1] == 0xFFFF)
	{
		crc[1] = RFOadCrcCalc();

#if defined FEATURE_RF_OAD_BIM	// If download image is made to run in-place, enable it here.
		uint16 addr = RF_OAD_IMG_DST_PAGE * RF_OAD_FLASH_PAGE_MULT + RF_OAD_IMG_CRC_OSET / HAL_FLASH_WORD_SIZE;
		crc[0] = 0xFFFF;
		HalFlashWrite(addr, (uint8 *)crc, 1);
		HalFlashRead(RF_OAD_IMG_DST_PAGE, RF_OAD_IMG_CRC_OSET, (uint8 *)crc, sizeof(crc));
#endif
	}

	return (crc[0] == crc[1]);
}

/*********************************************************************
*********************************************************************/

