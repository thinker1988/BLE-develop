/***************************************************************************
	@file			UPLBSUpgrade.h

	@brief		UPLBS system upgrade header. 

****************************************************************************/
#ifndef UPLBSUPGRADE_H
#define UPLBSUPGRADE_H

#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
 * INCLUDES
 */
#include "hal_aes.h"

/*********************************************************************
 * MACROS
 */

// Set LSB for Imange
#define SET_IMG_VER(ver)					((uint16)((ver)<<0x01 | 0x01)) 

/*********************************************************************
 * CONSTANTS
 */
// Version number
#if (!defined IMAGE_VERSION_NUM )
#define IMAGE_VERSION_NUM					0
#endif	/* IMAGE_VERSION_NUM */

// Version information
#define UPLBS_SOFTWARE_VERSION			"SW_V01.00.00.01_REL"
#define UPLBS_HARDWARE_VERSION			"HW_V01.00.00.01_REL"
#define UPLBS_RELEASE_DATE 				"20150304"


#if ( defined OAD_UPGRADE )

// Image begin from page 8
#if (!defined IMG_BEG_PAGE)
#define IMG_BEG_PAGE						8
#endif	/* IMG_BEG */

//Image takes 77 pages i.e. 154k
#if (!defined IMG_PG_SIZE)
#define IMG_PG_SIZE						77
#endif	/* IMG_PG_SIZE */

// CRC position in flash
#define FLASH_PAGE_IN_WORD				((uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE))

// CRC position in flash
#define BL_IMG_B_BEG_FLASH_ADDR			(FLASH_PAGE_IN_WORD * IMG_BEG_PAGE)

// Image user id
#define IMAGE_USER_ID						{'L','B','S','\0'}

/*********************************************************************
 * TYPEDEFS
 */
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


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */
extern void prepare_oad_upgrade(void);
#endif	/* OAD_UPGRADE */

extern void ReadOwnMac(uint8 * MacAddr);

extern uint32 t_htonl(uint32 h);
extern uint32 t_ntohl(uint32 n);
extern uint16 t_htons(uint16 h);
extern uint16 t_ntohs(uint16 n);


#ifdef __cplusplus
}
#endif

#endif /* UPLBSUPGRADE_H */
