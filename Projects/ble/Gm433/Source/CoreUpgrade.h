#ifndef CORE_UPGRADE_H_
#define CORE_UPGRADE_H_

#include "hal_aes.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if ( defined VERSION_NUMBER )

#if ( defined GM_IMAGE_A )

#define RF_OAD_IMAGE_USER_ID	{ 'I', 'M', 'G', 'A' }
#if (VERSION_NUMBER & 0x01 != 0 )
	#error "Image make error, odd version number please use image B project."
#endif	// VESION_NUMBER is odd

#elif ( defined GM_IMAGE_B )	// GM_IMAGE_B

#define RF_OAD_IMAGE_USER_ID	{ 'I', 'M', 'G', 'B' }
#if (VERSION_NUMBER & 0x01 == 0 )
	#error "Image make error, even version number please use image A project."
#endif	// VESION_NUMBER is even

#else	// !GM_IMAGE_A && !GM_IMAGE_B
	#error "Need define image type in preprocessor, e.g. 'GM_IMAGE_A' or 'GM_IMAGE_B'."
#endif	// GM_IMAGE_A

#else	// !VERSION_NUMBER
	#error "Need define verion number in preprocessor, e.g. 'VERSION_NUMBER=0x0100'."
#endif


// Check image type, even version number is A, odd version number is B.
#define RF_OAD_IMG_ID( ver )		((ver) & 0x01 )			


// BLECore image size 64k = 32pages
#define BLECORE_IMG_PG_SIZE			32

#if !defined RF_OAD_IMG_A_PAGE
#define RF_OAD_IMG_A_PAGE			2
#define RF_OAD_IMG_A_AREA			BLECORE_IMG_PG_SIZE
#endif

#if !defined RF_OAD_IMG_B_PAGE
// Image-A/B can be very differently sized areas when implementing BIM vice OAD boot loader.
#define RF_OAD_IMG_B_PAGE			8
#define RF_OAD_IMG_B_AREA			BLECORE_IMG_PG_SIZE
#endif

#if defined GM_IMAGE_B
#define RF_OAD_IMG_DST_PAGE			RF_OAD_IMG_A_PAGE
#define RF_OAD_IMG_DST_AREA			RF_OAD_IMG_A_AREA
#define RF_OAD_IMG_ORG_PAGE			RF_OAD_IMG_B_PAGE
#define RF_OAD_IMG_ORG_AREA			RF_OAD_IMG_B_AREA
#else	 //#elif defined GM_IMAGE_A or a non-BIM-enabled OAD Image-A w/ constants in Bank 1 vice 5.
#define RF_OAD_IMG_DST_PAGE			RF_OAD_IMG_B_PAGE
#define RF_OAD_IMG_DST_AREA			RF_OAD_IMG_B_AREA
#define RF_OAD_IMG_ORG_PAGE			RF_OAD_IMG_A_PAGE
#define RF_OAD_IMG_ORG_AREA			RF_OAD_IMG_A_AREA
#endif

// Place of _BLECORE_imgHdr
#define RF_OAD_IMG_HDR_OSET			0x0002

// CRC offset in image
#define RF_OAD_IMG_CRC_OSET			0x0000 


// The Image is transporte in 64-byte blocks in order to avoid using blob operations.
#define RF_OAD_BLOCKS_PER_PAGE		(HAL_FLASH_PAGE_SIZE / RF_OAD_BLOCK_SIZE)
#define RF_OAD_BLOCK_MAX			(RF_OAD_BLOCKS_PER_PAGE * RF_OAD_IMG_DST_AREA)

// Word in per FLASH page
#define RF_OAD_FLASH_PAGE_MULT		((uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE))


// OAD Image Header
typedef struct {
	uint16 crc1;		// CRC-shadow must be 0xFFFF.
	// User-defined Image Version Number - default logic uses simple a '!=' comparison to start an OAD.
	uint16 ver;
	uint16 len;			// Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
	uint8	uid[4];		// User-defined Image Identification bytes.
	uint8	res[4];		// Reserved space for future use.
}oad_img_hdr_t;

// The AES Header must be encrypted and the Signature must include the Image Header.
typedef struct {
	uint8 signature[KEY_BLENGTH];	// The AES-128 CBC-MAC signature.
	uint8 nonce12[12];				// The 12-byte Nonce for calculating the signature.
	uint8 spare[4];
}oad_aes_hdr_t;

extern const __code oad_img_hdr_t _BLECORE_imgHdr;
extern const __code oad_aes_hdr_t _BLECORE_aesHdr;

#ifdef __cplusplus
}
#endif

#endif
