/**************************************************************************//**
		@file			 UPLBSPacket.h

		@brief		Header file for UPLBS packet.

******************************************************************************/
#ifndef UPLBSPKT_H
#define UPLBSPKT_H


/******************************************************************************
 * INCLUDES
 */

/*********************************************************************
 * MACROS
 */
#define SET_OUTPUT_TYPE(type) 				((uint8)(type) & 0x7F)

/*********************************************************************
 * CONSTANTS
 */

/*
 *	|                                 SERIAL PACKET FORMAT                                     |
 *	|   MARKER   |  DATA LEN  |   TYPE    |    PAYLOAD      |  CHECK SUM  |
 *	|        3        |        1         |      1       |  >=0  <=122   |          1           |
 *
 */
#define LBS_MARKER								"LBS"
#define LBS_MARKER_SIZE						3

#define LBS_LENGTH_POS							(LBS_MARKER_SIZE)
#define LBS_LENGTH_SIZE						1

#define LBS_TYPE_POS							(LBS_LENGTH_POS+LBS_LENGTH_SIZE)
#define LBS_TYPE_SIZE							1

#define LBS_PAYLOAD_POS						(LBS_TYPE_POS+LBS_TYPE_SIZE)

#define LBS_CHK_SUM_SIZE						1

// Marker size + data length size + check sum size
#define LBS_OVERHEAD_SIZE						(LBS_MARKER_SIZE+LBS_LENGTH_SIZE+LBS_CHK_SUM_SIZE)


enum
{
	BLE_DEV_ADDR=1,			// Connected BLE device MAC address
	BLE_DEV_VERSION=2,			// Connected BLE device version
	BLE_DEV_SET_PARAMS=3,		// Set connected BLE device parameters
	BLE_DEV_GET_PARAMS=4,		// Get connected BLE device parameters
	BLE_DEV_CAR_SCAN_DATA=5,	// Scan data from CARDROID/CARD
	BLE_DEV_START_SCAN=6,		// Connected BLE device force scan
	BLE_DEV_SCAN_RSLT=7,		// Connected BLE device scan and result
	
	BLE_DEV_UPGRADE=126,		// Connected BLE device prepare upgrade
	BLE_DEV_REBOOT=127			// Connected BLE device force reboot
};

enum
{
	CMD_SUCCESS,
	CMD_INVALID,
	CMD_NOT_READY,
};

/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
	PKT_LBS_HEADER,
	PKT_LBS_LEN,
	PKT_LBS_DATA,
	PKT_LBS_SUM
}pkt_state_t;

/******************************************************************************
 * FUNCTION PROTOTYPES
 */
#endif	/* UPLBSPKT_H */

