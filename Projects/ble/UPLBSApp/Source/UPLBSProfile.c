/**************************************************************************************************
	Filename:			 UPLBSprofile.c
	Revised:				$Date: 2015-01-23 08:56:11 -0700 $
	Revision:			 $Revision: 23333 $

	Description:		This file contains the UPLBS GATT profile sample GATT service 
									profile for use with the BLE sample application.

	Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "UPLBSprofile.h"
#include "UPLBSUpgrade.h"
#include "UPLBSSerial.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Max blob packet
#define MAX_PACKET_NUM					8

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// UPLBS GATT Profile Service UUID:0xF000
CONST uint8 UPLBSProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
	LO_UINT16(UPLBS_SERV_UUID), HI_UINT16(UPLBS_SERV_UUID)
};

// Characteristic 1 UUID:0xF001
CONST uint8 UPLBSCharVernUUID[ATT_BT_UUID_SIZE] =
{ 
	LO_UINT16(UPLBS_CHAR_VERN_UUID), HI_UINT16(UPLBS_CHAR_VERN_UUID)
};

// Characteristic 2 UUID:0xF002
CONST uint8 UPLBSProfileCharSysInfoUUID[ATT_BT_UUID_SIZE] =
{ 
	LO_UINT16(UPLBS_CHAR_SYS_INFO_UUID), HI_UINT16(UPLBS_CHAR_SYS_INFO_UUID)
};

// Characteristic 3 UUID: 0xF003
CONST uint8 UPLBSProfileCharTemprUUID[ATT_BT_UUID_SIZE] =
{ 
	LO_UINT16(UPLBS_CHAR_TEMPR_UUID), HI_UINT16(UPLBS_CHAR_TEMPR_UUID)
};

// Characteristic 4 UUID: 0xF004
CONST uint8 UPLBSProfileCharAccParamUUID[ATT_BT_UUID_SIZE] =
{ 
	LO_UINT16(UPLBS_CHAR_ACC_PARAM_UUID), HI_UINT16(UPLBS_CHAR_ACC_PARAM_UUID)
};

// Characteristic 5 UUID: 0xF005
CONST uint8 UPLBSProfileCharRunStateUUID[ATT_BT_UUID_SIZE] =
{ 
	LO_UINT16(UPLBS_CHAR_RUN_STATE_UUID), HI_UINT16(UPLBS_CHAR_RUN_STATE_UUID)
};

// Characteristic 6 UUID: 0xF006
CONST uint8 UPLBSProfileCharPkStateUUID[ATT_BT_UUID_SIZE] =
{ 
	LO_UINT16(UPLBS_CHAR_PARKING_STATE_UUID), HI_UINT16(UPLBS_CHAR_PARKING_STATE_UUID)
};

// Characteristic 6 UUID: 0xF006
CONST uint8 UPLBSProfileCharBeaconTypeUUID[ATT_BT_UUID_SIZE] =
{ 
	LO_UINT16(UPLBS_CHAR_BEACON_TYPE_UUID), HI_UINT16(UPLBS_CHAR_BEACON_TYPE_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 PktRcvStatus[MAX_PACKET_NUM]={0};

static UPLBSProfileCBs_t *UPLBSProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// UPLBS Profile Service attribute
static CONST gattAttrType_t UPLBSProfileService = { ATT_BT_UUID_SIZE, UPLBSProfileServUUID };

// UPLBS Profile Characteristic 1 Properties, Value, User Description
static uint8 UPLBSCharVernProps = GATT_PROP_READ;
static UPLBS_vern_t UPLBSCharVern;
static uint8 UPLBSCharVernUserDesp[17] = "Version\0";

// UPLBS Profile Characteristic 2 Properties, Value, User Description
static uint8 UPLBSCharSysInfoProps = GATT_PROP_WRITE|GATT_PROP_NOTIFY;
static uint8 UPLBSCharSysInfo;
static uint8 UPLBSCharSysInfoUserDesp[17] = "System Info\0";

// UPLBS Profile Characteristic 3 Properties, Value, User Description
static uint8 UPLBSCharTemprProps = GATT_PROP_WRITE|GATT_PROP_NOTIFY;
static int8 UPLBSCharTempr = 0;
static uint8 UPLBSCharTemprUserDesp[17] = "Temperature\0";

// UPLBS Profile Characteristic 4 Properties, Value, User Description
static uint8 UPLBSCharAccParamProps = GATT_PROP_WRITE|GATT_PROP_NOTIFY;
static uint8 UPLBSCharAccParam = 0;
static uint8 UPLBSCharAccParamUserDesp[17] = "Acc params\0";

// UPLBS Profile Characteristic 5 Properties, Value, User Description
static uint8 UPLBSCharRunStateProps =	GATT_PROP_READ|GATT_PROP_WRITE|GATT_PROP_NOTIFY;
static uint8 UPLBSCharRunState = 0;
static uint8 UPLBSCharRunStateUserDesp[17] = "Running state\0";

// UPLBS Profile Characteristic 6 Properties, Value, User Description
static uint8 UPLBSCharPKStateProps =	GATT_PROP_READ|GATT_PROP_WRITE;
static uint8 UPLBSCharPKState = 0;
static uint8 UPLBSCharPKStateUserDesp[17] = "Parking state\0";

// UPLBS Profile Characteristic 7 Properties, Value, User Description
static uint8 UPLBSCharBecTypeProps =	GATT_PROP_READ|GATT_PROP_WRITE;
static uint8 UPLBSCharBecType = 0;
static uint8 UPLBSCharBecTypeUserDesp[17] = "Beacon type\0";


// UPLBS GATT Characteristic Config
static gattCharCfg_t UPLBSClientCharConfig[GATT_MAX_NUM_CONN];

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t UPLBSProfileAttrTbl[] = 
{
	//[ NO. 0 ] UPLBS Profile Service
	{ 
		{ ATT_BT_UUID_SIZE, primaryServiceUUID },	/* type */
		GATT_PERMIT_READ,								/* permissions */
		0,												/* handle */
		(uint8 *)&UPLBSProfileService				/* pValue */
	},
	// [ NO. 1 ] Characteristic 1 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&UPLBSCharVernProps 
	},
	// [ NO. 2 ] Characteristic Value 1
	{ 
		{ ATT_BT_UUID_SIZE, UPLBSCharVernUUID },
		GATT_PERMIT_READ, 
		0, 
		(uint8 *)&UPLBSCharVern 
	},
	// [ NO. 3 ] Characteristic 1 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		UPLBSCharVernUserDesp 
	},			

	// [ NO. 4 ] Characteristic 2 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&UPLBSCharSysInfoProps 
	},
	// [ NO. 5 ] Characteristic Value 2
	{ 
		{ ATT_BT_UUID_SIZE, UPLBSProfileCharSysInfoUUID },
		GATT_PERMIT_READ, 
		0, 
		&UPLBSCharSysInfo 
	},
	// [ NO. 6 ] Characteristic 2 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		UPLBSCharSysInfoUserDesp 
	},
	// [ NO. 7 ] Characteristic 3 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&UPLBSCharTemprProps 
	},
	// [ NO. 8 ] Characteristic Value 3
	{ 
		{ ATT_BT_UUID_SIZE, UPLBSProfileCharTemprUUID },
		GATT_PERMIT_READ, 
		0, 
		(uint8 *)&UPLBSCharTempr 
	},
	// [ NO. 9 ] Characteristic 3 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		UPLBSCharTemprUserDesp 
	},
	// [ NO. 10 ] Characteristic 4 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&UPLBSCharAccParamProps 
	},
	// [ NO. 11 ] Characteristic Value 4
	{ 
		{ ATT_BT_UUID_SIZE, UPLBSProfileCharAccParamUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
		0, 
		&UPLBSCharAccParam 
	},
	// [ NO. 12 ] Characteristic 4 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		UPLBSCharAccParamUserDesp 
	},
	// [ NO. 13 ] Characteristic 5 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&UPLBSCharRunStateProps 
	},
	// [ NO. 14 ] Characteristic Value 5
	{ 
		{ ATT_BT_UUID_SIZE, UPLBSProfileCharRunStateUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, //GATT_PERMIT_AUTHEN_READ|GATT_PERMIT_AUTHEN_WRITE
		0, 
		&UPLBSCharRunState 
	},
	// [ NO. 15 ] Characteristic 5 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		UPLBSCharRunStateUserDesp 
	},
	// [ NO. 16 ] Characteristic 6 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&UPLBSCharPKStateProps 
	},
	// [ NO. 17 ] Characteristic Value 6
	{ 
		{ ATT_BT_UUID_SIZE, UPLBSProfileCharPkStateUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, //GATT_PERMIT_AUTHEN_READ|GATT_PERMIT_AUTHEN_WRITE
		0, 
		&UPLBSCharPKState 
	},
	// [ NO. 18 ] Characteristic 6 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		UPLBSCharPKStateUserDesp 
	},
	// [ NO. 19 ] Characteristic 7 Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		&UPLBSCharBecTypeProps
	},
	// [ NO. 20 ] Characteristic Value 7
	{
		{ ATT_BT_UUID_SIZE, UPLBSProfileCharBeaconTypeUUID},
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		&UPLBSCharBecType
	},
	// [ NO. 21 ] Characteristic 7 Declaration
	{
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		UPLBSCharBecTypeUserDesp
	},
	// [ NO. 22 ] Client characteristic config
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8 *) &UPLBSClientCharConfig
	},
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 UPLBSProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
									uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t UPLBSProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
									uint8 *pValue, uint8 len, uint16 offset );
static void UPLBSProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

static bStatus_t notify_data_by_idx(uint16 con_hdl,uint8 msg_idx, uint8 * noti_data , uint8 data_len);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// UPLBS Profile Service Callbacks
CONST gattServiceCBs_t UPLBSProfileCBs =
{
	UPLBSProfile_ReadAttrCB,		// Read callback function pointer
	UPLBSProfile_WriteAttrCB,	// Write callback function pointer
	NULL							// Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn			UPLBSProfile_AddService
 *
 * @brief	 Initializes the UPLBS Profile service by registering
 *					GATT attributes with the GATT server.
 *
 * @param	 services - services to add. This is a bit map and can
 *										 contain more than one service.
 *
 * @return	Success or Failure
 */
bStatus_t UPLBSProfile_AddService( uint32 services )
{
	uint8 status = SUCCESS;

	// Initialize Client Characteristic Configuration attributes
	GATTServApp_InitCharCfg( INVALID_CONNHANDLE, UPLBSClientCharConfig );

	// Register with Link DB to receive link status change callback
	VOID linkDB_Register( UPLBSProfile_HandleConnStatusCB );	
	
	if ( services & UPLBS_SERVICE )
	{
		// Register GATT attribute list and CBs with GATT Server App
		status = GATTServApp_RegisterService( UPLBSProfileAttrTbl,\
				GATT_NUM_ATTRS(UPLBSProfileAttrTbl), &UPLBSProfileCBs );
	}

	return ( status );
}


/*********************************************************************
 * @fn			UPLBSProfile_RegisterAppCBs
 *
 * @brief	 Registers the application callback function. Only call 
 *					this function once.
 *
 * @param	 callbacks - pointer to application callbacks.
 *
 * @return	SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t UPLBSProfile_RegisterAppCBs( UPLBSProfileCBs_t *appCallbacks )
{
	if ( appCallbacks )
	{
		UPLBSProfile_AppCBs = appCallbacks;
		
		return ( SUCCESS );
	}
	else
	{
		return ( bleAlreadyInRequestedMode );
	}
}
	

/*********************************************************************
 * @fn		UPLBSProfile_SetParameter
 *
 * @brief		Set a UPLBS Profile parameter.
 *
 * @param	param - Profile parameter ID
 * @param	len - length of data to right
 * @param	value - pointer to data to write.	This is dependent on
 *				the parameter ID and WILL be cast to the appropriate 
 *				data type (example: data type of uint16 will be cast to 
 *				uint16 pointer).
 *
 * @return	bStatus_t
 */
bStatus_t UPLBSProfile_SetParameter( uint8 param, uint8 len, void *value )
{
	bStatus_t ret = SUCCESS;
	switch ( param )
	{
		case UPLBS_CHAR_VERN:
		{
			// Do not need value
			VOID value;
			VOID len;	

			if (fill_vern_info(&UPLBSCharVern) == FALSE)
				ret = bleInvalidRange;	

			break;
		}
		case UPLBS_CHAR_SYS_INFO:
		{
			if ( len == sizeof(UPLBSCharSysInfo)) 
				UPLBSCharSysInfo= *((uint8 *)value);
			else
				ret = bleInvalidRange;

			break;
		}
		case UPLBS_CHAR_TEMPR:
		{
			if ( len == sizeof (UPLBSCharTempr) ) 
				UPLBSCharTempr = *((uint8*)value);
			else
				ret = bleInvalidRange;

			break;
		}
		case UPLBS_CHAR_ACC_PARAM:
		{
			if ( len == sizeof ( UPLBSCharAccParam ) ) 
				UPLBSCharAccParam = *((uint8*)value);
			else
				ret = bleInvalidRange;
			break;
		}
		case UPLBS_CHAR_RUN_STATE:
		{
			if ( len == sizeof ( UPLBSCharRunState ) ) 
				UPLBSCharRunState = *((uint8*)value);
			else
				ret = bleInvalidRange;
			break;
		}
		case UPLBS_CHAR_PARKING_STATE:
		{
			if ( len == sizeof ( UPLBSCharPKState ) ) 
				UPLBSCharPKState = *((uint8*)value);
			else
				ret = bleInvalidRange;

			break;
		}
		case UPLBS_CHAR_BEACON_TYPE:
		{
			if ( len == sizeof ( UPLBSCharBecType) ) 
				UPLBSCharBecType= *((uint8*)value);
			else
				ret = bleInvalidRange;

			break;
		}
		default:
			ret = INVALIDPARAMETER;
			break;
	}
	
	return ( ret );
}

/*********************************************************************
 * @fn			UPLBSProfile_GetParameter
 *
 * @brief	 Get a UPLBS Profile parameter.
 *
 * @param	 param - Profile parameter ID
 * @param	 value - pointer to data to put.	This is dependent on
 *					the parameter ID and WILL be cast to the appropriate 
 *					data type (example: data type of uint16 will be cast to 
 *					uint16 pointer).
 *
 * @return	bStatus_t
 */
bStatus_t UPLBSProfile_GetParameter( uint8 param, void *value )
{
	bStatus_t ret = SUCCESS;
	switch ( param )
	{
		case UPLBS_CHAR_VERN:
			VOID osal_memcpy( value, (uint8 *)&UPLBSCharVern, sizeof(UPLBSCharVern));
			break;

		case UPLBS_CHAR_SYS_INFO:
			VOID osal_memcpy( value, (uint8 *)&UPLBSCharSysInfo, sizeof(UPLBSCharSysInfo));
			break;			

		case UPLBS_CHAR_TEMPR:
			*((uint8*)value) = UPLBSCharTempr;
			break;	

		case UPLBS_CHAR_ACC_PARAM:
			*((uint8*)value) = UPLBSCharAccParam;
			break;

		case UPLBS_CHAR_RUN_STATE:
			*((uint8*)value) = UPLBSCharRunState;
			break;			
		case UPLBS_CHAR_PARKING_STATE:
			*((uint8*)value) = UPLBSCharPKState;
			break;
		case UPLBS_CHAR_BEACON_TYPE:
			*((uint8*)value) = UPLBSCharBecType;
			break;
		default:
			ret = INVALIDPARAMETER;
			break;
	}
	
	return ( ret );
}

/*********************************************************************
 * @fn			UPLBSProfile_ReadAttrCB
 *
 * @brief			Read an attribute.
 *
 * @param		connHandle - connection message was received on
 * @param		pAttr - pointer to attribute
 * @param		pValue - pointer to data to be read
 * @param		pLen - length of data to be read
 * @param		offset - offset of the first octet to be read
 * @param		maxLen - maximum length of data to be read
 *
 * @return		Success or Failure
 */
static uint8 UPLBSProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
						uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
	bStatus_t status = SUCCESS;

	// If attribute permissions require authorization to read, return error
	if ( gattPermitAuthorRead( pAttr->permissions ) )
		return ( ATT_ERR_INSUFFICIENT_AUTHOR );
	
	// Make sure it's not a blob operation (no attributes in the profile are long)
	if ( offset > 0 )
		return ( ATT_ERR_ATTR_NOT_LONG );
 
	if ( pAttr->type.len == ATT_BT_UUID_SIZE )
	{
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
		switch ( uuid )
		{
			// No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
			// gattserverapp handles those reads
			case UPLBS_CHAR_VERN_UUID:
			{
				*pLen =	sizeof(UPLBSCharVern);
				VOID osal_memcpy( pValue, pAttr->pValue,	sizeof(UPLBSCharVern) );
				break;
			}
			case UPLBS_CHAR_SYS_INFO_UUID:
			{
				*pLen =	sizeof(UPLBSCharSysInfo);
				VOID osal_memcpy( pValue, pAttr->pValue,	sizeof(UPLBSCharSysInfo) );
				break;
			}
			case UPLBS_CHAR_TEMPR_UUID:
			case UPLBS_CHAR_ACC_PARAM_UUID:
			case UPLBS_CHAR_RUN_STATE_UUID:
			case UPLBS_CHAR_BEACON_TYPE_UUID:
			{
				*pLen = 1;
				pValue[0] = *pAttr->pValue;
				break;
			}
			default:
				*pLen = 0;
				status = ATT_ERR_ATTR_NOT_FOUND;
				break;
		}
	}
	else
	{
		// 128-bit UUID
		*pLen = 0;
		status = ATT_ERR_INVALID_HANDLE;
	}

	return ( status );
}

/*********************************************************************
 * @fn		UPLBSProfile_WriteAttrCB
 *
 * @brief		Validate attribute data prior to a write operation
 *
 * @param	connHandle - connection message was received on
 * @param	pAttr - pointer to attribute
 * @param	pValue - pointer to data to be written
 * @param	len - length of data
 * @param	offset - offset of the first octet to be written
 *
 * @return	Success or Failure
 */
static bStatus_t UPLBSProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
							uint8 *pValue, uint8 len, uint16 offset )
{
	bStatus_t status = SUCCESS;
	uint8 notifyApp = 0xFF;
	
	// If attribute permissions require authorization to write, return error
	if ( gattPermitAuthorWrite( pAttr->permissions ) )
	{
		// Insufficient authorization
		return ( ATT_ERR_INSUFFICIENT_AUTHOR );
	}
	
	if ( pAttr->type.len == ATT_BT_UUID_SIZE )
	{
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
		switch ( uuid )
		{
			case UPLBS_CHAR_TEMPR_UUID:
			case UPLBS_CHAR_ACC_PARAM_UUID:
			case UPLBS_CHAR_RUN_STATE_UUID:
			case UPLBS_CHAR_PARKING_STATE_UUID:
			case UPLBS_CHAR_BEACON_TYPE_UUID:
			{
				//Validate the value
				// Make sure it's not a blob oper
				if ( offset == 0 )
				{
					if ( len != 1 )
						status = ATT_ERR_INVALID_VALUE_SIZE;
				}
				else
					status = ATT_ERR_ATTR_NOT_LONG;
				
				//Write the value
				if ( status == SUCCESS )
				{
					uint8 *pCurValue = (uint8 *)pAttr->pValue;				
					*pCurValue = pValue[0];

					if( pAttr->pValue == (uint8 *)&UPLBSCharTempr)
						notifyApp = UPLBS_CHAR_TEMPR;
					else if( pAttr->pValue == &UPLBSCharAccParam)
						notifyApp = UPLBS_CHAR_ACC_PARAM;			 
					else if (pAttr->pValue == & UPLBSCharRunState)
						notifyApp = UPLBS_CHAR_RUN_STATE;
					else if (pAttr->pValue == & UPLBSCharPKState)
						notifyApp = UPLBS_CHAR_PARKING_STATE;
					else if (pAttr->pValue == & UPLBSCharBecType)
						notifyApp = UPLBS_CHAR_BEACON_TYPE;
				}
						 
				break;
			}
			case GATT_CLIENT_CHAR_CFG_UUID:
			{
				status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,\
						offset, GATT_CLIENT_CFG_NOTIFY );
				break;
			}
			default:
				// Should never get here! (Some characteristics do not have write permissions)
				status = ATT_ERR_ATTR_NOT_FOUND;
				break;
		}
	}
	else	// 128-bit UUID
		status = ATT_ERR_INVALID_HANDLE;

	// If a charactersitic value changed then callback function to notify application of change
	if ( (notifyApp != 0xFF ) && UPLBSProfile_AppCBs && UPLBSProfile_AppCBs->pfnUPLBSProfileChange )
	{
		UPLBSProfile_AppCBs->pfnUPLBSProfileChange( notifyApp );	
	}
	
	return ( status );
}

/*********************************************************************
 * @fn		UPLBSProfile_PerformNotify
 *
 * @brief		Send a notification .
 *
 * @return	None.
 
static void UPLBSProfile_PerformNotify( void )
{
	// Execute linkDB callback to send notification of each connection
	linkDB_PerformFunc( UPLBSNotifyCB );
}

*********************************************************************
 * @fn		UPLBSNotifyCB
 *
 * @brief		Send a notification of the level state characteristic.
 *
 * @param	connHandle - linkDB item
 *
 * @return	None.

static void UPLBSNotifyCB( linkDBItem_t *pLinkItem )
{	
	if ( pLinkItem->stateFlags & LINK_CONNECTED )
	{
		LnkHdl= pLinkItem->connectionHandle;
		
		uint16 value = GATTServApp_ReadCharCfg( pLinkItem->connectionHandle,
										UPLBSClientCharConfig );
		if ( value & GATT_CLIENT_CFG_NOTIFY )
			VOID value;
	}
}
*/

/*********************************************************************
 * @fn		read_data_by_handle
 *
 * @brief	 	Central read data by handle.
 *
 * @return	none
 */
bStatus_t read_data_by_handle(uint16 conn_handle, uint16 read_handle, uint8 task_id)
{
	attReadReq_t rd_req;
	
	rd_req.handle = read_handle;

	return GATT_ReadCharValue( conn_handle, &rd_req, task_id );
}

/*********************************************************************
 * @fn		write_data_by_handle
 *
 * @brief	 	Central write data by handle.
 *
 * @return	none
 */
bStatus_t write_data_by_handle(uint16 conn_handle, uint16 write_handle,
			uint8 * send_data,uint8 send_len, uint8 task_id)
{
	attWriteReq_t wt_req;

	wt_req.handle = write_handle;
	wt_req.len = send_len;
	osal_memcpy(wt_req.value, send_data, send_len);
	wt_req.sig = 0;
	wt_req.cmd = 0;

	return GATT_WriteCharValue( conn_handle, &wt_req, task_id);
}

/*********************************************************************
 * @fn		init_blob_recv
 *
 * @brief	 	init blob recv data
 *
 * @param	none
 * @return	none
 */
void init_blob_recv(void)
{
	uint8 i;

	// start new blob packet
	for (i=0;i<MAX_PACKET_NUM;i++)
		PktRcvStatus[i]=FALSE;
}


/*********************************************************************
 * @fn		BLE_noti_recv_data
 *
 * @brief	 	receive notification data
 *
 * @param	none
 * @return	none
 */
bool BLE_noti_recv_data(attHandleValueNoti_t noti_src, uint8 *data, uint8 *len)
{
	uint8 i;

	UPLBS_pkt_fmt_t *recv_data =(UPLBS_pkt_fmt_t *) noti_src.value;

	if (recv_data->pkt_hdr.tot_pkt_cnt > MAX_PACKET_NUM)
		return TRUE;

	if (PktRcvStatus[recv_data->pkt_hdr.cur_pkt_num-1] == FALSE)
	{
//		PrintIntValue("\r\n:",*len,10);
//		PrintIntValue("\r\n:",recv_data->pkt_hdr.cur_pkt_num,10);
//		SendBinData(recv_data->pkt_txt,recv_data->pkt_hdr.pkt_len);
		
		osal_memcpy(data+(recv_data->pkt_hdr.cur_pkt_num-1)*UPLBS_MAX_PKT_TXT_LEN,\
				recv_data->pkt_txt, recv_data->pkt_hdr.pkt_len);
		*len = *len + recv_data->pkt_hdr.pkt_len;
		PktRcvStatus[recv_data->pkt_hdr.cur_pkt_num-1] = TRUE;
	}

	for (i=0;i<recv_data->pkt_hdr.tot_pkt_cnt-1;i++)
		if (PktRcvStatus[i] == FALSE)
			return FALSE;

	return TRUE;
}

/*********************************************************************
 * @fn		BLE_noti_send_data
 *
 * @brief	 	notification send data
 *
 * @param	none
 * @return	none
 */
void BLE_noti_send_data(uint16 con_hdl,uint8 hdl_idx, uint8 *data, int16 len)
{
	UPLBS_pkt_fmt_t basic_pkt;
	uint8 cur_send_cnt=0;
	
	basic_pkt.pkt_hdr.tot_pkt_cnt = len/UPLBS_MAX_PKT_TXT_LEN+1;
	basic_pkt.pkt_hdr.cur_pkt_num = 1;
	basic_pkt.pkt_hdr.pkt_status = IMAGE_VERSION_NUM;

	while (len > 0)
	{
		osal_memset(basic_pkt.pkt_txt,0,UPLBS_MAX_PKT_TXT_LEN);
		basic_pkt.pkt_hdr.pkt_len = (len > UPLBS_MAX_PKT_TXT_LEN? UPLBS_MAX_PKT_TXT_LEN: len);
		osal_memcpy(basic_pkt.pkt_txt, data+cur_send_cnt, basic_pkt.pkt_hdr.pkt_len);
		notify_data_by_idx(con_hdl,hdl_idx,(uint8 *)&basic_pkt,sizeof(basic_pkt));
		
		len -= basic_pkt.pkt_hdr.pkt_len;
		cur_send_cnt += basic_pkt.pkt_hdr.pkt_len;
		basic_pkt.pkt_hdr.cur_pkt_num ++;
	}

	return;
}


/*********************************************************************
 * @fn		notify_data_by_idx
 *
 * @brief		Send a notification of the level state characteristic.
 *
 * @param	connHandle - linkDB item
 *
 * @return	None.
 */
static bStatus_t notify_data_by_idx(uint16 con_hdl, uint8 msg_idx, uint8 * noti_data , uint8 data_len)
{
	attHandleValueNoti_t noti;

	noti.handle = UPLBSProfileAttrTbl[msg_idx].handle;
	noti.len = data_len;
	osal_memcpy(noti.value, noti_data, data_len);

	return GATT_Notification(con_hdl, &noti, FALSE );
}

/*********************************************************************
 * @fn			UPLBSProfile_HandleConnStatusCB
 *
 * @brief			UPLBS Profile link status change handler function.
 *
 * @param		connHandle - connection handle
 * @param		changeType - type of change
 *
 * @return		none
 */
static void UPLBSProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
	// Make sure this is not loopback connection
	if ( connHandle != LOOPBACK_CONNHANDLE )
	{
		// Reset Client Char Config if connection has dropped
		if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )			||
				 ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
					 ( !linkDB_Up( connHandle ) ) ) )
		{ 
			GATTServApp_InitCharCfg( connHandle, UPLBSClientCharConfig );
		}
	}
}

/*********************************************************************
 * @fn			fill_vern_info
 *
 * @brief			fill version information.
 *
 * @param		vern_info - version information struct
 *
 * @return		TRUE - fill success
 */

bool fill_vern_info(UPLBS_vern_t * vern_info)
{
	char *vrn_num_bgn;
	uint8 vrn_num_chk;

	vern_info->img_vern = t_htonl(IMAGE_VERSION_NUM);
	VOID osal_memcpy( vern_info->vern_date, UPLBS_RELEASE_DATE, VERSION_DATE_LEN);
			
	vrn_num_bgn = UPLBS_SOFTWARE_VERSION;
	vrn_num_chk = 0;
	// Find Character 'V'
	while(*vrn_num_bgn != 'V' && vrn_num_chk < osal_strlen(UPLBS_SOFTWARE_VERSION))
	{
		vrn_num_bgn ++;
		vrn_num_chk ++;
	}
	// Check over flow
	if (vrn_num_chk+VERSION_NUMBER_LEN >= osal_strlen(UPLBS_SOFTWARE_VERSION))
		return FALSE;
	VOID osal_memcpy( vern_info->sw_vern_num, ++vrn_num_bgn, VERSION_NUMBER_LEN);

	vrn_num_bgn = UPLBS_HARDWARE_VERSION;
	vrn_num_chk = 0;
	while(*vrn_num_bgn != 'V' && vrn_num_chk < osal_strlen(UPLBS_HARDWARE_VERSION))
	{
		vrn_num_bgn ++;
		vrn_num_chk ++;
	}
	if (vrn_num_chk+VERSION_NUMBER_LEN >= osal_strlen(UPLBS_HARDWARE_VERSION))
		return FALSE;
	VOID osal_memcpy( vern_info->hw_vern_num, ++vrn_num_bgn, VERSION_NUMBER_LEN);

	return TRUE;
}

/*********************************************************************
*********************************************************************/
