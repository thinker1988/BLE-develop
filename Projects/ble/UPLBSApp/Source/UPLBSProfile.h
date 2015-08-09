/**************************************************************************************************
	Filename:			 UPLBSprofile.h
	Revised:				$Date: 2015-01-23 08:56:11 -0700 $
	Revision:			 $Revision: 23333 $

	Description:		This file contains the UPLBS GATT profile definitions and
									prototypes.

	Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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

#ifndef UPLBSGATTPROFILE_H
#define UPLBSGATTPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "att.h"
/*********************************************************************
 * CONSTANTS
 */
// Version string from 'V' to last '_', e.g. "01.00.00.01"
#define VERSION_NUMBER_LEN				11
#define VERSION_DATE_LEN					8

// Profile Parameters
#define UPLBS_CHAR_VERN					0
#define UPLBS_CHAR_SYS_INFO				1
#define UPLBS_CHAR_TEMPR					2
#define UPLBS_CHAR_ACC_PARAM				3
#define UPLBS_CHAR_RUN_STATE				4
#define UPLBS_CHAR_PARKING_STATE			5
#define UPLBS_CHAR_BEACON_TYPE			6

// UPLBS Profile Service UUID
#define UPLBS_SERV_UUID					0xF000
// Characteristic UUID
#define UPLBS_CHAR_VERN_UUID				0xF001
#define UPLBS_CHAR_SYS_INFO_UUID			0xF002
#define UPLBS_CHAR_TEMPR_UUID				0xF003
#define UPLBS_CHAR_ACC_PARAM_UUID		0xF004
#define UPLBS_CHAR_RUN_STATE_UUID		0xF005
#define UPLBS_CHAR_PARKING_STATE_UUID	0xF006
#define UPLBS_CHAR_BEACON_TYPE_UUID		0xF007

// UPLBS Profile Services bit fields
#define UPLBS_SERVICE						0x00000001

// Notification data handle in attribute number position
#define BOOT_TIME_HDL_IDX					5	//UPLBS_CHAR_SYS_INFO
#define TEMPR_VAL_HDL_IDX					8	//UPLBS_CHAR_TEMPR
#define ACC_PARAMS_HDL_IDX				11	//UPLBS_CHAR_ACC_PARAM
#define SCAN_DATA_HDL_IDX					14	//UPLBS_CHAR_RUN_STATE


// Packet max length
#define UPLBS_MAX_PKT_TXT_LEN				16

// System working state
enum
{
	SYS_CENTRAL = 1,
	SYS_PERIPHERAL,
	SYS_SENT_OVER,
	SYS_DATA_RESEND,
	SYS_REBOOT,
	SYS_UPGRADE
};

// System parking state
enum
{
	CAR_DIR_IN,
	CAR_DIR_OUT,
	CAR_DIR_UDF
};

// UPLBS long data status
enum
{
	PKT_SEND=1,
	PKT_RECV,
	PKT_RESEND
};

/*********************************************************************
 * TYPEDEFS
 */
// Callback when a characteristic value has changed
typedef NULL_OK void (*UPLBSProfileChange_t)( uint8 paramID );

typedef struct
{
	UPLBSProfileChange_t	pfnUPLBSProfileChange;	// Called when characteristic value changes
} UPLBSProfileCBs_t;

// Version type
typedef struct
{
	uint32 img_vern;
	uint8 vern_date[VERSION_DATE_LEN];
	uint8 sw_vern_num[VERSION_NUMBER_LEN];
	uint8 hw_vern_num[VERSION_NUMBER_LEN];
}UPLBS_vern_t;

// UPLBS long data header format
typedef struct
{
	uint8 tot_pkt_cnt;
	uint8 cur_pkt_num;
	uint8 pkt_len;
	uint8 pkt_status;	// reserve
}UPLBS_pkt_hdr_t;

// UPLBS long data format
typedef struct
{
	UPLBS_pkt_hdr_t pkt_hdr;
	uint8 pkt_txt[UPLBS_MAX_PKT_TXT_LEN];
}UPLBS_pkt_fmt_t;


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */
		

/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * UPLBSProfile_AddService- Initializes the UPLBS GATT Profile service by registering
 *					GATT attributes with the GATT server.
 *
 * @param	 services - services to add. This is a bit map and can
 *										 contain more than one service.
 */

extern bStatus_t UPLBSProfile_AddService( uint32 services );

/*
 * UPLBSProfile_RegisterAppCBs - Registers the application callback function.
 *										Only call this function once.
 *
 *		appCallbacks - pointer to application callbacks.
 */
extern bStatus_t UPLBSProfile_RegisterAppCBs( UPLBSProfileCBs_t *appCallbacks );

/*
 * UPLBSProfile_SetParameter - Set a UPLBS GATT Profile parameter.
 *
 *		param - Profile parameter ID
 *		len - length of data to right
 *		value - pointer to data to write.	This is dependent on
 *					the parameter ID and WILL be cast to the appropriate 
 *					data type (example: data type of uint16 will be cast to 
 *					uint16 pointer).
 */
extern bStatus_t UPLBSProfile_SetParameter( uint8 param, uint8 len, void *value );
	
/*
 * UPLBSProfile_GetParameter - Get a UPLBS GATT Profile parameter.
 *
 *		param - Profile parameter ID
 *		value - pointer to data to write.	This is dependent on
 *					the parameter ID and WILL be cast to the appropriate 
 *					data type (example: data type of uint16 will be cast to 
 *					uint16 pointer).
 */
extern bStatus_t UPLBSProfile_GetParameter( uint8 param, void *value );

extern bStatus_t read_data_by_handle(uint16 conn_handle,uint16 read_handle,uint8 task_id);
extern bStatus_t write_data_by_handle(uint16 conn_handle,uint16 write_handle,uint8 * send_data,uint8 send_len,uint8 task_id);

extern void init_blob_recv(void);


extern bool BLE_noti_recv_data(attHandleValueNoti_t noti_src, uint8 *data, uint8 *len);
extern void BLE_noti_send_data(uint16 con_hdl,uint8 hdl_idx, uint8 *data, int16 len);


extern bool fill_vern_info(UPLBS_vern_t * vern_info);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* UPLBSGATTPROFILE_H */
