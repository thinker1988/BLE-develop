#ifndef PKT_FMT_H_
#define PKT_FMT_H_


#ifdef __cplusplus
extern "C"
{
#endif

#include "bcomdef.h"

/********************************************************************************
| ID | Tot len | Sub type | reserved | chk sum | src ID | dest ID | version | payload |
| 1  |    1     |    1     |    2     |    1    |    2   |    2    |    2    |  128-12 |
*/
/****************RF ID*******************/
#define GMS_ID_POS		0
#define GMS_ID_SIZE		1

#define GME_SRC_ID		0xC8
#define GDE_SRC_ID		0x55
#define GTE_SRC_ID		0x66

/**************total length*******************/
#define GMS_TOT_LEN_POS		(GMS_ID_POS+GMS_ID_SIZE)	//0+1
#define GMS_TOT_LEN_SIZE	1

#define GMS_PKT_MAX_LEN	128

/**************sub type*******************/
#define GMS_SUB_TYPE_POS	(GMS_TOT_LEN_POS+GMS_TOT_LEN_SIZE)
#define GMS_SUB_TYPE_SIZE	1

// Direction: GME==>GDE
#define GME_ST_HRTBEAT_ACK	1
#define GME_ST_CARINFO_ACK	2
#define GME_ST_TMSYN_ACK	3
#define GME_ST_UPGD_REQ		4

// Direction: GDE==>GME
#define GDE_ST_HRTBEAT_REQ	21
#define GDE_ST_CARINFO_REQ	22
#define GDE_ST_TMSYN_REQ	23
#define GDE_ST_UPGD_REQ_ACK	24
#define GDE_ST_UPGD_ACK		25
// Direction: GDE==>GTE
#define GDE_ST_T_REQ_ACK	26
#define GDE_ST_T_READ_ACK	27
#define GDE_ST_T_SET_ACK	28

// Direction: GTE==>GDE
#define GTE_ST_PARAM_REQ	41
#define GTE_ST_PARAM_READ	42
#define GTE_ST_PARAM_SET	43
#define GTE_ST_UPGD_REQ		44

/**************reserved*******************/
#define GMS_RESERVE_POS		(GMS_SUB_TYPE_POS+GMS_SUB_TYPE_SIZE)
#define GMS_RESERVE_SIZE	2
#define GMS_RESERVE_STR		"RF"

#define GMS_CHK_SUM_POS		(GMS_RESERVE_POS+GMS_RESERVE_SIZE)
#define GMS_CHK_SUM_SIZE	1


#define GMS_SRC_ADDR_POS	(GMS_CHK_SUM_POS+GMS_CHK_SUM_SIZE)	//5+1
#define GMS_SRC_ADDR_SIZE	2


#define GMS_DEST_ADDR_POS	(GMS_SRC_ADDR_POS+GMS_SRC_ADDR_SIZE)	//6+2
#define GMS_DEST_ADDR_SIZE	2


#define GMS_VERSION_POS		(GMS_DEST_ADDR_POS+GMS_DEST_ADDR_SIZE)	//8+2
#define GMS_VERSION_SIZE	2


#define GMS_PKT_HDR_SIZE	(GMS_VERSION_POS+GMS_VERSION_SIZE)	//12
#define GMS_PKT_PLD_MAX_LEN	(GMS_PKT_MAX_LEN-GMS_PKT_HDR_SIZE)	//128-12

#define GMS_PKT_PAYLOAD_POS	GMS_PKT_HDR_SIZE		//12

#define EID_SIZE			1
#define EVAL_LEN_SIZE		1
#define ELM_HDR_SIZE		(EID_SIZE+EVAL_LEN_SIZE)	// 2

// List of element ID
#define EID_UTCL			1
#define EID_GDE_HRTBT		2
#define EID_GDE_CAR_INFO	3
#define EID_GME_INFO_ACK	4
#define EID_GTE_SET			5
#define EID_GDE_SET_ACK		6
#define EID_GDE_TMSYN		7
#define EID_GME_TMSYN_ACK	8
#define EID_GMS_UPGD		9
#define EID_GDE_UPGD_ACK	10
#define EID_GTE_SET_REQ		11
#define EID_GDE_SET_RESP	12
#define EID_GTE_READ		13
#define EID_GDE_READ_ACK	14


// Element length of different ID
// EID = 1
#define UTCL_EVAL_LEN		(UTCL_YEAR_POS+ UTCL_YEAR_SIZE)// 6
// EID = 2
#define GDE_HRTBT_LEN		(HRT_BT_STAT_POS+HRT_BT_STAT_SIZE)// 9
// EID = 3
#define GDE_CAR_INFO_LEN	GDE_HRTBT_LEN
// EID = 4
#define GME_INFO_ACK_LEN	1
// EID = 5
#define GTE_SET_LEN			(ST_VERSION_L_POS+ST_VERSION_L_SIZE)// 21
// EID = 6
#define GDE_SET_ACK_LEN		1
// EID = 7
#define GDE_TMSYN_LEN		1
// EID = 8
#define GME_TMSYN_ACK_LEN	6
// EID = 9
#define GMS_UPGD_LEN		7
// EID = 10
#define GDE_UPGD_ACK_LEN	1
// EID = 11
#define GDE_PREREQ_LEN		(ST_PREUPGD_FREQ_POS+ST_PREUPGD_FREQ_SIZE)// 1
// EID = 12
#define GDE_PREREQ_ACK_LEN	GDE_PREREQ_LEN
// EID = 13
#define GDE_READ_LEN		1
// EID = 14
#define GDE_READ_ACK_LEN	GTE_SET_LEN

// UTC local element position define
// Element ID = 1
#define UTCL_HOUR_POS		0
#define UTCL_HOUR_SIZE		1

#define UTCL_MINTS_POS		(UTCL_HOUR_POS+UTCL_HOUR_SIZE)	// 1
#define UTCL_MINTS_SIZE		1

#define UTCL_SECND_POS		(UTCL_MINTS_POS+UTCL_MINTS_SIZE)	// 2
#define UTCL_SECND_SIZE		1

#define UTCL_DAY_POS		(UTCL_SECND_POS+UTCL_SECND_SIZE)	// 3
#define UTCL_DAY_SIZE		1

#define UTCL_MONTH_POS		(UTCL_DAY_POS+UTCL_DAY_SIZE)	// 4
#define UTCL_MONTH_SIZE		1

#define UTCL_YEAR_POS		(UTCL_MONTH_POS+UTCL_MONTH_SIZE)	// 5
#define UTCL_YEAR_SIZE		1


// Heart beat data element position define
// Element ID = 2 & 3
#define HRT_BT_BATT_POS		0	// 0
#define HRT_BT_BATT_SIZE	1

#define HRT_BT_TMPR_POS		(HRT_BT_BATT_POS+HRT_BT_BATT_SIZE)	// 1
#define HRT_BT_TMPR_SIZE	1

#define HRT_BT_XVAL_POS		(HRT_BT_TMPR_POS+HRT_BT_TMPR_SIZE)	// 2
#define HRT_BT_XVAL_SIZE	2

#define HRT_BT_YVAL_POS		(HRT_BT_XVAL_POS+HRT_BT_XVAL_SIZE)	// 4
#define HRT_BT_YVAL_SIZE	2

#define HRT_BT_ZVAL_POS		(HRT_BT_YVAL_POS+HRT_BT_YVAL_SIZE)	// 6
#define HRT_BT_ZVAL_SIZE	2

#define HRT_BT_STAT_POS		(HRT_BT_ZVAL_POS+HRT_BT_ZVAL_SIZE)	// 8
#define HRT_BT_STAT_SIZE	1

// GTE set or read data element position define
// Element ID = 5
#define ST_RF_FREQ_POS		0	// 0
#define ST_RF_FREQ_SIZE		1

#define ST_RF_ST_FREQ_POS		(ST_RF_FREQ_POS+ST_RF_FREQ_SIZE)	// 1
#define ST_RF_ST_FREQ_SIZE		1

#define ST_RF_UPGD_FREQ_POS		(ST_RF_ST_FREQ_POS+ST_RF_ST_FREQ_SIZE)	// 2
#define ST_RF_UPGD_FREQ_SIZE	1

#define ST_RF_BAUD_POS		(ST_RF_UPGD_FREQ_POS+ST_RF_UPGD_FREQ_SIZE)	// 3
#define ST_RF_BAUD_SIZE		1

#define ST_RF_PWR_LVL_POS	(ST_RF_BAUD_POS+ST_RF_BAUD_SIZE)	// 4
#define ST_RF_PWR_LVL_SIZE	1

#define ST_GM_HB_FREQ_POS	(ST_RF_PWR_LVL_POS+ST_RF_PWR_LVL_SIZE)	// 5
#define ST_GM_HB_FREQ_SIZE	1

#define ST_GM_DTCT_VAL_POS	(ST_GM_HB_FREQ_POS+ST_GM_HB_FREQ_SIZE)	// 6
#define ST_GM_DTCT_VAL_SIZE	1

#define ST_GM_DTCT_ALG_POS	(ST_GM_DTCT_VAL_POS+ST_GM_DTCT_VAL_SIZE)	// 7
#define ST_GM_DTCT_ALG_SIZE	1

#define ST_GM_STATUS_POS	(ST_GM_DTCT_ALG_POS+ST_GM_DTCT_ALG_SIZE)	// 8
#define ST_GM_STATUS_SIZE	1

#define ST_GM_X_H_BCHMRK_POS	(ST_GM_STATUS_POS+ST_GM_STATUS_SIZE) 	// 9
#define ST_GM_X_H_BCHMRK_SIZE	1
#define ST_GM_X_L_BCHMRK_POS	(ST_GM_X_H_BCHMRK_POS+ST_GM_X_H_BCHMRK_SIZE) 	// 10
#define ST_GM_X_L_BCHMRK_SIZE	1

#define ST_GM_Y_H_BCHMRK_POS	(ST_GM_X_L_BCHMRK_POS+ST_GM_X_L_BCHMRK_SIZE) 	// 11
#define ST_GM_Y_H_BCHMRK_SIZE	1
#define ST_GM_Y_L_BCHMRK_POS	(ST_GM_Y_H_BCHMRK_POS+ST_GM_Y_H_BCHMRK_SIZE) 	// 12
#define ST_GM_Y_L_BCHMRK_SIZE	1

#define ST_GM_Z_H_BCHMRK_POS	(ST_GM_Y_L_BCHMRK_POS+ST_GM_Y_L_BCHMRK_SIZE) 	// 13
#define ST_GM_Z_H_BCHMRK_SIZE	1
#define ST_GM_Z_L_BCHMRK_POS	(ST_GM_Z_H_BCHMRK_POS+ST_GM_Z_H_BCHMRK_SIZE) 	// 14
#define ST_GM_Z_L_BCHMRK_SIZE	1

#define ST_GDE_ADDR_H_POS	(ST_GM_Z_L_BCHMRK_POS+ST_GM_Z_L_BCHMRK_SIZE)	// 15
#define ST_GDE_ADDR_H_SIZE	1
#define ST_GDE_ADDR_L_POS	(ST_GDE_ADDR_H_POS+ST_GDE_ADDR_H_SIZE)	// 16
#define ST_GDE_ADDR_L_SIZE	1

#define ST_GME_ADDR_H_POS	(ST_GDE_ADDR_L_POS+ST_GDE_ADDR_L_SIZE)	// 17
#define ST_GME_ADDR_H_SIZE	1
#define ST_GME_ADDR_L_POS	(ST_GME_ADDR_H_POS+ST_GME_ADDR_H_SIZE)	// 18
#define ST_GME_ADDR_L_SIZE	1

#define ST_VERSION_H_POS	(ST_GME_ADDR_L_POS+ST_GME_ADDR_L_SIZE)	// 19
#define ST_VERSION_H_SIZE	1
#define ST_VERSION_L_POS	(ST_VERSION_H_POS+ST_VERSION_H_SIZE)	// 20
#define ST_VERSION_L_SIZE	1

// GTE set or read data element position define
// Element ID = 11 & 12
#define ST_PREUPGD_FREQ_POS		0	// 0
#define ST_PREUPGD_FREQ_SIZE	1


typedef enum rferr
{
	RF_SUCCESS,	//0
	PKT_DENY,
	PKT_ERR,
	SUB_TYPE_ERR,
	CHKSUM_ERR,
	BAD_ADDR,
	DATA_ERR
}rferr_t;


extern void InitDevID(void);

extern void GMSPktForm(uint8 *rawbuf, uint8 rawlen);

extern rferr_t RFDataParse(uint8 *rfdata,uint8 len);

extern rferr_t RFDataForm(uint8 subtype, uint8 *data, uint8 datalen);

extern rferr_t RFDataSend(uint8 *buf, uint8 len);

#ifdef __cplusplus
}
#endif

#endif
