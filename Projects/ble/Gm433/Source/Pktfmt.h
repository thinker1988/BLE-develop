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
#define GME_SUBTYPE_HRTBEAT_ACK		1	// ELE 6+9+8+1
#define GME_SUBTYPE_CARINFO_ACK		2	// ELE 4
#define GME_SUBTYPE_TMSYN_ACK		3	// ELE 8
#define GME_SUBTYPE_UPGD_PKT		4	// ELE 11
#define GME_SUBTYPE_RST_BENCH_PKT	5	// ELE 3

// Direction: GDE==>GME
#define GDE_SUBTYPE_HRTBEAT_REQ		21	// ELE 1+2
#define GDE_SUBTYPE_CARINFO_REQ		22	// ELE 1+2+3
#define GDE_SUBTYPE_TMSYN_REQ		23	// ELE 7
#define GDE_SUBTYPE_UPGD_REQ_ACK	24	// ELE 6
#define GDE_SUBTYPE_UPGD_ACK		25	// ELE 4

// Direction: GDE==>GTE
#define GDE_SUBTYPE_T_PRESET_ACK	26	// ELE 6
#define GDE_SUBTYPE_T_READ_ACK		27	// ELE 5+3
#define GDE_SUBTYPE_T_SET_ACK		28	// ELE 4
#define GDE_SUBTYPE_T_UPGD_REQ_ACK	29	// ELE 6
#define GDE_SUBTYPE_T_UPGD_ACK		30	// ELE 4


// Direction: GTE==>GDE
#define GTE_SUBTYPE_PRESET_REQ		41	// ELE 6
#define GTE_SUBTYPE_PARAM_READ		42	// ELE 10
#define GTE_SUBTYPE_PARAM_SET		43	// ELE 5
#define GTE_SUBTYPE_UPGD_REQ		44	// ELE 6+9
#define GTE_SUBTYPE_UPGD_PKT		45	// ELE 11
#define GTE_SUBTYPE_RST_BENCH_PKT	46	// ELE 3


/**************reserved*******************/
#define GMS_RESERVE_POS		(GMS_SUB_TYPE_POS+GMS_SUB_TYPE_SIZE)
#define GMS_RESERVE_SIZE	2
#define GMS_RESERVE_STR		"RF"

/************XOR checksum****************/
#define GMS_CHK_SUM_POS		(GMS_RESERVE_POS+GMS_RESERVE_SIZE)
#define GMS_CHK_SUM_SIZE	1

/**************source address*****************/
#define GMS_SRC_ADDR_POS	(GMS_CHK_SUM_POS+GMS_CHK_SUM_SIZE)	//5+1
#define GMS_SRC_ADDR_SIZE	2

/**************destination address*****************/
#define GMS_DEST_ADDR_POS	(GMS_SRC_ADDR_POS+GMS_SRC_ADDR_SIZE)	//6+2
#define GMS_DEST_ADDR_SIZE	2

/**************version number****************/
#define GMS_VERSION_POS		(GMS_DEST_ADDR_POS+GMS_DEST_ADDR_SIZE)	//8+2
#define GMS_VERSION_SIZE	2

/**************packet header*****************/
#define GMS_PKT_HDR_SIZE	(GMS_VERSION_POS+GMS_VERSION_SIZE)	//12
#define GMS_PKT_PLD_MAX_LEN	(GMS_PKT_MAX_LEN-GMS_PKT_HDR_SIZE)	//128-12

// Payload start position
#define GMS_PKT_PAYLOAD_POS	GMS_PKT_HDR_SIZE	//12

/**************element header*****************/
#define EID_SIZE			1
#define EVAL_LEN_SIZE		1
#define ELM_HDR_SIZE		(EID_SIZE+EVAL_LEN_SIZE)	// 2

/************** list of element ID*****************/
#define EID_GDE_LOC_TM		1
#define EID_GDE_HRTBT		2
#define EID_GDE_BENCH_INFO	3
#define EID_GMS_INFO_ACK	4
#define EID_GDE_PARAMS		5
#define EID_GMS_RF_FREQ		6
#define EID_GDE_TMSYN		7
#define EID_GME_NT_TM		8
#define EID_GMS_FW_INFO		9
#define EID_GTE_READ		10
#define EID_GMS_UPGD		11

/*
#define EID_GTE_PRESET_REQ	11
#define EID_GDE_PRESET_RESP	12
#define EID_GTE_READ		13
#define EID_GMS_UPGD_REQ	14
#define EID_GDE_UPGD_RESP	15
*/

/**************element length of different ID*****************/
// EID = 1
#define GDE_LOC_TM_LEN		(UTCL_YEAR_POS+ UTCL_YEAR_SIZE)	// 6
// EID = 2
#define GDE_HRTBT_LEN		(HRT_BT_STAT_POS+HRT_BT_STAT_SIZE)	// 9
// EID = 3
#define GDE_BENCH_INFO_LEN	(GDE_Z_L_BCHMRK_POS+GDE_Z_L_BCHMRK_SIZE)	// 6
// EID = 4
#define GMS_INFO_ACK_LEN	(GME_INFO_ACK_MSG_POS+GME_INFO_ACK_MSG_SIZE)	// 1
// EID = 5
#define GDE_PARAMS_LEN		(ST_GME_ADDR_L_POS+ST_GME_ADDR_L_SIZE)	// 13
// EID = 6
#define GMS_RF_FREQ_LEN		(GMS_RF_FREQ_MSG_POS+GMS_RF_FREQ_MSG_SIZE)	// 1
// EID = 7
#define GDE_TMSYN_LEN		(GDE_TMSYNC_MSG_POS+GDE_TMSYNC_MSG_SIZE)	// 1
// EID = 8
#define GME_NT_TM_LEN		GDE_LOC_TM_LEN
// EID = 9
#define GMS_FW_INFO_LEN		(NEW_FW_UPGD_SECND_POS+NEW_FW_UPGD_SECND_SIZE)	// 13
// EID = 10
#define GTE_READ_LEN		(GTE_RD_REQ_MSG_POS+GTE_RD_REQ_MSG_SIZE)	// 1

// EID = 11
// Caution : this length does not include firmware block length, it should add at most RF_OAD_BLOCK_SIZE. 
#define GMS_UPGD_LEN		(NEW_FW_CUR_BLK_L_POS+NEW_FW_CUR_BLK_L_SIZE+RF_OAD_BLOCK_SIZE)	// 66

/*
// EID = 11
#define GDE_MOD_CHG_LEN		()

#define GTE_PRESET_REQ_LEN	(PRESET_REQ_MSG_POS+PRESET_REQ_MSG_SIZE)	// 1
// EID = 12
#define GDE_PRESET_RESP_LEN	(PRESET_RESP_MSG_POS+PRESET_RESP_MSG_SIZE)	// 1
// EID = 13
#define GTE_READ_LEN		(GTE_RD_REQ_MSG_POS+GTE_RD_REQ_MSG_SIZE)	// 1
// EID = 14
#define GMS_UPGD_REQ_LEN	(PREUPGD_REQ_MSG_POS+PREUPGD_REQ_MSG_SIZE)	// 1
// EID = 15
#define GDE_UPGD_RESP_LEN	(PREUPGD_RESP_MSG_POS+PREUPGD_RESP_MSG_SIZE)	// 1
*/


/**************element value content length*****************/

// UTC local element position define
// Element ID = 1, Len = GDE_LOC_TM_LEN
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
// Element ID = 2, Len = GDE_HRTBT_LEN
#define HRT_BT_BATT_POS		0	// 0
#define HRT_BT_BATT_SIZE	1

#define HRT_BT_TMPR_POS		(HRT_BT_BATT_POS+HRT_BT_BATT_SIZE)	// 1
#define HRT_BT_TMPR_SIZE	1

#define HRT_BT_XVAL_H_POS	(HRT_BT_TMPR_POS+HRT_BT_TMPR_SIZE)	// 2
#define HRT_BT_XVAL_H_SIZE	1
#define HRT_BT_XVAL_L_POS	(HRT_BT_XVAL_H_POS+HRT_BT_XVAL_H_SIZE)	// 3
#define HRT_BT_XVAL_L_SIZE	1

#define HRT_BT_YVAL_H_POS	(HRT_BT_XVAL_L_POS+HRT_BT_XVAL_L_SIZE)	// 4
#define HRT_BT_YVAL_H_SIZE	1
#define HRT_BT_YVAL_L_POS	(HRT_BT_YVAL_H_POS+HRT_BT_YVAL_H_SIZE)	// 5
#define HRT_BT_YVAL_L_SIZE	1

#define HRT_BT_ZVAL_H_POS	(HRT_BT_YVAL_L_POS+HRT_BT_YVAL_L_SIZE)	// 6
#define HRT_BT_ZVAL_H_SIZE	1
#define HRT_BT_ZVAL_L_POS	(HRT_BT_ZVAL_H_POS+HRT_BT_ZVAL_H_SIZE)	// 7
#define HRT_BT_ZVAL_L_SIZE	1

#define HRT_BT_STAT_POS		(HRT_BT_ZVAL_L_POS+HRT_BT_ZVAL_L_SIZE)	// 8
#define HRT_BT_STAT_SIZE	1


// GDE benchmark information element position define
// Element ID = 3, Len = GDE_BENCH_INFO_LEN
#define GDE_X_H_BCHMRK_POS	0	// 0
#define GDE_X_H_BCHMRK_SIZE	1
#define GDE_X_L_BCHMRK_POS	(GDE_X_H_BCHMRK_POS+GDE_X_H_BCHMRK_SIZE) 	// 1
#define GDE_X_L_BCHMRK_SIZE	1

#define GDE_Y_H_BCHMRK_POS	(GDE_X_L_BCHMRK_POS+GDE_X_L_BCHMRK_SIZE) 	// 2
#define GDE_Y_H_BCHMRK_SIZE	1
#define GDE_Y_L_BCHMRK_POS	(GDE_Y_H_BCHMRK_POS+GDE_Y_H_BCHMRK_SIZE) 	// 3
#define GDE_Y_L_BCHMRK_SIZE	1

#define GDE_Z_H_BCHMRK_POS	(GDE_Y_L_BCHMRK_POS+GDE_Y_L_BCHMRK_SIZE) 	// 4
#define GDE_Z_H_BCHMRK_SIZE	1
#define GDE_Z_L_BCHMRK_POS	(GDE_Z_H_BCHMRK_POS+GDE_Z_H_BCHMRK_SIZE) 	// 5
#define GDE_Z_L_BCHMRK_SIZE	1


// GME information acknowledge element position define
// Element ID = 4, Len = GMS_INFO_ACK_LEN
#define GME_INFO_ACK_MSG_POS	0	// 0
#define GME_INFO_ACK_MSG_SIZE	1


// GDE parameters element position define
// Element ID = 5, Len = GDE_PARAMS_LEN
#define ST_RF_WK_FREQ_POS		0	// 0
#define ST_RF_WK_FREQ_SIZE		1

#define ST_RF_ST_FREQ_POS		(ST_RF_WK_FREQ_POS+ST_RF_WK_FREQ_SIZE)	// 1
#define ST_RF_ST_FREQ_SIZE		1

#define ST_RF_UPGD_FREQ_POS		(ST_RF_ST_FREQ_POS+ST_RF_ST_FREQ_SIZE)	// 2
#define ST_RF_UPGD_FREQ_SIZE	1

#define ST_RF_AIR_BAUD_POS		(ST_RF_UPGD_FREQ_POS+ST_RF_UPGD_FREQ_SIZE)	// 3
#define ST_RF_AIR_BAUD_SIZE		1

#define ST_RF_PWR_LVL_POS		(ST_RF_AIR_BAUD_POS+ST_RF_AIR_BAUD_SIZE)	// 4
#define ST_RF_PWR_LVL_SIZE		1

#define ST_GM_HB_FREQ_POS		(ST_RF_PWR_LVL_POS+ST_RF_PWR_LVL_SIZE)	// 5
#define ST_GM_HB_FREQ_SIZE		1

#define ST_GM_DTCT_SENS_POS		(ST_GM_HB_FREQ_POS+ST_GM_HB_FREQ_SIZE)	// 6
#define ST_GM_DTCT_SENS_SIZE	1

#define ST_GM_BENCH_ALG_POS		(ST_GM_DTCT_SENS_POS+ST_GM_DTCT_SENS_SIZE)	// 7
#define ST_GM_BENCH_ALG_SIZE	1

#define ST_GM_STATUS_POS		(ST_GM_BENCH_ALG_POS+ST_GM_BENCH_ALG_SIZE)	// 8
#define ST_GM_STATUS_SIZE		1

#define ST_GDE_ADDR_H_POS		(ST_GM_STATUS_POS+ST_GM_STATUS_SIZE) 	// 9
#define ST_GDE_ADDR_H_SIZE		1
#define ST_GDE_ADDR_L_POS		(ST_GDE_ADDR_H_POS+ST_GDE_ADDR_H_SIZE)	// 10
#define ST_GDE_ADDR_L_SIZE		1

#define ST_GME_ADDR_H_POS		(ST_GDE_ADDR_L_POS+ST_GDE_ADDR_L_SIZE)	// 11
#define ST_GME_ADDR_H_SIZE		1
#define ST_GME_ADDR_L_POS		(ST_GME_ADDR_H_POS+ST_GME_ADDR_H_SIZE)	// 12
#define ST_GME_ADDR_L_SIZE		1


// GDE answer set command element position define
// Element ID = 6, Len = GMS_RF_FREQ_LEN
#define GMS_RF_FREQ_MSG_POS		0	// 0
#define GMS_RF_FREQ_MSG_SIZE	1

// GDE time synchronization element position define
// Element ID = 7, Len = GDE_TMSYN_LEN
#define GDE_TMSYNC_MSG_POS		0	// 0
#define GDE_TMSYNC_MSG_SIZE		1

// GME time synchronization respose element position define
// Element ID = 8, Len = GDE_LOC_TM_LEN

// GME upgrade element position define
// Element ID = 9, Len = GMS_FW_INFO_LEN
#define NEW_FW_VERN_NUM_H_POS	0
#define NEW_FW_VERN_NUM_H_SIZE	1
#define NEW_FW_VERN_NUM_L_POS	(NEW_FW_VERN_NUM_H_POS+NEW_FW_VERN_NUM_H_SIZE)	// 1
#define NEW_FW_VERN_NUM_L_SIZE	1

#define NEW_FW_TOT_LEN_HH_POS	(NEW_FW_VERN_NUM_L_POS+NEW_FW_VERN_NUM_L_SIZE)	// 2
#define NEW_FW_TOT_LEN_HH_SIZE	1
#define NEW_FW_TOT_LEN_HL_POS	(NEW_FW_TOT_LEN_HH_POS+NEW_FW_TOT_LEN_HH_SIZE)	// 3
#define NEW_FW_TOT_LEN_HL_SIZE	1
#define NEW_FW_CUR_LEN_LH_POS	(NEW_FW_TOT_LEN_HL_POS+NEW_FW_TOT_LEN_HL_SIZE)	// 4
#define NEW_FW_CUR_LEN_LH_SIZE	1
#define NEW_FW_CUR_LEN_LL_POS	(NEW_FW_CUR_LEN_LH_POS+NEW_FW_CUR_LEN_LH_SIZE)	// 5
#define NEW_FW_CUR_LEN_LL_SIZE	1


#define NEW_FW_CRC_H_POS		(NEW_FW_CUR_LEN_LL_POS+NEW_FW_CUR_LEN_LL_SIZE)	// 6
#define NEW_FW_CRC_H_SIZE		1
#define NEW_FW_CRC_L_POS		(NEW_FW_CRC_H_POS+NEW_FW_CRC_H_SIZE)	// 7
#define NEW_FW_CRC_L_SIZE		1

#define NEW_FW_TOT_BLK_H_POS	(NEW_FW_CRC_L_POS+NEW_FW_CRC_L_SIZE)	// 8
#define NEW_FW_TOT_BLK_H_SIZE	1
#define NEW_FW_TOT_BLK_L_POS	(NEW_FW_TOT_BLK_H_POS+NEW_FW_TOT_BLK_H_SIZE)	// 9
#define NEW_FW_TOT_BLK_L_SIZE	1

#define NEW_FW_UPGD_HOUR_POS	(NEW_FW_TOT_BLK_L_POS+NEW_FW_TOT_BLK_L_SIZE)	// 10
#define NEW_FW_UPGD_HOUR_SIZE	1

#define NEW_FW_UPGD_MINTS_POS	(NEW_FW_UPGD_HOUR_POS+NEW_FW_UPGD_HOUR_SIZE)	// 11
#define NEW_FW_UPGD_MINTS_SIZE	1

#define NEW_FW_UPGD_SECND_POS	(NEW_FW_UPGD_MINTS_POS+NEW_FW_UPGD_MINTS_SIZE)	// 12
#define NEW_FW_UPGD_SECND_SIZE	1


// GTE read params command element position define
// Element ID = 10, Len = GTE_READ_LEN
#define GTE_RD_REQ_MSG_POS		0	// 0
#define GTE_RD_REQ_MSG_SIZE		1

// GME upgrade element position define
// Element ID = 11, Len = GMS_UPGD_LEN
#define NEW_FW_CUR_BLK_H_POS	0	// 0
#define NEW_FW_CUR_BLK_H_SIZE	1
#define NEW_FW_CUR_BLK_L_POS	(NEW_FW_CUR_BLK_H_POS+NEW_FW_CUR_BLK_H_SIZE)	// 1
#define NEW_FW_CUR_BLK_L_SIZE	1

#define RF_OAD_BLOCK_SIZE		64

/*
// GDE upgrade finish respose element position define
// Element ID = 10, Len = GDE_UPGD_ACK_LEN
#define UPGD_FIN_ACK_MSG_POS		0	// 0
#define UPGD_FIN_ACK_MSG_SIZE		1

// GTE preset request element position define
// Element ID = 11, Len = GTE_PRESET_REQ_LEN
#define PRESET_REQ_MSG_POS		0	// 0
#define PRESET_REQ_MSG_SIZE		1

// GDE preset response element position define
// Element ID = 12, Len = GDE_PRESET_RESP_LEN
#define PRESET_RESP_MSG_POS		0	// 0
#define PRESET_RESP_MSG_SIZE	1

// GTE read params command element position define
// Element ID = 13, Len = GTE_READ_LEN
#define GTE_RD_REQ_MSG_POS		0	// 0
#define GTE_RD_REQ_MSG_SIZE		1

// GMS preupgrade request element position define
// Element ID = 14, Len = GMS_UPGD_REQ_LEN
#define PREUPGD_REQ_MSG_POS		0	// 0
#define PREUPGD_REQ_MSG_SIZE	1

// GDE preupgrade response element position define
// Element ID = 15, Len = GDE_UPGD_RESP_LEN
#define PREUPGD_RESP_MSG_POS		0	// 0
#define PREUPGD_RESP_MSG_SIZE		1
*/

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
