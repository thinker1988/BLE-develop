#ifndef PKT_FMT_H_
#define PKT_FMT_H_


#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __IAR_SYSTEMS_ICC__
#include "comdef.h"
#endif

#define GET_RF_SRC_ADDR(rfbuf)	\
	(BUILD_UINT16(rfbuf[GMS_SRC_ADDR_L_POS],rfbuf[GMS_SRC_ADDR_H_POS]))

#define GET_RF_DEST_ADDR(rfbuf)	\
	(BUILD_UINT16(rfbuf[GMS_DEST_ADDR_L_POS],rfbuf[GMS_DEST_ADDR_H_POS]))

#define GET_PKT_VERN_NUM(rfbuf)	\
	(BUILD_UINT16(rfbuf[GMS_VERSION_L_POS],rfbuf[GMS_VERSION_H_POS]))

/********************************************************************************
| ID | Tot len | Sub type | reserved | chk sum | src ID | dest ID | version |     payload    |
| 1  |    1      |        1      |     2       |      1      |     2   |     2      |      2     |    128-12     |
*/
/****************RF ID*******************/
#define GMS_ID_POS		0		// 0
#define GMS_ID_SIZE		1

#define GME_SRC_ID		0xC8
#define GDE_SRC_ID		0x55
#define GTE_SRC_ID		0x66

/**************total length*******************/
#define GMS_TOT_LEN_POS		(GMS_ID_POS+GMS_ID_SIZE)	// 1
#define GMS_TOT_LEN_SIZE	1

#define GMS_PKT_MAX_LEN		128

/**************sub type*******************/
#define GMS_SUB_TYPE_POS	(GMS_TOT_LEN_POS+GMS_TOT_LEN_SIZE)	// 2
#define GMS_SUB_TYPE_SIZE	1

// Direction: GME==>GDE
#define GME_SUBTYPE_ORDER_UPGD_REQ	1	// ELE 6+8+9
#define GME_SUBTYPE_CARINFO_RESP	2	// ELE 4
#define GME_SUBTYPE_TMSYN_RESP		3	// ELE 8
#define GME_SUBTYPE_UPGD_PKT		4	// ELE 11
#define GME_SUBTYPE_RST_BENCH_PKT	5	// ELE 3
#define GME_SUBTYPE_CHNG_ID_PKT		6	// ELE 12

// Direction: GDE==>GME
#define GDE_SUBTYPE_HRTBEAT_REQ		21	// ELE 1+2
#define GDE_SUBTYPE_CARINFO_REQ		22	// ELE 1+2+3
#define GDE_SUBTYPE_TMSYN_REQ		23	// ELE 7
#define GDE_SUBTYPE_ORDER_RESP		24	// ELE 4+6
#define GDE_SUBTYPE_UPGD_ACK		25	// ELE 4

// Direction: GDE==>GTE
#define GDE_SUBTYPE_T_PRESET_RESP	26	// ELE 6
#define GDE_SUBTYPE_T_READ_RESP		27	// ELE 5+3+12
#define GDE_SUBTYPE_T_SET_RESP		28	// ELE 4
#define GDE_SUBTYPE_T_ORDER_RESP	29	// ELE 4+6
#define GDE_SUBTYPE_T_UPGD_ACK		30	// ELE 4


// Direction: GTE==>GDE
#define GTE_SUBTYPE_PRESET_REQ		41	// ELE 6
#define GTE_SUBTYPE_PARAM_READ_REQ	42	// ELE 10
#define GTE_SUBTYPE_PARAM_SET_REQ	43	// ELE 5+12
#define GTE_SUBTYPE_ORDER_UPGD_REQ	44	// ELE 6+9
#define GTE_SUBTYPE_UPGD_PKT		45	// ELE 11
#define GTE_SUBTYPE_SET_STATE_PKT	46	// ELE 3+12


/**************reserved*******************/
#define GMS_RESERVE_POS			(GMS_SUB_TYPE_POS+GMS_SUB_TYPE_SIZE)	// 4
#define GMS_RESERVE_SIZE		2
#define GMS_RESERVE_STR			"RF"

/************XOR checksum****************/
#define GMS_CHK_SUM_POS			(GMS_RESERVE_POS+GMS_RESERVE_SIZE)	// 5
#define GMS_CHK_SUM_SIZE		1

/**************source address*****************/
#define GMS_SRC_ADDR_H_POS		(GMS_CHK_SUM_POS+GMS_CHK_SUM_SIZE)	// 6
#define GMS_SRC_ADDR_H_SIZE		1
#define GMS_SRC_ADDR_L_POS		(GMS_SRC_ADDR_H_POS+GMS_SRC_ADDR_H_SIZE)	// 7
#define GMS_SRC_ADDR_L_SIZE		1


/**************destination address*****************/
#define GMS_DEST_ADDR_H_POS		(GMS_SRC_ADDR_L_POS+GMS_SRC_ADDR_L_SIZE)	// 8
#define GMS_DEST_ADDR_H_SIZE	1
#define GMS_DEST_ADDR_L_POS		(GMS_DEST_ADDR_H_POS+GMS_DEST_ADDR_H_SIZE)	// 9
#define GMS_DEST_ADDR_L_SIZE	1


// Special address in GMS
#define GDE_ADV_ID		7999
#define GME_ADV_ID		8999
#define GTE_ADV_ID		9999


/**************version number****************/
#define GMS_VERSION_H_POS		(GMS_DEST_ADDR_L_POS+GMS_DEST_ADDR_L_SIZE)	// 10
#define GMS_VERSION_H_SIZE		1
#define GMS_VERSION_L_POS		(GMS_VERSION_H_POS+GMS_VERSION_H_SIZE)	// 11
#define GMS_VERSION_L_SIZE		1


/**************packet header*****************/
#define GMS_PKT_HDR_SIZE		(GMS_VERSION_L_POS+GMS_VERSION_L_SIZE)	// 12
#define GMS_PKT_PLD_MAX_LEN		(GMS_PKT_MAX_LEN-GMS_PKT_HDR_SIZE)	// 128-12
#define GMS_PKT_PAYLOAD_POS		GMS_PKT_HDR_SIZE	// 12

/**********payload length of subtypes***********/
#define GME_SUBTYPE_ORDER_UPGD_REQ_PL_LEN	(ELM_HDR_SIZE*3+EVLEN_GMS_RF_FREQ+EVLEN_GME_NT_TM+EVLEN_GMS_FW_INFO)// ELE 6+8+9
#define GME_SUBTYPE_CARINFO_RESP_PL_LEN		(ELM_HDR_SIZE+EVLEN_GMS_INFO_ACK)	// ELE 4
#define GME_SUBTYPE_TMSYN_RESP_PL_LEN		(ELM_HDR_SIZE+EVLEN_GME_NT_TM)	// ELE 8
#define GME_SUBTYPE_UPGD_PKT_PL_LEN			(ELM_HDR_SIZE+EVLEN_GMS_UPGD)	// ELE 11
#define GME_SUBTYPE_RST_BENCH_PKT_PL_LEN	(ELM_HDR_SIZE+EVLEN_GDE_BENCH_INFO)	// ELE 3
#define GME_SUBTYPE_CHNG_ID_PKT_PL_LEN		(ELM_HDR_SIZE+EVLEN_CHNG_ID)	// ELE 12

#define GDE_SUBTYPE_HRTBEAT_REQ_PL_LEN		(ELM_HDR_SIZE*2+EVLEN_GDE_LOC_TM+EVLEN_GDE_HRTBT)	// ELE 1+2
#define GDE_SUBTYPE_CARINFO_REQ_PL_LEN		(ELM_HDR_SIZE*3+EVLEN_GDE_LOC_TM+EVLEN_GDE_HRTBT+EVLEN_GDE_BENCH_INFO)	// ELE 1+2+3
#define GDE_SUBTYPE_TMSYN_REQ_PL_LEN		(ELM_HDR_SIZE+EVLEN_GDE_TMSYN)	// ELE 7
#define GDE_SUBTYPE_ORDER_RESP_PL_LEN		(ELM_HDR_SIZE*2+EVLEN_GMS_RF_FREQ+EVLEN_GMS_INFO_ACK)	// ELE 6+4
#define GDE_SUBTYPE_UPGD_ACK_PL_LEN			(ELM_HDR_SIZE+EVLEN_GMS_INFO_ACK)	// ELE 4
#define GDE_SUBTYPE_T_PRESET_RESP_PL_LEN	(ELM_HDR_SIZE+EVLEN_GMS_RF_FREQ)	// ELE 6
#define GDE_SUBTYPE_T_READ_RESP_PL_LEN		(ELM_HDR_SIZE*3+EVLEN_GDE_PARAMS+EVLEN_GDE_BENCH_INFO+EVLEN_CHNG_ID)	// ELE 5+3+12
#define GDE_SUBTYPE_T_SET_RESP_PL_LEN		(ELM_HDR_SIZE+EVLEN_GMS_INFO_ACK)	// ELE 4
#define GDE_SUBTYPE_T_ORDER_RESP_PL_LEN		(ELM_HDR_SIZE*2+EVLEN_GMS_RF_FREQ+EVLEN_GMS_INFO_ACK)	// ELE 6+4
#define GDE_SUBTYPE_T_UPGD_ACK_PL_LEN		(ELM_HDR_SIZE+EVLEN_GMS_INFO_ACK)	// ELE 4

#define GTE_SUBTYPE_PRESET_REQ_PL_LEN		(ELM_HDR_SIZE+EVLEN_GMS_RF_FREQ)	// ELE 6
#define GTE_SUBTYPE_PARAM_READ_REQ_PL_LEN	(ELM_HDR_SIZE+EVLEN_GTE_READ)	// ELE 10
#define GTE_SUBTYPE_PARAM_SET_REQ_PL_LEN	(ELM_HDR_SIZE*2+EVLEN_GDE_PARAMS+EVLEN_CHNG_ID)	// ELE 5+12
#define GTE_SUBTYPE_ORDER_UPGD_REQ_PL_LEN	(ELM_HDR_SIZE*2+EVLEN_GMS_RF_FREQ+EVLEN_GMS_FW_INFO)	// ELE 6+9
#define GTE_SUBTYPE_UPGD_PKT_PL_LEN			(ELM_HDR_SIZE+EVLEN_GMS_UPGD)	// ELE 11
#define GTE_SUBTYPE_SET_STATE_PKT_PL_LEN	(ELM_HDR_SIZE*2+EVLEN_GDE_BENCH_INFO+EVLEN_CHNG_ID)	// ELE 3+12

/**************element header*****************/
#define EID_SIZE			1

#define EVAL_LEN_POS		(EID_SIZE)
#define EVAL_LEN_SIZE		1

#define ELM_HDR_SIZE		(EVAL_LEN_POS+EVAL_LEN_SIZE)	// 2

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
#define EID_CHNG_ID			12

/**************element length of different ID*****************/
// EID = 1
#define EVLEN_GDE_LOC_TM	(UTCL_YEAR_POS+ UTCL_YEAR_SIZE)	// 6
// EID = 2
#define EVLEN_GDE_HRTBT		(HRT_BT_STAT_POS+HRT_BT_STAT_SIZE)	// 9
// EID = 3
#define EVLEN_GDE_BENCH_INFO	(GDE_Z_L_BCHMRK_POS+GDE_Z_L_BCHMRK_SIZE)	// 6
// EID = 4
#define EVLEN_GMS_INFO_ACK	(GMS_INFO_ACK_MSG_POS+GMS_INFO_ACK_MSG_SIZE)	// 1
// EID = 5
#define EVLEN_GDE_PARAMS	(ST_GM_STATUS_POS+ST_GM_STATUS_SIZE)	// 9
// EID = 6
#define EVLEN_GMS_RF_FREQ	(GMS_RF_FREQ_MSG_POS+GMS_RF_FREQ_MSG_SIZE)	// 1
// EID = 7
#define EVLEN_GDE_TMSYN		(GDE_TMSYNC_MSG_POS+GDE_TMSYNC_MSG_SIZE)	// 1
// EID = 8
#define EVLEN_GME_NT_TM		EVLEN_GDE_LOC_TM
// EID = 9
#define EVLEN_GMS_FW_INFO	(NEW_FW_UPGD_SECND_POS+NEW_FW_UPGD_SECND_SIZE)	// 13
// EID = 10
#define EVLEN_GTE_READ		(GTE_RD_REQ_MSG_POS+GTE_RD_REQ_MSG_SIZE)	// 1
// EID = 11
#define EVLEN_GMS_UPGD		(RF_OAD_BLOCK_BEG_POS+RF_OAD_BLOCK_SIZE)	// 66
// EID = 12
#define EVLEN_CHNG_ID		(CHNG_VERN_NUM_L_POS+CHNG_VERN_NUM_L_SIZE)	// 6


/**************element value content length*****************/

// UTC local element position define
// Element ID = 1, Len = EVLEN_GDE_LOC_TM
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
// Element ID = 2, Len = EVLEN_GDE_HRTBT
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
// Element ID = 3, Len = EVLEN_GDE_BENCH_INFO
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
// Element ID = 4, Len = EVLEN_GMS_INFO_ACK
#define GMS_INFO_ACK_MSG_POS	0	// 0
#define GMS_INFO_ACK_MSG_SIZE	1


// GDE parameters element position define
// Element ID = 5, Len = EVLEN_GDE_PARAMS
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



// GDE answer set command element position define
// Element ID = 6, Len = EVLEN_GMS_RF_FREQ
#define GMS_RF_FREQ_MSG_POS		0	// 0
#define GMS_RF_FREQ_MSG_SIZE	1

// GDE time synchronization element position define
// Element ID = 7, Len = EVLEN_GDE_TMSYN
#define GDE_TMSYNC_MSG_POS		0	// 0
#define GDE_TMSYNC_MSG_SIZE		1

// GME time synchronization respose element position define
// Element ID = 8, Len = EVLEN_GDE_LOC_TM

// GME upgrade element position define
// Element ID = 9, Len = EVLEN_GMS_FW_INFO
#define NEW_FW_VERN_NUM_H_POS	0
#define NEW_FW_VERN_NUM_H_SIZE	1
#define NEW_FW_VERN_NUM_L_POS	(NEW_FW_VERN_NUM_H_POS+NEW_FW_VERN_NUM_H_SIZE)	// 1
#define NEW_FW_VERN_NUM_L_SIZE	1

#define NEW_FW_TOT_LEN_HH_POS	(NEW_FW_VERN_NUM_L_POS+NEW_FW_VERN_NUM_L_SIZE)	// 2
#define NEW_FW_TOT_LEN_HH_SIZE	1
#define NEW_FW_TOT_LEN_HL_POS	(NEW_FW_TOT_LEN_HH_POS+NEW_FW_TOT_LEN_HH_SIZE)	// 3
#define NEW_FW_TOT_LEN_HL_SIZE	1
#define NEW_FW_TOT_LEN_LH_POS	(NEW_FW_TOT_LEN_HL_POS+NEW_FW_TOT_LEN_HL_SIZE)	// 4
#define NEW_FW_TOT_LEN_LH_SIZE	1
#define NEW_FW_TOT_LEN_LL_POS	(NEW_FW_TOT_LEN_LH_POS+NEW_FW_TOT_LEN_LH_SIZE)	// 5
#define NEW_FW_TOT_LEN_LL_SIZE	1


#define NEW_FW_CRC_H_POS		(NEW_FW_TOT_LEN_LL_POS+NEW_FW_TOT_LEN_LL_SIZE)	// 6
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
// Element ID = 10, Len = EVLEN_GTE_READ
#define GTE_RD_REQ_MSG_POS		0	// 0
#define GTE_RD_REQ_MSG_SIZE		1

// GME upgrade element position define
// Element ID = 11, Len = EVLEN_GMS_UPGD
#define NEW_FW_CUR_BLK_H_POS	0	// 0
#define NEW_FW_CUR_BLK_H_SIZE	1
#define NEW_FW_CUR_BLK_L_POS	(NEW_FW_CUR_BLK_H_POS+NEW_FW_CUR_BLK_H_SIZE)	// 1
#define NEW_FW_CUR_BLK_L_SIZE	1

#define RF_OAD_BLOCK_BEG_POS	(NEW_FW_CUR_BLK_L_POS+NEW_FW_CUR_BLK_L_SIZE)	// 2
#define RF_OAD_BLOCK_SIZE		64

// GME reset GDE version define
// Element ID = 12, Len = EVLEN_CHNG_ID
#define CHNG_GDE_ADDR_H_POS		0 	// 0
#define CHNG_GDE_ADDR_H_SIZE	1
#define CHNG_GDE_ADDR_L_POS		(CHNG_GDE_ADDR_H_POS+CHNG_GDE_ADDR_H_SIZE)	// 1
#define CHNG_GDE_ADDR_L_SIZE	1

#define CHNG_GME_ADDR_H_POS		(CHNG_GDE_ADDR_L_POS+CHNG_GDE_ADDR_L_SIZE)	// 2
#define CHNG_GME_ADDR_H_SIZE	1
#define CHNG_GME_ADDR_L_POS		(CHNG_GME_ADDR_H_POS+CHNG_GME_ADDR_H_SIZE)	// 3
#define CHNG_GME_ADDR_L_SIZE	1

#define CHNG_VERN_NUM_H_POS		(CHNG_GME_ADDR_L_POS+CHNG_GME_ADDR_L_SIZE)	// 4
#define CHNG_VERN_NUM_H_SIZE	1
#define CHNG_VERN_NUM_L_POS		(CHNG_VERN_NUM_H_POS+CHNG_VERN_NUM_H_SIZE)	// 5
#define CHNG_VERN_NUM_L_SIZE	1


typedef enum rfpkterr
{
	RF_SUCCESS,	// 0 - Success
	RF_NOT_GMS,	// 1 - Not GMS packet ID or reserved value
	RF_PKTLEN_ERR,	// 2 - Packet length error or not fully recieved
	RF_SUBTYPE_UNK,	// 3 - Unknow subtype
	RF_CHKSUM_ERR,	// 4 - Checksum error
	RF_ADDR_DENY,	// 5 - Not expected address(i.e. device ID or adv ID)
	RF_EID_UNK,	// 6 - Unknow element ID
	RF_PLD_ERR	// 7 - Payload length or value error
}rfpkterr_t;

typedef enum msgerrcd
{
	MSG_SUCCESS=0,	// Message success
	IMG_TYPE_ERR,	// Image type error(same with current working image)
	IMG_SIZE_ERR,	// Image size error
	IMG_CRC_FAIL	// Image crc fail
}msgerrcd_t;

typedef enum pktgms
{
	PKT_GMS_ID,
	PKT_GMS_LEN,
	PKT_GMS_SUBTYP,
	PKT_GMS_RSRV,
	PKT_GMS_DATA
}pktgms_t;


#ifdef __IAR_SYSTEMS_ICC__
extern void InitCommDevID(void);
extern void SetDevID(uint16 GDEaddr, uint16 GMEaddr, uint16 vern);

extern void GMSPktForm(uint8 *rawbuf, uint8 rawlen);

extern rfpkterr_t RFDataParse(uint8 *rfdata,uint8 len);

extern rfpkterr_t RFDataForm(uint8 subtype, uint8 *data, uint8 datalen);

extern rfpkterr_t RFDataSend(uint8 *buf, uint8 len);
#endif

#ifdef __cplusplus
}
#endif

#endif
