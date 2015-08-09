#ifndef PKT_FMT_H_
#define PKT_FMT_H_


#ifdef __cplusplus
extern "C"
{
#endif

#include "bcomdef.h"

/********************************************************************************
| ID | Tot len | Sub type | reserved | chk sum | src ID | dest ID | version | payload |
| 1  |    1    |    1     |    2     |    1    |    2   |    2    |    2    |  128-12 |
*/
#define GMS_PKT_MAX_LEN	128

#define GMS_ID_POS		0
#define GMS_ID_SIZE		1

#define GME_SRC_ID		0xC8
#define GDE_SRC_ID		0x55
#define GTE_SRC_ID		0x66


#define GMS_TOT_LEN_POS		(GMS_ID_POS+GMS_ID_SIZE)	//0+1
#define GMS_TOT_LEN_SIZE	1


#define GMS_SUB_TYPE_POS	(GMS_TOT_LEN_POS+GMS_TOT_LEN_SIZE)
#define GMS_SUB_TYPE_SIZE	1

#define GME_ST_HRTBEAT_ACK	1
#define GME_ST_CARINFO_ACK	2
#define GME_ST_TMSYN_ACK	3
#define GME_ST_UPGD_REQ		4

#define GDE_ST_HRTBEAT_REQ	21
#define GDE_ST_CARINFO_REQ	22
#define GDE_ST_TMSYN_REQ	23
#define GDE_ST_M_UPGD_ACK	24
#define GDE_ST_T_UPGD_ACK	25
#define GDE_ST_T_SET_ACK	26

#define GTE_ST_PARAM_SET	41
#define GTE_ST_UPGD_REQ		42


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
#define ELM_HDR_SIZE		(EID_SIZE+EVAL_LEN_SIZE)

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

#define UTCL_EVAL_LEN		6
#define GDE_HRTBT_LEN		9
#define GDE_CAR_INFO_LEN	9
#define GME_INFO_ACK_LEN	1
#define GTE_SET_LEN			15
#define GDE_SET_ACK_LEN		1
#define GDE_TMSYN_LEN		1
#define GME_TMSYN_ACK_LEN	6
#define GMS_UPGD_LEN		7
#define GDE_UPGD_ACK_LEN	1

/**********************************************************************

*/
#define HRT_BT_BATT_POS		0
#define HRT_BT_BATT_SIZE	1

#define HRT_BT_TMPR_POS		(HRT_BT_BATT_POS+HRT_BT_BATT_SIZE)
#define HRT_BT_TMPR_SIZE	1

#define HRT_BT_XVAL_POS		(HRT_BT_TMPR_POS+HRT_BT_TMPR_SIZE)
#define HRT_BT_XVAL_SIZE	2

#define HRT_BT_YVAL_POS		(HRT_BT_XVAL_POS+HRT_BT_XVAL_SIZE)
#define HRT_BT_YVAL_SIZE	2

#define HRT_BT_ZVAL_POS		(HRT_BT_YVAL_POS+HRT_BT_YVAL_SIZE)
#define HRT_BT_ZVAL_SIZE	2

#define HRT_BT_STAT_POS		(HRT_BT_ZVAL_POS+HRT_BT_ZVAL_SIZE)
#define HRT_BT_STAT_SIZE	1


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


extern void syncUTCtimereq(void);

extern void gmspktform(uint8 *rawbuf, uint8 rawlen);

extern rferr_t rfdataparse(uint8 *rfdata,uint8 len);

extern rferr_t rfdataform(uint8 subtype, uint8 *data, uint8 datalen);

#ifdef __cplusplus
}
#endif

#endif
