#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include<time.h>

/* for http to data center */
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <netinet/in.h>
#include <limits.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <ctype.h>

#define DEBUG_SWITCH                              1

#define VOID_RMC_LEN 25
#define MAX_RMC_LEN 100

#define MIN_GME_RECEIVE_RF_LEN               15
#define MAX_GME_RECEIVE_RF_LEN              31
#define GME_BUFFER_LEN                              256

#define GME_RESPONSE_BUFFER_LEN            20

#define GDE_HEAD                                          0x55
#define  GTE_HEAD                                         0x66
#define   GME_HEAD                                       0xC8

#define TTY_RAM "/tmp/gde_info"
#define TTY_USB "/dev/ttyUSB4"


/* WP543 must use uboot mode
root@OpenWrt:/# cat /proc/mtd 
dev:    size   erasesize  name
mtd0: 00040000 00010000 "uboot"
mtd1: 00010000 00010000 "env"
mtd2: 00130000 00010000 "kernel"
mtd3: 00670000 00010000 "rootfs"
mtd4: 003e0000 00010000 "rootfs_data"
mtd5: 00010000 00010000 "cfg"
*/
/* cfg information, gme_id is stored here in flash, ff ff is default, and after set, it is like 03 E9(1001), etc */
#ifdef CONFIG_NVRAM
#define NVRAM  "/dev/nvram"
#define NVRAM_OFFSET 0
#else
#define NVRAM  "/dev/mtdblock5"
#define NVRAM_OFFSET 0
#endif


/*********************************************************************
 * MACROS
 */

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define GET_RF_SRC_ADDR(rfbuf)	\
	(BUILD_UINT16(rfbuf[GMS_SRC_ADDR_POS+1],rfbuf[GMS_SRC_ADDR_POS]))

#define GET_RF_DEST_ADDR(rfbuf)	\
	(BUILD_UINT16(rfbuf[GMS_DEST_ADDR_POS+1],rfbuf[GMS_DEST_ADDR_POS]))
	
/*********************************************************************
 * CONSTANTS
 */

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

#define UTCL_EVAL_LEN		(UTCL_YEAR_POS+ UTCL_YEAR_SIZE)// 6
#define GDE_HRTBT_LEN		(HRT_BT_STAT_POS+HRT_BT_STAT_SIZE)// 9
#define GDE_CAR_INFO_LEN	9
#define GME_INFO_ACK_LEN	(GDE_RESP_POS+GDE_RESP_SIZE)// 1
#define GTE_SET_LEN			(ST_VERSION_L_POS+ST_VERSION_L_SIZE)// 15
#define GDE_SET_ACK_LEN		1
#define GDE_TMSYN_LEN		1
#define GME_TMSYN_ACK_LEN	6
#define GMS_UPGD_LEN		7
#define GDE_UPGD_ACK_LEN	1

/**********************************************************************
 * UTC local element position define
*/
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

/**********************************************************************
 * GDE response define
*/
#define GDE_RESP_POS		0
#define GDE_RESP_SIZE		1


/**********************************************************************
 * Heart beat data element position define
*/
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


#define GDE_ADV_ID		999

#define GME_ADV_ID		1999

#define GTE_ADV_ID		2999


#ifndef GDE_DEV_ID
#define GDE_DEV_ID		GDE_ADV_ID
#endif	// GDE_DEV_ID

#ifndef GME_DEV_ID
#define GME_DEV_ID		1001
#endif	// GME_DEV_ID


// Default version 1.0
#ifndef VERSION_NUMBER
#define VERSION_NUMBER		0x0100
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif


/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
	PKT_GMS_ID,
	PKT_GMS_LEN,
	PKT_GMS_DATA
}pktgms_t;

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

typedef signed   char   int8;     //!< Signed 8 bit integer
typedef unsigned char   uint8;    //!< Unsigned 8 bit integer

typedef signed   short  int16;    //!< Signed 16 bit integer
typedef unsigned short  uint16;   //!< Unsigned 16 bit integer

typedef signed   long   int32;    //!< Signed 32 bit integer
typedef unsigned long   uint32;   //!< Unsigned 32 bit integer

typedef unsigned char   bool;     //!< Boolean data type

/*********************************************************************
 * GLOBAL VARIABLES
 */
int gme_fd, fd2;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Read status
static pktgms_t pktgmsst=PKT_GMS_ID;

// Static temp buffer
static uint8 gmsrdpkt[GMS_PKT_MAX_LEN]={0};

// Current read len and total len
static uint8 currdlen=0;
static uint8 totrdlen=0;

// RF ID and version
uint16 RFdevID = GME_DEV_ID;
uint16 RFdestID = GDE_DEV_ID;
uint16 version = VERSION_NUMBER;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static unsigned char calcGMchksum(unsigned char* chkbuf, unsigned short len);
static uint8 fillGDEresp(uint8 * respbuf, uint8 * data, uint8 len);
static void gmspktform(uint8 * rawbuf, uint8 rawlen);

static rferr_t rfdataparse(uint8 *rfdata,uint8 len);
static uint8 filllocaltime(uint8 *timebuf);
static uint8 fillGDEresp(uint8 *respbuf, uint8 *data, uint8 len);
static rferr_t  formGMEpkt(uint8 subtype, uint8 *data, uint8 len);

static rferr_t rfdatasend(uint8 * buf, uint8 len);
static void httpsend(uint8 * buf, uint8 len);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

static unsigned char calcGMchksum(unsigned char* chkbuf, unsigned short len)
{
	unsigned char sum;
	unsigned short i;
	sum = 0;

	for(i=0;i<len;i++)
	{
		if (i == GMS_CHK_SUM_POS)
			continue;
		
		sum ^= chkbuf[i];
	}
	
	return sum;
}




/* 
//GDE packet format, hex format
(1) heart beat from GDE->GME,
55 means from GDE, 1F means packet head+payload is 31 bytes, 15 means RF sub type--digit 21 heart beat, 52 46 means "RF"--reserved, crc is , 00 01 is GDE number--1, 03 

E9 is GME number--1001, 01 00 is version 1.0, 01 06 means UTC time, data length is 6, 05 0E 30 means 5 hour 14 minute 48 second, 1C 0C 0C means 2012 year 12 month 28 

day. 02 09 means GDE heart beat, data length is 9, 20 means left energy is 32%, 21 means 33 temperture, 00 01 means x arix value(17), 00 22 means y arix value(34), 00 

33 means z airx value(51), 00 means no car.

55 1F 15 52 46 crc 00 01 03 E9 01 00 01 06 05 0E 30 1C 0C 0C 02 09 20 21 00 11 00 22 00 33 00
 

(2) car change packet from GDE->GME,
55 means from GDE, 20 means packet head+payload is 31 bytes, crc is , 16 means RF sub type--digit 22 car change information, 00 01 is GDE number--1, 03 E9 is GME 

number--1001, 01 00 is version 1.0, 01 06 means UTC time, data length is 6, 05 0E 30 means 5 hour 14 minute 48 second, 1C 0C 0C means 2012 year 12 month 28 day. 03 09 

means GDE car change information, data length is 9, 20 means left energy is 32%, 21 means 33 temperture, 00 01 means x arix value(17), 00 22 means y arix value(34), 00 

33 means z airx value(51), 00 means no car.

55 1F 16 52 46 crc 00 01 03 E9 01 00 01 06 05 0E 30 1C 0C 0C 02 09 20 21 00 11 00 22 00 33 00

(3) time sync request from GDE->GME,
55 means from GDE, 1F means packet head+payload is xx bytes, 17 means RF sub type--digit 23 time sync request, 52 46 means "RF"--reserved, crc is , 00 01 is GDE 

number--1, 03 E9 is GME number--1001, 01 00 is version 1.0, 07 means time sync request, 01 means length, 01 means request time
55 0F 17 52 46 crc 00 01 03 E9 01 00 07 01 01


//GME packet format, hex format
(4)response from GME->GDE for car change packet
C8 means from GME, 0F means packet head+payload is 15 bytes, 02 means RF sub type--digit 02 GME response for GDE car change info, 52 46 means "RF"--reserved, crc is 

,03 E9 is GME number--1001, 00 01 is GDE number--1, 01 00 is version 1.0, 04 01 means GME repsonse to GDE, data length is 1, 01 means GME receive GDE car change info 

correctly. (00  means receive error)

C8 0F 02 52 46 crc 03 E9 00 01 01 00 04 01 01    correct

C8 0F 02 52 46 crc 03 E9 00 01 01 00 04 01 00    error

(5) response from GME->GDE for time sync request packet
C8 means from GME, 0F means packet head+payload is 15 bytes, 02 means RF sub type--digit 02 GME response for GDE car change info, 52 46 means "RF"--reserved, crc is 

,03 E9 is GME number--1001, 00 01 is GDE number--1, 01 00 is version 1.0, 08 06 means GME repsonse to GDE, data length is 6, 05 0E 30 1C 0C 0C means GME UTC time,     

05 0E 30 means 5 hour 14 minute 48 second, 1C 0C 0C means 2012 year 12 month 28 day.

C8 0F 03 52 46 crc 03 E9 00 01 01 00 08 06 05 0E 30 1C 0C 0C


*/

void cfg_init();

int wrt2file(int fd, char *filebuf)
{
	
	return 0;
}

int read_gme_id()
{
	return RFdevID;
}

int write_gme_id(int gmeid)
{
	//write gme_id to WP543
	RFdevID = gmeid;
	return RFdevID;
}

int serial_set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if  ( tcgetattr( fd,&oldtio)  !=  0) { 
		perror("SetupSerial");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch( nBits )
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}

	switch( nEvent )
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E': 
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':  
		newtio.c_cflag &= ~PARENB;
		newtio.c_iflag &= ~INPCK;
		break;
	}

	switch( nSpeed )
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	case 460800:
		cfsetispeed(&newtio, B460800);
		cfsetospeed(&newtio, B460800);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}
	if( nStop == 1 )
		newtio.c_cflag &=  ~CSTOPB;
	else if ( nStop == 2 )
		newtio.c_cflag |=  CSTOPB;

	newtio.c_cc[VTIME] = 0;	
	newtio.c_cc[VMIN]=0;

	tcflush(fd,TCIOFLUSH);
	
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}
	
	return 0;
}

void db(unsigned char* buf, unsigned int len)
{
	int i;
	
	printf("buf=0x%x, len (byte) =%d\r\n",(int)buf,len);
	
	for(i= 0; i<len; i++)
	{	
		printf("%.2x ",buf[i]);	
	}			

	printf("\r\n");
}


static void gmspktform(uint8 *rawbuf, uint8 rawlen)
{
	uint8 i;

	for (i=0; i<rawlen; i++)
	{
		switch(pktgmsst)
		{
			case PKT_GMS_ID:
				if (rawbuf[i]==GDE_SRC_ID || rawbuf[i]==GTE_SRC_ID)
				{
					pktgmsst=PKT_GMS_LEN;
					gmsrdpkt[currdlen++] = rawbuf[i];
				}
				else
					printf("%c",rawbuf[i]);
				break;
			case PKT_GMS_LEN:
				totrdlen= rawbuf[i];
				if (totrdlen >= GMS_PKT_MAX_LEN)
				{
					pktgmsst = PKT_GMS_ID;
					currdlen = 0;
				}
				else
				{
					gmsrdpkt[currdlen++] =totrdlen;
					pktgmsst=PKT_GMS_DATA;
				}
				break;
			case PKT_GMS_DATA:
				gmsrdpkt[currdlen++]= rawbuf[i];
				if (currdlen == totrdlen)
				{
					rfdataparse(gmsrdpkt, currdlen);
					pktgmsst=PKT_GMS_ID;
					currdlen = 0;
					totrdlen = 0;
				}
				break;
			default:
				break;
		}
	}
}


static rferr_t rfdataparse(uint8 *rfdata,uint8 len)
{
	if (rfdata[GMS_ID_POS]!=GDE_SRC_ID && rfdata[GMS_ID_POS]!=GTE_SRC_ID)
		return PKT_DENY;
	
	if (rfdata[GMS_TOT_LEN_POS] != len)
		return PKT_ERR;

	if (memcmp(rfdata+GMS_RESERVE_POS, GMS_RESERVE_STR, GMS_RESERVE_SIZE) != 0)
		return PKT_DENY;
	
	if (GET_RF_DEST_ADDR(rfdata) != RFdevID && GET_RF_DEST_ADDR(rfdata) != GME_ADV_ID)
		return BAD_ADDR;
		
	if (calcGMchksum(rfdata,len) != rfdata[GMS_CHK_SUM_POS])
		return CHKSUM_ERR;

	RFdestID = GET_RF_SRC_ADDR(rfdata);


	if (rfdata[GMS_ID_POS] == GDE_SRC_ID)
	{
		switch(rfdata[GMS_SUB_TYPE_POS])
		{
			case GDE_ST_HRTBEAT_REQ:
				httpsend(rfdata, len);
				break;
			case GDE_ST_CARINFO_REQ:
			{
				// GDE carinfo
				uint8 val = TRUE;
				formGMEpkt(GME_ST_CARINFO_ACK, &val, sizeof(val));
				httpsend(rfdata, len);
				break;
			}
			case GDE_ST_TMSYN_REQ:
				// set GDE time
				formGMEpkt(GME_ST_TMSYN_ACK, NULL, 0);
				break;
			case GDE_ST_M_UPGD_ACK:
				// upgrade process
				break;
			case GDE_ST_T_UPGD_ACK:
			case GDE_ST_T_SET_ACK:
			default:
				return SUB_TYPE_ERR;
		}
	}


	return RF_SUCCESS;
}


static uint8 filllocaltime(uint8 *timebuf)
{
	time_t timep;
	struct tm *p;
	
	time(&timep);
	p = localtime(&timep); //get local time

	printf("++++++++++time:%d/%d/%d.\r\n",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday);
	printf("++++++++++%d:%d:%d\r\n",p->tm_hour, p->tm_min, p->tm_sec);

	timebuf[0] = EID_GME_TMSYN_ACK;
	timebuf[EID_SIZE] = GME_TMSYN_ACK_LEN;

	timebuf[ELM_HDR_SIZE+UTCL_HOUR_POS]=(unsigned char)p->tm_hour;
	timebuf[ELM_HDR_SIZE+UTCL_MINTS_POS]=(unsigned char)p->tm_min;
	timebuf[ELM_HDR_SIZE+UTCL_SECND_POS]=(unsigned char)p->tm_sec;//second
	timebuf[ELM_HDR_SIZE+UTCL_DAY_POS]=(unsigned char)p->tm_mday;
	timebuf[ELM_HDR_SIZE+UTCL_MONTH_POS]=(unsigned char)(p->tm_mon+1);
	timebuf[ELM_HDR_SIZE+UTCL_YEAR_POS]=(unsigned char)(p->tm_year + 1900 -2000); //year, should plus 2000

	return (ELM_HDR_SIZE+GME_TMSYN_ACK_LEN);
}


static uint8 fillGDEresp(uint8 *respbuf, uint8 *data, uint8 len)
{
	(void) len;

	respbuf[0] = EID_GME_INFO_ACK;
	respbuf[EID_SIZE] = GME_INFO_ACK_LEN;
	respbuf[ELM_HDR_SIZE+GDE_RESP_POS]= *data;

	return (ELM_HDR_SIZE+GME_INFO_ACK_LEN);
}


static rferr_t  formGMEpkt(uint8 subtype, uint8 *data, uint8 len)
{
	uint8 rfbuf[GMS_PKT_MAX_LEN]={0};
	uint8 curpldpos = GMS_PKT_PAYLOAD_POS;

	rfbuf[GMS_ID_POS]=GME_SRC_ID;
	
	rfbuf[GMS_SUB_TYPE_POS]=subtype;

	memcpy(rfbuf+GMS_RESERVE_POS, GMS_RESERVE_STR, GMS_RESERVE_SIZE);

	rfbuf[GMS_SRC_ADDR_POS] = HI_UINT16(RFdevID);
	rfbuf[GMS_SRC_ADDR_POS+1] = LO_UINT16(RFdevID);

	rfbuf[GMS_DEST_ADDR_POS] = HI_UINT16(RFdestID);	// need mutex?
	rfbuf[GMS_DEST_ADDR_POS+1] = LO_UINT16(RFdestID);

	rfbuf[GMS_VERSION_POS] = HI_UINT16(version);	// how to fill fist update
	rfbuf[GMS_VERSION_POS+1] = LO_UINT16(version);

	switch(subtype)
	{
		case GME_ST_HRTBEAT_ACK:
			break;
		case GME_ST_CARINFO_ACK:
			if (len != GME_INFO_ACK_LEN)
				return DATA_ERR;
			// heart beat packet format
			curpldpos += fillGDEresp(rfbuf+GMS_PKT_PAYLOAD_POS, data, len);
			break;
		case GME_ST_TMSYN_ACK:
			// car info request
			curpldpos += filllocaltime(rfbuf+GMS_PKT_PAYLOAD_POS);
			break;
		case GME_ST_UPGD_REQ:
			break;
		default:
			return SUB_TYPE_ERR;
	}
	
	rfbuf[GMS_TOT_LEN_POS]=curpldpos;

	rfbuf[GMS_CHK_SUM_POS]=calcGMchksum(rfbuf, rfbuf[GMS_TOT_LEN_POS]);

	return rfdatasend(rfbuf,rfbuf[GMS_TOT_LEN_POS]);

}

static rferr_t rfdatasend(uint8 *buf, uint8 len)
{
	write(gme_fd, buf, len);

	return RF_SUCCESS;
}


static void httpsend(uint8 *buf, uint8 len)
{
	struct hostent *host;
	struct sockaddr_in server_addr;

	int sockfd;
	char recvbuffer[1024]={0};
	char request[1024]={0}; 
	char sendtext[512];
	int port;
	//char *host_addr = "120.26.103.149";
	char *host_addr = "10.30.242.154";

	// Ascii data len should be 2*MAX_LEN of bin data
	char data[GMS_PKT_MAX_LEN*2];
	int sendstate;
	int recvstate;
	int con_flag;
	int i,j;

	unsigned char gdehb[31] = {0x55,0x1F,0x15,0x52,0x46,0x8B,0x00,0x01,0x03,\
			0xE9,0x01,0x00,0x01,0x06,0x05,0x0E,0x30,0x1C,0x0C,0x0C,\
			0x02,0x09,0x20,0x21,0x00,0x11,0x00,0x22,0x00,0x33,0x00};
	//120.26.103.149:9999/gmehttpget?data=551F1552468B000103E901000106050E301C0C0C0209202100110022003300
	unsigned char gdep[31] ={0x55,0x1F,0x16,0x52,0x46,0x88,0x00,0x01,0x03,\
			0xE9,0x01,0x00,0x01,0x06,0x05,0x0E,0x30,0x1C,0x0C,0x0C,\
			0x02,0x09,0x20,0x21,0x00,0x11,0x00,0x22,0x00,0x33,0x00};

	printf("+++++enter 3G send.\r\n");

	for(i=0;i<len;i++)
		sprintf(data+i*2,"%02x",buf[i]);
	
	//sprintf(data+i*2,"%02x",gdehb[i]);
	//printf("%s\n",host_addr); 

	host=gethostbyname(host_addr);
	port = 9999;
	sockfd=socket(AF_INET,SOCK_STREAM,0);
	bzero(&server_addr,sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(port);
	server_addr.sin_addr = *((struct in_addr *)host->h_addr);
	con_flag = connect(sockfd,(struct sockaddr *)(&server_addr),sizeof(struct sockaddr));

	sprintf(request, "GET /gmehttpget?data=%s HTTP/1.1\r\nHost: %s:%d\r\nConnection: Close\r\n\r\n",\
			data, host_addr, port);

	printf("%s", request);
	sendstate = -1;
	j=0;

	//con_flag = 0 means network is OK send 3 times if failed
	while((-1 == sendstate)&&(j<3)&&(0 == con_flag))
	{
		sendstate = send(sockfd,request,strlen(request),0);
		j++;
		printf("++++++sendsate:%d,j:%d.\r\n",sendstate,j);
	}
	printf("++++++send:%d\r\n",sendstate);
	recvstate = -1; 
	while((recvstate)&&(0  == con_flag))
	{
		recvstate = recv(sockfd, recvbuffer, sizeof(recvbuffer), 0);
		printf("%s", recvbuffer);//把http信息打印在屏幕上	 
	}

	close(sockfd);
	//return 0; 
}


int asciitobin(char *ascbuf, int len, char *bbuf)
{
	int i;
	
	for (i = 0; i < len; i++)
	{
		if (ascbuf[i] >= '0' && ascbuf[i] <= '9')
			ascbuf[i] -= '0';
		else if (ascbuf[i] >= 'A' && ascbuf[i] <= 'F')
			ascbuf[i] = ascbuf[i] - 'A' + 0x0A;
		else if (ascbuf[i] >= 'a' && ascbuf[i] <= 'f')
			ascbuf[i] = ascbuf[i] - 'a' + 0x0A;
		else
			return 0;
	}
	
	for (i = 0; i < len/2; i++)
	{
		bbuf[i] = (ascbuf[i*2]<<4) + ascbuf[i*2+1];
	}
	
	return len/2;
}

void usage()
{
	printf("Usage:\r\n");
	printf("\t(app name) (serial tty file) (serial baud) [option params]\r\n");
	printf("\tserial tty file, e.g. /dev/ttyUSB0\r\n");
	printf("\tserial baud, e.g. 9600\r\n");
	printf("\toption params: -a -s\r\n");
	printf("\t\t-a: set GME ID\r\n");
	printf("\t\t-s: set CC1120 RF params\r\n");
	exit(1);
}
int main(int argc, char ** argv)
{
	char *c,*p;
	int nset1,len,ret,Param;
	unsigned char buf[GME_BUFFER_LEN];
	unsigned char crc_buf[GME_BUFFER_LEN];
	char setflg=0;

#if ( defined RECV_DEBUG)
	printf("RECV DEBUG\r\n");
#else
	printf("NORMAL\r\n");
#endif

	if (argc < 3 )
	{
		usage();
	}
	/* read parameter */
	for (Param = 3; Param < argc; Param++)
	{
		c = p = argv[Param];
#ifdef __linux__
/* We don't accept an option beginning with a '/' because it could be a file name. */
		if(*c == '-')
#else
		if((*c == '-') || (*c == '/'))
#endif
		{
			p = c + 2;
			/* Parameter may follow immediately after option */
			/* Skips after parameter to next option when it is not the last option*/
			if (*p == '\0' && (Param < argc-1))
				p = argv[++Param];
			
			switch(*(++c))
			{
				case 's':
					setflg = 1;
					break;
				case 'a':
					RFdevID = atoi(p);
					break;
				case 'v':
					//display_verion();
					break;
				default:
					usage();
			}
		}
		else
		{
			usage();
		}
	}

	// 3G module use ttyUSB3-ttyUSB6, RF work module use ttyUSB0, RF upgrade module use ttyUSB1
	gme_fd = open(argv[1], O_RDWR|O_NOCTTY|O_NONBLOCK);
	if(gme_fd == -1)
	{
		printf("Open serial wrong!\n");
		exit(1);
	}
	
	nset1 = serial_set_opt(gme_fd,atoi(argv[2]),8,'N',1);
	if(nset1 == -1)
	{
		printf("Serial set wrong!\n");
		exit(1);
	}

#if 0
       // 3G module use ttyUSB0-ttyUSB3, RF work module use ttyUSB4, RF upgrade module use ttyUSB5
	fd2 = open("/tmp/rf1", O_RDWR|O_NOCTTY|O_NONBLOCK|O_CREAT|O_APPEND, S_IRWXG);

   	if(fd2 == -1)
	{
		printf("Open tmp rf1 file wrong!\n");
		exit(1);
	}
#endif

	while (1)
	{
		memset(buf, 0, GME_BUFFER_LEN);	
		if (setflg == 0)
		{
			len = read(gme_fd, buf, GME_BUFFER_LEN);
			if (len > 0)
			{
#if ( defined RECV_DEBUG)
				static int totlen = 0;
				totlen += len;
				printf("\r\nRecv: %d, total: %d\r\n",len,totlen);
				db(buf,len);
#else
				gmspktform(buf,len);
#endif	// RECV_DEBUG
			}
			usleep(200*1000);
		}
		else
		{
			printf("set cc1120 module:\r\n");
			scanf("%s",buf);
			len = strlen(buf);
			if ( len > 0 )
			{
				if (strcmp(buf,"exit") == 0)
				{
					setflg = 0;
					printf("back to read mode:\r\n");
				}
				else
				{
					len = asciitobin(buf,len,crc_buf);
					db(crc_buf,len);
					write(gme_fd,crc_buf,len);	
				}
			}
			sleep(1);
			if (len > 0 && setflg == 1)
			{
				len = read(gme_fd,buf,sizeof(buf));
				db(buf,len);
			}
		}
	}
	
	close(gme_fd);
	
	return 0;
}
