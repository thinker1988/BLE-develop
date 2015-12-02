#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include<time.h>
#include<signal.h>

/* for http to data center */
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <netinet/in.h>
#include <limits.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <ctype.h>

#include "Pktfmt.h"


/*********************************************************************
 * MACROS
 */

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)


#define BigLittleSwapShort(A)	((((uint16)(A) & 0xff00) >> 8) | (((uint16)(A) & 0x00ff) << 8))

/*********************************************************************
 * CONSTANTS
 */

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

#define MAX_MNG_GDE_CNT			100

/*********************************************************************
 * TYPEDEFS
 */
typedef signed   char   int8;     //!< Signed 8 bit integer
typedef unsigned char   uint8;    //!< Unsigned 8 bit integer

typedef signed   short  int16;    //!< Signed 16 bit integer
typedef unsigned short  uint16;   //!< Unsigned 16 bit integer

typedef signed   long   int32;    //!< Signed 32 bit integer
typedef unsigned long   uint32;   //!< Unsigned 32 bit integer

typedef unsigned char   bool;     //!< Boolean data type

typedef enum
{
	NORMAL_MODE,
	DEBUG_MODE,
	SETUP_MODE
}sworkstate_t;


typedef struct fw_img_hdr
{
	uint16 crcraw;
	uint16 crcshdw;
	uint16 oadvern;
	uint16 oadfwdlen;
	
}fw_img_hdr_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Device serial fd
static int gme_sfd;// fd2;

// RF ID and version
static uint16 RFdevID = GME_DEV_ID;
static uint16 RFdestID = GDE_DEV_ID;
static uint16 version = VERSION_NUMBER;

//char *host_addr = "120.26.103.149";
static char *host_addr = "10.30.242.154";
static int host_port = 9999;

// RF read GM system packet status
static pktgms_t pktgmsst=PKT_GMS_ID;
// Temporary save packet buffer
static uint8 gmsrdpkt[GMS_PKT_MAX_LEN]={0};
// Current read len and total len
static uint8 currdlen=0;
static uint8 totrdlen=0;

// GDE ID wait to reset benchmark
static uint16 rstgdeid = 0;

// Prepare upgrade flag
static bool prepupgdflg = FALSE;
// Upgrade bin file string
static char *binfstr = NULL;
// Upgrade file pointer
static FILE *binfp;
// Manage GDE state
static uint8 mnggdest[MAX_MNG_GDE_CNT];

// Upgrade firmware version
static uint16 fwvern = 0;
static uint32 fwlen = 0;
static uint16 fwcrc = 0;
static uint16 fwtotblk = 0;
static uint8 alrmhh,alrmmm,alrmss;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void gmspktform(uint8 * rawbuf, uint8 rawlen);
static rfpkterr_t rfdataparse(uint8 *rfdata,uint8 len);

static rfpkterr_t GDEPktElmProc(uint8 subtype, uint8* pldbuf, uint8 pldlen);
static void controlGDE(void);
static void procGDEack(uint8 subtype, msgerrcd_t ecd);


static rfpkterr_t  formGMEpkt(uint8 subtype, uint8 *data, uint8 len);
static uint8 fillupgdfreq(uint8 * freqbuf);
static uint8 fillupgfwinfo(uint8 * fwinfobuf);
static uint8 filllocaltime(uint8 *timebuf);
static uint8 fillGDEresp(uint8 * respbuf, uint8 * data, uint8 len);
static uint8 fillupgblk(uint8 * pldbuf, uint8 * data, uint8 len);
static uint8 fillrstbench(uint8 * pldbuf);

static rfpkterr_t rfdatasend(uint8 * buf, uint8 len);

static void PrintGDETime(uint8* gdetm);
static void PrintHrtbtData(uint8* hrtbtval);
static void PrintBnchmk(uint8* bnchmk);

static void upgrade_gde_fw(int iSignNo);
static uint32 get_file_sz(FILE * fp);
static void prep_upgrade(char * binstr);
static void set_upgrade_alarm(char * hmstmstr);


static void httpsend(uint8 * buf, uint8 len);

static unsigned char calcGMchksum(unsigned char* chkbuf, unsigned short len);
static int asciitobin(char * ascbuf, int len, char * bbuf);

static void display_verion(void);
static void timeerror(void);
static void usage(void);

static uint8 CPUBigEndian(void);
static uint16 le_tohs(uint16 n);


/*********************************************************************
 * PUBLIC FUNCTIONS
 */


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
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
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


static rfpkterr_t rfdataparse(uint8 *rfdata,uint8 len)
{
	uint8 pldlen = len-GMS_PKT_HDR_SIZE;

	if (rfdata[GMS_ID_POS]!=GDE_SRC_ID) //&& rfdata[GMS_ID_POS]!=GTE_SRC_ID)
		return RF_NOT_GMS;
	
	if (rfdata[GMS_TOT_LEN_POS] != len)
		return RF_PKTLEN_ERR;

	if (memcmp(rfdata+GMS_RESERVE_POS, GMS_RESERVE_STR, GMS_RESERVE_SIZE) != 0)
		return RF_NOT_GMS;
	
	if (GET_RF_DEST_ADDR(rfdata) != RFdevID && GET_RF_DEST_ADDR(rfdata) != GME_ADV_ID)
		return RF_ADDR_DENY;
		
	if (calcGMchksum(rfdata,len) != rfdata[GMS_CHK_SUM_POS])
		return RF_CHKSUM_ERR;

	RFdestID = GET_RF_SRC_ADDR(rfdata);

	printf("\r\nFROM ID:%d, Vern: V%02x.%02x, Subtype: %d\r\n",RFdestID,rfdata[GMS_VERSION_H_POS],\
			rfdata[GMS_VERSION_L_POS],rfdata[GMS_SUB_TYPE_POS]);

	if (rfdata[GMS_ID_POS] == GDE_SRC_ID)
	{
		switch(rfdata[GMS_SUB_TYPE_POS])
		{
			case GDE_SUBTYPE_HRTBEAT_REQ:	// GDE heart beat recieved success
				if (pldlen == GDE_SUBTYPE_HRTBEAT_REQ_PL_LEN)
				{
					//httpsend(rfdata, len);
					printf("===>Heart beat.\r\n");
					break;
				}
			case GDE_SUBTYPE_CARINFO_REQ:	// GDE carinfo recieved success
				if (pldlen == GDE_SUBTYPE_CARINFO_REQ_PL_LEN)
				{
					//httpsend(rfdata, len);
					printf("===>Car detected.\r\n");
					break;
				}
			case GDE_SUBTYPE_TMSYN_REQ:	// Prepare time synchronizing
				if (pldlen == GDE_SUBTYPE_TMSYN_REQ_PL_LEN)
				{
					printf("===>Time sync.\r\n");
					break;
				}
			case GDE_SUBTYPE_ORDER_RESP:	// Finish order
				if (pldlen == GDE_SUBTYPE_ORDER_RESP_PL_LEN)
				{
					printf("===>Finish order.\r\n");
					break;
				}
			case GDE_SUBTYPE_UPGD_ACK:	// Upgrade finish state
				if (pldlen == GDE_SUBTYPE_TMSYN_REQ_PL_LEN)
				{
					printf("===>Upgrade finish.\r\n");
					break;
				}
				printf("===>Unknow payload\r\n");
				return RF_PLD_ERR;

			default:
				printf("===>Unknow subtype\r\n");
				return RF_SUBTYPE_UNK;
		}
	}
	GDEPktElmProc(rfdata[GMS_SUB_TYPE_POS],rfdata+GMS_PKT_HDR_SIZE, pldlen);

	return RF_SUCCESS;
}

static rfpkterr_t GDEPktElmProc(uint8 subtype, uint8* pldbuf, uint8 pldlen)
{
	msgerrcd_t val = MSG_SUCCESS;
	uint8 elmpos = 0;

	while(elmpos<pldlen)
	{
		switch(pldbuf[elmpos])
		{
			case EID_GDE_LOC_TM:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_LOC_TM)
				{
					PrintGDETime(pldbuf+elmpos+ELM_HDR_SIZE);
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_LOC_TM;
					break;
				}
			case EID_GDE_HRTBT:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_HRTBT)
				{
					PrintHrtbtData(pldbuf+elmpos+ELM_HDR_SIZE);
					switch (subtype)
					{
						case GDE_SUBTYPE_HRTBEAT_REQ:
							controlGDE();
							break;
						case GDE_SUBTYPE_CARINFO_REQ:
							val = MSG_SUCCESS;
							formGMEpkt(GME_SUBTYPE_CARINFO_RESP, (uint8 *)&val, GME_SUBTYPE_CARINFO_RESP_PL_LEN);
							break;
						default:
							;// do not break
					}
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_HRTBT;
				}
			case EID_GDE_BENCH_INFO:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_BENCH_INFO)
				{
					PrintBnchmk(pldbuf+elmpos+ELM_HDR_SIZE);
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_BENCH_INFO;
					break;
				}
			case EID_GMS_INFO_ACK:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GMS_INFO_ACK)
				{
					val = pldbuf[ELM_HDR_SIZE];
					procGDEack(subtype, val);
					elmpos += ELM_HDR_SIZE+EVLEN_GMS_INFO_ACK;
					break;
				}
			case EID_GDE_PARAMS:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_PARAMS)
				{
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_PARAMS;
					break;
				}
			case EID_GMS_RF_FREQ:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GMS_RF_FREQ)	// should change frequency
				{
					val = pldbuf[ELM_HDR_SIZE];
					printf("Upgrade frequency:%d\r\n",val);
					elmpos += ELM_HDR_SIZE+EVLEN_GMS_RF_FREQ;
					break;
				}
			case EID_GDE_TMSYN:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GDE_TMSYN)
				{
					formGMEpkt(GME_SUBTYPE_TMSYN_RESP, NULL, GDE_SUBTYPE_TMSYN_REQ_PL_LEN);
					elmpos += ELM_HDR_SIZE+EVLEN_GDE_TMSYN;
					break;
				}
			case EID_GME_NT_TM:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GME_NT_TM)
				{
					elmpos += ELM_HDR_SIZE+EVLEN_GME_NT_TM;
					break;
				}
			case EID_GMS_FW_INFO:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GMS_FW_INFO)
				{
					elmpos += ELM_HDR_SIZE+EVLEN_GMS_FW_INFO;
					break;
				}

			case EID_GTE_READ:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GTE_READ)
				{
					elmpos += ELM_HDR_SIZE+EVLEN_GTE_READ;
					break;
				}
			case EID_GMS_UPGD:
				if (pldbuf[elmpos+EVAL_LEN_POS] == EVLEN_GMS_UPGD)
				{
					elmpos += ELM_HDR_SIZE+EVLEN_GMS_UPGD;
					break;
				}

				return RF_PLD_ERR;
			default:
				return RF_EID_UNK;
		}
	}
	return RF_SUCCESS;		
}

static void controlGDE(void)
{
	if (prepupgdflg == TRUE)
	{
		if (mnggdest[(RFdestID-1)%MAX_MNG_GDE_CNT] == FALSE)
		{
			formGMEpkt(GME_SUBTYPE_ORDER_UPGD_REQ, NULL, GME_SUBTYPE_ORDER_UPGD_REQ_PL_LEN);
			printf("Send %d order upgrade...\r\n", RFdestID);
		}
	}

	if (rstgdeid == RFdestID)
	{
		formGMEpkt(GME_SUBTYPE_RST_BENCH_PKT, NULL, GME_SUBTYPE_RST_BENCH_PKT_PL_LEN);
		rstgdeid = 0;
		printf("Reset benchmark...\r\n");
	}	
}

static void procGDEack(uint8 subtype, msgerrcd_t ecd)
{
	switch(ecd)
	{
		case MSG_SUCCESS:
			if (subtype == GDE_SUBTYPE_ORDER_RESP)
			{
				printf("Order %d OK.\r\n", RFdestID);
				mnggdest[(RFdestID-1)%MAX_MNG_GDE_CNT] = TRUE;
			}
			else if (subtype == GDE_SUBTYPE_UPGD_ACK)
			{
				printf("%d upgrade OK.\r\n", RFdestID);
				mnggdest[(RFdestID-1)%MAX_MNG_GDE_CNT] = FALSE;
			}
			break;
		case IMG_TYPE_ERR:
		case IMG_SIZE_ERR:
		case IMG_CRC_FAIL:
			printf("%d upgrade failed, reason %d.\r\n", RFdestID, ecd);
			prepupgdflg = FALSE;
			break;
		default:
			break;
	}
}

static uint8 fillupgdfreq(uint8* freqbuf)
{
	freqbuf[0] = EID_GMS_RF_FREQ;
	freqbuf[EVAL_LEN_POS] = EVLEN_GMS_RF_FREQ;
	freqbuf[ELM_HDR_SIZE] = 0;

	return (ELM_HDR_SIZE+EVLEN_GMS_RF_FREQ);
}

static uint8 fillupgfwinfo(uint8 *fwinfobuf)
{
	fwinfobuf[0] = EID_GMS_FW_INFO;
	fwinfobuf[EVAL_LEN_POS] = EVLEN_GMS_FW_INFO;

	fwinfobuf[ELM_HDR_SIZE+NEW_FW_VERN_NUM_H_POS] = HI_UINT16(fwvern);
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_VERN_NUM_L_POS] = LO_UINT16(fwvern);

	fwinfobuf[ELM_HDR_SIZE+NEW_FW_TOT_LEN_HH_POS] = HI_UINT16(fwlen>>16);
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_TOT_LEN_HL_POS] = LO_UINT16(fwlen>>16);
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_TOT_LEN_LH_POS] = HI_UINT16(fwlen&0xFFFF);
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_TOT_LEN_LL_POS] = LO_UINT16(fwlen&0xFFFF);
	
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_CRC_H_POS] = HI_UINT16(fwcrc);
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_CRC_L_POS] = LO_UINT16(fwcrc);
	
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_TOT_BLK_H_POS] = HI_UINT16(fwtotblk);
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_TOT_BLK_L_POS] = LO_UINT16(fwtotblk);

	fwinfobuf[ELM_HDR_SIZE+NEW_FW_UPGD_HOUR_POS] = (alrmhh>=24&&alrmmm!=0&&alrmss!=0 ? alrmhh-24: alrmhh);
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_UPGD_MINTS_POS] = alrmmm;
	fwinfobuf[ELM_HDR_SIZE+NEW_FW_UPGD_SECND_POS] = alrmss;

	return (ELM_HDR_SIZE+EVLEN_GMS_FW_INFO);
}


static uint8 filllocaltime(uint8 *timebuf)
{
	time_t timep;
	struct tm *p;
	
	time(&timep);
	p = localtime(&timep); //get local time

	printf("++++++++++time:%d/%d/%d.\r\n",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday);
	printf("++++++++++%d:%d:%d\r\n",p->tm_hour, p->tm_min, p->tm_sec);

	timebuf[0] = EID_GME_NT_TM;
	timebuf[EVAL_LEN_POS] = EVLEN_GME_NT_TM;

	timebuf[ELM_HDR_SIZE+UTCL_HOUR_POS]=(unsigned char)p->tm_hour;
	timebuf[ELM_HDR_SIZE+UTCL_MINTS_POS]=(unsigned char)p->tm_min;
	timebuf[ELM_HDR_SIZE+UTCL_SECND_POS]=(unsigned char)p->tm_sec;//second
	timebuf[ELM_HDR_SIZE+UTCL_DAY_POS]=(unsigned char)p->tm_mday;
	timebuf[ELM_HDR_SIZE+UTCL_MONTH_POS]=(unsigned char)(p->tm_mon+1);
	timebuf[ELM_HDR_SIZE+UTCL_YEAR_POS]=(unsigned char)(p->tm_year + 1900 -2000); //year, should plus 2000

	return (ELM_HDR_SIZE+EVLEN_GME_NT_TM);
}


static uint8 fillGDEresp(uint8 *respbuf, uint8 *data, uint8 len)
{
	(void) len;

	respbuf[0] = EID_GMS_INFO_ACK;
	respbuf[EVAL_LEN_POS] = EVLEN_GMS_INFO_ACK;
	respbuf[ELM_HDR_SIZE+GMS_INFO_ACK_MSG_POS]= *data;

	return (ELM_HDR_SIZE+GMS_INFO_ACK_MSG_SIZE);
}

static uint8 fillupgblk(uint8 *pldbuf, uint8 *data, uint8 len)
{
	if (len != EVLEN_GMS_UPGD)
		return 0;

	pldbuf[0] = EID_GMS_UPGD;
	pldbuf[EVAL_LEN_POS] = EVLEN_GMS_UPGD;
	memcpy(pldbuf+ELM_HDR_SIZE,data,len);

	return (ELM_HDR_SIZE+EVLEN_GMS_UPGD);
}

static uint8 fillrstbench(uint8 *pldbuf)
{
	pldbuf[0] = EID_GDE_BENCH_INFO;
	pldbuf[EVAL_LEN_POS] = EVLEN_GDE_BENCH_INFO;
	memset(pldbuf+ELM_HDR_SIZE, 0, EVLEN_GDE_BENCH_INFO);

	return (ELM_HDR_SIZE+EVLEN_GDE_BENCH_INFO);
}

static rfpkterr_t  formGMEpkt(uint8 subtype, uint8 *data, uint8 len)
{
	uint8 rfbuf[GMS_PKT_MAX_LEN]={0};
	uint8 curpldpos = GMS_PKT_PAYLOAD_POS;

	rfbuf[GMS_ID_POS]=GME_SRC_ID;
	
	rfbuf[GMS_SUB_TYPE_POS]=subtype;

	memcpy(rfbuf+GMS_RESERVE_POS, GMS_RESERVE_STR, GMS_RESERVE_SIZE);

	rfbuf[GMS_SRC_ADDR_H_POS] = HI_UINT16(RFdevID);
	rfbuf[GMS_SRC_ADDR_L_POS] = LO_UINT16(RFdevID);

	rfbuf[GMS_DEST_ADDR_H_POS] = HI_UINT16(RFdestID);	// need mutex?
	rfbuf[GMS_DEST_ADDR_L_POS] = LO_UINT16(RFdestID);

	rfbuf[GMS_VERSION_H_POS] = HI_UINT16(version);	// how to fill fist update
	rfbuf[GMS_VERSION_L_POS] = LO_UINT16(version);

	switch(subtype)
	{
		case GME_SUBTYPE_ORDER_UPGD_REQ:
			// Send upgrade firmware information
			curpldpos += fillupgdfreq(rfbuf+curpldpos);
			curpldpos += filllocaltime(rfbuf+curpldpos);
			curpldpos += fillupgfwinfo(rfbuf+curpldpos);
			break;
		case GME_SUBTYPE_CARINFO_RESP:
			// Car information response
			curpldpos += fillGDEresp(rfbuf+curpldpos, data, len);
			break;
		case GME_SUBTYPE_TMSYN_RESP:
			// Time synchronization response
			curpldpos += filllocaltime(rfbuf+curpldpos);
			break;
		case GME_SUBTYPE_UPGD_PKT:
			// Fill upgrade packet
			curpldpos += fillupgblk(rfbuf+curpldpos, data, len);
			break;
		case GME_SUBTYPE_RST_BENCH_PKT:
			// Fill reset benchmark packet
			curpldpos += fillrstbench(rfbuf+curpldpos);
			break;
		default:
			return RF_SUBTYPE_UNK;
	}
	
	rfbuf[GMS_TOT_LEN_POS]=curpldpos;

	rfbuf[GMS_CHK_SUM_POS]=calcGMchksum(rfbuf, rfbuf[GMS_TOT_LEN_POS]);

	return rfdatasend(rfbuf,rfbuf[GMS_TOT_LEN_POS]);

}

static rfpkterr_t rfdatasend(uint8 *buf, uint8 len)
{
	write(gme_sfd, buf, len);

	return RF_SUCCESS;
}

static void PrintGDETime(uint8* gdetm)
{
	printf("\tRecv time %d-%02d-%02d %02d:%02d:%02d\r\n",2000+gdetm[UTCL_YEAR_POS],gdetm[UTCL_MONTH_POS],\
			gdetm[UTCL_DAY_POS],gdetm[UTCL_HOUR_POS],gdetm[UTCL_MINTS_POS],gdetm[UTCL_SECND_POS]);
}

static void PrintHrtbtData(uint8* hrtbtval)
{
	printf("\tBat: %3d  Tmpr: %3d  X: %5d  Y: %5d  Z: %5d  Status: %c\r\n",\
			hrtbtval[HRT_BT_BATT_POS],(int8)hrtbtval[HRT_BT_TMPR_POS],\
			(int16)BUILD_UINT16(hrtbtval[HRT_BT_XVAL_L_POS], hrtbtval[HRT_BT_XVAL_H_POS]),\
			(int16)BUILD_UINT16(hrtbtval[HRT_BT_YVAL_L_POS], hrtbtval[HRT_BT_YVAL_H_POS]),\
			(int16)BUILD_UINT16(hrtbtval[HRT_BT_ZVAL_L_POS], hrtbtval[HRT_BT_ZVAL_H_POS]),
			(hrtbtval[HRT_BT_STAT_POS])? 'Y': 'N');
}
static void PrintBnchmk(uint8* bnchmk)
{
	printf("\tXB: %5d  YB: %5d  ZB %5d\r\n",(int16)BUILD_UINT16(bnchmk[GDE_X_L_BCHMRK_POS],bnchmk[GDE_X_H_BCHMRK_POS]),\
			(int16)BUILD_UINT16(bnchmk[GDE_Y_L_BCHMRK_POS],bnchmk[GDE_Y_H_BCHMRK_POS]),\
			(int16)BUILD_UINT16(bnchmk[GDE_Z_L_BCHMRK_POS],bnchmk[GDE_Z_H_BCHMRK_POS]));
}

static void upgrade_gde_fw(int iSignNo)
{
	uint16 fwblk,i;
	uint8 upgdbuf[EVLEN_GMS_UPGD];

	printf("Prepare state:");
	for (i=0; i<MAX_MNG_GDE_CNT; i++)
	{
		if (mnggdest[i]!=0)
			printf(" %d",i+1);
	}
	printf("\r\n");

	sleep(10);
	binfp=fopen(binfstr,"rb");
	for (fwblk=0; fwblk<fwtotblk; fwblk++)
	{
		upgdbuf[NEW_FW_CUR_BLK_H_POS] = HI_UINT16(fwblk);
		upgdbuf[NEW_FW_CUR_BLK_L_POS] = LO_UINT16(fwblk);
		fread(upgdbuf+RF_OAD_BLOCK_BEG_POS, 1, RF_OAD_BLOCK_SIZE, binfp);
		printf("Write NO.%d block...\r\n",fwblk+1);
		RFdestID = GDE_ADV_ID;
		formGMEpkt(GME_SUBTYPE_UPGD_PKT, upgdbuf, EVLEN_GMS_UPGD);
		usleep(150*1000);
		RFdestID = GDE_ADV_ID;
		formGMEpkt(GME_SUBTYPE_UPGD_PKT, upgdbuf, EVLEN_GMS_UPGD);
		usleep(150*1000);
	}
	printf("Firmware send finish!\r\n");
	fclose(binfp);
}

static uint32 get_file_sz(FILE * fp)
{
	uint32 f_len = 0;
	fseek(fp, 0, SEEK_END);
	f_len = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	return f_len;
}

static void prep_upgrade(char* binstr)
{
	uint8 len;
	fw_img_hdr_t tmpbuf;

	binfstr = binstr;
	if ((binfp=fopen(binfstr,"rb")) == NULL)
	{
		perror("Open bin file failed");
		exit(1);
	}
	len = get_file_sz(binfp);

	fread((uint8*)&tmpbuf, 1, sizeof(fw_img_hdr_t), binfp);
	fwvern = le_tohs(tmpbuf.oadvern);
	fwlen = (uint32)(le_tohs(tmpbuf.oadfwdlen))*4;
	fwcrc = le_tohs(tmpbuf.crcraw);
	fwtotblk = fwlen/64;

	fclose(binfp);

	if (fwvern!=0 && fwcrc!= 0 && fwlen!=0 && fwtotblk!=0 && fwlen%64==0 && len!=fwlen)
	{
		printf("Firmware version: V%02x.%02x, length: %dBytes, CRC: 0x%04x, blocks: %d.\r\n",\
				HI_UINT16(fwvern),LO_UINT16(fwvern), fwlen, fwcrc, fwtotblk);
		prepupgdflg = TRUE;
		memset(mnggdest,0,sizeof(mnggdest));
	}
	else
	{
		printf("Firmware bin file format error.\r\n");
		exit(1);
	}
}

static void set_upgrade_alarm(char* hmstmstr)
{
	uint8 pos,coloncnt=0,totlen;
	char* pminsnd[2];
	time_t timep;
	struct tm *pcurtm;
	uint32 alrmsnd;

	totlen = strlen(hmstmstr);

	// Start check from the second index
	for (pos=1; pos<totlen;pos++)
	{
		if (hmstmstr[pos] == ':')
		{
			if (++coloncnt > 2)
				timeerror();
			hmstmstr[pos] = '\0';
			pminsnd[coloncnt-1] = hmstmstr+pos+1;
		}
	}
	if (coloncnt != 2)
		timeerror();
	alrmhh = atoi(hmstmstr);
	alrmmm = atoi(pminsnd[0]);
	alrmss = atoi(pminsnd[1]);

	if (alrmhh>24 || (alrmhh==24 && alrmmm!=0 && alrmss!=0) || alrmhh <0)
		timeerror();
	if (alrmmm > 59 || alrmmm <0 || alrmss<0 || alrmss>59)
		timeerror();

	if (alrmhh==0 && alrmmm==0 && alrmss==0)
	{
		//upgrade_gde_fw();
		//return;
		// should regard as 24:00:00
	}
	
	time(&timep);
	pcurtm = localtime(&timep); //get local time

	if (alrmhh < pcurtm->tm_hour)
		alrmhh += 24;
	else if (alrmhh==pcurtm->tm_hour && alrmmm<pcurtm->tm_min )
		alrmhh += 24;
	else if (alrmhh==pcurtm->tm_hour && alrmmm==pcurtm->tm_min && alrmss<pcurtm->tm_sec)
		alrmhh += 24;

	
	printf("\r\n%d:%d:%d-->%d:%d:%d\r\n",pcurtm->tm_hour,pcurtm->tm_min,pcurtm->tm_sec,alrmhh,alrmmm,alrmss);

	alrmsnd = (uint32)(alrmhh-pcurtm->tm_hour)*60*60+(int32)(alrmmm-pcurtm->tm_min)*60+alrmss-pcurtm->tm_sec;

	printf("Wait %ld seconds to alarm.\r\n",alrmsnd);

	signal(SIGALRM, upgrade_gde_fw);
	alarm(alrmsnd);
	
}



static void httpsend(uint8 *buf, uint8 len)
{
	struct hostent *host;
	struct sockaddr_in server_addr;

	int sockfd;
	char recvbuffer[1024]={0};
	char request[1024]={0}; 
	
	// Ascii data len should be 2*MAX_LEN of bin data
	char data[GMS_PKT_MAX_LEN*2];
	int sendstate;
	int recvstate;
	int con_flag;
	int i,j;

/*	unsigned char gdehb[31] = {0x55,0x1F,0x15,0x52,0x46,0x8B,0x00,0x01,0x03,\
			0xE9,0x01,0x00,0x01,0x06,0x05,0x0E,0x30,0x1C,0x0C,0x0C,\
			0x02,0x09,0x20,0x21,0x00,0x11,0x00,0x22,0x00,0x33,0x00};
	//120.26.103.149:9999/gmehttpget?data=551F1552468B000103E901000106050E301C0C0C0209202100110022003300
	unsigned char gdep[31] ={0x55,0x1F,0x16,0x52,0x46,0x88,0x00,0x01,0x03,\
			0xE9,0x01,0x00,0x01,0x06,0x05,0x0E,0x30,0x1C,0x0C,0x0C,\
			0x02,0x09,0x20,0x21,0x00,0x11,0x00,0x22,0x00,0x33,0x00};*/

	for(i=0;i<len;i++)
		sprintf(data+i*2,"%02x",buf[i]);
	
	//sprintf(data+i*2,"%02x",gdehb[i]);
	//printf("%s\n",host_addr); 

	host=gethostbyname(host_addr);
	sockfd=socket(AF_INET,SOCK_STREAM,0);
	bzero(&server_addr,sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(host_port);
	server_addr.sin_addr = *((struct in_addr *)host->h_addr);
	con_flag = connect(sockfd,(struct sockaddr *)(&server_addr),sizeof(struct sockaddr));

	sprintf(request, "GET /gmehttpget?data=%s HTTP/1.1\r\nHost: %s:%d\r\nConnection: Close\r\n\r\n",\
			data, host_addr, host_port);

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
//		printf("%s", recvbuffer);//把http信息打印在屏幕上	 
	}

	close(sockfd);
	//return 0; 
}

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

static int asciitobin(char *ascbuf, int len, char *bbuf)
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

static void display_verion()
{
	printf("\r\nGME working application V%02d.%02d",HI_UINT16(version),LO_UINT16(version));
	exit(0);
}

static void timeerror(void)
{
	printf("Order upgrade time error, example HH:MM:SS (00:00:00~24:00:00).\r\n");
	printf("00:00:00 means upgrade now\r\n");
	exit(1);
}

static void usage()
{
	printf("Usage:\r\n");
	printf("\t(app name) (serial tty file) (serial baud) [option params]\r\n");
	printf("\tserial tty file, e.g. /dev/ttyUSB0\r\n");
	printf("\tserial baud, e.g. 9600\r\n");
	printf("\toption params: -d -a -b -s -r -p -u -t -v\r\n");
	printf("\t\t-d: serial debug print mode\r\n");
	printf("\t\t-a: set GME ID\r\n");
	printf("\t\t-b: reset GDE benchmark\r\n");
	printf("\t\t-s: set serial transparent module params\r\n");
	printf("\t\t-r: remote server IP address\r\n");
	printf("\t\t-p: remote server port\r\n");
	printf("\t\t-u: upgrade file\r\n");
	printf("\t\t-t: upgrade time, suggest at least 10min later\r\n");
	printf("\t\t-v: show application version\r\n");
	exit(1);
}

int main(int argc, char ** argv)
{
	char *c,*p;
	int nset1,len,Param;
	unsigned char buf[GMS_PKT_MAX_LEN];
	unsigned char crc_buf[GMS_PKT_MAX_LEN];
	sworkstate_t wkst=NORMAL_MODE;

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
				case 'd':
					wkst = DEBUG_MODE;
					--Param;
					break;
				case 's':
					wkst = SETUP_MODE;
					--Param;
					break;
				case 'a':
					RFdevID = atoi(p);
					break;
				case 'b':
					rstgdeid = atoi(p);
					break;
				case 'r':
					host_addr = p;
					break;
				case 'p':
					host_port = atoi(p);
					break;
				case 'u':
					prep_upgrade(p);
					break;
				case 't':
					set_upgrade_alarm(p);
					break;
				case 'v':
					display_verion();
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

	if (prepupgdflg==TRUE && alrmhh==0 && alrmmm==0 && alrmss==0)
	{
		printf("Need input upgrade time. see usage:\r\n");
		usage();
	}

	// 3G module use ttyUSB3-ttyUSB6, RF work module use ttyUSB0, RF upgrade module use ttyUSB1
	gme_sfd = open(argv[1], O_RDWR|O_NOCTTY|O_NONBLOCK);
	if(gme_sfd == -1)
	{
		printf("Open serial wrong!\n");
		exit(1);
	}
	
	nset1 = serial_set_opt(gme_sfd, atoi(argv[2]), 8, 'N', 1);
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
		memset(buf, 0, GMS_PKT_MAX_LEN);	
		if (wkst == NORMAL_MODE)
		{
			len = read(gme_sfd, buf, GMS_PKT_MAX_LEN);
			if (len > 0)
			{
				gmspktform(buf,len);
			}
			usleep(200*1000);
		}
		else if (wkst == DEBUG_MODE)
		{
			len = read(gme_sfd, buf, GMS_PKT_MAX_LEN);
			if (len > 0)
			{
				static int totlen = 0;
				totlen += len;
				printf("\r\nRecv: %d, total: %d\r\n",len,totlen);
				db(buf,len);
			}
		}
		else if (wkst == SETUP_MODE)
		{
			printf("Serial command set module:\r\n");
			scanf("%s",buf);
			len = strlen((char *)buf);
			if ( len > 0 )
			{
				if (strcmp((char *)buf,"exit") == 0)
				{
					wkst = NORMAL_MODE;
					printf("back to read mode:\r\n");
					continue;
				}
				else
				{
					len = asciitobin((char *)buf,len,(char *)crc_buf);
					printf("\r\nSend bin command: ");
					db(crc_buf,len);
					write(gme_sfd,crc_buf,len);	
				}
			}
			sleep(1);
			if (len > 0)
			{
				len = read(gme_sfd,buf,sizeof(buf));
				printf("\r\nRecieve command response: ");
				db(buf,len);
			}
		}
	}
	
	close(gme_sfd);
	
	return 0;
}


/*********************************************************************
 * @fn		CPUBigEndian
 *
 * @brief	check CPU endian
 *
 * @param	none
 *
 * @return	TRUE - big endian
 */
static uint8 CPUBigEndian(void)
{
	union{
		uint32 i;
		uint8 s[4];
	}c;

	c.i = 0x12345678;
	return (0x12 == c.s[0]);
}

/*********************************************************************
 * @fn		le_tohs
 *
 * @brief	Little endian short int data change to host.
 *
 * @param	none
 *
 * @return	Host endian value
 */
static uint16 le_tohs(uint16 n)
{
	return CPUBigEndian() ? BigLittleSwapShort(n) : n;
}

