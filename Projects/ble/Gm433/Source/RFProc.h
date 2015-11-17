//*****************************************************************************
//! @file			RFProc.h
//! @brief			Template for CC112x register export from SmartRF Studio 
//
//	Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//	Redistribution and use in source and binary forms, with or without
//	modification, are permitted provided that the following conditions
//	are met:
//
//			Redistributions of source code must retain the above copyright
//			notice, this list of conditions and the following disclaimer.
//
//			Redistributions in binary form must reproduce the above copyright
//			notice, this list of conditions and the following disclaimer in the
//			documentation and/or other materials provided with the distribution.
//
//			Neither the name of Texas Instruments Incorporated nor the names of
//			its contributors may be used to endorse or promote products derived
//			from this software without specific prior written permission.
//
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//	OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef RFPROC_H
#define RFPROC_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "Com433.h"
#include "BLECore.h"
#include "Pktfmt.h"

#if ( defined USE_SX1278_RF )
#include "Sx1278.h"
#elif ( defined USE_CC112X_RF )
#include "Cc112x.h"
#elif ( defined USE_TEN308_RF )
#include "Ten308.h"
#else
	#error "Not defined RF!"
#endif


/******************************************************************************
 * CONSTANTS
 */
#define WAIT_RF_PRESET_PERIOD		100


#if ( defined HAL_SPI_MASTER )
//modified by lan 15.8.3, control by app
#define SPI_CSN_PXDIR				P1DIR
#define SPI_CSN_PIN					P1_4
#define SPI_CSN_BIT					BV(4)
#define SPI_MISO_PIN				P1_7

// CC112X GPIO2 INT -> P1.3
#define RF_SYNC_INT_PINSEL			P1SEL
#define RF_SYNC_INT_PINDIR			P1DIR
#define RF_SYNC_INT_IFG				P1IFG
#define RF_SYNC_INT_IF				P1IF
#define RF_SYNC_INT_IEN				P1IEN
#define RF_SYNC_INT_IE				BV(3)

/*********************************************************************
 * MACROS
 */

#define TRXEN_SPI_BEGIN()			st(SPI_CSN_PIN = 0; asm("NOP");)
#define TRXEN_SPI_END()				st(asm("NOP"); SPI_CSN_PIN = 1;)

#define WAIT_SO_STABLE()			st(while(SPI_MISO_PIN == 1);)

#define RF_SYNC_INT_ENABLE()		st(RF_SYNC_INT_IEN |= RF_SYNC_INT_IE;)
#define RF_SYNC_INT_DISABLE()		st(RF_SYNC_INT_IEN &= ~RF_SYNC_INT_IE;)

#endif	// HAL_SPI_MASTER

/******************************************************************************
 * TYPEDEFS
 */


// TEN & CC112X RF working state
typedef enum
{
	RF_PRESET,	// TEN & CC112X RF first boot up
	RF_BEG_SET,	// TEN & CC112X RF send setup command
#if ( defined USE_CC112X_RF || defined USE_SX1278_RF )
	RF_SEND,	// CC112X RF send
	RF_RECV,	// CC112X RF receive
#endif

#if ( defined USE_TEN308_RF || defined USE_SX1278_RF )
	RF_WAIT_RESET,	// TEN RF device reset
#endif

#if ( defined USE_TEN308_RF )
	RF_WAKEUP,	// TEN RF device wake up
	RF_WORK,	// TEN RF data process
#endif	// USE_CC112X_RF
	RF_SLEEP	// TEN & CC112X RF sleep
}rfstate_t;

typedef uint8 rfStatus_t;


/******************************************************************************
 * PROTPTYPES
 */
extern void SetRFstate(rfstate_t newrfstate);
extern rfstate_t GetRFstate(void);

extern void RF_working(uint8 task_id, rfstate_t newrfstate);

extern void ReadRFParam(uint8 * rdbuf);
extern bool SetRFParam(uint8 wkfreq, uint8 setfreq, uint8 upgdfreq, uint8 baud, uint8 pwlvl);
extern uint8 GetCurFreq(sysstate_t state);

#if ( defined HAL_SPI_MASTER )
extern void SetRTxRdyFlg(bool flag);
extern bool GetRTxRdyFlg();
extern void trxSingleTX(uint8 data);
extern uint8 trxSingleRX(void);
#endif	//HAL_SPI_MASTER

#ifdef	__cplusplus
}
#endif

#endif
