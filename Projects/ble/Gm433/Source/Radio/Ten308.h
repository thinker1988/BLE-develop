//*****************************************************************************
//! @file		Ten308.h
//! @brief		Template for Ten308
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

#ifndef TEN308_CONFIG_H
#define TEN308_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
 * INCLUDES
 */

/******************************************************************************
 * CONSTANTS
 */

// Wait 100ms for serial command data send
#define WAIT_TEN_CMD_PERIOD			100

// Wait 50ms for TEN module enter sleep
#define WAIT_TEN_STOP_PERIOD		50

// Wait 250ms for TEN module RF ready to RX/TX
#define WAIT_TEN_RF_RDY_PERIOD		250

// Total RF work period
#define WAIT_RF_WORK_PERIOD			(WAIT_RF_START_PERIOD+WAIT_TEN_CMD_PERIOD+WAIT_TEN_STOP_PERIOD+WAIT_TEN_RF_RDY_PERIOD)

// Max length of TEN308 cmd
#define TEN_RF_CMD_MAX_LEN			20

#define TEN308_DIOA_PINSEL			P0SEL
#define TEN308_DIOA_PINDIR			P0DIR
#define TEN308_DIOA_PININP			P0INP
#define TEN308_DIOA_GPIO			BV(6)
#define TEN308_DIOA_PIN				P0_6

#define TEN308_DIOB_PINSEL			P1SEL
#define TEN308_DIOB_PINDIR			P1DIR
#define TEN308_DIOB_PININP			P1INP
#define TEN308_DIOB_GPIO			BV(2)
#define TEN308_DIOB_PIN				P1_2


/*********************************************************************
 * MACROS
 */
// TEN308 have 2 RF frequency
#define CHECK_FREQ_VALID(freq)		(((freq)>0 && (freq)<sizeof(tenRFfreq)/sizeof(tenRFfreq[0])+1)? TRUE: FALSE)

#define CHECK_BAUD_VALID(baud)		(((baud)>0 && (baud)<sizeof(tenRFairbaud)+1)? TRUE: FALSE)

#define CHECK_PWR_VALID(plvl)		(((plvl)>0 && (plvl)<sizeof(tenRFpwr)+1)? TRUE: FALSE)


/******************************************************************************
 * VARIABLES
 */

// TEN308 RF frequency
// 433 & 470
static const uint8 tenRFfreq[][3] =
{
	{0x06,0x9B,0x68},	// 433.0
	{0x07,0x2B,0xF0},	// 470.0
};

// TEN308 RF air baud
// 00~0.81k  01~1.46k  02~2.6k  03~4.56k  04~9.11k  05~18.23k
static const uint8 tenRFairbaud[] =
{0x00,0x01,0x02,0x03,0x04,0x05};

// TEN308 RF send power
// 00~6dBm  01~8dBm  02~10dBm  03~12dBm  04~14dBm  05~16dBm  06~18dBm  07~20dBm
static const uint8 tenRFpwr[] =
{0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};



#ifdef	__cplusplus
 }
#endif
 
#endif
