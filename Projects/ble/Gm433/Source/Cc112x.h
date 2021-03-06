//*****************************************************************************
//! @file       cc112x.h
//! @brief      Template for CC112x register export from SmartRF Studio 
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//      Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//      Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//      Neither the name of Texas Instruments Incorporated nor the names of
//      its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/

#ifndef CC112X_EASY_LINK_REG_CONFIG_H
#define CC112X_EASY_LINK_REG_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
 * INCLUDES
 */
/* configuration registers */
#define CC112X_IOCFG3                   0x0000
#define CC112X_IOCFG2                   0x0001
#define CC112X_IOCFG1                   0x0002
#define CC112X_IOCFG0                   0x0003
#define CC112X_SYNC3                    0x0004
#define CC112X_SYNC2                    0x0005
#define CC112X_SYNC1                    0x0006
#define CC112X_SYNC0                    0x0007
#define CC112X_SYNC_CFG1                0x0008
#define CC112X_SYNC_CFG0                0x0009
#define CC112X_DEVIATION_M              0x000A
#define CC112X_MODCFG_DEV_E             0x000B
#define CC112X_DCFILT_CFG               0x000C
#define CC112X_PREAMBLE_CFG1            0x000D
#define CC112X_PREAMBLE_CFG0            0x000E
#define CC112X_FREQ_IF_CFG              0x000F
#define CC112X_IQIC                     0x0010
#define CC112X_CHAN_BW                  0x0011
#define CC112X_MDMCFG1                  0x0012
#define CC112X_MDMCFG0                  0x0013
#define CC112X_SYMBOL_RATE2             0x0014
#define CC112X_SYMBOL_RATE1             0x0015
#define CC112X_SYMBOL_RATE0             0x0016
#define CC112X_AGC_REF                  0x0017
#define CC112X_AGC_CS_THR               0x0018
#define CC112X_AGC_GAIN_ADJUST          0x0019
#define CC112X_AGC_CFG3                 0x001A
#define CC112X_AGC_CFG2                 0x001B
#define CC112X_AGC_CFG1                 0x001C
#define CC112X_AGC_CFG0                 0x001D
#define CC112X_FIFO_CFG                 0x001E
#define CC112X_DEV_ADDR                 0x001F
#define CC112X_SETTLING_CFG             0x0020
#define CC112X_FS_CFG                   0x0021
#define CC112X_WOR_CFG1                 0x0022
#define CC112X_WOR_CFG0                 0x0023
#define CC112X_WOR_EVENT0_MSB           0x0024
#define CC112X_WOR_EVENT0_LSB           0x0025
#define CC112X_PKT_CFG2                 0x0026
#define CC112X_PKT_CFG1                 0x0027
#define CC112X_PKT_CFG0                 0x0028
#define CC112X_RFEND_CFG1               0x0029
#define CC112X_RFEND_CFG0               0x002A
#define CC112X_PA_CFG2                  0x002B
#define CC112X_PA_CFG1                  0x002C
#define CC112X_PA_CFG0                  0x002D
#define CC112X_PKT_LEN                  0x002E

/* Extended Configuration Registers */
#define CC112X_IF_MIX_CFG               0x2F00
#define CC112X_FREQOFF_CFG              0x2F01
#define CC112X_TOC_CFG                  0x2F02
#define CC112X_MARC_SPARE               0x2F03
#define CC112X_ECG_CFG                  0x2F04
#define CC112X_CFM_DATA_CFG             0x2F05
#define CC112X_EXT_CTRL                 0x2F06
#define CC112X_RCCAL_FINE               0x2F07
#define CC112X_RCCAL_COARSE             0x2F08
#define CC112X_RCCAL_OFFSET             0x2F09
#define CC112X_FREQOFF1                 0x2F0A
#define CC112X_FREQOFF0                 0x2F0B
#define CC112X_FREQ2                    0x2F0C
#define CC112X_FREQ1                    0x2F0D
#define CC112X_FREQ0                    0x2F0E
#define CC112X_IF_ADC2                  0x2F0F
#define CC112X_IF_ADC1                  0x2F10
#define CC112X_IF_ADC0                  0x2F11
#define CC112X_FS_DIG1                  0x2F12
#define CC112X_FS_DIG0                  0x2F13
#define CC112X_FS_CAL3                  0x2F14
#define CC112X_FS_CAL2                  0x2F15
#define CC112X_FS_CAL1                  0x2F16
#define CC112X_FS_CAL0                  0x2F17
#define CC112X_FS_CHP                   0x2F18
#define CC112X_FS_DIVTWO                0x2F19
#define CC112X_FS_DSM1                  0x2F1A
#define CC112X_FS_DSM0                  0x2F1B
#define CC112X_FS_DVC1                  0x2F1C
#define CC112X_FS_DVC0                  0x2F1D
#define CC112X_FS_LBI                   0x2F1E
#define CC112X_FS_PFD                   0x2F1F
#define CC112X_FS_PRE                   0x2F20
#define CC112X_FS_REG_DIV_CML           0x2F21
#define CC112X_FS_SPARE                 0x2F22
#define CC112X_FS_VCO4                  0x2F23
#define CC112X_FS_VCO3                  0x2F24
#define CC112X_FS_VCO2                  0x2F25
#define CC112X_FS_VCO1                  0x2F26
#define CC112X_FS_VCO0                  0x2F27
#define CC112X_GBIAS6                   0x2F28
#define CC112X_GBIAS5                   0x2F29
#define CC112X_GBIAS4                   0x2F2A
#define CC112X_GBIAS3                   0x2F2B
#define CC112X_GBIAS2                   0x2F2C
#define CC112X_GBIAS1                   0x2F2D
#define CC112X_GBIAS0                   0x2F2E
#define CC112X_IFAMP                    0x2F2F
#define CC112X_LNA                      0x2F30
#define CC112X_RXMIX                    0x2F31
#define CC112X_XOSC5                    0x2F32
#define CC112X_XOSC4                    0x2F33
#define CC112X_XOSC3                    0x2F34
#define CC112X_XOSC2                    0x2F35
#define CC112X_XOSC1                    0x2F36
#define CC112X_XOSC0                    0x2F37
#define CC112X_ANALOG_SPARE             0x2F38
#define CC112X_PA_CFG3                  0x2F39
#define CC112X_IRQ0M                    0x2F3F
#define CC112X_IRQ0F                    0x2F40 

/* Status Registers */
#define CC112X_WOR_TIME1                0x2F64
#define CC112X_WOR_TIME0                0x2F65
#define CC112X_WOR_CAPTURE1             0x2F66
#define CC112X_WOR_CAPTURE0             0x2F67
#define CC112X_BIST                     0x2F68
#define CC112X_DCFILTOFFSET_I1          0x2F69
#define CC112X_DCFILTOFFSET_I0          0x2F6A
#define CC112X_DCFILTOFFSET_Q1          0x2F6B
#define CC112X_DCFILTOFFSET_Q0          0x2F6C
#define CC112X_IQIE_I1                  0x2F6D
#define CC112X_IQIE_I0                  0x2F6E
#define CC112X_IQIE_Q1                  0x2F6F
#define CC112X_IQIE_Q0                  0x2F70
#define CC112X_RSSI1                    0x2F71
#define CC112X_RSSI0                    0x2F72
#define CC112X_MARCSTATE                0x2F73
#define CC112X_LQI_VAL                  0x2F74
#define CC112X_PQT_SYNC_ERR             0x2F75
#define CC112X_DEM_STATUS               0x2F76
#define CC112X_FREQOFF_EST1             0x2F77
#define CC112X_FREQOFF_EST0             0x2F78
#define CC112X_AGC_GAIN3                0x2F79
#define CC112X_AGC_GAIN2                0x2F7A
#define CC112X_AGC_GAIN1                0x2F7B
#define CC112X_AGC_GAIN0                0x2F7C
#define CC112X_CFM_RX_DATA_OUT          0x2F7D
#define CC112X_CFM_TX_DATA_IN           0x2F7E
#define CC112X_ASK_SOFT_RX_DATA         0x2F7F
#define CC112X_RNDGEN                   0x2F80
#define CC112X_MAGN2                    0x2F81
#define CC112X_MAGN1                    0x2F82
#define CC112X_MAGN0                    0x2F83
#define CC112X_ANG1                     0x2F84
#define CC112X_ANG0                     0x2F85
#define CC112X_CHFILT_I2                0x2F86
#define CC112X_CHFILT_I1                0x2F87
#define CC112X_CHFILT_I0                0x2F88
#define CC112X_CHFILT_Q2                0x2F89
#define CC112X_CHFILT_Q1                0x2F8A
#define CC112X_CHFILT_Q0                0x2F8B
#define CC112X_GPIO_STATUS              0x2F8C
#define CC112X_FSCAL_CTRL               0x2F8D
#define CC112X_PHASE_ADJUST             0x2F8E
#define CC112X_PARTNUMBER               0x2F8F
#define CC112X_PARTVERSION              0x2F90
#define CC112X_SERIAL_STATUS            0x2F91
#define CC112X_MODEM_STATUS1            0x2F92
#define CC112X_MODEM_STATUS0            0x2F93
#define CC112X_MARC_STATUS1             0x2F94
#define CC112X_MARC_STATUS0             0x2F95
#define CC112X_PA_IFAMP_TEST            0x2F96
#define CC112X_FSRF_TEST                0x2F97
#define CC112X_PRE_TEST                 0x2F98
#define CC112X_PRE_OVR                  0x2F99
#define CC112X_ADC_TEST                 0x2F9A
#define CC112X_DVC_TEST                 0x2F9B
#define CC112X_ATEST                    0x2F9C
#define CC112X_ATEST_LVDS               0x2F9D
#define CC112X_ATEST_MODE               0x2F9E
#define CC112X_XOSC_TEST1               0x2F9F
#define CC112X_XOSC_TEST0               0x2FA0  
                                        
#define CC112X_RXFIRST                  0x2FD2   
#define CC112X_TXFIRST                  0x2FD3   
#define CC112X_RXLAST                   0x2FD4 
#define CC112X_TXLAST                   0x2FD5 
#define CC112X_NUM_TXBYTES              0x2FD6  /* Number of bytes in TXFIFO */ 
#define CC112X_NUM_RXBYTES              0x2FD7  /* Number of bytes in RXFIFO */
#define CC112X_FIFO_NUM_TXBYTES         0x2FD8  
#define CC112X_FIFO_NUM_RXBYTES         0x2FD9  

                                                                                                                                                
/* DATA FIFO Access */
#define CC112X_SINGLE_TXFIFO            0x003F      /*  TXFIFO  - Single accecss to Transmit FIFO */
#define CC112X_BURST_TXFIFO             0x007F      /*  TXFIFO  - Burst accecss to Transmit FIFO  */
#define CC112X_SINGLE_RXFIFO            0x00BF      /*  RXFIFO  - Single accecss to Receive FIFO  */
#define CC112X_BURST_RXFIFO             0x00FF      /*  RXFIFO  - Busrrst ccecss to Receive FIFO  */

#define CC112X_LQI_CRC_OK_BM            0x80
#define CC112X_LQI_EST_BM               0x7F



/* Command strobe registers */
#define CC112X_SRES                     0x30      /*  SRES    - Reset chip. */
#define CC112X_SFSTXON                  0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define CC112X_SXOFF                    0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define CC112X_SCAL                     0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define CC112X_SRX                      0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define CC112X_STX                      0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define CC112X_SIDLE                    0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define CC112X_SWOR                     0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define CC112X_SPWD                     0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define CC112X_SFRX                     0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define CC112X_SFTX                     0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define CC112X_SWORRST                  0x3C      /*  SWORRST - Reset real time clock. */
#define CC112X_SNOP                     0x3D      /*  SNOP    - No operation. Returns status byte. */
#define CC112X_AFC                      0x37      /*  AFC     - Automatic Frequency Correction */

/* Chip states returned in status byte */
#define CC112X_STATE_IDLE               0x00
#define CC112X_STATE_RX                 0x10
#define CC112X_STATE_TX                 0x20
#define CC112X_STATE_FSTXON             0x30
#define CC112X_STATE_CALIBRATE          0x40
#define CC112X_STATE_SETTLING           0x50
#define CC112X_STATE_RXFIFO_ERROR       0x60
#define CC112X_STATE_TXFIFO_ERROR       0x70


#define RADIO_BURST_ACCESS   0x40
#define RADIO_SINGLE_ACCESS  0x00
#define RADIO_READ_ACCESS    0x80
#define RADIO_WRITE_ACCESS   0x00


/* Bit fields in the chip status byte */
#define STATUS_CHIP_RDYn_BM             0x80
#define STATUS_STATE_BM                 0x70
#define STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F



typedef struct
{
  uint16  addr;
  uint8   data;
}registerSetting_t;

typedef uint8 rfStatus_t;


/******************************************************************************
 * VARIABLES
 */

#if defined CC1120
#ifndef _RF_SETTINGS_H
#define _RF_SETTINGS_H

#define SMARTRF_SETTING_IOCFG3           0xB0
#define SMARTRF_SETTING_IOCFG2           0x06
#define SMARTRF_SETTING_IOCFG1           0xB0
#define SMARTRF_SETTING_IOCFG0           0x40
#define SMARTRF_SETTING_SYNC_CFG1        0x0B
#define SMARTRF_SETTING_DEVIATION_M      0x48
#define SMARTRF_SETTING_MODCFG_DEV_E     0x05
#define SMARTRF_SETTING_DCFILT_CFG       0x1C
#define SMARTRF_SETTING_IQIC             0x46
#define SMARTRF_SETTING_CHAN_BW          0x02
#define SMARTRF_SETTING_MDMCFG0          0x05
#define SMARTRF_SETTING_SYMBOL_RATE2     0x73
#define SMARTRF_SETTING_AGC_REF          0x20
#define SMARTRF_SETTING_AGC_CS_THR       0x19
#define SMARTRF_SETTING_AGC_CFG1         0xA9
#define SMARTRF_SETTING_AGC_CFG0         0xCF
#define SMARTRF_SETTING_FIFO_CFG         0x00
#define SMARTRF_SETTING_SETTLING_CFG     0x03
#define SMARTRF_SETTING_FS_CFG           0x14
#define SMARTRF_SETTING_PKT_CFG0         0x20
#define SMARTRF_SETTING_PA_CFG0          0x7D
#define SMARTRF_SETTING_PKT_LEN          0xFF
#define SMARTRF_SETTING_IF_MIX_CFG       0x00
#define SMARTRF_SETTING_FREQOFF_CFG      0x22
#define SMARTRF_SETTING_FREQ2            0x6C
#define SMARTRF_SETTING_FREQ1            0x40
#define SMARTRF_SETTING_FS_DIG1          0x00
#define SMARTRF_SETTING_FS_DIG0          0x5F
#define SMARTRF_SETTING_FS_CAL1          0x40
#define SMARTRF_SETTING_FS_CAL0          0x0E
#define SMARTRF_SETTING_FS_DIVTWO        0x03
#define SMARTRF_SETTING_FS_DSM0          0x33
#define SMARTRF_SETTING_FS_DVC0          0x17
#define SMARTRF_SETTING_FS_PFD           0x50
#define SMARTRF_SETTING_FS_PRE           0x6E
#define SMARTRF_SETTING_FS_REG_DIV_CML   0x14
#define SMARTRF_SETTING_FS_SPARE         0xAC
#define SMARTRF_SETTING_FS_VCO0          0xB4
#define SMARTRF_SETTING_XOSC5            0x0E
#define SMARTRF_SETTING_XOSC1            0x03

#define SMARTRF_SETTING_SYNC3            0xD3
#define SMARTRF_SETTING_SYNC2            0x91
#define SMARTRF_SETTING_SYNC1            0xD3
#define SMARTRF_SETTING_SYNC0            0x91
#define SMARTRF_SETTING_SYMBOL_RATE1     0xA9
#define SMARTRF_SETTING_SYMBOL_RATE0     0x2A
#define SMARTRF_SETTING_PKT_CFG2         0x04
#define SMARTRF_SETTING_PKT_CFG1         0x05
#define SMARTRF_SETTING_PREAMBLE_CFG1    0x18  
#define SMARTRF_SETTING_PREAMBLE_CFG0    0x2A  
#define SMARTRF_SETTING_SYNC_CFG0        0x17 

#endif	/* _RF_SETTINGS_H */


static const uint8 rfLogicalChanTable[][4] =
{
  {0x6B, 0x00, 0x00}, /* 428MHz */
  {0x6B, 0x0C, 0xCD}, 
  {0x6B, 0x19, 0x99}, 
  {0x6B, 0x26, 0x66},
  {0x6B, 0x33, 0x33},
  {0x6B, 0x40, 0x00}, /* 429MHz */  
  {0x6B, 0x4C, 0xCD}, 
  {0x6B, 0x59, 0x99}, 
  {0x6B, 0x66, 0x66},
  {0x6B, 0x73, 0x33},
  {0x6B, 0x80, 0x00}, /* 430MHz */ 
  {0x6B, 0x8C, 0xCD}, 
  {0x6B, 0x99, 0x99}, 
  {0x6B, 0xA6, 0x66},
  {0x6B, 0xB3, 0x33},
  {0x6B, 0xC0, 0x00}, /* 431MHz */
  {0x6B, 0xCC, 0xCD}, 
  {0x6B, 0xD9, 0x99}, 
  {0x6B, 0xE6, 0x66},
  {0x6B, 0xF3, 0x33},
  {0x6C, 0x00, 0x00}, /* 432MHz */
  {0x6C, 0x0C, 0xCD}, 
  {0x6C, 0x19, 0x99}, 
  {0x6C, 0x26, 0x66},
  {0x6C, 0x33, 0x33},
  {0x6C, 0x40, 0x00}, /* 433MHz */
  {0x6C, 0x4C, 0xCD}, 
  {0x6C, 0x59, 0x99}, 
  {0x6C, 0x66, 0x66},
  {0x6C, 0x73, 0x33},
  {0x6C, 0x80, 0x00}, /* 434MHz */
  {0x6C, 0x8C, 0xCD}, 
  {0x6C, 0x99, 0x99}, 
  {0x6C, 0xA6, 0x66},
  {0x6C, 0xB3, 0x33},
  {0x6C, 0xC0, 0x00}, /* 435MHz */
  {0x6C, 0xCC, 0xCD}, 
  {0x6C, 0xD9, 0x99}, 
  {0x6C, 0xE6, 0x66},
  {0x6C, 0xF3, 0x33},
  {0x6D, 0x00, 0x00}, /* 436MHz */
  {0x6D, 0x0C, 0xCD}, 
  {0x6D, 0x19, 0x99}, 
  {0x6D, 0x26, 0x66},
  {0x6D, 0x33, 0x33},
  {0x6D, 0x40, 0x00}, /* 437MHz */
  {0x6D, 0x4C, 0xCD}, 
  {0x6D, 0x59, 0x99}, 
  {0x6D, 0x66, 0x66},
  {0x6D, 0x73, 0x33},
  {0x6D, 0x80, 0x00}, /* 438MHz */
};

static const uint8 rfRFPowerTable[] =
{
  0x7F, /* 15dBm */
  0x7D, /* 14dBm */
  0x7B, /* 13dBm */
  0x79, /* 12dBm */
  0x77, /* 11dBm */
  0x74, /* 10dBm */
  0x72, /* 9dBm */
  0x6F, /* 8dBm */
  0x6D, /* 7dBm */
  0x6B, /* 6dBm */
  0x69, /* 5dBm */
  0x66, /* 4dBm */
  0x64, /* 3dBm */
  0x62, /* 2dBm */
  0x5F, /* 1dBm */
  0x5D, /* 0dBm */
  0x56, /* -3dBm */
  0x4F, /* -6dBm */
  0x43, /* -11dBm */
};

/* Common configuration for diffrent data rate */
static const registerSetting_t rfRadioCfgCm[] =
{
  {CC112X_IOCFG0,            SMARTRF_SETTING_IOCFG0},
//  {CC112X_RFEND_CFG1,        SMARTRF_SETTING_RFEND_CFG1},
//  {CC112X_RFEND_CFG0,        SMARTRF_SETTING_RFEND_CFG0},
  {CC112X_PKT_CFG2,          SMARTRF_SETTING_PKT_CFG2},
  {CC112X_PKT_CFG1,          SMARTRF_SETTING_PKT_CFG1},
  {CC112X_PKT_CFG0,          SMARTRF_SETTING_PKT_CFG0},
  {CC112X_SETTLING_CFG,      SMARTRF_SETTING_SETTLING_CFG},
  {CC112X_PKT_LEN,           SMARTRF_SETTING_PKT_LEN},
  {CC112X_FIFO_CFG,          SMARTRF_SETTING_FIFO_CFG},
  {CC112X_SYNC3,             SMARTRF_SETTING_SYNC3},  
  {CC112X_SYNC2,             SMARTRF_SETTING_SYNC2},  
  {CC112X_SYNC1,             SMARTRF_SETTING_SYNC1},  
  {CC112X_SYNC0,             SMARTRF_SETTING_SYNC0}, 
  {CC112X_MDMCFG0,           SMARTRF_SETTING_MDMCFG0},
  {CC112X_AGC_CFG1,          SMARTRF_SETTING_AGC_CFG1},
  {CC112X_IF_MIX_CFG,        SMARTRF_SETTING_IF_MIX_CFG},
  {CC112X_FS_DIG1,           SMARTRF_SETTING_FS_DIG1},
  {CC112X_FS_DIG0,           SMARTRF_SETTING_FS_DIG0},
  {CC112X_FS_CAL1,           SMARTRF_SETTING_FS_CAL1},
  {CC112X_FS_CAL0,           SMARTRF_SETTING_FS_CAL0},
  {CC112X_FS_DIVTWO,         SMARTRF_SETTING_FS_DIVTWO},
  {CC112X_FS_DSM0,           SMARTRF_SETTING_FS_DSM0},
  {CC112X_FS_DVC0,           SMARTRF_SETTING_FS_DVC0},
  {CC112X_FS_PFD,            SMARTRF_SETTING_FS_PFD},
  {CC112X_FS_PRE,            SMARTRF_SETTING_FS_PRE},
  {CC112X_FS_REG_DIV_CML,    SMARTRF_SETTING_FS_REG_DIV_CML},
  {CC112X_FS_SPARE,          SMARTRF_SETTING_FS_SPARE},
  {CC112X_FS_VCO0,           SMARTRF_SETTING_FS_VCO0},
  {CC112X_XOSC5,             SMARTRF_SETTING_XOSC5},
  {CC112X_XOSC1,             SMARTRF_SETTING_XOSC1},


	{CC112X_IOCFG3,			 0xB0},  // not use GPIO3, 3-state
	{CC112X_IOCFG2,			  0x06}, // GPIO2 - PKT_SYNC_RXTX
	{CC112X_IOCFG1,			  0xB0}, // not use GPIO1, 3-state

  	{CC112X_FREQ2,			   0x6B}, // M: 6C
	{CC112X_FREQ1,			   0x00}, // M: 80	New 428
	{CC112X_FREQ0,			   0x00},
};

/* Special configuration for different data rate.
 * [1.2k][2.4k][4.8k][9.6k][19.2k][38.4k][50k][100k][150k][200k][REG] 
 */
static const uint16 rfRadioCfgSp[][11] =
{
  {0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x08, 0x07, 0x07, CC112X_SYNC_CFG1},
  {0x06, 0x06, 0x3B, 0x48, 0x48, 0x48, 0x99, 0x99, 0x53, 0x53, CC112X_DEVIATION_M},
  {0x03, 0x03, 0x04, 0x05, 0x05, 0x05, 0x05, 0x2D, 0x2F, 0x2F, CC112X_MODCFG_DEV_E},
  {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x15, 0x15, 0x04, 0x04, CC112X_DCFILT_CFG},
  {0x18, 0x18, 0x18, 0x18, 0x18, 0x20, 0x18, 0x18, 0x18, 0x18, CC112X_PREAMBLE_CFG0},////
  {0x18, 0x18, 0x18, 0x18, 0x18, 0x14, 0x18, 0x18, 0x18, 0x18, CC112X_PREAMBLE_CFG1},
  {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x3A, 0x3A, 0x00, 0x00, CC112X_FREQ_IF_CFG},
  {0xC6, 0xC6, 0x46, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, CC112X_IQIC},
  {0x08, 0x08, 0x04, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, CC112X_CHAN_BW},
  {0x43, 0x53, 0x63, 0x73, 0x83, 0x93, 0x99, 0x99, 0xA3, 0xA9, CC112X_SYMBOL_RATE2},
  {0xA9, 0xA9, 0xA9, 0xA9, 0xA9, 0xA9, 0x99, 0x99, 0x33, 0x99, CC112X_SYMBOL_RATE1},
  {0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x99, 0x9A, 0x33, 0x99, CC112X_SYMBOL_RATE0},
  {0x20, 0x20, 0x20, 0x20, 0x20, 0x36, 0x3C, 0x3C, 0x3C, 0x3C, CC112X_AGC_REF},
  {0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0xEF, 0xEF, 0xEC, 0xEC, CC112X_AGC_CS_THR},
  {0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x83, 0x83, CC112X_AGC_CFG3},
  {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x60, 0x60, CC112X_AGC_CFG2},
  {0xCF, 0xCF, 0xCF, 0xCF, 0xCF, 0xCF, 0xC0, 0xC0, 0xC0, 0xC0, CC112X_AGC_CFG0},
  {0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, CC112X_FS_CFG},
  {0x7E, 0x7E, 0x7E, 0x7D, 0x7C, 0x7B, 0x79, 0x7B, 0x02, 0x01, CC112X_PA_CFG0},
  {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x20, 0x20, CC112X_FREQOFF_CFG},
  {0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0A, 0x0A, 0x0A, 0x0A, CC112X_TOC_CFG},
};

#else
// RX filter BW = 50.000000 
// Address config = No address check 
// Packet length = 255 
// Symbol rate = 1.2 
// PA ramping = true 
// Performance mode = High Performance 
// Carrier frequency = 434.000000 
// Bit rate = 1.2 
// Packet bit length = 0 
// Whitening = false 
// Manchester enable = false 
// Modulation format = 2-FSK 
// Packet length mode = Variable 
// Device address = 0 
// TX power = 15 
// Deviation = 3.997803
static const registerSetting_t preferredSettings434n[]= 
{
	{CC112X_IOCFG3, 		   0xB0},
	{CC112X_IOCFG2, 		   0x06},
	{CC112X_IOCFG1, 		   0xB0},
	{CC112X_IOCFG0, 		   0x40},

	{CC112X_FS_DIG1,		   0x00},
	{CC112X_FS_DIG0,		   0x5F},
	{CC112X_FS_CAL0,		   0x0E},
	{CC112X_FS_DIVTWO,		   0x03},
	{CC112X_FS_DSM0,		   0x33},
	{CC112X_FS_DVC0,		   0x17},
	{CC112X_FS_PFD, 		   0x50},
	{CC112X_FS_PRE, 		   0x6E},
	{CC112X_FS_REG_DIV_CML,    0x14},
	{CC112X_FS_SPARE,		   0xAC},
	{CC112X_XOSC5,			   0x0E},
	{CC112X_XOSC4,			   0xA0},
	{CC112X_XOSC3,			   0xC7},
	{CC112X_XOSC1,			   0x03},
	{CC112X_ANALOG_SPARE,	   0x00},
	{CC112X_FIFO_CFG,		   0x80}, // D: auto flushes bit turned on
	{CC112X_DEV_ADDR,		   0x00},
	{CC112X_SETTLING_CFG,	   0x03},
	{CC112X_FS_CFG, 		   0x14}, // M:12 D: 12 may be error, changed to 14
	{CC112X_PKT_CFG2,		   0x00}, // always give CCA indication
	{CC112X_PKT_CFG1,		   0x05},
	{CC112X_PKT_CFG0,		   0x20},
	{CC112X_PKT_LEN,		   0xFF},
	{CC112X_RFEND_CFG1,		   0x0E}, // D: omit PQT and CS
	{CC112X_RFEND_CFG0,		   0x20}, // D: TXOFF return to TX
	{CC112X_FREQ2,			   0x6B}, // M: 6C
	{CC112X_FREQ1,			   0x00}, // M: 80	New 428
	{CC112X_FREQ0,			   0x00},
	{CC112X_SYNC3,			   0x93},
	{CC112X_SYNC2,	  		   0x0B},
	{CC112X_SYNC1,			   0x51},
	{CC112X_SYNC0,	   		   0xDE},
	{CC112X_SYNC_CFG1,		   0x0B},
	{CC112X_SYNC_CFG0,		   0x17},
	{CC112X_DEVIATION_M,	   0x06}, // M: 06 D: deviation = 3.997803
	{CC112X_MODCFG_DEV_E,	   0x03}, // M: 03	New deviation = 20.019531 48 05
	{CC112X_DCFILT_CFG, 	   0x1C},
	{CC112X_PREAMBLE_CFG1, 	   0x14}, // D: minimum 4 bytes preamble bits
	{CC112X_PREAMBLE_CFG0,	   0x2A},
	{CC112X_FREQ_IF_CFG,	   0x40},
	{CC112X_IQIC,			   0xC6}, // D: enable IQ
	{CC112X_CHAN_BW,		   0x08}, // D: 1120 25Hz	
	{CC112X_MDMCFG1,		   0x46},
	{CC112X_MDMCFG0,		   0x05},
	{CC112X_SYMBOL_RATE2,	   0x43},
	{CC112X_SYMBOL_RATE1,	   0xA9},
	{CC112X_SYMBOL_RATE0,	   0x2A},
	{CC112X_AGC_REF,		   0x20},
	{CC112X_AGC_CS_THR, 	   0x19},
	{CC112X_AGC_GAIN_ADJUST,   0x00},
	{CC112X_AGC_CFG3,		   0x91},
	{CC112X_AGC_CFG2,		   0x20},
	{CC112X_AGC_CFG1,		   0xA9},
	{CC112X_AGC_CFG0,		   0xCF},
	{CC112X_PA_CFG2, 	       0x7F},
	{CC112X_PA_CFG1,		   0x56},
	{CC112X_PA_CFG0,		   0x7C},
	{CC112X_IF_MIX_CFG, 	   0x00},
	{CC112X_FREQOFF_CFG,	   0x22},
	{CC112X_TOC_CFG, 	       0x0B},
	{CC112X_CFM_DATA_CFG, 	   0x00},
};


// RX filter BW = 50.000000 
// Address config = No address check 
// Packet length = 255 
// Symbol rate = 1.2 
// PA ramping = true 
// Performance mode = High Performance 
// Carrier frequency = 434.000000 
// Bit rate = 1.2 
// Packet bit length = 0 
// Whitening = false 
// Manchester enable = false 
// Modulation format = 2-FSK 
// Packet length mode = Variable 
// Device address = 0 
// TX power = 15 
// Deviation = 20.019531 
static const registerSetting_t preferredSettings434[]= 
{
	{CC112X_IOCFG3, 		   0xB0},
	{CC112X_IOCFG2, 		   0x06},
	{CC112X_IOCFG1, 		   0xB0},
	{CC112X_IOCFG0, 		   0x40},
	{CC112X_SYNC_CFG1,		   0x0B},
//	{CC112X_DEVIATION_M,	   0x48},
//	{CC112X_MODCFG_DEV_E,	   0x05},
	{CC112X_DCFILT_CFG, 	   0x1C},
	{CC112X_IQIC,			   0x46},
	{CC112X_CHAN_BW,		   0x04},
	{CC112X_MDMCFG0,		   0x05},
	{CC112X_AGC_REF,		   0x20},
	{CC112X_AGC_CS_THR, 	   0x19},
	//add agc mode
	{CC112X_AGC_CFG2,		   0x00},

	{CC112X_AGC_CFG1,		   0xA9},
	{CC112X_AGC_CFG0,		   0xCF},
	{CC112X_FIFO_CFG,		   0x00},
	{CC112X_SETTLING_CFG,	   0x03},
	{CC112X_FS_CFG, 		   0x14},
	{CC112X_PKT_CFG0,		   0x20},
	{CC112X_PKT_LEN,		   0xFF},
	{CC112X_IF_MIX_CFG, 	   0x00},
	{CC112X_FREQOFF_CFG,	   0x22},
	{CC112X_FREQ2,			   0x6C},
	{CC112X_FREQ1,			   0x80},
	{CC112X_FS_DIG1,		   0x00},
	{CC112X_FS_DIG0,		   0x5F},
	{CC112X_FS_CAL1,		   0x40},
	{CC112X_FS_CAL0,		   0x0E},
	{CC112X_FS_DIVTWO,		   0x03},
	{CC112X_FS_DSM0,		   0x33},
	{CC112X_FS_DVC0,		   0x17},
	{CC112X_FS_PFD, 		   0x50},
	{CC112X_FS_PRE, 		   0x6E},
	{CC112X_FS_REG_DIV_CML,    0x14},
	{CC112X_FS_SPARE,		   0xAC},
	{CC112X_FS_VCO0,		   0xB4},
	{CC112X_XOSC5,			   0x0E},
	{CC112X_XOSC1,			   0x03},
};

// RX filter BW = 41.666667 
// Address config = No address check 
// Packet length = 255 
// Symbol rate = 1.2 
// PA ramping = true 
// Performance mode = High Performance 
// Carrier frequency = 868.000000 
// Bit rate = 1.2 
// Packet bit length = 0 
// Whitening = false 
// Manchester enable = false 
// Modulation format = 2-FSK 
// Packet length mode = Variable 
// Device address = 0 
// TX power = 15 
// Deviation = 10.009766
static const registerSetting_t preferredSettings868[]= 
{
	{CC112X_IOCFG3, 		   0xB0},
	{CC112X_IOCFG2, 		   0x06},
	{CC112X_IOCFG1, 		   0xB0},
	{CC112X_IOCFG0, 		   0x40},
	{CC112X_SYNC_CFG1,		   0x0B},
	{CC112X_DEVIATION_M,	   0x48},
	{CC112X_MODCFG_DEV_E,	   0x04},
	{CC112X_DCFILT_CFG, 	   0x1C},
	{CC112X_IQIC,			   0xC6},
	{CC112X_CHAN_BW,		   0x43},
	{CC112X_MDMCFG0,		   0x05},
	{CC112X_AGC_REF,		   0x20},
	{CC112X_AGC_CS_THR ,	   0x19},
	{CC112X_AGC_CFG1 ,		   0xA9},
	{CC112X_AGC_CFG0 ,		   0xCF},
	{CC112X_FIFO_CFG,		   0x00},
	{CC112X_SETTLING_CFG ,	   0x03},
	{CC112X_FS_CFG, 		   0x12},
	{CC112X_PKT_CFG0   ,	   0x20},
	{CC112X_PKT_LEN  ,		   0xFF},
	{CC112X_IF_MIX_CFG ,	   0x00},
	{CC112X_FREQOFF_CFG ,	   0x22},
	{CC112X_FREQ2,			   0x6C},
	{CC112X_FREQ1 , 		   0x80},
	{CC112X_FS_DIG1  ,		   0x00},
	{CC112X_FS_DIG0  ,		   0x5F},
	{CC112X_FS_CAL1  ,		   0x40},
	{CC112X_FS_CAL0   , 	   0x0E},
	{CC112X_FS_DIVTWO  ,	   0x03},
	{CC112X_FS_DSM0   , 	   0x33},
	{CC112X_FS_DVC0,		   0x17},
	{CC112X_FS_PFD	,		   0x50},
	{CC112X_FS_PRE	,		   0x6E},
	{CC112X_FS_REG_DIV_CML	,  0x14},
	{CC112X_FS_SPARE   ,	   0xAC},
	{CC112X_FS_VCO0    ,	   0xB4},
	{CC112X_XOSC5	   ,	   0x0E},
	{CC112X_XOSC1	   ,	   0x03},
};
// Address config = No address check 
// Packet length = 255 (variable)
// Modulation format = 2-GFSK 
// PA ramping = true 
// Packet length mode = Variable 
// Bit rate = 9.6 k
// Deviation = 1.499176 
// Packet bit length = 0 
// Performance mode = High Performance 
// Carrier frequency = 470.000000 
// RX filter BW = 41.67 k
// Manchester enable = false 
// Symbol rate = 9.6 k 
// TX power = 15 
// Device address = 0 
// Whitening = false 
static const registerSetting_t preferredSettings470[]= 
{
  {CC112X_IOCFG3,            0xB0},	// not use GPIO3, 3-state
  {CC112X_IOCFG2,            0x06}, // GPIO2 - PKT_SYNC_RXTX
  {CC112X_IOCFG1,            0xB0}, // not use GPIO1, 3-state
  {CC112X_IOCFG0,            0x40}, // GPIO1 (invert output) -  RXFIFO-THR
  {CC112X_SYNC_CFG1,         0x08}, // sync threshold

  {CC112X_DCFILT_CFG,        0x1C}, // 256 sample
  {CC112X_IQIC,              0xC6}, //
  {CC112X_MDMCFG0,			 0x04},

  {CC112X_AGC_REF,           0x20}, // AGC ref = 0x20
  {CC112X_AGC_CS_THR,        0x19}, //
  {CC112X_AGC_CFG1,          0xA9}, // freeze AGC gain and RSSI, 32 samples
  {CC112X_AGC_CFG0,          0xCF}, // AGC 10dB...
  {CC112X_FIFO_CFG,          0x00}, // disable automatic flushes
  {CC112X_SETTLING_CFG,      0x03}, // manual calibrate
  {CC112X_FS_CFG,            0x14}, // enable out of lock detector, LO divider = 8
  {CC112X_PKT_CFG0,          0x20}, // variable packet length mode, length after sync
  {CC112X_PA_CFG0,           0x7E}, // output power=??14dbm , UNSAMPLER P=64
  {CC112X_PKT_LEN,           0xFF}, // packet length = 255
  {CC112X_IF_MIX_CFG,        0x00}, //
  {CC112X_FREQOFF_CFG,       0x30}, // enable frequency offset correction, loop gain factor 1/256
  {CC112X_FREQ2,             0x75}, // FREQ = 758000H f(RF) = 470 MHz
  {CC112X_FREQ1,             0x80}, // 758H/4=470 MHz
  {CC112X_FS_DIG1,           0x00}, // not use
  {CC112X_FS_DIG0,           0x5F}, // 
  {CC112X_FS_CAL1,           0x40}, // 
  {CC112X_FS_CAL0,           0x0E}, // 
  {CC112X_FS_DIVTWO,         0x03},
  {CC112X_FS_DSM0,           0x33},
  {CC112X_FS_DVC0,           0x17},
  {CC112X_FS_PFD,            0x50},
  {CC112X_FS_PRE,            0x6E},
  {CC112X_FS_REG_DIV_CML,    0x14},
  {CC112X_FS_SPARE,          0xAC},
  {CC112X_FS_VCO0,           0xB4}, //
  {CC112X_LNA,               0x03}, // not use
  {CC112X_XOSC5,             0x0E}, //
  {CC112X_XOSC1,             0x03}, // low phase noise
};


static const registerSetting_t br_1200_cfg[]=
{
	{CC112X_DEVIATION_M,	   0x3B}, // frequency deviation 1.2 kHz
	{CC112X_MODCFG_DEV_E,	   0x09}, // normal mode, 2-GFSk, DEV_E=1
	{CC112X_CHAN_BW,		   0x43}, // enable channel filter, decimation factor 32, RX filter BW=1000k/8/3=41.67k
	{CC112X_SYMBOL_RATE2,	   0x43}, // data rate/symbol rate=1(2-GFSK), SRATE_E=4, SRATE_M=3A92A	BR=1200
};

static const registerSetting_t br_4800_cfg[]=
{
	{CC112X_DEVIATION_M,	 0x3B}, // frequency deviation 4.806kHz
	{CC112X_MODCFG_DEV_E,	 0x0B}, // normal mode, 2-GFSk, DEV_E=3
	{CC112X_CHAN_BW,         0x42}, // enable channel filter, decimation factor 32, RX filter BW=1000k/8/2=62.5k
	{CC112X_SYMBOL_RATE2,    0x63}, // data rate/symbol rate=1(2-GFSK), SRATE_E=6, SRATE_M=3A92A  BR=4800
};

static const registerSetting_t br_9600_cfg[]=
{
	{CC112X_DEVIATION_M,	 0x3B}, // frequency deviation 9.612 kHz
	{CC112X_MODCFG_DEV_E,	 0x0C}, // normal mode, 2-GFSk, DEV_E=4
	{CC112X_CHAN_BW,         0x42}, // enable channel filter, decimation factor
	{CC112X_SYMBOL_RATE2,    0x73}, // data rate/symbol rate=1(2-GFSK), SRATE_E=7, SRATE_M=3A92A  BR=9600
};
#endif
/******************************************************************************
 * PROTPTYPES
 */ 
void initRFcfg(void);
void RFentersleep(void);
void RFmodechange(void);


void txdata(uint8 *txbuf, uint8 len);
void rxdata(void);

void tx_test(void);
void rx_test(void);

/* basic set of access functions */
rfStatus_t cc112xSpiReadReg(uint16 addr, uint8 *data, uint8 len);
rfStatus_t cc112xSpiWriteReg(uint16 addr, uint8 *data, uint8 len);

rfStatus_t cc112xSpiWriteTxFifo(uint8 *pWriteData, uint8 len);
rfStatus_t cc112xSpiReadRxFifo(uint8 *pReadData, uint8 len);

rfStatus_t cc112xGetTxStatus(void);
rfStatus_t cc112xGetRxStatus(void);

#ifdef  __cplusplus
}
#endif

#endif
