/******************************************************************************
 * @file     usbd_audio.h
 * @brief    NuMicro series USB driver header file
 * @version  1.0.0
 * @date     22, December, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_UAC_H__
#define __USBD_UAC_H__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0x1284

#define AUDIO_RATE      PLAY_RATE
#define AUDIO_CHANNELS  PLAY_CHANNELS
#define AUDIO_BIT_RATE  PLAY_BIT_RATE

#define UAC_MICROPHONE  0
#define UAC_SPEAKER     1

/*!<Define Audio information */
#define PLAY_RATE       16000   /* The audo play sampling rate. The setting is 16KHz */
#define PLAY_CHANNELS   2
#define PLAY_BIT_RATE   0x10    /* 16-bit data rate */

#define REC_RATE        16000   /* The record sampling rate. It is 16KHz */
#define REC_CHANNELS    1
#define REC_BIT_RATE    0x10    /* 16-bit data rate */


#define REC_FEATURE_UNITID      0x05
#define PLAY_FEATURE_UNITID     0x06

#define BUFF_LEN    32*12
#define REC_LEN     REC_RATE / 1000

/* Define Descriptor information */
#if(PLAY_CHANNELS == 1)
#define PLAY_CH_CFG     1
#endif
#if(PLAY_CHANNELS == 2)
#define PLAY_CH_CFG     3
#endif

#if(REC_CHANNELS == 1)
#define REC_CH_CFG     1
#endif
#if(REC_CHANNELS == 2)
#define REC_CH_CFG     3
#endif

#define PLAY_RATE_LO    (PLAY_RATE & 0xFF)
#define PLAY_RATE_MD    ((PLAY_RATE >> 8) & 0xFF)
#define PLAY_RATE_HI    ((PLAY_RATE >> 16) & 0xFF)

#define REC_RATE_LO     (REC_RATE & 0xFF)
#define REC_RATE_MD     ((REC_RATE >> 8) & 0xFF)
#define REC_RATE_HI     ((REC_RATE >> 16) & 0xFF)

/********************************************/
/* Audio Class Current State                */
/********************************************/
/*!<Define Audio Class Current State */
#define UAC_STOP_AUDIO_RECORD           0
#define UAC_START_AUDIO_RECORD          1
#define UAC_PROCESSING_AUDIO_RECORD     2
#define UAC_BUSY_AUDIO_RECORD           3

/***************************************************/
/*      Audio Class-Specific Request Codes         */
/***************************************************/
/*!<Define Audio Class Specific Request */
#define UAC_REQUEST_CODE_UNDEFINED  0x00
#define UAC_SET_CUR                 0x01
#define UAC_GET_CUR                 0x81
#define UAC_SET_MIN                 0x02
#define UAC_GET_MIN                 0x82
#define UAC_SET_MAX                 0x03
#define UAC_GET_MAX                 0x83
#define UAC_SET_RES                 0x04
#define UAC_GET_RES                 0x84
#define UAC_SET_MEM                 0x05
#define UAC_GET_MEM                 0x85
#define UAC_GET_STAT                0xFF

#define MUTE_CONTROL                0x01
#define VOLUME_CONTROL              0x02

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    64
#define EP1_MAX_PKT_SIZE    EP0_MAX_PKT_SIZE
#define EP2_MAX_PKT_SIZE    REC_RATE*REC_CHANNELS*2/1000
#define EP3_MAX_PKT_SIZE    PLAY_RATE*PLAY_CHANNELS*2/1000

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         EP3_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define ISO_IN_EP_NUM    0x01
#define ISO_OUT_EP_NUM   0x02

/*-------------------------------------------------------------*/
extern uint32_t g_usbd_UsbAudioState;

void UAC_DeviceEnable(uint8_t u8Object);
void UAC_DeviceDisable(uint8_t u8Object);
void UAC_SendRecData(void);
void UAC_GetPlayData(int16_t *pi16src, int16_t i16Samples);


/*-------------------------------------------------------------*/
void UAC_Init(void);
void UAC_ClassRequest(void);
void UAC_SetInterface(uint32_t u32AltInterface);

void EP2_Handler(void);
void EP3_Handler(void);

void WAU8822_Setup(void);
void timer_init(void);

#endif  /* __USBD_UAC_H_ */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
