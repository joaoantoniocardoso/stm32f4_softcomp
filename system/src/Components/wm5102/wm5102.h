/**
  ******************************************************************************
  * @file    wm5102_.h
  * @author  IFSC
  * @version V1.0.0
  * @date    01-April-2015
  * @brief   This file contains all the functions prototypes for the
  *          wm5102_.c driver.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WM5102_H
#define __WM5102_H

/* Includes ------------------------------------------------------------------*/
#include "../Common/audio.h"
#include "stm32f4xx_hal.h"
#include "wm5102_register_map.h"

/** @addtogroup WM5102
  * @{
  */

/** @defgroup WM5102_Exported_Types
  * @{
  *
  */
typedef enum {
	WM5102_FILTER_DISABLED = 0,
	WM5102_FILTER_LOW_PASS,
	WM5102_FILTER_HIGH_PASS,
} WM5102_FilterConfigModesDef;

typedef enum {
	WM5102_BLOCK_NONE = 0,
	WM5102_BLOCK_FILTER,
	WM5102_BLOCK_EQUALIZER,
} WM5102_BlockTypeDef;

typedef enum {
	WM5102_EQUALIZER_DISABLED = 0,
	WM5102_EQUALIZER_SHELVING_FILTER,
	WM5102_EQUALIZER_PEAK_FILTER,
} WM5102_EqualizerConfigModesDef;


/**
  * @}
  */

/** @defgroup WM5102_Exported_Constants
  * @{
  */

/******************************************************************************/
/***************************  Codec User defines ******************************/
/******************************************************************************/
#define MODE_MASTER		  0
#define MODE_SLAVE		  1

#define AUDIO_CONTROL_INTERFACE		  2
#define AUDIO_DATA_INTERFACE		  1
#define AUDIO_DATA_MODE				  MODE_SLAVE

/* Codec output DEVICE */
#define OUTPUT_DEVICE_LINE_OUT                ((uint8_t)0x01)
#define OUTPUT_DEVICE_HEADPHONE               ((uint8_t)0x02)
#define OUTPUT_DEVICE_BOTH                    ((uint8_t)0x03)
#define OUTPUT_DEVICE_AUTO                    ((uint8_t)0x04)
#define INPUT_DEVICE_DIGITAL_MICROPHONE       ((uint8_t)0x01)
#define INPUT_DEVICE_MICROPHONE			      ((uint8_t)0x02)
#define INPUT_DEVICE_LINE_IN               	  ((uint8_t)0x03)

/* Volume Levels values */
#define DEFAULT_VOLMIN                0x00
#define DEFAULT_VOLMAX                0xFF
#define DEFAULT_VOLSTEP               0x01

#define AUDIO_PAUSE                   0
#define AUDIO_RESUME                  1

/* Codec POWER DOWN modes */
#define CODEC_PDWN_HW                 1
#define CODEC_PDWN_SW                 2

/* MUTE commands */
#define AUDIO_MUTE_ON                 1
#define AUDIO_MUTE_OFF                0

/* AUDIO FREQUENCY */
#define AUDIO_FREQUENCY_192K          ((uint32_t)192000)
#define AUDIO_FREQUENCY_96K           ((uint32_t)96000)
#define AUDIO_FREQUENCY_48K           ((uint32_t)48000)
#define AUDIO_FREQUENCY_44K           ((uint32_t)44100)
#define AUDIO_FREQUENCY_32K           ((uint32_t)32000)
#define AUDIO_FREQUENCY_22K           ((uint32_t)22050)
#define AUDIO_FREQUENCY_16K           ((uint32_t)16000)
#define AUDIO_FREQUENCY_11K           ((uint32_t)11025)
#define AUDIO_FREQUENCY_8K            ((uint32_t)8000)

#define VOLUME_CONVERT(Volume)    (((Volume) > 100)? 100:((uint8_t)(((Volume) * 191) / 100)))

/******************************************************************************/
/****************************** REGISTER MAPPING ******************************/
/******************************************************************************/
/**
  * @brief  WM5102 ID
  */
#define  WM5102_ID    0x5102

/**
  * @brief Device ID Register: Reading from this register will indicate device
  *                            family ID 5102h
  */
#define WM5102_CHIPID_ADDR                  0x00

#define WM5102_EQUALIZER_GAIN_MAX(g)		((g>12)?12:((g<-12)?-12:g))
#define WM5102_EQUALIZER_GAIN_CONVERT(g)	(g+12)

/**
  * @}
  */

/** @defgroup WM5102_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup WM5102_Exported_Functions
  * @{
  */

/*------------------------------------------------------------------------------
                           Audio Codec functions
------------------------------------------------------------------------------*/
/* High Layer codec functions */
uint32_t wm5102_Init(uint16_t DeviceAddr, uint16_t OutputInputDevice, uint8_t Volume, uint32_t AudioFreq);
void wm5102_DeInit(void);
uint32_t wm5102_ReadID(uint16_t DeviceAddrq);
uint32_t wm5102_Play(uint16_t DeviceAddr, uint16_t* pBuffer, uint16_t Size);
uint32_t wm5102_Pause(uint16_t DeviceAddr);
uint32_t wm5102_Resume(uint16_t DeviceAddr);
uint32_t wm5102_Stop(uint16_t DeviceAddr, uint32_t Cmd);
uint32_t wm5102_SetVolume(uint16_t DeviceAddr, uint8_t Volume);
uint32_t wm5102_SetMute(uint16_t DeviceAddr, uint32_t Cmd);
void wm5102_SetFilter(uint16_t DeviceAddr, WM5102_FilterConfigModesDef ModeL, WM5102_FilterConfigModesDef ModeR, uint16_t CoeffL, uint16_t CoeffR);
void wm5102_SetEqualizer(uint16_t DeviceAddr, WM5102_EqualizerConfigModesDef ModeL, WM5102_EqualizerConfigModesDef ModeR,
		int8_t *GainL, int8_t * GainR, uint16_t * CoeffL, uint16_t * CoeffR);
void wm5102_SetMixerFilter(uint8_t ModeL, uint8_t ModeR, uint16_t CoeffL, uint16_t CoeffR);
uint32_t wm5102_SetInputMode(uint16_t DeviceAddr, uint8_t Output);
uint32_t wm5102_SetOutputMode(uint16_t DeviceAddr, uint8_t Output);
uint32_t wm5102_SetFrequency(uint16_t DeviceAddr, uint32_t AudioFreq);
uint32_t wm5102_Reset(uint16_t DeviceAddr);

/* EXTERNAL AUDIO IO functions */
void    EXTERNAL_AUDIO_IO_Init(void);
void    EXTERNAL_AUDIO_IO_DeInit(void);
void 	EXTERNAL_AUDIO_IO_Write (uint8_t Addr, uint32_t Reg, uint16_t Value);
uint16_t EXTERNAL_AUDIO_IO_Read(uint8_t Addr, uint32_t Reg);
void    EXTERNAL_AUDIO_IO_Delay(uint32_t Delay);

/* Audio driver structure */
extern AUDIO_DrvTypeDef   wm5102_drv;

#endif /* __WM5102_H */

/**
  * @}
  */

/**
  * @}
  */
