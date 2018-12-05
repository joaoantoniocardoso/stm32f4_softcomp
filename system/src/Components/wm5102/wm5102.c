/**
  ******************************************************************************
  * @file    wm5102.h
  * @author  IFSC
  * @version V1.0.0
  * @date    01-April-2015
  * @brief   This file contains all the functions prototypes for the
  *          wm5102.c driver.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "wm5102.h"

/** @addtogroup wm5102
  * @brief     This file provides a set of functions needed to drive the
  *            WM5102 audio codec.
  * @{
  */

/** @defgroup WM5102_Private_Types
  * @{
  */

typedef struct {
	uint32_t fMin;
	uint32_t fMax;
	uint8_t FRatio;
	uint8_t Gain;
} WM5102_FLLFRatioGainDef;

typedef struct {
	uint32_t AudioFreq;
	uint8_t SampleRate1;
	uint8_t BClkRate;
} WM5102_SampleRateConfigDef;

typedef struct {
	WM5102_FilterConfigModesDef ModeL;
	WM5102_FilterConfigModesDef ModeR;
	uint16_t CoeffL;
	uint16_t CoeffR;
} WM5102_MixerFilterConfigDef;

typedef struct {
	WM5102_EqualizerConfigModesDef ModeL;
	WM5102_EqualizerConfigModesDef ModeR;
	int8_t GainL[5];
	int8_t GainR[5];
	uint16_t CoeffL[19];
	uint16_t CoeffR[19];
} WM5102_EqualizerConfigDef;

/**
  * @}
  */

/** @defgroup WM5102_Private_Defines
  * @{
  */
/* Uncomment this line to enable verifying data sent to codec after each write
   operation (for debug purpose) */
#if !defined (VERIFY_WRITTENDATA)
/* #define VERIFY_WRITTENDATA */
#endif /* VERIFY_WRITTENDATA */
/**
  * @}
  */

/** @defgroup WM5102_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup WM5102_Private_Variables
  * @{
  */

/* Audio codec driver structure initialization */
AUDIO_DrvTypeDef wm5102_drv =
{
	wm5102_Init,
	wm5102_DeInit,
	wm5102_ReadID,
	wm5102_Play,
	wm5102_Pause,
	wm5102_Resume,
	wm5102_Stop,
	wm5102_SetFrequency,
	wm5102_SetVolume,
	wm5102_SetMute,
	wm5102_SetInputMode,
	wm5102_SetOutputMode,
	wm5102_Reset
};

const WM5102_FLLFRatioGainDef  FLLFRatioGain[] = {
		{1000000, 13500000, 1, 16},
		{256000, 1000000, 2, 4},
		{128000, 256000, 4, 1},
		{64000, 128000, 8, 1},
		{0, 64000, 16, 1},
};

const WM5102_SampleRateConfigDef SampleRateConfig[] = {
	{AUDIO_FREQUENCY_8K, 0x11, 0x06},
	{AUDIO_FREQUENCY_16K, 0x12, 0x08},
	{AUDIO_FREQUENCY_32K, 0x13, 0x0A},
	{AUDIO_FREQUENCY_48K, 0x03, 0x0B},
	{AUDIO_FREQUENCY_96K, 0x04, 0x0D},
	{AUDIO_FREQUENCY_192K, 0x05, 0x0F},
	{AUDIO_FREQUENCY_11K, 0x09, 0x07},
	{AUDIO_FREQUENCY_22K, 0x0A, 0x11},
	{AUDIO_FREQUENCY_44K, 0x0B, 0x0B},
	{0, 0, 0},
};

/**
  * @}
  */

/** @defgroup WM5102_Function_Prototypes
  * @{
  */
static uint8_t CODEC_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
static uint8_t CODEC_IO_Read(uint8_t Addr, uint16_t Reg);
/**
  * @}
  */

/** @defgroup WM5102_Private_Functions
  * @{
  */

static uint32_t wm5102_Log2PowerOfTwo(uint32_t powerOfTwo)
{
	uint32_t result = (powerOfTwo & 0xAAAAAAAA) != 0;
	result |= ((powerOfTwo & 0xFFFF0000) != 0) << 4;
	result |= ((powerOfTwo & 0xFF00FF00) != 0) << 3;
	result |= ((powerOfTwo & 0xF0F0F0F0) != 0) << 2;
	result |= ((powerOfTwo & 0xCCCCCCCC) != 0) << 1;
	return result;
}

static uint32_t wm5102_ApplyWolfsonPatch(uint16_t DeviceAddr)
{
	uint32_t counter = 0;

	EXTERNAL_AUDIO_IO_Delay(5);

	counter += CODEC_IO_Write(DeviceAddr, 0x0019, 0x0001);   // patch codec (supplied by Wolfson)
	counter += CODEC_IO_Write(DeviceAddr, 0x0080, 0x0003);
	counter += CODEC_IO_Write(DeviceAddr, 0x0081, 0xE022);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUTPUT_PATH_CONFIG_1L, 0x6080);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUTPUT_PATH_CONFIG_2L, 0xa080);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUTPUT_PATH_CONFIG_3L, 0xa080);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUTPUT_PATH_CONFIG_4L, 0xE000);
	counter += CODEC_IO_Write(DeviceAddr, 0x0443, 0xDC1A);
	counter += CODEC_IO_Write(DeviceAddr, 0x04B0, 0x0066);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_NOISE_GATE_CONTROL, 0x000B);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_LDO1_CONTROL_2, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL1_CONTROL_1, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, 0x035E, 0x000C);
	counter += CODEC_IO_Write(DeviceAddr, 0x02D4, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, 0x0080, 0x0000);  // end of patch

	EXTERNAL_AUDIO_IO_Delay(5);

	return counter;
}

static uint32_t wm5102_ConfigureI2S(uint16_t DeviceAddr)
{
	uint32_t counter = 0;
	uint16_t AIFFormat = 0x0002;		//I2S format
#if (AUDIO_DATA_MODE == MODE_MASTER)
	uint16_t RxPinCtrl = 0x0005;
#else
	uint16_t RxPinCtrl = 0x0004;
#endif
	uint16_t FrameCtrl1 = 0x1010;		//TX WL and SLOT_LEN both 16-bit
	uint16_t FrameCtrl2 = 0x1010;		//RX WL and SLOT_LEN both 16-bit
	uint16_t RxEnables = 0x0003;		//enable AIF1 RX channels (L and R)
	uint16_t TxEnables = 0x0003;		//enable AIF1 TX channels (L and R)
	uint16_t RxBCLKRate = 0x0020;		//32 BCLK cycles per LRC RX frame
	uint16_t TxBCLKRate = 0x0020;		//32 BCLK cycles per LRC TX frame


#if (AUDIO_DATA_INTERFACE == 1)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_FORMAT, AIFFormat);   	  // AIF format
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_RX_PIN_CTRL, RxPinCtrl);   // AIF LRCLK - In master mode this takes LRC high, reliably I hope...
#elif (AUDIO_DATA_INTERFACE == 2)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_FORMAT, AIFFormat);   	  // AIF I2S format
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_RX_PIN_CTRL, RxPinCtrl);   // AIF LRCLK - In master mode this takes LRC high, reliably I hope...
#else
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_FORMAT, AIFFormat);   	  // AIF I2S format
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_RX_PIN_CTRL, RxPinCtrl);   // AIF LRCLK - In master mode this takes LRC high, reliably I hope...
#endif

	EXTERNAL_AUDIO_IO_Delay(1);


#if (AUDIO_DATA_INTERFACE == 1)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_RX_ENABLES, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_TX_ENABLES, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_FRAME_CTRL_1, FrameCtrl1);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_FRAME_CTRL_2, FrameCtrl2);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_RX_BCLK_RATE, RxBCLKRate);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_TX_BCLK_RATE, TxBCLKRate);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_RX_ENABLES, RxEnables);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_TX_ENABLES, TxEnables);
#elif (AUDIO_DATA_INTERFACE == 2)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_RX_ENABLES, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_TX_ENABLES, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_FRAME_CTRL_1, FrameCtrl1);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_FRAME_CTRL_2, FrameCtrl2);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_RX_BCLK_RATE, RxBCLKRate);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_TX_BCLK_RATE, TxBCLKRate);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_RX_ENABLES, RxEnables);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_TX_ENABLES, TxEnables);
#else
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_RX_ENABLES, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_TX_ENABLES, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_FRAME_CTRL_1, FrameCtrl1);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_FRAME_CTRL_2, FrameCtrl2);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_RX_BCLK_RATE, RxBCLKRate);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_TX_BCLK_RATE, TxBCLKRate);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_RX_ENABLES, RxEnables);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_TX_ENABLES, TxEnables);
#endif

	return counter;
}

static uint32_t wm5102_ConfigureSampleRate(uint16_t DeviceAddr, uint32_t AudioFreq)
{
	uint32_t counter = 0;
	uint8_t i;

	//Sample rate configuration
	for(i = 0; SampleRateConfig[i].AudioFreq; i++)
	{
		if(SampleRateConfig[i].AudioFreq == AudioFreq)
		{
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_SAMPLE_RATE_1, SampleRateConfig[i].SampleRate1);

#if (AUDIO_DATA_MODE == MODE_MASTER)
	#if (AUDIO_DATA_INTERFACE == 1)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_BCLK_CTRL, 0x20 | SampleRateConfig[i].BClkRate);
	#elif (AUDIO_DATA_INTERFACE == 2)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_BCLK_CTRL, 0x20 | SampleRateConfig[i].BClkRate);
	#else
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_BCLK_CTRL, 0x20 | SampleRateConfig[i].BClkRate);
	#endif
#else
	#if (AUDIO_DATA_INTERFACE == 1)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1_BCLK_CTRL, SampleRateConfig[i].BClkRate);
	#elif (AUDIO_DATA_INTERFACE == 2)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2_BCLK_CTRL, SampleRateConfig[i].BClkRate);
	#else
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3_BCLK_CTRL, SampleRateConfig[i].BClkRate);
	#endif
#endif
			break;
		}
	}

	return counter;
}

static uint32_t wm5102_ConfigureSYSCLK(uint16_t DeviceAddr, uint32_t AudioFreq)
{
	uint32_t counter = 0;

	uint32_t fRef;
	uint32_t fVCO;
	uint32_t fOut;

	uint8_t fllRefClkDiv = 1;
	uint16_t fllN = 96;
	uint16_t fllTheta = 0;
	uint16_t fllLambda = 1;
	uint8_t fllFRatio = 1;
	uint8_t fllGain = 16;
	uint8_t fllOutDiv = 4; //For 24.576Mhz and 22.5792Mhz SYSCLK, 90Mhz <= fVCO <= 104 Mhz
	uint8_t fllRefClkSrc = 0;

	uint8_t i;


	//Clock frequency related to sample rate
	switch(AudioFreq)
	{
		case AUDIO_FREQUENCY_8K:
		case AUDIO_FREQUENCY_16K:
		case AUDIO_FREQUENCY_32K:
		case AUDIO_FREQUENCY_48K:
		case AUDIO_FREQUENCY_96K:
		case AUDIO_FREQUENCY_192K:
			fOut = 24576000;
			break;
		case AUDIO_FREQUENCY_11K:
		case AUDIO_FREQUENCY_22K:
		case AUDIO_FREQUENCY_44K:
			fOut = 22579200;
			break;
		default:
			fOut = 24576000;
	}


#if (AUDIO_DATA_MODE == MODE_MASTER)
	//When use 12Mhz MCLK
	fRef = 12000000 / fllRefClkDiv; // MCLK1
	fllRefClkSrc = 0;

	if(fOut == 24576000)
	{
		fllTheta = 24;
		fllLambda = 125;
	}else{
		fllTheta = 329;
		fllLambda = 625;
	}
#else
	//When use I2S clock
	fRef = (AudioFreq * 32) / fllRefClkDiv;


#if (AUDIO_DATA_INTERFACE == 1)
	fllRefClkSrc = 8;
#elif (AUDIO_DATA_INTERFACE == 2)
	fllRefClkSrc = 9;
#else
	fllRefClkSrc = 10;
#endif

	//N.K is integer
	fllTheta = 0;
	fllLambda = 1;
#endif

	for(i = 0; FLLFRatioGain[i].fMin; i++)
	{
		if((fRef>=FLLFRatioGain[i].fMin) && (fRef<FLLFRatioGain[i].fMax))
		{
			fllFRatio = FLLFRatioGain[i].FRatio;
			fllGain = FLLFRatioGain[i].Gain;
			break;
		}
	}

	fVCO = fOut*fllOutDiv;
	fllN = fVCO/fRef;

	//Configure FLL
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_CONTROL_1, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_CONTROL_2, 0x8000 | fllN);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_CONTROL_3, fllTheta);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_CONTROL_4, fllLambda);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_CONTROL_5, (wm5102_Log2PowerOfTwo(fllFRatio) << 8) | (fllOutDiv << 1));
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_CONTROL_6, (wm5102_Log2PowerOfTwo(fllFRatio) << 6) | fllRefClkSrc);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_CONTROL_7, (wm5102_Log2PowerOfTwo(fllGain) << 2));
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_CONTROL_1, 0x0001);


	//Configure monitor pin
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_GPIO5_CTRL, 0x0006);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_GPIO_CLOCK, 0x0000);
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_FLL2_GPIO_CLOCK, 0x0009);

	EXTERNAL_AUDIO_IO_Delay(5);

	//Enable SYSCLK
	if(fOut == 24576000)
	{
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_SYSTEM_CLOCK_1,
				(0x00 << 15)  // SYSCLK is a multiple of 6.144MHz
				| (0x02 << 8) // 24.576MHz
				| (0x01 << 6) // SYSCLK Control Enabled
				| 0x05 		  // SYSCLK Source
				);


	}else{
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_SYSTEM_CLOCK_1,
				(0x01 << 15)  // SYSCLK is a multiple of 5.6448MHz
				| (0x02 << 8) // 22.5792MHz
				| (0x01 << 6) // SYSCLK Control Enabled
				| 0x05 		  // SYSCLK Source
				);
	}
	return counter;
}

static void wm5102_GetInsertionPoint(uint16_t DeviceAddr, WM5102_BlockTypeDef Block,
		uint16_t *InputSourceL, uint16_t *InputSourceR, uint16_t *InputDestinationL, uint16_t *InputDestinationR)
{
#if (AUDIO_DATA_INTERFACE == 1)
	*InputDestinationL = WM5102_REG_AIF1TX1MIX_INPUT_1_SOURCE;
	*InputDestinationR = WM5102_REG_AIF1TX2MIX_INPUT_1_SOURCE;
#elif (AUDIO_DATA_INTERFACE == 2)
	*InputDestinationL = WM5102_REG_AIF2TX1MIX_INPUT_1_SOURCE;
	*InputDestinationR = WM5102_REG_AIF2TX2MIX_INPUT_1_SOURCE;
#else
	*InputDestinationL = WM5102_REG_AIF3TX1MIX_INPUT_1_SOURCE;
	*InputDestinationR = WM5102_REG_AIF3TX2MIX_INPUT_1_SOURCE;
#endif


	*InputSourceL = CODEC_IO_Read(DeviceAddr, *InputDestinationL);
	*InputSourceR = CODEC_IO_Read(DeviceAddr, *InputDestinationR);

	do{
		//Reverse walk through path
		if(*InputSourceL == 0x0050)
		{
			//Equalizer already enabled
			//Get input from equalizer
			*InputSourceL = CODEC_IO_Read(DeviceAddr, WM5102_REG_EQ1MIX_INPUT_1_SOURCE);

			//Change destination to equalizer if target is another block
			if(Block != WM5102_BLOCK_EQUALIZER)
				*InputDestinationL = WM5102_REG_EQ1MIX_INPUT_1_SOURCE;
		}

		if(*InputSourceR == 0x0051)
		{
			//Equalizer already enabled
			//Get input from equalizer
			*InputSourceR = CODEC_IO_Read(DeviceAddr, WM5102_REG_EQ2MIX_INPUT_1_SOURCE);

			//Change destination to equalizer if target is another block
			if(Block != WM5102_BLOCK_EQUALIZER)
				*InputDestinationR = WM5102_REG_EQ2MIX_INPUT_1_SOURCE;
		}

		if(Block == WM5102_BLOCK_EQUALIZER)
			break;

		if(*InputSourceL == 0x0060)
		{
			//Filter already enabled
			//Get input from filter
			*InputSourceL = CODEC_IO_Read(DeviceAddr, WM5102_REG_HPLP1MIX_INPUT_1_SOURCE);

			//Change destination to filter if target is another block
			if(Block != WM5102_BLOCK_FILTER)
				*InputDestinationL = WM5102_REG_HPLP1MIX_INPUT_1_SOURCE;
		}

		if(*InputSourceR == 0x0061)
		{
			//Filter already enabled
			//Get input from filter
			*InputSourceR = CODEC_IO_Read(DeviceAddr, WM5102_REG_HPLP2MIX_INPUT_1_SOURCE);

			//Change destination to filter if target is another block
			if(Block != WM5102_BLOCK_FILTER)
				*InputDestinationR = WM5102_REG_HPLP2MIX_INPUT_1_SOURCE;
		}
	}while(0);
}


/**
  * @brief Initializes the audio codec and the control interface.
  * @param DeviceAddr: Device address on communication Bus.
  * @param OutputDevice: can be OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *                       OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO .
  * @param Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param AudioFreq: Audio Frequency
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_Init(uint16_t DeviceAddr, uint16_t OutputInputDevice, uint8_t Volume, uint32_t AudioFreq)
{
	uint32_t counter = 0;
	uint8_t OutputDevice = OutputInputDevice & 0xFF;
	uint8_t InputDevice = (OutputInputDevice & 0xFF00) >> 8;

	/* Initialize the Control and Data interface of the Audio Codec */
	EXTERNAL_AUDIO_IO_Init();

	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_SOFTWARE_RESET, 0x0000);   // reset codec

	wm5102_ApplyWolfsonPatch(DeviceAddr);

	counter += wm5102_ConfigureSYSCLK(DeviceAddr, AudioFreq);

	counter += wm5102_ConfigureSampleRate(DeviceAddr, AudioFreq);

	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_NOISE_GATE_CONTROL, 0x0009);   // output noise gate enabled, threshold -84dB (important??)

	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_MIC_CHARGE_PUMP_1, 0x0001);   // not used prior to 20 March but I think necessary for CP2/LDO2 - analog inputs
																				   // Wolfson example write 0x0007 but bit 0 is CP2_ENA
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_LDO1_CONTROL_1, 0x00D5);   // LDO1 control 0x00D5 -> LDO1 enabled, normal, 1V2


	wm5102_ConfigureI2S(DeviceAddr);

	wm5102_SetOutputMode(DeviceAddr, OutputDevice);

	wm5102_SetInputMode(DeviceAddr, InputDevice);

	wm5102_SetVolume(DeviceAddr, Volume);

	/* Return communication control value */
	return counter;
}

/**
  * @brief  Deinitializes the audio codec.
  * @param  None
  * @retval  None
  */
void wm5102_DeInit(void)
{
  /* Deinitialize Audio Codec interface */
  EXTERNAL_AUDIO_IO_DeInit();
}

/**
  * @brief  Get the WM5102 ID.
  * @param DeviceAddr: Device address on communication Bus.
  * @retval The WM5102 ID
  */
uint32_t wm5102_ReadID(uint16_t DeviceAddr)
{
  /* Initialize the Control interface of the Audio Codec */
  EXTERNAL_AUDIO_IO_Init();

  return ((uint32_t)EXTERNAL_AUDIO_IO_Read(DeviceAddr, WM5102_CHIPID_ADDR));
}

/**
  * @brief Start the audio Codec play feature.
  * @note For this codec no Play options are required.
  * @param DeviceAddr: Device address on communication Bus.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_Play(uint16_t DeviceAddr, uint16_t* pBuffer, uint16_t Size)
{
	UNUSED(pBuffer);
	UNUSED(Size);
	uint32_t counter = 0;

	/* Resumes the audio file playing */
	/* Unmute the output first */
	counter += wm5102_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

	return counter;
}

/**
  * @brief Pauses playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_Pause(uint16_t DeviceAddr)
{
  uint32_t counter = 0;

  /* Pause the audio file playing */
  /* Mute the output first */
  counter += wm5102_SetMute(DeviceAddr, AUDIO_MUTE_ON);


  return counter;
}

/**
  * @brief Resumes playing on the audio codec.
  * @param DeviceAddr: Device address on communication Bus.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_Resume(uint16_t DeviceAddr)
{
  uint32_t counter = 0;

  /* Resumes the audio file playing */
  /* Unmute the output first */
  counter += wm5102_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

  return counter;
}

/**
  * @brief Stops audio Codec playing. It powers down the codec.
  * @param DeviceAddr: Device address on communication Bus.
  * @param CodecPdwnMode: selects the  power down mode.
  *          - CODEC_PDWN_SW: only mutes the audio codec. When resuming from this
  *                           mode the codec keeps the previous initialization
  *                           (no need to re-Initialize the codec registers).
  *          - CODEC_PDWN_HW: Physically power down the codec. When resuming from this
  *                           mode, the codec is set to default configuration
  *                           (user should re-Initialize the codec in order to
  *                            play again the audio stream).
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_Stop(uint16_t DeviceAddr, uint32_t CodecPdwnMode)
{
  uint32_t counter = 0;

  /* Mute the output first */
  counter += wm5102_SetMute(DeviceAddr, AUDIO_MUTE_ON);

  if (CodecPdwnMode == CODEC_PDWN_SW)
  {
     /* Only output mute required*/
  }
  else /* CODEC_PDWN_HW */
  {

  }

  return counter;
}

/**
  * @brief Sets higher or lower the codec volume level.
  * @param DeviceAddr: Device address on communication Bus.
  * @param Volume: a byte value from 0 to 255 (refer to codec registers
  *         description for more details).
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_SetVolume(uint16_t DeviceAddr, uint8_t Volume)
{
  uint32_t counter = 0;
  uint8_t convertedvol = VOLUME_CONVERT(Volume);

  if(convertedvol > 0xBF)
  {
    /* Unmute audio codec */
    counter += wm5102_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_2L, 0x00BF | 0x0200); // DAC 2 volume L 0dB (LINE OUT)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_2R, 0x00BF | 0x0200); // DAC 2 volume R 0dB (LINE OUT)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_1L, 0x00BF | 0x0200); // DAC 1 volume L 0dB (HP OUT)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_1R, 0x00BF | 0x0200); // DAC 1 volume R 0dB (HP OUT)
  }
  else if (Volume == 0)
  {
    /* Mute audio codec */
    counter += wm5102_SetMute(DeviceAddr, AUDIO_MUTE_ON);
  }
  else
  {
    /* Unmute audio codec */
    counter += wm5102_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_2L, convertedvol | 0x0200); // DAC 2 volume L 0dB (LINE OUT)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_2R, convertedvol | 0x0200); // DAC 2 volume R 0dB (LINE OUT)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_1L, convertedvol | 0x0200); // DAC 1 volume L 0dB (HP OUT)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_1R, convertedvol | 0x0200); // DAC 1 volume R 0dB (HP OUT)
  }

  return counter;
}

/**
  * @brief Enables or disables the mute feature on the audio codec.
  * @param DeviceAddr: Device address on communication Bus.
  * @param Cmd: AUDIO_MUTE_ON to enable the mute or AUDIO_MUTE_OFF to disable the
  *             mute mode.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_SetMute(uint16_t DeviceAddr, uint32_t Cmd)
{
	uint32_t counter = 0;

	/* Set the Mute mode */
	if(Cmd == AUDIO_MUTE_ON)
	{
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_2L, 0x0300); // DAC 2 volume L 0dB (LINE OUT)
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_2R, 0x0300); // DAC 2 volume R 0dB (LINE OUT)
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_1L, 0x0300); // DAC 1 volume L 0dB (HP OUT)
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_1R, 0x0300); // DAC 1 volume R 0dB (HP OUT)
	}
	else /* AUDIO_MUTE_OFF Disable the Mute */
	{
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_2L, 0x0080 | 0x0200); // DAC 2 volume L 0dB (LINE OUT)
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_2R, 0x0080 | 0x0200); // DAC 2 volume R 0dB (LINE OUT)
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_1L, 0x0080 | 0x0200); // DAC 1 volume L 0dB (HP OUT)
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_DAC_DIGITAL_VOLUME_1R, 0x0080 | 0x0200); // DAC 1 volume R 0dB (HP OUT)
	}
	return counter;
}

void wm5102_SetFilter(uint16_t DeviceAddr, WM5102_FilterConfigModesDef ModeL, WM5102_FilterConfigModesDef ModeR, uint16_t CoeffL, uint16_t CoeffR)
{
	uint32_t counter = 0;
	uint16_t InputSourceL;
	uint16_t InputSourceR;
	uint16_t InputDestinationL;
	uint16_t InputDestinationR;

	wm5102_GetInsertionPoint(DeviceAddr, WM5102_BLOCK_FILTER,
			&InputSourceL, &InputSourceR, &InputDestinationL, &InputDestinationR);

	//Reconfigure path
	if(ModeL != WM5102_FILTER_DISABLED)
	{
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_HPLPF1_1, ((ModeL == WM5102_FILTER_LOW_PASS)?0x0000:0x0002) | 0x0001); // LHPF1 HPF enabled
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_HPLPF1_2, CoeffL); // LHPF1 cutoff frequency in Hz depends on fs
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_HPLP1MIX_INPUT_1_SOURCE, InputSourceL);
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_HPLP1MIX_INPUT_1_VOLUME, 0x0080); // LHPF1 mixer source 1 gain 0dB
		InputSourceL = 0x0060;
	}

	if(ModeR != WM5102_FILTER_DISABLED)
	{
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_HPLPF2_1, ((ModeR == WM5102_FILTER_LOW_PASS)?0x0000:0x0002) | 0x0001); // LHPF2 HPF enabled
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_HPLPF2_2, CoeffR); // LHPF2 cutoff frequency in Hz depends on fs
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_HPLP2MIX_INPUT_1_SOURCE, InputSourceR);
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_HPLP2MIX_INPUT_1_VOLUME, 0x0080); // LHPF2 mixer source 1 gain 0dB
		InputSourceR = 0x0061;
	}

	counter += CODEC_IO_Write(DeviceAddr, InputDestinationL, InputSourceL);
	counter += CODEC_IO_Write(DeviceAddr, InputDestinationR, InputSourceR);


}

void wm5102_SetEqualizer(uint16_t DeviceAddr, WM5102_EqualizerConfigModesDef ModeL, WM5102_EqualizerConfigModesDef ModeR,
		int8_t *GainL, int8_t * GainR, uint16_t * CoeffL, uint16_t * CoeffR)
{
	uint32_t counter = 0;
	uint16_t InputSourceL;
	uint16_t InputSourceR;
	uint16_t InputDestinationL;
	uint16_t InputDestinationR;
	uint16_t RegOffset;

	wm5102_GetInsertionPoint(DeviceAddr, WM5102_BLOCK_EQUALIZER,
			&InputSourceL, &InputSourceR, &InputDestinationL, &InputDestinationR);


	//Reconfigure path
	if(ModeL != WM5102_EQUALIZER_DISABLED)
	{
		if(GainL)
		{
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ1_1, (WM5102_EQUALIZER_GAIN_CONVERT(GainL[0]) << 11)
																  | (WM5102_EQUALIZER_GAIN_CONVERT(GainL[1]) << 6)
																  | (WM5102_EQUALIZER_GAIN_CONVERT(GainL[2]) << 1)
																  | 0x0001);

			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ1_2, (WM5102_EQUALIZER_GAIN_CONVERT(GainL[3]) << 11)
																  | (WM5102_EQUALIZER_GAIN_CONVERT(GainL[4]) << 6)
																  | (ModeL == WM5102_EQUALIZER_SHELVING_FILTER)?0x0000:0x0001);
		}

		if(CoeffL)
			for(RegOffset = 0; RegOffset < 19; RegOffset++)
				counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ1_3 + RegOffset, CoeffL[RegOffset]);

		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ1MIX_INPUT_1_SOURCE, InputSourceL);
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ1MIX_INPUT_1_VOLUME, 0x0080); // EQ1 mixer source 1 gain 0dB

		InputSourceL = 0x0050;
	}

	if(ModeR != WM5102_EQUALIZER_DISABLED)
	{
		if(GainR)
		{
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ2_1, (WM5102_EQUALIZER_GAIN_CONVERT(GainR[0]) << 11)
																  | (WM5102_EQUALIZER_GAIN_CONVERT(GainR[1]) << 6)
																  | (WM5102_EQUALIZER_GAIN_CONVERT(GainR[2]) << 1)
																  | 0x0001);

			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ2_2, (WM5102_EQUALIZER_GAIN_CONVERT(GainR[3]) << 11)
																  | (WM5102_EQUALIZER_GAIN_CONVERT(GainR[4]) << 6)
																  | (ModeR == WM5102_EQUALIZER_SHELVING_FILTER)?0x0000:0x0001);
		}

		if(CoeffR)
			for(RegOffset = 0; RegOffset < 19; RegOffset++)
				counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ2_3 + RegOffset, CoeffR[RegOffset]);

		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ2MIX_INPUT_1_SOURCE, InputSourceR);
		counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_EQ2MIX_INPUT_1_VOLUME, 0x0080); // EQ2 mixer source 1 gain 0dB

		InputSourceR = 0x0051;
	}

	counter += CODEC_IO_Write(DeviceAddr, InputDestinationL, InputSourceL);
	counter += CODEC_IO_Write(DeviceAddr, InputDestinationR, InputSourceR);
}

/**
  * @brief Switch dynamically (while audio file is played) the input device
  *         (microphone or line-in).
  * @param DeviceAddr: Device address on communication Bus.
  * @param Input: specifies the audio output target: INPUT_DEVICE_MIC_IN,
  * INPUT_DEVICE_LINE_IN, INPUT_DEVICE_DMIC_IN
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_SetInputMode(uint16_t DeviceAddr, uint8_t Input)
{
	uint32_t counter = 0;
	uint16_t InputSourceL;
	uint16_t InputSourceR;
	uint16_t InputDestinationL;
	uint16_t InputDestinationR;

	wm5102_GetInsertionPoint(DeviceAddr, WM5102_BLOCK_NONE,
			&InputSourceL, &InputSourceR, &InputDestinationL, &InputDestinationR);

	switch(Input)
	{
		case INPUT_DEVICE_DIGITAL_MICROPHONE:
			InputSourceL = 0x0012; // LHPF1 mixer from IN2 (DMIC IN)
			InputSourceR = 0x0013; // LHPF2 mixer from IN2
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_INPUT_ENABLES, 0x000C); // enable IN2L and IN2R 000C DMIC IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_IN2L_CONTROL, 0x3480); // IN2 DMIC IN IN2L PGA vol 0dB
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_ADC_DIGITAL_VOLUME_2R, 0x0280); // IN2R ADC volume 0dB DMIC IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_ADC_DIGITAL_VOLUME_2L, 0x0280); // IN2L ADC volume 0dB DMIC IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_MIC_BIAS_CTRL_2, 0x01A7); // MICBIAS2 enable DMIC IN
			break;
		case INPUT_DEVICE_MICROPHONE:
			InputSourceL = 0x0010; // LHPF1 mixer from IN1 (MIC IN)
			InputSourceR = 0x0011; // LHPF2 mixer from IN1
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_INPUT_ENABLES, 0x0003); // enable IN1L and IN1R 0003 MIC IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_IN1L_CONTROL, 0x2A80); // IN1L PGA vol 0dB MIC IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_IN1R_CONTROL, 0x0080); // IN1R PGA vol 0dB MIC IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_ADC_DIGITAL_VOLUME_1L, 0x0280); // IN1L ADC volume 0dB MIC IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_ADC_DIGITAL_VOLUME_1R, 0x0280); // IN1R ADC volume 0dB DMIC IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_MIC_BIAS_CTRL_1, 0x01A7); // MICBIAS1 enable MIC IN
			break;
		case INPUT_DEVICE_LINE_IN:
			InputSourceL = 0x0014; // LHPF1 mixer from IN3 (LINE IN)
			InputSourceR = 0x0015; // LHPF2 mixer from IN3
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_INPUT_ENABLES, 0x0030); // enable IN3L and IN3R 0030 LINE IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_IN3L_CONTROL, 0x2290); // IN3L PGA gain +8.0dB LINE IN (potential divider comp.)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_ADC_DIGITAL_VOLUME_3L, 0x0280); // IN3L ADC volume 0dB LINE IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_IN3R_CONTROL, 0x0090); // IN3R PGA gain +8.0dB LINE IN (potential divider comp.)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_ADC_DIGITAL_VOLUME_3R, 0x0280); // IN3R ADC volume 0dB LINE IN
			break;
		default:
			InputSourceL = 0x0014; // LHPF1 mixer from IN3 (LINE IN)
			InputSourceR = 0x0015; // LHPF2 mixer from IN3
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_INPUT_ENABLES, 0x0030); // enable IN3L and IN3R 0030 LINE IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_IN3L_CONTROL, 0x2290); // IN3L PGA gain +8.0dB LINE IN (potential divider comp.)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_ADC_DIGITAL_VOLUME_3L, 0x0280); // IN3L ADC volume 0dB LINE IN
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_IN3R_CONTROL, 0x0090); // IN3R PGA gain +8.0dB LINE IN (potential divider comp.)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_ADC_DIGITAL_VOLUME_3R, 0x0280); // IN3R ADC volume 0dB LINE IN
			break;
	}

	counter += CODEC_IO_Write(DeviceAddr, InputDestinationL, InputSourceL);
	counter += CODEC_IO_Write(DeviceAddr, InputDestinationR, InputSourceR);

	//Set audio interface volume
#if (AUDIO_DATA_INTERFACE == 1)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1TX1MIX_INPUT_1_VOLUME, 0x0080); // AIF1TX mixer gain 0dB
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF1TX2MIX_INPUT_1_VOLUME, 0x0080); // AIF1TX mixer gain 0dB
#elif (AUDIO_DATA_INTERFACE == 2)
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2TX1MIX_INPUT_1_VOLUME, 0x0080); // AIF2TX mixer gain 0dB
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF2TX2MIX_INPUT_1_VOLUME, 0x0080); // AIF2TX mixer gain 0dB
#else
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3TX1MIX_INPUT_1_VOLUME, 0x0080); // AIF3TX mixer gain 0dB
	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_AIF3TX2MIX_INPUT_1_VOLUME, 0x0080); // AIF3TX mixer gain 0dB
#endif

	return counter;
}

/**
  * @brief Switch dynamically (while audio file is played) the output target
  *         (speaker or headphone).
  * @param DeviceAddr: Device address on communication Bus.
  * @param Output: specifies the audio output target: OUTPUT_DEVICE_SPEAKER,
  *         OUTPUT_DEVICE_HEADPHONE, OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_SetOutputMode(uint16_t DeviceAddr, uint8_t Output)
{
	uint32_t counter = 0;
	uint8_t OutputSource[2];
#if (AUDIO_DATA_INTERFACE == 1)
	OutputSource[0] = 0x20;
	OutputSource[1] = 0x21;
#elif (AUDIO_DATA_INTERFACE == 2)
	OutputSource[0] = 0x28;
	OutputSource[1] = 0x29;
#else
	OutputSource[0] = 0x30;
	OutputSource[1] = 0x31;
#endif

	//Route mixers to AIF
	switch(Output)
	{
		case OUTPUT_DEVICE_LINE_OUT:
			// LINE OUT enabled and HP OUT in silence
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2LMIX_INPUT_1_SOURCE, OutputSource[0]); // OUT2L (LINE OUT) mixer input is AIF1 RX1 (from I2S) 30
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2LMIX_INPUT_1_VOLUME, 0x0080); // associated volume is 0dB

			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2RMIX_INPUT_1_SOURCE, OutputSource[1]); // OUT2R (LINE OUT) mixer input is AIF1 RX2 (from I2S) 31
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2RMIX_INPUT_1_VOLUME, 0x0080); // associated volume is 0dB

			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1LMIX_INPUT_1_SOURCE, 0x00); // OUT1L (HP OUT) mixer input is Silence (Mute)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1RMIX_INPUT_1_SOURCE, 0x00); // OUT1R (HP OUT) mixer input is Silence (Mute)
			break;
		case OUTPUT_DEVICE_HEADPHONE:
			// LINE OUT in silence and HP OUT enabled
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2LMIX_INPUT_1_SOURCE, 0x00); // OUT2L (LINE OUT) mixer input is Silence (Mute)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2RMIX_INPUT_1_SOURCE, 0x00); // OUT2R (LINE OUT) mixer input is Silence (Mute)

			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1LMIX_INPUT_1_SOURCE, OutputSource[0]); // OUT1L (HP OUT) mixer input is AIF1 RX1 (from I2S)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1LMIX_INPUT_1_VOLUME, 0x0080); // associated volume is 0dB

			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1RMIX_INPUT_1_SOURCE, OutputSource[1]); // OUT1R (HP OUT) mixer input is AIF1 RX2 (from I2S)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1RMIX_INPUT_1_VOLUME, 0x0080); // associated volume is 0dB
			break;
		case OUTPUT_DEVICE_BOTH:
		case OUTPUT_DEVICE_AUTO:
			// LINE OUT and HP OUT enabled in parallel
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2LMIX_INPUT_1_SOURCE, OutputSource[0]); // OUT2L (LINE OUT) mixer input is AIF1 RX1 (from I2S) 30
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2LMIX_INPUT_1_VOLUME, 0x0080); // associated volume is 0dB

			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2RMIX_INPUT_1_SOURCE, OutputSource[1]); // OUT2R (LINE OUT) mixer input is AIF1 RX2 (from I2S) 31
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT2RMIX_INPUT_1_VOLUME, 0x0080); // associated volume is 0dB

			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1LMIX_INPUT_1_SOURCE, OutputSource[0]); // OUT1L (HP OUT) mixer input is AIF1 RX1 (from I2S)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1LMIX_INPUT_1_VOLUME, 0x0080); // associated volume is 0dB

			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1RMIX_INPUT_1_SOURCE, OutputSource[1]); // OUT1R (HP OUT) mixer input is AIF1 RX2 (from I2S)
			counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUT1RMIX_INPUT_1_VOLUME, 0x0080); // associated volume is 0dB
			break;
	}

	counter += CODEC_IO_Write(DeviceAddr, WM5102_REG_OUTPUT_ENABLES_1, 0x000F); // enable outputs OUT2L, OUT2R, OUT1L, OUT1R

	return counter;
}

/**
  * @brief Sets new frequency.
  * @param DeviceAddr: Device address on communication Bus.
  * @param AudioFreq: Audio frequency used to play the audio stream.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_SetFrequency(uint16_t DeviceAddr, uint32_t AudioFreq)
{
  uint32_t counter = 0;

  wm5102_ConfigureSampleRate(DeviceAddr, AudioFreq);

  return counter;
}

/**
  * @brief Resets wm5102 registers.
  * @param DeviceAddr: Device address on communication Bus.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t wm5102_Reset(uint16_t DeviceAddr)
{
  uint32_t counter = 0;

  /* Reset Codec by wrinting in 0x0000 address register */
  counter = CODEC_IO_Write(DeviceAddr, 0x0000, 0x0000);

  return counter;
}

/**
  * @brief  Writes/Read a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
static uint8_t CODEC_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value)
{
	uint32_t result = 0;

	EXTERNAL_AUDIO_IO_Write(Addr, Reg, Value);

	#ifdef VERIFY_WRITTENDATA
	/* Verify that the data has been correctly written */
	result = (EXTERNAL_AUDIO_IO_Read(Addr, Reg) == Value)? 0:1;
	#endif /* VERIFY_WRITTENDATA */

	return result;
}

/**
  * @brief  Read a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
static uint8_t CODEC_IO_Read(uint8_t Addr, uint16_t Reg)
{
	uint32_t result = 0;

	result =  EXTERNAL_AUDIO_IO_Read(Addr, Reg);

	return result;
}

/**
  * @}
  */

/**
  * @}
  */
