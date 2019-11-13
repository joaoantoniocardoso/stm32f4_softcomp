#include <tests.h>
#include "filter.h"

typedef enum TestType {
	TEST_NONE = 0,
	TEST_PLAY,
	TEST_VOLUME,
	TEST_SET_FREQUENCY_8K,
	TEST_SET_FREQUENCY_48K,
	TEST_SET_FILTER_LOW_PASS,
	TEST_SET_FILTER_HIGH_PASS,
	TEST_SET_FILTER_DISABLE,
	TEST_SET_EQUALIZER_LR,
	TEST_SET_EQUALIZER_RL,
	TEST_SET_EQUALIZER_CUSTOM,
	TEST_SET_EQUALIZER_DISABLE,
	TEST_PAUSE,
	TEST_RESUME,
	TEST_FIR_FLOAT32,
	TEST_FIR_Q15,
	TEST_STOP
} TestTypeDef;

typedef struct {
	const TestTypeDef TestType;
	const char * TestName;
} TestDescDef;

static const TestDescDef TestDesc[] = {
		{TEST_PLAY, "Play"},
		{TEST_VOLUME, "Volume"},
		{TEST_SET_FREQUENCY_8K, "Set frequency to 8k"},
		{TEST_SET_FREQUENCY_48K, "Set frequency to 48k"},
		{TEST_SET_FILTER_LOW_PASS, "Set filter to low pass"},
		{TEST_SET_FILTER_HIGH_PASS, "Set filter to high pass"},
		{TEST_SET_FILTER_DISABLE, "Disable filter"},
		{TEST_SET_EQUALIZER_LR, "Set equalizer with gains L= {12,6,0,-6,-12}, R={-12,-6,0,6,12}"},
		{TEST_SET_EQUALIZER_RL, "Set equalizer with gains R= {12,6,0,-6,-12}, L={-12,-6,0,6,12}"},
		{TEST_SET_EQUALIZER_CUSTOM, "Set equalizer with custom gains"},
		{TEST_SET_EQUALIZER_DISABLE, "Disable equalizer"},
		{TEST_PAUSE, "Pause"},
		{TEST_RESUME, "Resume"},
		{TEST_FIR_FLOAT32, "Set FIR filter to FLOAT32"},
		{TEST_FIR_Q15, "Set FIR filter to Q15"},
		{TEST_STOP, "Stop"},
		{TEST_NONE, NULL},
};

extern int16_t TxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
extern int16_t RxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
extern __IO uint8_t Volume;

extern FilterTypeDef filterType;

uint16_t FTCoeffL = 0xFBB6;
uint16_t FTCoeffR = 0xFBB6;

int8_t EQGainL[5] = {12,6,0,-6,-12};
int8_t EQGainR[5] = {-12,-6,0,6,12};
__IO int8_t CustomEQGainL[5] = {0,0,0,0,0};
__IO int8_t CustomEQGainR[5] = {0,0,0,0,0};

uint16_t EQCoeffL[19] = {
		0x0FC8,
		0x03FE,
		0x00E0,
		0x1EC4,
		0xF136,
		0x040A,
		0x04CC,
		0x1C9B,
		0xF337,
		0x040A,
		0x0CBB,
		0x16F8,
		0xF7D9,
		0x040A,
		0x1F14,
		0x058C,
		0x0563,
		0x4000,
		0x0000
};

uint16_t EQCoeffR[19] = {
		0x0FC8,
		0x03FE,
		0x00E0,
		0x1EC4,
		0xF136,
		0x0409,
		0x04CC,
		0x1C9B,
		0xF337,
		0x040B,
		0x0CBB,
		0x16F8,
		0xF7D9,
		0x040A,
		0x1F14,
		0x058C,
		0x0563,
		0x4000,
		0x0B75,
};

#define TEST_FIRST TEST_PLAY
#define TEST_LAST TEST_STOP

__IO TestTypeDef TestPending = TEST_NONE;
__IO uint32_t TestDebounce = 0, TestDebounce2 = 0;

//HAL Configure 1 Tick = 1ms
#define TEST_DEBOUNCE_TICKS 20

static void TEST_GetEQGains()
{
	uint32_t i;

	for(i=0; i<5; i++)
	{
		trace_printf("\nGain L - Band %d (-12dB - +12dB)", (i+1));
		utilsGetOption("\n>", UTILS_OPTION_TYPE_INT8, &CustomEQGainL[i], NULL);
	}

	for(i=0; i<5; i++)
	{
		trace_printf("\nGain R - Band %d", (i+1));
		utilsGetOption("\n>", UTILS_OPTION_TYPE_INT8, &CustomEQGainR[i], NULL);
	}
}

void TEST_Init()
{
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
}

void TEST_Main()
{
	if(TestPending != TEST_NONE)
	{
		switch(TestPending)
		{
			case TEST_PLAY:
				WOLFSON_PI_AUDIO_Play(TxBuffer, RxBuffer, WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE);
				break;
			case TEST_VOLUME:
				WOLFSON_PI_AUDIO_SetVolume((uint8_t) Volume);
				break;
			case TEST_SET_FREQUENCY_8K:
				WOLFSON_PI_AUDIO_Stop(0);
				WOLFSON_PI_AUDIO_SetFrequency(AUDIO_FREQUENCY_8K);
				WOLFSON_PI_AUDIO_Play(TxBuffer, RxBuffer, WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE);
				break;
			case TEST_SET_FREQUENCY_48K:
				WOLFSON_PI_AUDIO_Stop(0);
				WOLFSON_PI_AUDIO_SetFrequency(AUDIO_FREQUENCY_48K);
				WOLFSON_PI_AUDIO_Play(TxBuffer, RxBuffer, WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE);
				break;
			case TEST_SET_FILTER_LOW_PASS:
				WOLFSON_PI_AUDIO_SetFilter(WM5102_FILTER_LOW_PASS, WM5102_FILTER_LOW_PASS, FTCoeffL, FTCoeffR);
				break;
			case TEST_SET_FILTER_HIGH_PASS:
				WOLFSON_PI_AUDIO_SetFilter(WM5102_FILTER_HIGH_PASS, WM5102_FILTER_HIGH_PASS, FTCoeffL, FTCoeffR);
				break;
			case TEST_SET_FILTER_DISABLE:
				WOLFSON_PI_AUDIO_SetFilter(WM5102_FILTER_DISABLED, WM5102_FILTER_DISABLED, 0, 0);
				break;
			case TEST_SET_EQUALIZER_LR:
				WOLFSON_PI_AUDIO_SetEqualizer(WM5102_EQUALIZER_SHELVING_FILTER, WM5102_EQUALIZER_SHELVING_FILTER,
									EQGainL, EQGainR, EQCoeffL, EQCoeffR);
				break;
			case TEST_SET_EQUALIZER_RL:
				WOLFSON_PI_AUDIO_SetEqualizer(WM5102_EQUALIZER_SHELVING_FILTER, WM5102_EQUALIZER_SHELVING_FILTER,
									EQGainR, EQGainL, EQCoeffL, EQCoeffR);
				break;
			case TEST_SET_EQUALIZER_CUSTOM:
				WOLFSON_PI_AUDIO_SetEqualizer(WM5102_EQUALIZER_SHELVING_FILTER, WM5102_EQUALIZER_SHELVING_FILTER,
									(int8_t *) CustomEQGainL, (int8_t *) CustomEQGainR, EQCoeffL, EQCoeffR);
				break;
			case TEST_SET_EQUALIZER_DISABLE:
				WOLFSON_PI_AUDIO_SetEqualizer(WM5102_EQUALIZER_DISABLED, WM5102_EQUALIZER_DISABLED,
									NULL, NULL, NULL, NULL);
				break;
			case TEST_PAUSE:
				WOLFSON_PI_AUDIO_Pause();
				break;
			case TEST_RESUME:
				WOLFSON_PI_AUDIO_Resume();
				break;
			case TEST_FIR_FLOAT32:
				WOLFSON_PI_AUDIO_Pause();
				//filterType=FIR_FLOAT32;
				WOLFSON_PI_AUDIO_Resume();
				break;
			case TEST_FIR_Q15:
				WOLFSON_PI_AUDIO_Pause();
				//filterType=FIR_Q15;
				WOLFSON_PI_AUDIO_Resume();
				break;
			case TEST_STOP:
				WOLFSON_PI_AUDIO_Stop(0);
				break;
			default:
				break;
		}

		TestPending = TEST_NONE;
	}
}




/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t i;
	uint8_t option;

	if (KEY_BUTTON_PIN == GPIO_Pin)
	{
		if(BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_SET)
		{
			TestDebounce = HAL_GetTick();
		}
		else
		{
			if((HAL_GetTick() - TestDebounce) > TEST_DEBOUNCE_TICKS)
			{
				WOLFSON_PI_AUDIO_Pause();

				trace_printf("\nSelect test to run:\n");

				for(i=0; TestDesc[i].TestType; i++)
				{
					trace_printf("\n%d - %s.", TestDesc[i].TestType, TestDesc[i].TestName);
				}

				utilsGetOption("\nTest>", UTILS_OPTION_TYPE_UINT8, &option, NULL);

				TestPending = option;

				switch(TestPending)
				{
					case TEST_PLAY:
						break;
					case TEST_VOLUME:
						trace_printf("\nCurrent volume: %d", Volume);
						utilsGetOption("\nVolume>", UTILS_OPTION_TYPE_UINT8, &Volume, NULL);
						break;
					case TEST_SET_FREQUENCY_8K:
					case TEST_SET_FREQUENCY_48K:
					case TEST_SET_FILTER_LOW_PASS:
					case TEST_SET_FILTER_HIGH_PASS:
					case TEST_SET_FILTER_DISABLE:
					case TEST_SET_EQUALIZER_LR:
					case TEST_SET_EQUALIZER_RL:
						break;
					case TEST_SET_EQUALIZER_CUSTOM:
						TEST_GetEQGains();
						break;
					case TEST_SET_EQUALIZER_DISABLE:
					case TEST_PAUSE:
					case TEST_RESUME:
					case TEST_FIR_FLOAT32:
					case TEST_FIR_Q15:
					case TEST_STOP:
					default:
						break;
				}

				WOLFSON_PI_AUDIO_Resume();

				TestDebounce = HAL_GetTick();
			}
		}
	}
}




