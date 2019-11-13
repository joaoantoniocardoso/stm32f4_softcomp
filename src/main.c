//#include <stm32f4xx.h>
//#include <stm32f4xx_hal.h>
//#include <stm32f4xx_hal_gpio.h>
//#include <stm32f4_discovery.h>
#include <dwt.h>
#include "compressor.h"
#include "audio_config.h"
#include <wolfson_pi_audio.h>

int16_t TxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
int16_t RxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];

__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
__IO uint8_t Volume = VOLUME;
GPIO_InitTypeDef GPIO_InitStructure;

void wolfson_pi_init(void);
void stm_init(void);

void wolfson_pi_init(void)
{
    WOLFSON_PI_AUDIO_Init((INPUT_DEVICE_LINE_IN << 8) | OUTPUT_DEVICE_BOTH, 80, AUDIO_FREQUENCY);
    WOLFSON_PI_AUDIO_SetInputMode(INPUT_DEVICE_LINE_IN);
    WOLFSON_PI_AUDIO_SetMute(AUDIO_MUTE_ON);
    WOLFSON_PI_AUDIO_Play(TxBuffer, RxBuffer, WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE);
    WOLFSON_PI_AUDIO_SetVolume(Volume);
}

void stm_init(void)
{
	// Initialise the HAL Library; it must be the first
	// instruction to be executed in the main program.
	HAL_Init();
	DWT_Enable();

    //Push Button
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   = GPIO_PIN_0;
	GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull  = GPIO_NOPULL;
	//GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	//LED:
	__GPIOD_CLK_ENABLE();
	GPIO_InitStructure.Pin   = GPIO_PIN_12;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	__GPIOD_CLK_ENABLE();
	GPIO_InitStructure.Pin   = GPIO_PIN_13;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	__GPIOD_CLK_ENABLE();
	GPIO_InitStructure.Pin   = GPIO_PIN_14;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	__GPIOD_CLK_ENABLE();
	GPIO_InitStructure.Pin   = GPIO_PIN_15;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    LED0_on();
    LED1_on();
    LED2_on();
    LED3_on();
}

int main(int argc, char* argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    stm_init();
    wolfson_pi_init();

    // Compressor initialization
    compressor_parameter_t compressor_parameter;
    compressor_t compressor;
    compressor_init(&compressor_parameter, &compressor);

    LED0_off();
    LED1_off();
    LED2_off();
    LED3_off();

    // Left-Right matrix selection macros
    #ifdef BYPASS_L
    #define FUN_L(i)  RxBuffer[i]
    #else
    #define FUN_L(i)  compressor_stage(RxBuffer[i], &compressor)
    #endif
    #ifdef BYPASS_R
    #define FUN_R(i)  RxBuffer[i]
    #else
    #define FUN_R(i)  compressor_stage(RxBuffer[i], &compressor)
    #endif

    for(;;){
    	static uint32_t i = 0;
		if(buffer_offset == BUFFER_OFFSET_HALF){
			DWT_Reset();
			for(i = 0; i < (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE / 2); i++){
					TxBuffer[i] = FUN_L(i);
					i++;
					TxBuffer[i] = FUN_R(i);
			}
			buffer_offset = BUFFER_OFFSET_NONE;
		}

		if(buffer_offset == BUFFER_OFFSET_FULL){
			DWT_Reset();
			for(i = (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE / 2); i < WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++){
					TxBuffer[i] = FUN_L(i);
					i++;
					TxBuffer[i] = FUN_R(i);
			}
			buffer_offset = BUFFER_OFFSET_NONE;
		}
	}

    return 0;
}

/*--------------------------------------------------------
 Callbacks implementation:
 --------------------------------------------------------*/

/**
 * @brief    Manages the DMA full Transfer complete event.
 */
void WOLFSON_PI_AUDIO_TransferComplete_CallBack (void)
{
    buffer_offset = BUFFER_OFFSET_FULL;
}

/**
 * @brief    Manages the DMA Half Transfer complete event.
 */
void WOLFSON_PI_AUDIO_HalfTransfer_CallBack (void)
{
    buffer_offset = BUFFER_OFFSET_HALF;
}

/**
 * @brief    Manages the DMA FIFO error interrupt.
 * @param    None
 * @retval None
 */
void WOLFSON_PI_AUDIO_OUT_Error_CallBack (void)
{
    /* Stop the program with an infinite loop */
    for(;;);
}
