#include <stm32f4xx.h>
#include <arm_math.h>
#include <stm32f4_discovery.h>
#include <stm32f4_discovery_accelerometer.h>
#include <wolfson_pi_audio.h>
#include <diag/Trace.h>
#include <tests.h>
#include <dwt.h>

int16_t TxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
int16_t RxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];

__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
__IO uint8_t Volume = 70;


void init (void)
{
    // Initialise the HAL Library; it must be the first
    // instruction to be executed in the main program.
    HAL_Init ();

    DWT_Enable ();

    WOLFSON_PI_AUDIO_Init ((INPUT_DEVICE_LINE_IN << 8) | OUTPUT_DEVICE_BOTH, 80,
    AUDIO_FREQUENCY_48K);

    WOLFSON_PI_AUDIO_SetInputMode (INPUT_DEVICE_LINE_IN);

    WOLFSON_PI_AUDIO_SetMute (AUDIO_MUTE_ON);

    WOLFSON_PI_AUDIO_Play (TxBuffer, RxBuffer, WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE);

    WOLFSON_PI_AUDIO_SetVolume (Volume);


    TEST_Init ();
}

int main (int argc, char* argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    init();
    uint32_t i;

    for (;;){
        // Add your code here.
        if (buffer_offset == BUFFER_OFFSET_HALF){
            DWT_Reset();

            for (i = 0; i < (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE / 2); i++){
                if (i % 2){
                    TxBuffer[i] = 0.5*RxBuffer[i];
                }else{
                    TxBuffer[i] = RxBuffer[i];
                }
            }

            buffer_offset = BUFFER_OFFSET_NONE;
        }

        if (buffer_offset == BUFFER_OFFSET_FULL){
            DWT_Reset();

            for (i = (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE / 2);
                i < WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++){
                if (i % 2){
                    TxBuffer[i] = RxBuffer[i];
                }else{
                    TxBuffer[i] = RxBuffer[i];
                }
            }

            buffer_offset = BUFFER_OFFSET_NONE;
        }

        TEST_Main ();
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
    while (1) ;
}
