#include <stm32f4xx.h>
#include <arm_math.h>
#include <stm32f4_discovery.h>
#include <stm32f4_discovery_accelerometer.h>
#include <wolfson_pi_audio.h>
#include <diag/Trace.h>
#include <tests.h>
#include <dwt.h>
// #include <math.h>
#include <fasteraprox/fastpow.h>
#include <fasteraprox/fastlog.h>
#include <fasteraprox/fastexp.h>

int16_t TxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
int16_t RxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];

__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
__IO uint8_t Volume = 70;

#define i2f(x)  ((float32_t)x/(2*32768.0))
#define f2i(x)  ((int16_t)(x*2*32768))
#define FALSE 0
#define TRUE 1

/**
 * @brief   Compressor with pre-gain, makeup, saturation and clipping adjustments.
 * @param   x is the signal (not in dB), array from -1 to 1;
 * @param   g is the pre-gain level (dB), from -10 to 10;
 * @param   r is the compression ratio, from 0 to 100;
 * @param   th is the threshold level (dB),from -100 to 0;
 * @param   k is the knee-width (dB), from 0 to threshold dB level
 * @param   att is the attack time (ms), from 0 to 500ms
 * @param   rlt is the release time (ms), from 0 to 500ms
 * @param   m is the makeup gain level (dB), from -100 to 100;
 * @param   c is the clip level (dB), from 0 to -100;
 * @param   d is the saturation ratio, from 0 to 1
 */
typedef struct comp_conf_t{
    float32_t x;
    float32_t g;
    float32_t r;
    float32_t th;
    float32_t k;
    float32_t att;
    float32_t rlt; 
    float32_t m;
    float32_t c;
    float32_t d;
    float32_t g_;
    float32_t m_;
    float32_t c_;
    float32_t k1;
    float32_t k2;
    float32_t satt;
}comp_conf_t;

typedef struct comp_t{
    comp_conf_t *param;
    uint32_t sba;
    uint32_t sbr;
    uint32_t gate;
    int32_t s;
}comp_t;

void init(void)
{
    // Initialise the HAL Library; it must be the first
    // instruction to be executed in the main program.
    HAL_Init();
    DWT_Enable();
    WOLFSON_PI_AUDIO_Init((INPUT_DEVICE_LINE_IN << 8) | OUTPUT_DEVICE_BOTH, 80,
    AUDIO_FREQUENCY_48K);
    WOLFSON_PI_AUDIO_SetInputMode(INPUT_DEVICE_LINE_IN);
    WOLFSON_PI_AUDIO_SetMute(AUDIO_MUTE_ON);
    WOLFSON_PI_AUDIO_Play(TxBuffer, RxBuffer, WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE);
    WOLFSON_PI_AUDIO_SetVolume(Volume);
    //TEST_Init();
}

comp_t comp_init(comp_conf_t *c)
{
    // Apply parameters limits
    if(c->g > 10)          c->g = 10;
    else if(c->g < -10)    c->g = -10;
    if(c->r > 100)         c->r = 100;
    else if(c->r <= 0)     c->r = 0;
    if(c->th > 0)          c->th = 0;
    else if(c->th < -100)  c->th = -100;
    if(c->k < 0)           c->k = 0;
    else if(c->k > -c->th) c->k = -c->th;
    if(c->att > 500)       c->att = 500;
    if(c->rlt > 500)       c->rlt = 500;
    if(c->m > 100)         c->m = 100;
    else if(c->m < -100)   c->m = -100;
    if(c->c > 0)           c->c = 0;
    else if(c->c < -10)    c->c = -10;
    if(c->d < 0)           c->d = 0;
    else if(c->d > 1)      c->d = 1;

    // Convert from dB to level
    c->g_ = fasterpow(10, c->g / 20);      // pre-gain level
    c->m_ = fasterpow(10, c->m / 20);      // makeup gain level
    c->c_ = fasterpow(10, c->c / 20);      // clip level
    c->k = 0.5 * c->k;                // knee-width level
    c->k1 = c->th -c->k;                 // knee minimum level (dB)
    c->k2 = c->th +c->k;                 // knee maximum level (dB)
    //if(r == 0) r = 1;           // protects for division
    c->r = 1 / c->r;                  // ratio to multiply instead divide
    c->d = c->d / 10;                 // distortion ratio
    c->satt = AUDIO_FREQUENCY_48K * c->att / 1000;     // attack (samples)
    //srlt = fs * rlt / 1000;     // release (samples)

    comp_t r;
    r.param = c;
    return r;
}

void comp(int16_t *b, comp_t *c)
{
    // Convert to float
    c->param->x = i2f(*b);

    // Pre-gain
    c->param->x *= c->param->g_;

    // Sign detection
    if(c->param->x < 0)     c->s = -1;
    else                    c->s = 1;

    // Threshold detection
    if((c->s * c->param->x) > c->param->k1){
        // Count samples before Attack
        if(c->sba < c->param->satt) c->sba++;
        else{
            c->sbr = 0;
            c->gate = TRUE;
        }
    }else{
        // Threshold detection
        if((c->s * c->param->x) < c->param->k1) c->sbr++;
        else{
            // todo: release -> srlt
            c->sba = 0;
            c->gate = FALSE;
        }
    }

    // Compression
    if(c->gate){
        // Convert to dB
        c->param->x = 20 * fasterlog(c->s * c->param->x);
    
        // Soft knee
        if(c->param->x > c->param->k2){
            // Above knee region, hard compress:
            c->param->x = ((c->param->x -c->param->th) * c->param->r) +c->param->th;
        }else if(c->param->x > c->param->k1){
            // On knee region, compress using a second order interpolation
            c->param->x += ((c->param->r-1) * (c->param->x -c->param->th +c->param->k) * (c->param->x -c->param->th +c->param->k) / (4 * c->param->k));
        }
    
        // Get back from dB
        c->param->x = c->s * fasterpow(10, c->param->x / 20);

        // Adds a bit of saturation
        c->param->x -= (c->s * c->param->d * (-1 +fasterexp(c->s * c->param->x)));
    }

    // Makeup gain
    c->param->x *= c->param->m_;

    // Hard clip
    if((c->s * c->param->x) > c->param->c_) c->param->x = c->s * c->param->c_;


    // Convert back to int
    *b = f2i(c->param->x);
}

int main(int argc, char* argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    uint32_t i = 0;
    init();

    // Creates the compressor setup
    comp_conf_t c;
    c.g = 0;
    c.r = 20;
    c.th = -40;
    c.k = 4;
    c.att = 1;
    c.rlt = 0.5;
    c.m = -10;
    c.c = -0.1;
    c.d = 0;

    // Creates a stereo compressor sharing the configuration
    comp_t cL = comp_init(&c);
    comp_t cR = comp_init(&c);
   
    for(;;){

        // Compressor block
        if(i % 2)   comp(&RxBuffer[i], &cL);
        else        comp(&RxBuffer[i], &cR);

        // Buffer block
        if(buffer_offset == BUFFER_OFFSET_HALF){
            DWT_Reset();
            for(i = 0; i < (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE / 2); i++){
                if (i % 2){
                    TxBuffer[i] = RxBuffer[i];
                }else{
                    TxBuffer[i] = RxBuffer[i];
                }
            }
            buffer_offset = BUFFER_OFFSET_NONE;
        }

        if(buffer_offset == BUFFER_OFFSET_FULL){
            DWT_Reset();
            for(i = (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE / 2); i < WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++){
                if (i % 2){
                    TxBuffer[i] = RxBuffer[i];
                }else{
                    TxBuffer[i] = RxBuffer[i];
                }
            }
            buffer_offset = BUFFER_OFFSET_NONE;
        }

        //TEST_Main ();
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
