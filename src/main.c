#include <arm_math.h>
//#include <stm32f4xx.h>
//#include <stm32f4xx_hal.h>
//#include <stm32f4xx_hal_gpio.h>
//#include <stm32f4_discovery.h>
#include <wolfson_pi_audio.h>
#include <dwt.h>
#include <fasteraprox/fastpow.h>
#include <fasteraprox/fastlog.h>
#include <fasteraprox/fastexp.h>
#define fasterlog10(x)  (0.3010299956639812f * fasterlog2(x))

#define SETUP_LIVE      // SETUP_LIVE or SETUP_TEST?

#ifdef SETUP_LIVE
  #define INPUT_MONO_R
  //#define INPUT_MONO_L
  //#define INPUT_STEREO
  //#define INPUT_CROSS
  #define OUTPUT_STEREO
  //#define COMP_SETUP_NONE
  #define COMP_SETUP_CUSTOM
  #define VOLUME  70
#endif

#ifdef SETUP_TEST
  #define INPUT_STEREO
  #define BYPASS_R
  #define VOLUME  76
  #define COMP_SETUP_CLEANGUITARS
#endif

int16_t TxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
int16_t RxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];

__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
__IO uint8_t Volume = VOLUME;
GPIO_InitTypeDef GPIO_InitStructure;

#define i2f(x)  ((float32_t)x/(2*32768.0))
#define f2i(x)  ((int16_t)(x*2*32768))
#define FALSE 0
#define TRUE 1

// Leds macros
// LED0 -> green    right
// LED1 -> orange   up
// LED2 -> red      left
// LED3 -> blue     down
#define LED0_on()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)
#define LED1_on()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)
#define LED2_on()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)
#define LED3_on()     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET)
#define LED0_off()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
#define LED1_off()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED2_off()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED3_off()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET)
#define LED0_tg()     HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) ? LED0_on() : LED0_off()
#define LED1_tg()     HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) ? LED1_on() : LED1_off()
#define LED2_tg()     HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) ? LED2_on() : LED2_off()
#define LED3_tg()     HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) ? LED3_on() : LED3_off()

// Buttons Macros
#define BT0_on()      (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
#define BT0_off()     (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)

// Left-Right matrix selection macros
#ifdef BYPASS_L
  #define FUN_L(i)  RxBuffer[i]
#else
  #define FUN_L(i)  comp(RxBuffer[i], &cL);
#endif
#ifdef BYPASS_R
  #define FUN_R(i)  RxBuffer[i]
#else
  #define FUN_R(i)  comp(RxBuffer[i], &cR);
#endif

/**
 * @brief   Compressor with pre-gain, makeup, saturation and clipping adjustments.
 * @param   x is the signal (not in dB), array from -1 to 1;
 * @param   g is the pre-gain level (dB), from -10 to 10;
 * @param   r is the compression ratio, from 0 to 30;
 * @param   th is the threshold level (dB),from -40 to 0;
 * @param   k is the knee-width (dB), from 0 to threshold dB level or -10;
 * @param   att is the attack time (ms), from 0 to 100ms;
 * @param   rlt is the release time (ms), from 0 to 1000ms;
 * @param   m is the makeup gain level (dB), from -10 to 10;
 * @param   c is the clip level (dB), from 0 to -10;
 * @param   d is the saturation ratio, from 0 to 1
 */
typedef struct comp_conf_t{
    float32_t g;
    float32_t g_;
    float32_t r;
    float32_t th;
    float32_t k;
    float32_t k1;
    float32_t k2;
    float32_t k1_;
    float32_t att;
    float32_t rlt; 
    uint32_t satt;
    uint32_t srlt;
    float32_t d;
    float32_t m;
    float32_t m_;
    float32_t c;
    float32_t c_;
}comp_conf_t;

typedef struct comp_t{
    comp_conf_t *param;
    float32_t x;
    uint32_t sba;
    uint32_t sbr;
    uint32_t gate;
    uint32_t attacking;
    uint32_t releasing;
    uint32_t rd;
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

void comp_param_init(comp_conf_t *c)
{
    // Apply parameters limits
    if(c->g > 10)          c->g = 10;
    else if(c->g < -10)    c->g = -10;
    if(c->r > 30)          c->r = 30;
    else if(c->r < 1)      c->r = 1;
    if(c->th > 0)          c->th = 0;
    else if(c->th < -40)   c->th = -40;
    if(c->k < 0)           c->k = 0;
    else if(c->k > -c->th) c->k = -c->th;
    if(c->k < -10)         c->k = -10;
    if(c->att > 100)       c->att = 100;
    if(c->att < 0)         c->att = 0;
    if(c->rlt > 1000)      c->rlt = 1000;
    if(c->rlt < 0)         c->rlt = 0;
    if(c->m > 10)          c->m = 10;
    else if(c->m < -10)    c->m = -10;
    if(c->c > 0)           c->c = 0;
    else if(c->c < -10)    c->c = -10;
    if(c->d < 0)           c->d = 0;
    else if(c->d > 1)      c->d = 1;

    // Convert from dB to level
    c->g_ = powf(10, c->g / 20);      // pre-gain level
    c->m_ = powf(10, c->m / 20);      // makeup gain level
    c->c_ = powf(10, c->c / 20);      // clip level
    c->k = 0.5 * c->k;                // knee-width level
    c->k1 = c->th -c->k;              // knee minimum level (dB)
    c->k1_ = powf(10, c->k1 / 20);    // knee minimum level
    c->k2 = c->th +c->k;              // knee maximum level (dB)

    //if(r == 0) r = 1;               // protects for division
    c->r = 1 / c->r;                  // ratio to multiply instead divide
    c->d = c->d / 10;                 // distortion ratio
    c->satt = floor(AUDIO_FREQUENCY_48K * c->att / 1000);   // attack (samples)
    c->srlt = floor(AUDIO_FREQUENCY_48K * c->rlt / 1000);   // release (samples)
}

void comp_init_mono(comp_conf_t *c, comp_t *m)
{
    comp_param_init(c);
    m->param = c;
    m->sba = m->sbr = m->gate = m->attacking = m->releasing = 0;
    m->s = 1;
    m->rd = 1;
}

void comp_init_stereo(comp_conf_t *c, comp_t *sl, comp_t *sr)
{
    comp_param_init(c);
    sl->param = sr->param = c;
    sl->sba = sl->sbr = sl->gate = sl->attacking = sl->releasing = 0;
    sr->sba = sr->sbr = sr->gate = sr->attacking = sr->releasing = 0;
    sl->s = sr->s = 1;
    sl->rd = sr->rd = 1;
}

int16_t comp(int16_t b, comp_t *c)
{
    // Convert to float
    c->x = i2f(b);

    // Pre-gain
    c->x *= c->param->g_;

    // Sign detection
    if(c->x < 0)     c->s = -1;
    else             c->s = 1;

    // Threshold detection
    if((c->s * c->x) >= c->param->k1_){      // Attack
        LED0_on();
        if(!c->param->satt){                // zero-attack case
            c->gate = TRUE;
            c->rd = c->param->r;            // Fully closed
        }else if(!c->attacking && !c->releasing){
            c->attacking = TRUE;
            c->gate = FALSE;
            c->sba = 0;
        }
    }else{                                  // Release
        LED0_off();
        if(!c->param->srlt){                // zero-release case
            c->gate = FALSE;
            c->rd = 1;                      // Fully open
        }else if(c->gate){
            if(!c->releasing && !c->attacking){
                c->releasing = TRUE;
                c->gate = FALSE;
                c->sbr = 0;
            }
        }
    }

    // Ballistic
    if(c->attacking){
        LED1_on();
        if(c->sba < c->param->satt){
            // Count samples before Attack
            // Closing transition
            c->rd = c->param->r +(1 -c->param->r) * fasterexp(-5 * c->sba / c->param->satt);
            //c->rd = c->param->r +(1 -c->param->r) * expf(-5 * c->sba / c->param->satt);
            c->sba++;
        }else{
            c->attacking = FALSE;
            c->gate = TRUE;
        }
    }else if(c->releasing){
        LED1_on();
        c->gate = FALSE;
        // Count samples before Release
        if(c->sbr < c->param->srlt){
            // Opening transition
            c->rd = 1 +(1 -c->param->r) * fasterexp(-5 * c->sbr / c->param->srlt);
            //c->rd = 1 +(1 -c->param->r) * expf(-5 * c->sbr / c->param->srlt);
            c->sbr++;
        }else{
            c->releasing = FALSE;
        }
    }

    // Adds a bit of saturation
    //if(c->param->srlt && c->param->satt){
    //    c->x -= (c->s * c->param->d * (-1 +fasterexp(c->s * c->x)));
        //c->x -= (c->s * c->param->d * (-1 +expf(c->s * c->x)));
    //}

    // Buttons interface
    // A/B test
    if(BT0_on()){
        LED3_on();
        c->rd = 1;
    }

    // Convert to dB
    c->x = 20 * fasterlog10(c->s * c->x);
    //c->x = 20 * log10f(c->s * c->x);

    // Soft knee
    if(c->x >= c->param->k2){
        LED3_on();
        // Above knee region, hard compress:
        c->x = ((c->x -c->param->th) * c->rd) +c->param->th;
    }else if(c->x >= c->param->k1){
        LED3_on();
        // On knee region, compress using a second order interpolation
        c->x += ((c->rd -1) * (c->x -c->param->th +c->param->k) * (c->x -c->param->th +c->param->k) / (4 * c->param->k));
    }

    // Get back from dB
    c->x = c->s * fasterpow(10, c->x / 20);
    //c->x = c->s * powf(10, c->x / 20);

    // Makeup gain
    c->x *= c->param->m_;

    // Hard clip
    if((c->s * c->x) > c->param->c_){
        LED2_on();
        c->x = c->s * c->param->c_;
    }else{
        LED0_off();
        LED1_off();
        LED2_off();
        LED3_off();
    }

    // Convert back to int
    return f2i(c->x);
}

int main(int argc, char* argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    uint32_t i = 0;
    init();

    // Creates the compressor setup
    comp_conf_t c;

#ifdef COMP_SETUP_CUSTOM
    c.g = 0;                        // pre-gain level (dB)
    c.r = 20;                       // compress ratio
    c.th = -30;                     // threshold level (dB)
    c.k = 0;                        // knee-width (dB)
    c.att = 50;                      // attack (us)
    c.rlt = 100;                      // release (ms)
    c.m = 0;                        // makeup gain level (dB)
    c.c = -0.1;                     // clipping level (dB)
    c.d = 0;                        // distortion ratio
#endif

#ifdef COMP_SETUP_NONE
    c.g = 0;                        // pre-gain level (dB)
    c.r = 0;                        // compress ratio
    c.th = -0;                      // threshold level (dB)
    c.k = 0;                        // knee-width (dB)
    c.att = 0;                      // attack (us)
    c.rlt = 0;                      // release (ms)
    c.m = 0;                        // makeup gain level (dB)
    c.c = -0.0;                     // clipping level (dB)    
    c.d = 0;                        // distortion ratio   

#endif

#ifdef COMP_SETUP_SNARE
    c.g = 10;                       // pre-gain level (dB)
    c.r = 20;                       // compress ratio
    c.th = -16;                     // threshold level (dB)
    c.k = 0;                        // knee-width (dB)
    c.att = 0;                      // attack (us)
    c.rlt = 250;                    // release (ms)
    c.m = 0;                        // makeup gain level (dB)
    c.c = -0.1;                     // clipping level (dB)    
    c.d = 0;                        // distortion ratio   
#endif

#ifdef COMP_SETUP_DRUMS
    c.g = 10;                       // pre-gain level (dB)
    c.r = 30;                       // compress ratio
    c.th = -26;                     // threshold level (dB)
    c.k = 0;                        // knee-width (dB)
    c.att = 0;                      // attack (us)
    c.rlt = 250;                    // release (ms)
    c.m = 0;                        // makeup gain level (dB)
    c.c = -0.1;                     // clipping level (dB)
    c.d = 0;                        // distortion ratio
#endif

#ifdef COMP_SETUP_CLEANGUITARS
    c.g = 10;                       // pre-gain level (dB)
    c.r = 8;                        // compress ratio
    c.th = -10;                     // threshold level (dB)
    c.k = 2;                        // knee-width (dB)
    c.att = 0;                      // attack (us)
    c.rlt = 120;                    // release (ms)
    c.m = 0;                        // makeup gain level (dB)
    c.c = -0.1;                     // clipping level (dB)
    c.d = 0;                        // distortion ratio
#endif


#ifdef OUTPUT_STEREO
    // Creates a stereo compressor sharing the configuration
    comp_t cL, cR;
    comp_init_stereo(&c, &cL, &cR);
#endif
#ifdef BYPASS_L
    comp_t cR;
    comp_init_mono(&c, &cR);
#endif
#ifdef BYPASS_R
    comp_t cL;
    comp_init_mono(&c, &cL);
#endif

    LED0_off();
    LED1_off();
    LED2_off();
    LED3_off();

    //int32_t _r = c.r;

    for(;;){

        if(buffer_offset == BUFFER_OFFSET_HALF){
            DWT_Reset();
            for(i = 0; i < (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE / 2); i++){
#ifdef INPUT_STEREO                                 // L -> L, R -> R:
                    TxBuffer[i] = FUN_L(i);
                    i++;
                    TxBuffer[i] = FUN_R(i);
#endif
#ifdef INPUT_MONO_L                                 // L -> L, L -> R:
                    TxBuffer[i] = FUN_L(i);
                    TxBuffer[i+1] = FUN_R(i);
                    i++;
#endif
#ifdef INPUT_MONO_R                                 // R -> L, R -> R:
                    TxBuffer[i] = FUN_L(i+1);
                    TxBuffer[i+1] = FUN_R(i+1);
                    i++;
#endif
#ifdef INPUT_CROSS                                  // L -> R, R -> L:
                    TxBuffer[i] = FUN_L(i+1);
                    TxBuffer[i+1] = FUN_R(i);
                    i++;
#endif
            }
            buffer_offset = BUFFER_OFFSET_NONE;
        }

        if(buffer_offset == BUFFER_OFFSET_FULL){
            DWT_Reset();
            for(i = (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE / 2); i < WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++){
#ifdef INPUT_STEREO                                 // L -> L, R -> R:
                    TxBuffer[i] = FUN_L(i);
                    i++;
                    TxBuffer[i] = FUN_R(i);
#endif
#ifdef INPUT_MONO_L                                 // L -> L, L -> R:
                    TxBuffer[i] = FUN_L(i);
                    TxBuffer[i+1] = FUN_R(i);
                    i++;
#endif
#ifdef INPUT_MONO_R                                 // R -> L, R -> R:
                    TxBuffer[i] = FUN_L(i+1);
                    TxBuffer[i+1] = FUN_R(i+1);
                    i++;
#endif
#ifdef INPUT_CROSS                                  // L -> R, R -> L:
                    TxBuffer[i] = FUN_L(i+1);
                    TxBuffer[i+1] = FUN_R(i);
                    i++;
#endif
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
