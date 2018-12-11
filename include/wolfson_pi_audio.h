#ifndef WOLFSON_PI_AUDIO_H_
#define WOLFSON_PI_AUDIO_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <wm5102/wm5102.h>

/* Size of buffer (Multiple of 4096, RAM_BUFFER_SIZE used in BSP) */
#define WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE           				0x1000

#define WOLFSON_PI_AUDIO_LED_PIN                            		GPIO_PIN_10
#define WOLFSON_PI_AUDIO_LED_CLK_ENABLE()                   		__GPIOE_CLK_ENABLE()
#define WOLFSON_PI_AUDIO_LED_GPIO_PORT                        		GPIOE

#define WOLFSON_PI_AUDIO_POWER_PIN                            		GPIO_PIN_10
#define WOLFSON_PI_AUDIO_POWER_CLK_ENABLE()                   		__GPIOE_CLK_ENABLE()
#define WOLFSON_PI_AUDIO_POWER_GPIO_PORT                        	GPIOE
#define WOLFSON_PI_AUDIO_RESET_PIN                            		GPIO_PIN_11
#define WOLFSON_PI_AUDIO_RESET_CLK_ENABLE()                   		__GPIOE_CLK_ENABLE()
#define WOLFSON_PI_AUDIO_RESET_GPIO_PORT                        	GPIOE


#define WOLFSON_PI_AUDIO_I2Cx_SPEED                            		100000
#define WOLFSON_PI_AUDIO_I2Cx_ADDR                            		0x34
#define WOLFSON_PI_AUDIO_OK                        					0
#define WOLFSON_PI_AUDIO_ERROR                     					1
#define WOLFSON_PI_AUDIO_TIMEOUT                   					2

typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

/* Select the interrupt preemption priority and subpriority for the DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO           5   /* Select the preemption priority level(0 is the highest) */
/* Select the interrupt preemption priority and subpriority for the IT/DMA interrupt */
#define AUDIO_IN_IRQ_PREPRIO            6   /* Select the preemption priority level(0 is the highest) */

#define AUDIODATA_SIZE                  2   /* 16-bits audio data size */

/* Audio status definition */
#define AUDIO_OK                        0
#define AUDIO_ERROR                     1
#define AUDIO_TIMEOUT                   2


#if (AUDIO_CONTROL_INTERFACE == 1)
    /* I2C peripheral configuration defines (CIF1 - control interface of the audio codec) */
    #define WOLFSON_PI_AUDIO_I2Cx                                    	I2C2
    #define WOLFSON_PI_AUDIO_I2Cx_CLK_ENABLE()                       	__I2C2_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()          	__GPIOB_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_I2Cx_SCL_SDA_AF                         	GPIO_AF4_I2C2
    #define WOLFSON_PI_AUDIO_I2Cx_SCL_SDA_GPIO_PORT                  	GPIOB
    #define WOLFSON_PI_AUDIO_I2Cx_SCL_PIN                            	GPIO_PIN_10
    #define WOLFSON_PI_AUDIO_I2Cx_SDA_PIN                            	GPIO_PIN_11

    #define WOLFSON_PI_AUDIO_I2Cx_FORCE_RESET()                      	__I2C2_FORCE_RESET()
    #define WOLFSON_PI_AUDIO_I2Cx_RELEASE_RESET()                    	__I2C2_RELEASE_RESET()
	#define WOLFSON_PI_AUDIO_I2Cx_TIMEOUT								0x1000
#elif (AUDIO_CONTROL_INTERFACE == 2)
    /* SPI peripheral configuration defines (CIF2 - control interface of the audio codec) */
    #define WOLFSON_PI_AUDIO_SPIx                                  		SPI2
    #define WOLFSON_PI_AUDIO_SPIx_CLK_ENABLE()                     		__SPI2_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_SPIx_GPIO_PORT                        		GPIOB
    #define WOLFSON_PI_AUDIO_SPIx_AF                               		GPIO_AF5_SPI2
    #define WOLFSON_PI_AUDIO_SPIx_GPIO_CLK_ENABLE()                		__GPIOB_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_SPIx_GPIO_CLK_DISABLE()               		__GPIOB_CLK_DISABLE()
    #define WOLFSON_PI_AUDIO_SPIx_SS_PIN                           		GPIO_PIN_12
    #define WOLFSON_PI_AUDIO_SPIx_SCK_PIN                          		GPIO_PIN_13
    #define WOLFSON_PI_AUDIO_SPIx_MISO_PIN                         		GPIO_PIN_14
    #define WOLFSON_PI_AUDIO_SPIx_MOSI_PIN                         		GPIO_PIN_15
    #define WOLFSON_PI_AUDIO_SPIx_FORCE_RESET()                     	__SPI2_FORCE_RESET()
    #define WOLFSON_PI_AUDIO_SPIx_RELEASE_RESET()                   	__SPI2_RELEASE_RESET()
	#define WOLFSON_PI_AUDIO_SPIx_TIMEOUT						   		0x1000
	#define WOLFSON_PI_AUDIO_SPIx_OP_READ(addr)             			(addr | (1 << 31))
	#define WOLFSON_PI_AUDIO_SPIx_OP_WRITE(addr)               			(addr & ~(1 << 31))
#endif

#if (AUDIO_DATA_INTERFACE == 1)
    /* I2S peripheral configuration defines */
    #define WOLFSON_PI_AUDIO_I2Sx                                     	SPI3
    #define WOLFSON_PI_AUDIO_I2Sxext                                  	I2S3ext
    #define WOLFSON_PI_AUDIO_I2Sx_IRQn                            	  	SPI3_IRQn
    #define WOLFSON_PI_AUDIO_I2Sx_IRQ_Handler                         	SPI3_IRQHandler
    #define WOLFSON_PI_AUDIO_I2Sx_CLK_ENABLE()                        	__SPI3_CLK_ENABLE();
	#define WOLFSON_PI_AUDIO_I2Sx_TIMEOUT								0x1000

    #define WOLFSON_PI_AUDIO_I2Sx_WS_PIN                            	GPIO_PIN_4
    #define WOLFSON_PI_AUDIO_I2Sx_WS_AF                           		GPIO_AF6_SPI3
    #define WOLFSON_PI_AUDIO_I2Sx_WS_CLK_ENABLE()                   	__GPIOA_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_I2Sx_WS_GPIO_PORT                    		GPIOA

    #define WOLFSON_PI_AUDIO_I2Sx_SCK_PIN                            	GPIO_PIN_10
    #define WOLFSON_PI_AUDIO_I2Sx_SCK_AF                           		GPIO_AF6_SPI3
    #define WOLFSON_PI_AUDIO_I2Sx_SCK_CLK_ENABLE()                   	__GPIOC_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_I2Sx_SCK_GPIO_PORT                   		GPIOC

    #define WOLFSON_PI_AUDIO_I2Sx_SD_PIN                            	GPIO_PIN_12
    #define WOLFSON_PI_AUDIO_I2Sx_SD_AF                           		GPIO_AF6_SPI3
    #define WOLFSON_PI_AUDIO_I2Sx_SD_CLK_ENABLE()                   	__GPIOC_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_I2Sx_SD_GPIO_PORT                    		GPIOC

    #define WOLFSON_PI_AUDIO_I2Sxext_SD_PIN                        		GPIO_PIN_11
    #define WOLFSON_PI_AUDIO_I2Sxext_SD_AF                           	GPIO_AF5_I2S3ext
    #define WOLFSON_PI_AUDIO_I2Sxext_SD_CLK_ENABLE()               		__GPIOC_CLK_ENABLE()
	#define WOLFSON_PI_AUDIO_I2Sxext_SD_GPIO_PORT                   	 GPIOC

	#define WOLFSON_PI_AUDIO_I2Sx_DMAx_CLK_ENABLE()          			__DMA1_CLK_ENABLE()

	/* I2S DMA RX Stream definitions */
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_STREAM                		DMA1_Stream0
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_CHANNEL               		DMA_CHANNEL_0
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_IRQ                   		DMA1_Stream0_IRQn
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_PERIPH_DATA_SIZE      		DMA_PDATAALIGN_HALFWORD
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_MEM_DATA_SIZE					DMA_MDATAALIGN_HALFWORD
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMA_MAX_SZE                     	0xFFFF

	#define WOLFSON_PI_AUDIO_I2Sx_RX_IRQHandler                 		DMA1_Stream0_IRQHandler

	/* I2S DMA TX Stream definitions */
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_STREAM                		DMA1_Stream5
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_CHANNEL               		DMA_CHANNEL_2
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_IRQ                   		DMA1_Stream5_IRQn
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_PERIPH_DATA_SIZE      		DMA_PDATAALIGN_HALFWORD
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_MEM_DATA_SIZE					DMA_MDATAALIGN_HALFWORD
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMA_MAX_SZE                     	0xFFFF

	#define WOLFSON_PI_AUDIO_I2Sx_TX_IRQHandler                 		DMA1_Stream5_IRQHandler


	#define DMA_MAX_SIZE                     							0x7000
	#define DMA_MAX(_X_)                								(((_X_) <= DMA_MAX_SIZE)? (_X_):DMA_MAX_SIZE)
#elif (AUDIO_DATA_INTERFACE == 3)
        /* I2S peripheral configuration defines */
    #define WOLFSON_PI_AUDIO_I2Sx                                     	SPI2
    #define WOLFSON_PI_AUDIO_I2Sx_IRQn                            		SPI2_IRQn
    #define WOLFSON_PI_AUDIO_I2Sx_IRQ_Handler                         	SPI2_IRQHandler
    #define WOLFSON_PI_AUDIO_I2Sx_CLK_ENABLE()                       	 __SPI2_CLK_ENABLE()
	#define WOLFSON_PI_AUDIO_I2Sx_TIMEOUT								0x1000

    #define WOLFSON_PI_AUDIO_I2Sx_WS_PIN                            	GPIO_PIN_12
    #define WOLFSON_PI_AUDIO_I2Sx_WS_AF                           		GPIO_AF5_SPI2
    #define WOLFSON_PI_AUDIO_I2Sx_WS_CLK_ENABLE()                   	__GPIOB_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_I2Sx_WS_GPIO_PORT                    		GPIOB

    #define WOLFSON_PI_AUDIO_I2Sx_SCK_PIN                            	GPIO_PIN_13
    #define WOLFSON_PI_AUDIO_I2Sx_SCK_AF                           		GPIO_AF5_SPI2
    #define WOLFSON_PI_AUDIO_I2Sx_SCK_CLK_ENABLE()                   	__GPIOB_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_I2Sx_SCK_GPIO_PORT                    		GPIOB

    #define WOLFSON_PI_AUDIO_I2Sx_SD_PIN                            	GPIO_PIN_15
    #define WOLFSON_PI_AUDIO_I2Sx_SD_AF                           		GPIO_AF5_SPI2
    #define WOLFSON_PI_AUDIO_I2Sx_SD_CLK_ENABLE()                   	__GPIOB_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_I2Sx_SD_GPIO_PORT                    		GPIOB

    #define WOLFSON_PI_AUDIO_I2Sxext_SD_PIN                        		GPIO_PIN_14
    #define WOLFSON_PI_AUDIO_I2Sxext_SD_AF                           	GPIO_AF6_I2S2ext
    #define WOLFSON_PI_AUDIO_I2Sxext_SD_CLK_ENABLE()               		__GPIOB_CLK_ENABLE()
    #define WOLFSON_PI_AUDIO_I2Sxext_SD_GPIO_PORT                    	GPIOB

	/* I2S DMA RX Stream definitions */
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_CLK_ENABLE()          		__DMA1_CLK_ENABLE()
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_STREAM                		DMA1_Stream3
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_CHANNEL               		DMA_CHANNEL_0
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_IRQ                   		DMA1_Stream3_IRQn
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_PERIPH_DATA_SIZE      		DMA_PDATAALIGN_HALFWORD
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_MEM_DATA_SIZE					DMA_MDATAALIGN_HALFWORD
	#define WOLFSON_PI_AUDIO_I2Sx_RX_DMA_MAX_SZE                     	0xFFFF

	#define WOLFSON_PI_AUDIO_I2Sx_RX_IRQHandler                 		DMA1_Stream3_IRQHandler

	/* I2S DMA TX Stream definitions */
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_CLK_ENABLE()          		__DMA1_CLK_ENABLE()
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_STREAM                		DMA1_Stream4
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_CHANNEL               		DMA_CHANNEL_2
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_IRQ                   		DMA1_Stream4_IRQn
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_PERIPH_DATA_SIZE      		DMA_PDATAALIGN_HALFWORD
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_MEM_DATA_SIZE					DMA_MDATAALIGN_HALFWORD
	#define WOLFSON_PI_AUDIO_I2Sx_TX_DMA_MAX_SZE                     	0xFFFF

	#define WOLFSON_PI_AUDIO_I2Sx_TX_IRQHandler                 		DMA1_Stream4_IRQHandler
#endif


/* High Layer functions */
uint32_t WOLFSON_PI_AUDIO_Init(uint16_t OutputInputDevice, uint8_t Volume, uint32_t AudioFreq);
uint32_t WOLFSON_PI_AUDIO_ReadID();
uint32_t WOLFSON_PI_AUDIO_Play(int16_t* pTxData,  int16_t* pRxData, uint16_t Size);
uint32_t WOLFSON_PI_AUDIO_ChangeBuffer(int16_t* pTxData,  int16_t* pRxData, uint16_t Size);
uint32_t WOLFSON_PI_AUDIO_Pause();
uint32_t WOLFSON_PI_AUDIO_Resume();
uint32_t WOLFSON_PI_AUDIO_Stop(uint32_t Option);
uint32_t WOLFSON_PI_AUDIO_SetVolume(uint8_t Volume);
uint32_t WOLFSON_PI_AUDIO_SetMute(uint32_t Cmd);
void WOLFSON_PI_AUDIO_SetFilter(WM5102_FilterConfigModesDef ModeL, WM5102_FilterConfigModesDef ModeR, uint16_t CoeffL, uint16_t CoeffR);
void WOLFSON_PI_AUDIO_SetEqualizer(WM5102_EqualizerConfigModesDef ModeL, WM5102_EqualizerConfigModesDef ModeR,
		int8_t *GainL, int8_t * GainR, uint16_t * CoeffL, uint16_t * CoeffR);
uint32_t WOLFSON_PI_AUDIO_SetOutputMode(uint8_t Output);
uint32_t WOLFSON_PI_AUDIO_SetInputMode(uint8_t Input);
uint32_t WOLFSON_PI_AUDIO_SetFrequency(uint32_t AudioFreq);
uint32_t WOLFSON_PI_AUDIO_Reset();

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function is called when the requested data has been completely transferred. */
void    WOLFSON_PI_AUDIO_TransferComplete_CallBack(void);

/* This function is called when half of the requested buffer has been transferred. */
void    WOLFSON_PI_AUDIO_HalfTransfer_CallBack(void);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void    WOLFSON_PI_AUDIO_Error_CallBack(void);

#endif /* WOLFSON_PI_AUDIO_H_ */
