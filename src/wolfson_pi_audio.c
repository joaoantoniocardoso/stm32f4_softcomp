#include <wolfson_pi_audio.h>


///* These PLL parameters are valid when the f(VCO clock) = 1Mhz */
const uint32_t I2SFreq[8] = {8000, 16000, 32000, 48000, 96000};
const uint32_t I2SPLLN[8] = {192, 192, 256, 192, 384};
const uint32_t I2SPLLR[8] = {2, 3, 2, 5, 5};


/* AUDIO IO functions */
void    EXTERNAL_AUDIO_IO_Init(void);
void 	EXTERNAL_AUDIO_IO_Write (uint8_t Addr, uint32_t Reg, uint16_t Value);
uint16_t EXTERNAL_AUDIO_IO_Read(uint8_t Addr, uint32_t Reg);
void    EXTERNAL_AUDIO_IO_Delay(uint32_t Delay);

static void I2Sx_Msp_Init();
static void I2Sx_Init();

#if (AUDIO_CONTROL_INTERFACE == 1)
static void I2Cx_Msp_Init();
static void I2Cx_Init();
static void I2Cx_WriteData(uint8_t Addr, uint32_t Reg, uint16_t Value);
static uint16_t  I2Cx_ReadData(uint8_t Addr, uint8_t Reg);
static void I2Cx_Error(uint8_t Addr);
#else
static void SPIx_Msp_Init();
static void SPIx_Init();
static void SPIx_WriteData(uint32_t Reg, uint16_t Value);
static uint16_t  SPIx_ReadData(uint32_t Reg);
static void SPIx_Error();
#endif


static AUDIO_DrvTypeDef           *pAudioDrv = NULL;

SPI_HandleTypeDef WOLFSON_PI_AUDIO_SPIx_HandleStructure;
I2C_HandleTypeDef WOLFSON_PI_AUDIO_I2Cx_HandleStructure;
I2S_HandleTypeDef WOLFSON_PI_AUDIO_I2Sx_HandleStructure;
DMA_HandleTypeDef WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure;
DMA_HandleTypeDef WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure;


uint32_t CurrentAudioFreq;

static void I2Sx_Msp_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    WOLFSON_PI_AUDIO_I2Sx_WS_CLK_ENABLE();
    WOLFSON_PI_AUDIO_I2Sx_SCK_CLK_ENABLE();
    WOLFSON_PI_AUDIO_I2Sx_SD_CLK_ENABLE();
    WOLFSON_PI_AUDIO_I2Sxext_SD_CLK_ENABLE();

    // set up GPIO initialization structure for I2S pins WS, CK, SD and SD
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = WOLFSON_PI_AUDIO_I2Sx_WS_PIN;
    GPIO_InitStructure.Alternate = WOLFSON_PI_AUDIO_I2Sx_WS_AF;
    HAL_GPIO_Init(WOLFSON_PI_AUDIO_I2Sx_WS_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = WOLFSON_PI_AUDIO_I2Sx_SCK_PIN;
    GPIO_InitStructure.Alternate = WOLFSON_PI_AUDIO_I2Sx_SCK_AF;
    HAL_GPIO_Init(WOLFSON_PI_AUDIO_I2Sx_SCK_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = WOLFSON_PI_AUDIO_I2Sx_SD_PIN;
    GPIO_InitStructure.Alternate = WOLFSON_PI_AUDIO_I2Sx_SD_AF;
    HAL_GPIO_Init(WOLFSON_PI_AUDIO_I2Sx_SD_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = WOLFSON_PI_AUDIO_I2Sxext_SD_PIN;
    GPIO_InitStructure.Alternate = WOLFSON_PI_AUDIO_I2Sxext_SD_AF;
    HAL_GPIO_Init(WOLFSON_PI_AUDIO_I2Sxext_SD_GPIO_PORT, &GPIO_InitStructure);


    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    // I2S clock on
    WOLFSON_PI_AUDIO_I2Sx_CLK_ENABLE();

    /* Enable the DMA clock */
    WOLFSON_PI_AUDIO_I2Sx_DMAx_CLK_ENABLE();

	/* Configure the DMA RX handle parameters */
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.Channel             = WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_CHANNEL;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.PeriphInc           = DMA_PINC_DISABLE;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.MemInc              = DMA_MINC_ENABLE;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.PeriphDataAlignment = WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_PERIPH_DATA_SIZE;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.MemDataAlignment    = WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_MEM_DATA_SIZE;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.Mode                = DMA_CIRCULAR;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.Priority            = DMA_PRIORITY_HIGH;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.MemBurst            = DMA_MBURST_SINGLE;
    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Init.PeriphBurst         = DMA_MBURST_SINGLE;

    WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure.Instance = WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_STREAM;

    /* Configure the DMA TX handle parameters */
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.Channel             = WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_CHANNEL;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.PeriphInc           = DMA_PINC_DISABLE;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.MemInc              = DMA_MINC_ENABLE;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.PeriphDataAlignment = WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_PERIPH_DATA_SIZE;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.MemDataAlignment    = WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_MEM_DATA_SIZE;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.Mode                = DMA_CIRCULAR;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.Priority            = DMA_PRIORITY_HIGH;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.MemBurst            = DMA_MBURST_SINGLE;
	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Init.PeriphBurst         = DMA_PBURST_SINGLE;

	WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure.Instance                 = WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_STREAM;


	/* Deinitialize the Stream for new transfer */
	HAL_DMA_DeInit(&WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure);
    HAL_DMA_DeInit(&WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure);

	/* Configure the DMA Stream */
	HAL_DMA_Init(&WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure);
    HAL_DMA_Init(&WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure);

	/* Associate the DMA handle */
	__HAL_LINKDMA((&WOLFSON_PI_AUDIO_I2Sx_HandleStructure), hdmarx, WOLFSON_PI_AUDIO_I2Sx_DMAx_RX_HandleStructure);
    __HAL_LINKDMA((&WOLFSON_PI_AUDIO_I2Sx_HandleStructure), hdmatx, WOLFSON_PI_AUDIO_I2Sx_DMAx_TX_HandleStructure);

    /* I2S DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
    HAL_NVIC_SetPriority(WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
    HAL_NVIC_EnableIRQ(WOLFSON_PI_AUDIO_I2Sx_RX_DMAx_IRQ);
    HAL_NVIC_EnableIRQ(WOLFSON_PI_AUDIO_I2Sx_TX_DMAx_IRQ);


}

static void I2Sx_Init()
{
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Instance = WOLFSON_PI_AUDIO_I2Sx;

    __HAL_I2S_DISABLE(&WOLFSON_PI_AUDIO_I2Sx_HandleStructure);

    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.Standard = I2S_STANDARD_PHILIPS;
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.DataFormat = I2S_DATAFORMAT_16B;
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.ClockSource = I2S_CLOCK_PLL;

#if (AUDIO_DATA_MODE == MODE_MASTER)
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.AudioFreq = CurrentAudioFreq;
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.CPOL = I2S_CPOL_HIGH;
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.Mode = I2S_MODE_SLAVE_RX;
#else
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.AudioFreq = CurrentAudioFreq;
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.CPOL = I2S_CPOL_HIGH;
    WOLFSON_PI_AUDIO_I2Sx_HandleStructure.Init.Mode = I2S_MODE_MASTER_RX;
#endif

    /* Initialize the I2S peripheral with the structure above */
    if(HAL_I2S_GetState(&WOLFSON_PI_AUDIO_I2Sx_HandleStructure) == HAL_I2S_STATE_RESET)
    {
    	I2Sx_Msp_Init();
    }

    HAL_I2S_Init(&WOLFSON_PI_AUDIO_I2Sx_HandleStructure);

    HAL_Delay(5);
}

#if (AUDIO_CONTROL_INTERFACE == 1)

static void I2Cx_Msp_Init()
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable I2C GPIO clocks */
  WOLFSON_PI_AUDIO_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

  /* I2Cx SCL and SDA pins configuration ---------------------------*/
  GPIO_InitStruct.Pin = WOLFSON_PI_AUDIO_I2Cx_SCL_PIN | WOLFSON_PI_AUDIO_I2Cx_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Alternate  = WOLFSON_PI_AUDIO_I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(WOLFSON_PI_AUDIO_I2Sx_SCK_GPIO_PORT, &GPIO_InitStruct);

  /* Enable the I2Cx peripheral clock */
  WOLFSON_PI_AUDIO_I2Cx_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  WOLFSON_PI_AUDIO_I2Cx_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  WOLFSON_PI_AUDIO_I2Cx_RELEASE_RESET();
}

static void I2Cx_Init()
{
	if(HAL_I2C_GetState(&WOLFSON_PI_AUDIO_I2Cx_HandleStructure) == HAL_I2C_STATE_RESET)
	{
		/* DISCOVERY_I2Cx peripheral configuration */
		WOLFSON_PI_AUDIO_I2Cx_HandleStructure.Instance = WOLFSON_PI_AUDIO_I2Cx;
		WOLFSON_PI_AUDIO_I2Cx_HandleStructure.Init.ClockSpeed = WOLFSON_PI_AUDIO_I2Cx_SPEED;
		WOLFSON_PI_AUDIO_I2Cx_HandleStructure.Init.DutyCycle = I2C_DUTYCYCLE_2;
		WOLFSON_PI_AUDIO_I2Cx_HandleStructure.Init.OwnAddress1 = 0x33;
		WOLFSON_PI_AUDIO_I2Cx_HandleStructure.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

		/* Init the I2C */
		I2Cx_Msp_Init();

		HAL_I2C_Init(&WOLFSON_PI_AUDIO_I2Cx_HandleStructure);
	}
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written
  * @retval HAL status
  */
static void I2Cx_WriteData(uint8_t Addr, uint32_t Reg, uint16_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t Data[6];

	Data[0] = (Reg>>24) & 0xFF;
	Data[1] = (Reg>>16) & 0xFF;
	Data[2] = (Reg>>8) & 0xFF;
	Data[3] = (Reg) & 0xFF;
	Data[4] = (Value>>8) & 0xFF;
	Data[5] = (Value) & 0xFF;

	status = HAL_I2C_Master_Transmit(&WOLFSON_PI_AUDIO_I2Cx_HandleStructure, WOLFSON_PI_AUDIO_I2Cx_ADDR, Data, 6, WOLFSON_PI_AUDIO_I2Cx_TIMEOUT);

	if(status != HAL_OK)
		I2Cx_Error(Addr);
}

/**
  * @brief  Read a register of the device through BUS
  * @param  Addr: Device address on BUS
  * @param  Reg: The target register address to read
  * @retval HAL status
  */
static uint16_t  I2Cx_ReadData(uint8_t Addr, uint8_t Reg)
{
	uint8_t Value = 0;

	return Value;
}

static void I2Cx_Error(uint8_t Addr)
{
	(void)Addr;

	/* De-initialize the I2C communication bus */
	HAL_I2C_DeInit(&WOLFSON_PI_AUDIO_I2Cx_HandleStructure);

	/* Re-Initialize the I2C communication bus */
	I2Cx_Init();
}


#else

static void SPIx_Msp_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    WOLFSON_PI_AUDIO_SPIx_GPIO_CLK_ENABLE();

    // set up GPIO initialization structure for SPI pins SS, SCK, MOSI, MISO
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = WOLFSON_PI_AUDIO_SPIx_SS_PIN;
    HAL_GPIO_Init(WOLFSON_PI_AUDIO_SPIx_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pin = WOLFSON_PI_AUDIO_SPIx_SCK_PIN;
    GPIO_InitStructure.Alternate = WOLFSON_PI_AUDIO_SPIx_AF;
    HAL_GPIO_Init(WOLFSON_PI_AUDIO_SPIx_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = WOLFSON_PI_AUDIO_SPIx_MOSI_PIN;
    GPIO_InitStructure.Alternate = WOLFSON_PI_AUDIO_SPIx_AF;
    HAL_GPIO_Init(WOLFSON_PI_AUDIO_SPIx_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = WOLFSON_PI_AUDIO_SPIx_MISO_PIN;
    GPIO_InitStructure.Alternate = WOLFSON_PI_AUDIO_SPIx_AF;
    HAL_GPIO_Init(WOLFSON_PI_AUDIO_SPIx_GPIO_PORT, &GPIO_InitStructure);

    //SS = high
    HAL_GPIO_WritePin(WOLFSON_PI_AUDIO_SPIx_GPIO_PORT, WOLFSON_PI_AUDIO_SPIx_SS_PIN, GPIO_PIN_SET);

    WOLFSON_PI_AUDIO_SPIx_CLK_ENABLE();

    /* Force the SPI peripheral clock reset */
    //WOLFSON_PI_AUDIO_SPIx_FORCE_RESET();

    /* Release the SPI peripheral clock reset */
    //WOLFSON_PI_AUDIO_SPIx_RELEASE_RESET();

    //HAL_Delay(5);
}

static void SPIx_Init()
{
    SPIx_Msp_Init();

    // configure SPI
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Instance 					= WOLFSON_PI_AUDIO_SPIx;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.Direction 			= SPI_DIRECTION_2LINES;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.Mode 				= SPI_MODE_MASTER;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.DataSize 			= SPI_DATASIZE_16BIT;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.CLKPolarity 			= SPI_POLARITY_LOW;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.CLKPhase 			= SPI_PHASE_1EDGE;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_64;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.FirstBit 			= SPI_FIRSTBIT_MSB;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.CRCCalculation		= SPI_CRCCALCULATION_DISABLE;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.CRCPolynomial		= 7;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.NSS					= SPI_NSS_SOFT;
    WOLFSON_PI_AUDIO_SPIx_HandleStructure.Init.TIMode				= SPI_TIMODE_DISABLE;
    HAL_SPI_Init(&WOLFSON_PI_AUDIO_SPIx_HandleStructure);
}


/**
  * @brief  Write a value in a register of the device.
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written
  * @retval HAL status
  */
static void SPIx_WriteData(uint32_t Reg, uint16_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint16_t Data[4];

	Data[0] = (WOLFSON_PI_AUDIO_SPIx_OP_WRITE(Reg)>>16) & 0xFFFF;
	Data[1] = (Reg) & 0xFFFF;
	Data[2] = 0xFFFF;
	Data[3] = Value;

	HAL_GPIO_WritePin(WOLFSON_PI_AUDIO_SPIx_GPIO_PORT, WOLFSON_PI_AUDIO_SPIx_SS_PIN, GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&WOLFSON_PI_AUDIO_SPIx_HandleStructure, (uint8_t *) Data, 4, WOLFSON_PI_AUDIO_SPIx_TIMEOUT);

	HAL_GPIO_WritePin(WOLFSON_PI_AUDIO_SPIx_GPIO_PORT, WOLFSON_PI_AUDIO_SPIx_SS_PIN, GPIO_PIN_SET);

	if(status != HAL_OK)
		SPIx_Error();
}

/**
  * @brief  Read a register of the device
  * @param  Reg: The target register address to read
  * @retval HAL status
  */
static uint16_t  SPIx_ReadData(uint32_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint16_t TxData[4];
	uint16_t RxData[4];

	TxData[0] = (WOLFSON_PI_AUDIO_SPIx_OP_READ(Reg)>>16) & 0xFFFF;
	TxData[1] = (Reg) & 0xFFFF;
	TxData[2] = 0xFFFF;
	TxData[3] = 0x0000;

	HAL_GPIO_WritePin(WOLFSON_PI_AUDIO_SPIx_GPIO_PORT, WOLFSON_PI_AUDIO_SPIx_SS_PIN, GPIO_PIN_RESET);

	status = HAL_SPI_TransmitReceive(&WOLFSON_PI_AUDIO_SPIx_HandleStructure, (uint8_t *) TxData, (uint8_t *) RxData, 4, WOLFSON_PI_AUDIO_SPIx_TIMEOUT);

	HAL_GPIO_WritePin(WOLFSON_PI_AUDIO_SPIx_GPIO_PORT, WOLFSON_PI_AUDIO_SPIx_SS_PIN, GPIO_PIN_SET);

	if(status != HAL_OK)
		SPIx_Error();

	return RxData[3];
}

static void SPIx_Error()
{
	/* De-initialize the I2C communication bus */
	HAL_SPI_DeInit(&WOLFSON_PI_AUDIO_SPIx_HandleStructure);

	/* Re-Initialize the I2C communication bus */
	SPIx_Init();
}

#endif


/* High Layer functions */
uint32_t WOLFSON_PI_AUDIO_Init(uint16_t OutputInputDevice, uint8_t Volume, uint32_t AudioFreq)
{
	uint8_t ret = WOLFSON_PI_AUDIO_ERROR;
	uint32_t deviceid = 0x00;
	RCC_PeriphCLKInitTypeDef rccclkinit;
	uint8_t index = 0, freqindex = 0xFF;

	for(index = 0; index < 8; index++)
	{
		if(I2SFreq[index] == AudioFreq)
		{
		  freqindex = index;
		}
	}
	/* Enable PLLI2S clock */
	HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
	/* PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
	if (freqindex & 0x7)
		{
		/* I2S clock config
		PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLM)
		I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
		rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
		rccclkinit.PLLI2S.PLLI2SN = I2SPLLN[freqindex];
		rccclkinit.PLLI2S.PLLI2SR = I2SPLLR[freqindex];
		HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
	}
	else
	{
		/* I2S clock config
		PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLM)
		I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
		rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
		rccclkinit.PLLI2S.PLLI2SN = 258;
		rccclkinit.PLLI2S.PLLI2SR = 3;
		HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
	}

	CurrentAudioFreq = AudioFreq;

	deviceid = wm5102_drv.ReadID(WOLFSON_PI_AUDIO_I2Cx_ADDR);

	if(deviceid == WM5102_ID)
	{
		/* Initialize the audio driver structure */
		pAudioDrv = &wm5102_drv;
		ret = WOLFSON_PI_AUDIO_OK;
	}
	else
	{
		ret = WOLFSON_PI_AUDIO_ERROR;
	}

	if(ret == WOLFSON_PI_AUDIO_OK)
	{
		pAudioDrv->Init(WOLFSON_PI_AUDIO_I2Cx_ADDR, OutputInputDevice, Volume, AudioFreq);
	}

	return ret;
}

uint32_t WOLFSON_PI_AUDIO_ReadID()
{
	return pAudioDrv->ReadID(WOLFSON_PI_AUDIO_I2Cx_ADDR);
}

uint32_t WOLFSON_PI_AUDIO_Play(int16_t* pTxData,  int16_t* pRxData, uint16_t Size)
{
    HAL_I2SEx_TransmitReceive_DMA(&WOLFSON_PI_AUDIO_I2Sx_HandleStructure, (uint16_t* ) pTxData, (uint16_t* ) pRxData, DMA_MAX(Size));

    pAudioDrv->Play(WOLFSON_PI_AUDIO_I2Cx_ADDR, (uint16_t* ) pTxData, Size);

	return AUDIO_OK;
}

uint32_t WOLFSON_PI_AUDIO_Pause()
{
	uint32_t result;

	/* Call the Audio Codec Pause/Resume function */
	if(pAudioDrv->Pause(WOLFSON_PI_AUDIO_I2Cx_ADDR) != 0)
	{
		result = AUDIO_ERROR;
	}
	else
	{
		/* Call the Media layer pause function */
		HAL_I2S_DMAPause(&WOLFSON_PI_AUDIO_I2Sx_HandleStructure);

		/* Return AUDIO_OK when all operations are correctly done */
		result = AUDIO_OK;
	}

	return result;
}

uint32_t WOLFSON_PI_AUDIO_Resume()
{
	uint32_t result;

	/* Call the Audio Codec Pause/Resume function */
	if(pAudioDrv->Resume(WOLFSON_PI_AUDIO_I2Cx_ADDR) != 0)
	{
		result = AUDIO_ERROR;
	}
	else
	{
		/* Call the Media layer pause function */
		HAL_I2S_DMAResume(&WOLFSON_PI_AUDIO_I2Sx_HandleStructure);

		/* Return AUDIO_OK when all operations are correctly done */
		result = AUDIO_OK;
	}

	return result;
}

/**
  * @brief  Stops audio playing and Power down the Audio Codec.
  * @param  Option: could be one of the following parameters
  *           - CODEC_PDWN_HW: completely shut down the codec (physically).
  *                            Then need to reconfigure the Codec after power on.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */

uint32_t WOLFSON_PI_AUDIO_Stop(uint32_t Option)
{
	uint32_t result;

	/* Call DMA Stop to disable DMA stream before stopping codec */
	HAL_I2S_DMAStop(&WOLFSON_PI_AUDIO_I2Sx_HandleStructure);

	/* Call Audio Codec Stop function */
	if(pAudioDrv->Stop(WOLFSON_PI_AUDIO_I2Cx_ADDR, Option) != 0)
	{
		result = AUDIO_ERROR;
	}
	else
	{
		if(Option == CODEC_PDWN_HW)
		{
			/* Wait at least 1ms */
			HAL_Delay(1);

			/* Reset the pin */
			HAL_GPIO_WritePin(WOLFSON_PI_AUDIO_RESET_GPIO_PORT, WOLFSON_PI_AUDIO_RESET_PIN, GPIO_PIN_RESET);
		}

		/* Return AUDIO_OK when all operations are correctly done */
		result = AUDIO_OK;
	}

	return result;
}

uint32_t WOLFSON_PI_AUDIO_SetVolume(uint8_t Volume)
{
	return pAudioDrv->SetVolume(WOLFSON_PI_AUDIO_I2Cx_ADDR, Volume);
}

uint32_t WOLFSON_PI_AUDIO_SetMute(uint32_t Cmd)
{
	return pAudioDrv->SetMute(WOLFSON_PI_AUDIO_I2Cx_ADDR, Cmd);
}

void WOLFSON_PI_AUDIO_SetFilter(WM5102_FilterConfigModesDef ModeL, WM5102_FilterConfigModesDef ModeR, uint16_t CoeffL, uint16_t CoeffR)
{
	wm5102_SetFilter(WOLFSON_PI_AUDIO_I2Cx_ADDR, ModeL, ModeR, CoeffL, CoeffR);
}

void WOLFSON_PI_AUDIO_SetEqualizer(WM5102_EqualizerConfigModesDef ModeL, WM5102_EqualizerConfigModesDef ModeR,
		int8_t *GainL, int8_t * GainR, uint16_t * CoeffL, uint16_t * CoeffR)
{
	wm5102_SetEqualizer(WOLFSON_PI_AUDIO_I2Cx_ADDR, ModeL, ModeR, GainL, GainR, CoeffL, CoeffR);
}
uint32_t WOLFSON_PI_AUDIO_SetOutputMode(uint8_t Output)
{
	return pAudioDrv->SetOutputMode(WOLFSON_PI_AUDIO_I2Cx_ADDR, Output);
}

uint32_t WOLFSON_PI_AUDIO_SetInputMode(uint8_t Input)
{
    //wm5102_SetEqualizer(EQUALIZER_SHELVING_FILTER, EQUALIZER_SHELVING_FILTER, GainL, GainR, CoeffL, CoeffR);

	return pAudioDrv->SetInputMode(WOLFSON_PI_AUDIO_I2Cx_ADDR, Input);
}

uint32_t WOLFSON_PI_AUDIO_SetFrequency(uint32_t AudioFreq)
{
	RCC_PeriphCLKInitTypeDef rccclkinit;
	uint8_t index = 0, freqindex = 0xFF;

	for(index = 0; index < 8; index++)
	{
		if(I2SFreq[index] == AudioFreq)
		{
		  freqindex = index;
		}
	}
	/* Enable PLLI2S clock */
	HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
	/* PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
	if (freqindex & 0x7)
		{
		/* I2S clock config
		PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLM)
		I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
		rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
		rccclkinit.PLLI2S.PLLI2SN = I2SPLLN[freqindex];
		rccclkinit.PLLI2S.PLLI2SR = I2SPLLR[freqindex];
		HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
	}
	else
	{
		/* I2S clock config
		PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLM)
		I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
		rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
		rccclkinit.PLLI2S.PLLI2SN = 258;
		rccclkinit.PLLI2S.PLLI2SR = 3;
		HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
	}

	CurrentAudioFreq = AudioFreq;

	I2Sx_Init();

	return pAudioDrv->SetFrequency(WOLFSON_PI_AUDIO_I2Cx_ADDR, AudioFreq);
}

uint32_t WOLFSON_PI_AUDIO_Reset()
{
	return pAudioDrv->Reset(WOLFSON_PI_AUDIO_I2Cx_ADDR);
}


/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hi2s: I2S handle
  */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(hi2s->Instance == WOLFSON_PI_AUDIO_I2Sx)
  {
    /* Call the user function which will manage directly transfer complete */
    WOLFSON_PI_AUDIO_TransferComplete_CallBack();
  }
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hi2s: I2S handle
  */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(hi2s->Instance == WOLFSON_PI_AUDIO_I2Sx)
  {
    /* Manage the remaining file size and new address offset: This function should
       be coded by user (its prototype is already declared in stm32f4_discovery_audio.h) */
    WOLFSON_PI_AUDIO_HalfTransfer_CallBack();
  }
}

/**
  * @brief  I2S error callbacks.
  * @param  hi2s: I2S handle
  * @retval None
  */
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
  /* Manage the error generated on DMA FIFO: This function
     should be coded by user  */
  if(hi2s->Instance == WOLFSON_PI_AUDIO_I2Sx)
  {
    WOLFSON_PI_AUDIO_Error_CallBack();
  }
}



/**
  * @brief  Manages the DMA full Transfer complete event.
  */
__weak void WOLFSON_PI_AUDIO_TransferComplete_CallBack(void)
{
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  */
__weak void WOLFSON_PI_AUDIO_HalfTransfer_CallBack(void)
{
}

/**
  * @brief  Manages the DMA FIFO error event.
  */
__weak void WOLFSON_PI_AUDIO_Error_CallBack(void)
{
}

/**
  * @brief  Initializes Wolfson-Pi low level.
  * @param  None
  * @retval None
  */
void EXTERNAL_AUDIO_IO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable Reset GPIO Clock */
  WOLFSON_PI_AUDIO_RESET_CLK_ENABLE();

  /* Audio power and reset pin configuration */
  GPIO_InitStruct.Pin = WOLFSON_PI_AUDIO_POWER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(WOLFSON_PI_AUDIO_POWER_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = WOLFSON_PI_AUDIO_RESET_PIN;
  HAL_GPIO_Init(WOLFSON_PI_AUDIO_RESET_GPIO_PORT, &GPIO_InitStruct);
#if (AUDIO_CONTROL_INTERFACE == 1)
  I2Cx_Init();
#else
  SPIx_Init();
#endif

  /* Power on the codec */
  HAL_GPIO_WritePin(WOLFSON_PI_AUDIO_POWER_GPIO_PORT, WOLFSON_PI_AUDIO_POWER_PIN, GPIO_PIN_SET);

  /* Reset Codec */
  HAL_GPIO_WritePin(WOLFSON_PI_AUDIO_RESET_GPIO_PORT, WOLFSON_PI_AUDIO_RESET_PIN, GPIO_PIN_RESET);

  /* Wait for a delay to insure registers erasing */
  HAL_Delay(50);

  /* Release Reset */
  HAL_GPIO_WritePin(WOLFSON_PI_AUDIO_RESET_GPIO_PORT, WOLFSON_PI_AUDIO_RESET_PIN, GPIO_PIN_SET);

  /* Wait for a delay to insure registers erasing */
  HAL_Delay(5);

  I2Sx_Init();
}

/**
  * @brief  DeInitializes Audio low level.
  */
void EXTERNAL_AUDIO_IO_DeInit(void)
{

}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
void EXTERNAL_AUDIO_IO_Write (uint8_t Addr, uint32_t Reg, uint16_t Value)
{
#if (AUDIO_CONTROL_INTERFACE == 1)
	I2Cx_WriteData(Addr, Reg, Value);
#else
	UNUSED(Addr);
    SPIx_WriteData(Reg, Value);
#endif
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint16_t EXTERNAL_AUDIO_IO_Read(uint8_t Addr, uint32_t Reg)
{
    uint16_t Value = 0;

#if (AUDIO_CONTROL_INTERFACE == 1)
    Value = I2Cx_ReadData(Addr, Reg);
#else
	UNUSED(Addr);
    Value = SPIx_ReadData(Reg);
#endif

    return Value;
}

void EXTERNAL_AUDIO_IO_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}
