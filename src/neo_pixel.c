#include "neo_pixel.h"

#include "hal_include.h"

DMA_HandleTypeDef hdma_tim3_ch3;
TIM_HandleTypeDef htim3;

#define PWM_HI (38)
#define PWM_LO (19)

#define NUM_BPP (3)
#define NUM_PIXELS (2)
#define NUM_BYTES (NUM_BPP * NUM_PIXELS)

// LED color buffer
uint8_t rgb_arr[NUM_BYTES] = {0};

// LED write buffer
#define WR_BUF_LEN (NUM_BPP * 8 * 2)
uint8_t wr_buf[WR_BUF_LEN] = {0};
uint_fast8_t wr_buf_p = 0;

// Shuttle the data to the LEDs!
void neo_pixel_render(led_data_t *leds)
{
    if (hdma_tim3_ch3.State == HAL_DMA_STATE_BUSY)
        return;

    rgb_arr[0] = leds->led_state[led_rx].state ? 0x0F : 0x00;
    rgb_arr[1] = leds->led_state[led_tx].state ? 0x0F : 0x00;
    rgb_arr[2] = leds->led_state[led_st].state ? 0x0F : 0x00;
    rgb_arr[3] = rgb_arr[0];
    rgb_arr[4] = rgb_arr[1];
    rgb_arr[5] = rgb_arr[2];

    if(wr_buf_p != 0 || hdma_tim3_ch3.State != HAL_DMA_STATE_READY) {
        // Ongoing transfer, cancel!
        for(uint8_t i = 0; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
        wr_buf_p = 0;
        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
        return;
    }
    // Ooh boi the first data buffer half (and the second!)

    for(uint_fast8_t i = 0; i < 8; ++i) {
        wr_buf[i     ] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
        wr_buf[i +  8] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
        wr_buf[i + 16] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
        wr_buf[i + 24] = PWM_LO << (((rgb_arr[3] << i) & 0x80) > 0);
        wr_buf[i + 32] = PWM_LO << (((rgb_arr[4] << i) & 0x80) > 0);
        wr_buf[i + 40] = PWM_LO << (((rgb_arr[5] << i) & 0x80) > 0);
    }

    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)wr_buf, WR_BUF_LEN);
    wr_buf_p = 2; // Since we're ready for the next buffer
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if(htim != &htim3)
        return;

    // DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
    if(wr_buf_p < NUM_PIXELS) {
        // We're in. Fill the even buffer

        for(uint_fast8_t i = 0; i < 8; ++i) {
            wr_buf[i     ] = PWM_LO << (((rgb_arr[3 * wr_buf_p    ] << i) & 0x80) > 0);
            wr_buf[i +  8] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 1] << i) & 0x80) > 0);
            wr_buf[i + 16] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 2] << i) & 0x80) > 0);
        }
        wr_buf_p++;
    } else if (wr_buf_p < NUM_PIXELS + 2) {
        // Last two transfers are resets. SK6812: 64 * 1.25 us = 80 us == good enough reset
        // First half reset zero fill
        for(uint8_t i = 0; i < WR_BUF_LEN / 2; ++i) wr_buf[i] = 0;
        wr_buf_p++;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim != &htim3)
        return;

    // DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
    if(wr_buf_p < NUM_PIXELS) {
        // We're in. Fill the odd buffer

        for(uint_fast8_t i = 0; i < 8; ++i) {
            wr_buf[i + 24] = PWM_LO << (((rgb_arr[3 * wr_buf_p    ] << i) & 0x80) > 0);
            wr_buf[i + 32] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 1] << i) & 0x80) > 0);
            wr_buf[i + 40] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 2] << i) & 0x80) > 0);
        }
        wr_buf_p++;
    } else if (wr_buf_p < NUM_PIXELS + 2) {
        // Second half reset zero fill
        for(uint8_t i = WR_BUF_LEN / 2; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
        ++wr_buf_p;
    } else {
        // We're done. Lean back and until next time!
        wr_buf_p = 0;
        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
    }
}

void pretty_delay(led_data_t *leds, int delay_ms)
{
    const int states[] = {
        0x01,
        0x03,
        0x02,
        0x06,
        0x04,
        0x05,
        0x00
    };

    const int delay = 50;
    const int cycles = delay_ms / (6*delay);

    for(int i = 0; i < cycles; i++)
    {
        for(const int *s = states; *s; s++)
        {
            leds->led_state[0].state = !!(*s & 1);
            leds->led_state[1].state = !!(*s & 2);
            leds->led_state[2].state = !!(*s & 4);

            neo_pixel_render(leds);
            HAL_Delay(delay);
        }
    }

    led_set_mode(leds, led_mode_off);
}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
#ifdef STM32F4
		hdma_tim3_ch3.Instance = DMA2_Stream5; // just shutup compiler, this is not tested
#else
        hdma_tim3_ch3.Instance = DMA1_Channel2;
#endif
        hdma_tim3_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_tim3_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_tim3_ch3.Init.MemInc = DMA_MINC_ENABLE;
        hdma_tim3_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_tim3_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_tim3_ch3.Init.Mode = DMA_CIRCULAR;
        hdma_tim3_ch3.Init.Priority = DMA_PRIORITY_HIGH;
        HAL_DMA_Init(&hdma_tim3_ch3);

        __HAL_LINKDMA(htim_base, hdma[TIM_DMA_ID_CC3], hdma_tim3_ch3);

        HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM3)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
#ifdef STM32F4
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // just shutup compiler, this is not tested
#else
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
#endif

        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_DISABLE();
        HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC3]);
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
    }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void neo_pixel_init(void)
{
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 59;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim3);

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

    HAL_TIM_PWM_Init(&htim3);

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

    HAL_TIM_MspPostInit(&htim3);
}

