/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "config.h"
#include "neo_pixel.h"

#include "gpio.h"
#include "queue.h"
#include "gs_usb.h"
#include "can.h"
#include "led.h"
#include "dfu.h"
#include "timer.h"
#include "util.h"

#include "class/cdc/cdc_device.h"
#include "tusb_config.h"
#include "device/usbd.h"

void gs_usb_task(void);
void HAL_MspInit(void);
static void SystemClock_Config(void);
//static bool send_to_host_or_enqueue(struct gs_host_frame *frame);
//static void send_to_host(void);
static int run = 1;

can_data_t hCAN = {0};
led_data_t hLED = {0};

queue_t *q_frame_pool = NULL;
queue_t *q_from_host = NULL;
queue_t *q_to_host = NULL;

#ifdef PIN_DELAY_Pin
// Wait for DELAY to go active.
static void wait_for_the_go_signal(void)
{
    int cnt = 0;
    while(cnt < 10)
    {
        if(GPIO_PIN_SET == HAL_GPIO_ReadPin(PIN_DELAY_GPIO_Port, PIN_DELAY_Pin))
        {
            cnt++;
        }
        HAL_Delay(10);
    }
}
#endif

#ifdef PIN_READY_Pin
static void send_ready_signal(void)
{
#if (PIN_READY_Active_High == 1)
    HAL_GPIO_WritePin(PIN_READY_GPIO_Port, PIN_READY_Pin, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(PIN_READY_GPIO_Port, PIN_READY_Pin, GPIO_PIN_RESET);
#endif
}
#endif

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();

#ifdef STM32F4
	// just shutup compiler, this is not tested
#else
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
#endif

//    HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
}

static void cdc_task(void);

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	gpio_init();

	MX_DMA_Init();

    timer_init();

#ifdef NEO_LED_Pin
	neo_pixel_init();
#endif

    led_init(&hLED);

#ifdef LEDRX_Pin
	/* nice wake-up pattern */
	for(uint8_t i=0; i<10; i++)
	{
		HAL_GPIO_TogglePin(LEDRX_GPIO_Port, LEDRX_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LEDTX_GPIO_Port, LEDTX_Pin);
	}
#endif

	can_init(&hCAN, CAN_INTERFACE);
	can_disable(&hCAN);

#ifdef PIN_DELAY_Pin
    led_set_mode(&hLED, led_mode_boot);
    led_update(&hLED);
    wait_for_the_go_signal();
#endif

	q_frame_pool = queue_create(CAN_QUEUE_SIZE);
	q_from_host  = queue_create(CAN_QUEUE_SIZE);
	q_to_host	 = queue_create(CAN_QUEUE_SIZE);
	assert_basic(q_frame_pool && q_from_host && q_to_host);

	struct gs_host_frame *msgbuf = calloc(CAN_QUEUE_SIZE, sizeof(struct gs_host_frame));
	assert_basic(msgbuf);

	for (unsigned i=0; i<CAN_QUEUE_SIZE; i++) {
		queue_push_back(q_frame_pool, &msgbuf[i]);
	}

	// init device stack on configured roothub port
	tud_init(BOARD_TUD_RHPORT);

#ifdef PIN_READY_Pin
    // we need a long delay so virtualbox and windows enumerate consistently
	pretty_delay(&hLED, 4000);
    send_ready_signal();
#endif

    led_set_mode(&hLED, led_mode_off);
    led_update(&hLED);

#ifdef CAN_S_GPIO_Port
	HAL_GPIO_WritePin(CAN_S_GPIO_Port, CAN_S_Pin, GPIO_PIN_RESET);
#endif

	while(run) {
		tud_task();
		cdc_task();
		gs_usb_task();
	}

	dfu_run_bootloader();
	return 0;
}

static char buf[1024];
static int bufUsed;
int block = 0;

int _write(int file, char *data, int len)
{
	if ((file != 1) && (file != 2) && (file != 3)) {
	  return -1;
	}

	if (block)
		return -1;

	if (bufUsed + len >= 1024)
		return -1;

	memcpy(buf+bufUsed, data, len);
	bufUsed += len;
	buf[bufUsed] = 0;
	return 0;
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
static void cdc_task(void)
{
	static bool started = false;
	// connected() check for DTR bit
	// Most but not all terminal client set this when making connection
	// if ( tud_cdc_n_connected(0) )
	{
	  if ( tud_cdc_n_available(0) )
	  {
		  started = true;

		  char rxbuf[64];
		  uint32_t bytes = tud_cdc_n_read(0, rxbuf, sizeof(rxbuf));
		  if(rxbuf[0] == 'q')
			  run = 0;
		  tud_cdc_n_write(0, rxbuf, bytes);
		  tud_cdc_n_write_flush(0);
	  }
	}

	if (started)
	{
		block = 1;

		if (bufUsed)
		{
			char *s = buf;
			while(bufUsed > 64)
			{
				tud_cdc_n_write(0, s, 64);
				tud_cdc_n_write_flush(0);
				s += 64;
				bufUsed -= 64;
			}
			tud_cdc_n_write(0, s, bufUsed);
			tud_cdc_n_write_flush(0);
			bufUsed = 0;
		}

		tud_cdc_n_write_flush(0);

		block = 0;
	}
}

void HAL_MspInit(void)
{
	__HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

#if defined(STM32F4)
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
#endif
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void SystemClock_Config(void)
{
#if BOARD_wmc_usb_can
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);


    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

#elif defined(STM32F0)
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;
	RCC_CRSInitTypeDef       RCC_CRSInitStruct;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
								  RCC_CLOCKTYPE_SYSCLK |
								  RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__HAL_RCC_CRS_CLK_ENABLE();
	RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
	RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
	RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
	RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
	RCC_CRSInitStruct.ErrorLimitValue = 34;
	RCC_CRSInitStruct.HSI48CalibrationValue = 32;
	HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
#elif defined(STM32F4)
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
								  RCC_CLOCKTYPE_SYSCLK |
								  RCC_CLOCKTYPE_PCLK1 |
								  RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
#endif

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
