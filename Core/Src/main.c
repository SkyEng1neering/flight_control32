/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ibus.h"
#include "imu.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void usart_rx_byte(uint8_t byte) {
//	printf("%d\n", byte);
	ibus_data_process(byte);
}

uint32_t crop_data(uint32_t min, uint32_t max, uint32_t val) {
	if (val < min)
		return min;
	if (val > max)
		return max;
	return val;
}

uint32_t band2band(uint32_t b1_min, uint32_t b1_max, uint32_t b2_min, uint32_t b2_max, uint32_t val_from_b1) {
	uint32_t retval = ((((val_from_b1 - b1_min)*100)/(b1_max - b1_min))*(b2_max - b2_min))/100 + b2_min;
	return retval;
}

void handle_verticle_mode(struct IbusCannels* ch_struct_ptr) {
	float pitch_rate, roll_rate, yaw_rate;  // нос вверх, правое крыло вниз,нос влево
	if (get_gyro(&pitch_rate, &roll_rate, &yaw_rate) != true) {
		return;
	}

    //yaw_rate should be proportional to stick (left, horizontal)

    //pitch_rate should be zero. and propotional to derivative of stick(right, vertical)
    //OR pitch should be propotional to stick(right, vertical)

    //Roll should be proportional to stick(right, horizontal)

	//printf("pitch: %.2f, yaw: %.2f\n", pitch, yaw);
}

void handle_horizontal_mode(struct IbusCannels* ch_struct_ptr) {

}

void handle_super_ohuet_vip_3d_mode(struct IbusCannels* ch_struct_ptr) {
	int elevon_diff = ((int)ch_struct_ptr->ch1 - 1500);
	int propeller_diff = ((int)ch_struct_ptr->ch4 - 1500);
	uint32_t elevon_right_val = crop_data(1010, 1950, ch_struct_ptr->ch2 + elevon_diff);
	uint32_t elevon_left_val = crop_data(1010, 1950, 3000 - ch_struct_ptr->ch2 + elevon_diff);
	uint32_t propeller_right_val = ch_struct_ptr->ch3;
	uint32_t propeller_left_val = ch_struct_ptr->ch3;
	if (propeller_diff > 0) {
		propeller_right_val -= propeller_diff/2;
	} else if (propeller_diff < 0) {
		propeller_left_val += propeller_diff/2;
	}

//	printf("ch1: %d, ch2: %d, ch3: %d, ch4: %d, elevon_diff: %d, r_val: %lu, l_val: %lu\n", ch_struct_ptr->ch1, ch_struct_ptr->ch2, ch_struct_ptr->ch3, ch_struct_ptr->ch4, propeller_diff, propeller_right_val, propeller_left_val);
	printf("ch1: %d, ch2: %d, ch3: %d, ch4: %d, ch5: %d, ch6: %d, ch7: %d, ch8: %d, ch9: %d, ch10: %d, ch11: %d, ch12: %d, ch13: %d, ch14: %d\n",
			ch_struct_ptr->ch1, ch_struct_ptr->ch2, ch_struct_ptr->ch3, ch_struct_ptr->ch4, ch_struct_ptr->ch5, ch_struct_ptr->ch6, ch_struct_ptr->ch7,
			ch_struct_ptr->ch8, ch_struct_ptr->ch9, ch_struct_ptr->ch10, ch_struct_ptr->ch11, ch_struct_ptr->ch12, ch_struct_ptr->ch13, ch_struct_ptr->ch14);



//	printf("pitch: %.2f, yaw: %.2f\n", pitch, yaw);

	/* Right elevon */
	TIM2->CCR2 = elevon_right_val;

	/* Left elevon */
	TIM2->CCR1 = elevon_left_val;

	/* Right propeller */
	TIM4->CCR1 = band2band(1000, 2000, 20, 100, crop_data(1010, 1950, propeller_right_val));

	/* Left propeller */
	TIM4->CCR2 = band2band(1000, 2000, 20, 100, crop_data(1010, 1950, propeller_left_val));
}

void ch_data_callback(struct IbusCannels* ch_struct_ptr) {
	__disable_irq();
	if (ch_struct_ptr->ch5 == 1000) {
		handle_verticle_mode(ch_struct_ptr);
	} else {
		handle_horizontal_mode(ch_struct_ptr);
	}
	__enable_irq();
}
	/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  ibus_init(ch_data_callback);
  imu_init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  printf("Flight control started\n");
//  lsm6dsl_self_test();
//  lsm6dsl_read_data_polling();
//  TIM2->CCR1 = 600;
//  TIM2->CCR2 = 700;
//  TIM2->CCR3 = 30;
//  TIM2->CCR4 = 30;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

