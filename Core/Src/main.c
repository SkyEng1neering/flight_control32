#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "ibus.h"
#include "imu.h"
#include "stdbool.h"
#include "low_pass_simple.h"
#include "math.h"
#include "controls.h"
#include "flight_modes.h"
#include "config.h"
#include "controls.h"
#include "timebase_us.h"

#define BATT_VOLTAGE_DISARM_THRESHOLD_MV         6000
#define BATT_VOLTAGE_TURTLE_THRESHOLD_MV         6700

struct FlightConfig global_config;
bool disarm_flag = false;
bool turtle_mode_flag = false;

void usart_rx_byte(uint8_t byte) {
    ibus_data_process(byte);
}


void handle_verticle_mode(struct IbusCannels* ch_struct_ptr) {
    vertical_mode(ch_struct_ptr);
}

void handle_horizontal_mode(struct IbusCannels* ch_struct_ptr) {

}

void handle_super_ohuet_vip_3d_mode(struct IbusCannels* ch_struct_ptr) {

}

void ch_data_callback(struct IbusCannels* ch_struct_ptr) {
//    __disable_irq();
//    printf("%u %u %u %u\n", ch_struct_ptr->ch1, ch_struct_ptr->ch2, ch_struct_ptr->ch3, ch_struct_ptr->ch4);

    /* Check stick patterns */
    if ((ch_struct_ptr->ch1 < 1100) && (ch_struct_ptr->ch2 < 1100) &&
            (ch_struct_ptr->ch3 < 1100) && (ch_struct_ptr->ch4 < 1100)) {
        /* Calibration pattern */
        calibration();
    }

    if (ch_struct_ptr->ch5 == 1000) {
        handle_verticle_mode(ch_struct_ptr);
    } else {
//        handle_horizontal_mode(ch_struct_ptr);
//        handle_super_ohuet_vip_3d_mode(ch_struct_ptr);
    }
//    __enable_irq();
}

void SystemClock_Config();
void check_bat_voltage();

int main(void) {
    HAL_Init();
    SystemClock_Config();

    get_config(&global_config);

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_ADC1_Init();

    ibus_init(ch_data_callback);
    imu_init();
    filter_init(10.0, 1500.0);
    timebase_us_init();

    MX_TIM2_Init();
    MX_TIM4_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    MX_USART1_UART_Init();

    printf("Flight control started\n");
    //  lsm6dsl_self_test();
    //  lsm6dsl_read_data_polling();

    while (1) {
        check_bat_voltage();
        HAL_Delay(100);
    }
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

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}

float get_bat_voltage() {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    uint32_t raw_val = HAL_ADC_GetValue(&hadc1);
    float val_volt = filter_process(((float)raw_val*3300.0)/4095.0);
    float val_bat = val_volt*5.44;
//    printf("Voltage -> ADC: %.3f mV, battery: %.3f mV\n", val_volt, val_bat);
    return val_bat;
}

void check_bat_voltage() {
    float bat_voltage = get_bat_voltage();
    if (bat_voltage < BATT_VOLTAGE_DISARM_THRESHOLD_MV) {
        disarm_flag = true;
    } else {
        disarm_flag = false;
        turtle_mode_flag = false;
    }

    if (bat_voltage < BATT_VOLTAGE_TURTLE_THRESHOLD_MV) {
        turtle_mode_flag = true;
    }
}
