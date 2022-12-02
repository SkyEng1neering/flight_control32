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

#define BATT_VOLTAGE_DISARM_THRESHOLD_MV         6000
#define BATT_VOLTAGE_TURTLE_THRESHOLD_MV         6700

/* Start angles */
#define ANGLE_INIT_PITCH                         90.0
#define ANGLE_INIT_YAW                           0.0
#define ANGLE_INIT_ROLL                          0.0

static bool disarm_flag = false;
static bool turtle_mode_flag = false;

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

float fband2band(float b1_min, float b1_max, float b2_min, float b2_max, float val_from_b1) {
    float retval = ((((val_from_b1 - b1_min)*100.0)/(b1_max - b1_min))*(b2_max - b2_min))/100.0 + b2_min;
    return retval;
}

/* -100 to +100 */
void set_roll_pitch(float roll, float pitch) {
    int elevon_diff = ((int)fband2band(-100.0, 100.0, 1000.0, 2000.0, roll) - 1500);

    uint32_t elevon_right_val = crop_data(1010, 1950, (int)fband2band(-100.0, 100.0, 1000.0, 2000.0, pitch) + elevon_diff);
    uint32_t elevon_left_val = crop_data(1010, 1950, 3000 - (int)fband2band(-100.0, 100.0, 1000.0, 2000.0, pitch) + elevon_diff);

    /* Right elevon */
    TIM2->CCR2 = elevon_right_val;

    /* Left elevon */
    TIM2->CCR1 = elevon_left_val;
}

void handle_verticle_mode(struct IbusCannels* ch_struct_ptr) {
    float pitch, yaw;
    if (imu_get_pitch_yaw(&pitch, &yaw) != true) {
        return;
    }

//    printf("%.2f, %.2f\n", pitch, yaw);

//    float yaw_rate, roll_rate, pitch_rate;
//    if (get_gyro(&yaw_rate, &roll_rate, &pitch_rate) != true) {
//        return;
//    }

    //pitch -
    //roll - right wing down +
    //yaw - clockwise +

    //yaw_rate should be proportional to stick (left, horizontal)

    //pitch_rate should be zero. and propotional to derivative of stick(right, vertical)
    //OR pitch should be propotional to stick(right, vertical)

    //Roll should be proportional to stick(right, horizontal)

//    set_roll_pitch(fband2band(1000.0, 2000.0, -100.0, 100.0, (float)ch_struct_ptr->ch1),
//            fband2band(1000.0, 2000.0, -100.0, 100.0, (float)ch_struct_ptr->ch2));

//    printf("%.2f, %.2f, %.2f\n", pitch_rate/1000, roll_rate/1000, yaw_rate/1000);
}

void handle_horizontal_mode(struct IbusCannels* ch_struct_ptr) {
    /* Current angles */
    static float current_pitch = ANGLE_INIT_PITCH;

    /* Last calc tick */
    static uint32_t last_tick = 0;

    /* Initial run */
    if (last_tick == 0) {
        last_tick = HAL_GetTick();
        return;
    }

    /* Get gyro rates */
    float yaw_rate, roll_rate, pitch_rate;
    if (get_gyro(&yaw_rate, &roll_rate, &pitch_rate) != true) {
        return;
    }

    /* Get accel angles */
    float accel_pitch, accel_yaw;
    if (imu_get_pitch_yaw(&accel_pitch, &accel_yaw) != true) {
        return;
    }

    /* Get integral of rates and combine with accel */
    uint32_t current_tick = HAL_GetTick();
    float dt = (float)(current_tick - last_tick)/1000.0;
    float accel_k = 0.02;
    current_pitch = (current_pitch + (pitch_rate/1000.0) * dt) * (1 - accel_k) + accel_pitch*accel_k;

    last_tick = current_tick;

    printf("%.2f, %.2f\n", accel_pitch, current_pitch);

//    set_roll_pitch(current_yaw, ch_struct_ptr->ch2);
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
//	printf("ch1: %d, ch2: %d, ch3: %d, ch4: %d, ch5: %d, ch6: %d, ch7: %d, ch8: %d, ch9: %d, ch10: %d, ch11: %d, ch12: %d, ch13: %d, ch14: %d\n",
//			ch_struct_ptr->ch1, ch_struct_ptr->ch2, ch_struct_ptr->ch3, ch_struct_ptr->ch4, ch_struct_ptr->ch5, ch_struct_ptr->ch6, ch_struct_ptr->ch7,
//			ch_struct_ptr->ch8, ch_struct_ptr->ch9, ch_struct_ptr->ch10, ch_struct_ptr->ch11, ch_struct_ptr->ch12, ch_struct_ptr->ch13, ch_struct_ptr->ch14);



//	printf("pitch: %.2f, yaw: %.2f\n", pitch, yaw);

    /* Right elevon */
    TIM2->CCR2 = elevon_right_val;

    /* Left elevon */
    TIM2->CCR1 = elevon_left_val;

    /* Check disarm flag */
    if (disarm_flag == true) {
        TIM4->CCR1 = 0;
        TIM4->CCR2 = 0;
        return;
    }

    uint32_t prop_max_val = 1980;

    /* Check turtle mode flag */
    if (turtle_mode_flag == true) {
        prop_max_val = 1350;
    }

    /* Right propeller */
    TIM4->CCR1 = band2band(1000, 2000, 20, 100, crop_data(1010, prop_max_val, propeller_right_val));

    /* Left propeller */
    TIM4->CCR2 = band2band(1000, 2000, 20, 100, crop_data(1010, prop_max_val, propeller_left_val));
}

void ch_data_callback(struct IbusCannels* ch_struct_ptr) {
    __disable_irq();
    if (ch_struct_ptr->ch5 == 1000) {
        handle_verticle_mode(ch_struct_ptr);
    } else {
        handle_horizontal_mode(ch_struct_ptr);
//        handle_super_ohuet_vip_3d_mode(ch_struct_ptr);
    }
    __enable_irq();
}

void SystemClock_Config(void);

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
    }

    if (bat_voltage < BATT_VOLTAGE_TURTLE_THRESHOLD_MV) {
        turtle_mode_flag = true;
    } else {
        turtle_mode_flag = false;
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();
    MX_TIM4_Init();

    ibus_init(ch_data_callback);
    imu_init();
    filter_init(10.0, 1500.0);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

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

