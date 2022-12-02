#include "lsm6dsl_reg.h"
#include <string.h>
#include <stdio.h>
#include "imu.h"
#include "spi.h"
#include "gpio.h"
#include "math.h"
#include "stdbool.h"


#define SENSOR_BUS hspi1

/* Private macro -------------------------------------------------------------*/

#define    BOOT_TIME   15 //ms
#define    WAIT_TIME_A     100 //ms
#define    WAIT_TIME_G_01  150 //ms
#define    WAIT_TIME_G_02   50 //ms

/* Self test limits. */
#define    MIN_ST_LIMIT_mg         90.0f
#define    MAX_ST_LIMIT_mg       1700.0f
#define    MIN_ST_LIMIT_mdps   150000.0f
#define    MAX_ST_LIMIT_mdps   700000.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

stmdev_ctx_t dev_ctx;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

float rad2grad(float rad) {
	return rad*(180.0/M_PI);
}

bool imu_init() {
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &SENSOR_BUS;
	uint8_t rst;

	/* Init test platform */
	platform_init();
	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Restore default configuration */
	lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lsm6dsl_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_416Hz);
	lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_416Hz);
	/* Set full scale */
	lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
	lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_125dps);
	/* Configure filtering chain(No aux interface) */
	/* Accelerometer - analog filter */
	lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);
	/* Accelerometer - LPF1 path ( LPF2 not used )*/
	//lsm6dsl_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
	/* Accelerometer - LPF1 + LPF2 path */
	lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx,
						   LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);
	/* Accelerometer - High Pass / Slope path */
	//lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
	//lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);
	/* Gyroscope - filtering chain */
	lsm6dsl_gy_band_pass_set(&dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);
	return true;
}

bool imu_get_acc_mg(float* x, float* y, float* z) {
	lsm6dsl_reg_t reg;
	int16_t data_raw_acceleration[3];
	lsm6dsl_status_reg_get(&dev_ctx, &reg.status_reg);

	if (reg.status_reg.xlda) {
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
//		printf("%d, %d, %d\n", data_raw_acceleration[0], data_raw_acceleration[1], data_raw_acceleration[2]);
		if (x != NULL) {
			*x = lsm6dsl_from_fs2g_to_mg(data_raw_acceleration[0]);
		}
		if (y != NULL) {
			*y = lsm6dsl_from_fs2g_to_mg(data_raw_acceleration[1]);
		}
		if (z != NULL) {
			*z = lsm6dsl_from_fs2g_to_mg(data_raw_acceleration[2]);
		}
		return true;
	}
	return false;
}

bool get_gyro(float* wx, float* wy, float* wz) {
	lsm6dsl_reg_t reg;
	int16_t data_raw_gyro[3];
	lsm6dsl_status_reg_get(&dev_ctx, &reg.status_reg);

	if (reg.status_reg.xlda) {
		//memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw_gyro);
//		printf("%d, %d, %d\n", data_r2aw_acceleration[0], data_raw_acceleration[1], data_raw_acceleration[2]);
		if (wx != NULL) {
			*wx = lsm6dsl_from_fs125dps_to_mdps(data_raw_gyro[0]);
		}
		if (wy != NULL) {
			*wy = lsm6dsl_from_fs125dps_to_mdps(data_raw_gyro[1]);
		}
		if (wz != NULL) {
			*wz = lsm6dsl_from_fs125dps_to_mdps(data_raw_gyro[2]);
		}
		return true;
	}
	return false;
}

/*
bool imu_get_pitch_yaw(float* pitch, float* yaw) {
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	if (imu_get_acc_mg(&x, &y, &z) != true) {
		return false;
	}
//	printf("%.2f, %.2f, %.2f\n", x, y, z);
	float g_module = sqrtf(x*x + y*y + z*z);
	*pitch = rad2grad(acosf(x/g_module));
	*yaw = rad2grad(acosf(z/g_module));

	return true;
}

}
*/


static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;

void lsm6dsl_read_data_polling(void) {
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Restore default configuration */
  lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsl_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_416Hz);
  lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_416Hz);
  /* Set full scale */
  lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
  lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);
  /* Configure filtering chain(No aux interface) */
  /* Accelerometer - analog filter */
  lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);
  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  //lsm6dsl_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
  /* Accelerometer - LPF1 + LPF2 path */
  lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx,
                               LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);
  /* Accelerometer - High Pass / Slope path */
  //lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  //lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);
  /* Gyroscope - filtering chain */
  lsm6dsl_gy_band_pass_set(&dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);

  /* Read samples in polling mode (no int) */
  while (1) {
    /* Read output only if new value is available */
    lsm6dsl_reg_t reg;
    lsm6dsl_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.xlda) {
      /* Read magnetic field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] = lsm6dsl_from_fs2g_to_mg(
                             data_raw_acceleration[0]);
      acceleration_mg[1] = lsm6dsl_from_fs2g_to_mg(
                             data_raw_acceleration[1]);
      acceleration_mg[2] = lsm6dsl_from_fs2g_to_mg(
                             data_raw_acceleration[2]);
      float g_module = sqrtf(acceleration_mg[0]*acceleration_mg[0] + acceleration_mg[1]*acceleration_mg[1] + acceleration_mg[2]*acceleration_mg[2]);
      printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
    		  rad2grad(acosf(acceleration_mg[0]/g_module)), rad2grad(acosf(acceleration_mg[1]/g_module)), rad2grad(acosf(acceleration_mg[2]/g_module)), g_module);
    }

//    if (reg.status_reg.gda) {
//      /* Read magnetic field data */
//      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
//      lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
//      angular_rate_mdps[0] = lsm6dsl_from_fs2000dps_to_mdps(
//                               data_raw_angular_rate[0]);
//      angular_rate_mdps[1] = lsm6dsl_from_fs2000dps_to_mdps(
//                               data_raw_angular_rate[1]);
//      angular_rate_mdps[2] = lsm6dsl_from_fs2000dps_to_mdps(
//                               data_raw_angular_rate[2]);
//      printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
//    }
//
//    if (reg.status_reg.tda) {
//      /* Read temperature data */
//      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//      lsm6dsl_temperature_raw_get(&dev_ctx, &data_raw_temperature);
//      temperature_degC = lsm6dsl_from_lsb_to_celsius(
//                           data_raw_temperature );
//      printf("Temperature [degC]:%6.2f\r\n", temperature_degC );
//    }
  }
}


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register regs
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  reg |= 0x80;
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
  return 0;
}



/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{

}


