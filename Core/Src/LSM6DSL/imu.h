#ifndef SRC_LSM6DSL_IMU_H_
#define SRC_LSM6DSL_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"

float rad2grad(float rad);
void lsm6dsl_self_test(void);
void lsm6dsl_read_data_polling(void);
bool imu_get_pitch_roll(float* pitch, float* roll);
bool imu_init();
bool imu_get_acc_mg(float* x, float* y, float* z);

/* gyro in mdps (milli? degree per second)*/
bool get_gyro(float* wx, float* wy, float* wz);

#ifdef __cplusplus
}
#endif

#endif /* SRC_LSM6DSL_IMU_H_ */
