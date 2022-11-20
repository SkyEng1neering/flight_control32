#ifndef SRC_LSM6DSL_IMU_H_
#define SRC_LSM6DSL_IMU_H_

#include "stdbool.h"

void lsm6dsl_self_test(void);
void lsm6dsl_read_data_polling(void);
bool imu_get_pitch_yaw(float* pitch, float* yaw);
bool imu_init();
bool imu_get_acc_mg(float* x, float* y, float* z);

#endif /* SRC_LSM6DSL_IMU_H_ */
