#ifndef SRC_CONFIG_CONFIG_H_
#define SRC_CONFIG_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

struct FlightConfig {
    float accel_pitch_offset;
    float accel_yaw_offset;
    float gyro_pitch_rate_bias;
    float gyro_yaw_rate_bias;
    float gyro_roll_rate_bias;
};

void get_config(struct FlightConfig* conf_ptr);
bool save_config(struct FlightConfig* conf_ptr);

#ifdef __cplusplus
}
#endif

#endif /* SRC_CONFIG_CONFIG_H_ */
