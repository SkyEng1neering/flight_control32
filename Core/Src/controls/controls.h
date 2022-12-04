#ifndef SRC_CONTROLS_CONTROLS_H_
#define SRC_CONTROLS_CONTROLS_H_

#define CONTROLS_BAND_MIN                 -100.0
#define CONTROLS_BAND_MAX                 100.0

#define RC_BAND_MIN                       1000
#define RC_BAND_MIDDLE                    1500
#define RC_BAND_MAX                       2000

void set_roll_pitch(float roll, float pitch);
void set_yaw_throttle(float yaw, float throttle);
float get_fused_pitch(float pitch_rate, float accel_pitch);
float get_fused_roll(float roll_rate, float accel_roll);

#endif /* SRC_CONTROLS_CONTROLS_H_ */
