#include <stdio.h>
#include "flight_modes.h"
#include "imu.h"
#include "controls.h"
#include "pidlib.h"
#include "utils.h"

#define VERTICLE_PITCH_FULL_BAND            60.0 /* Grads */
#define VERTICLE_PITCH_BAND_PT              25.0 /* Percents */

#define VERTICLE_YAW_FULL_BAND              40.0 /* Grads */
#define VERTICLE_YAW_BAND_PT                40.0 /* Percents */

#define VERTICLE_ROLL_BAND_PT               25.0 /* Percents */

PID<float> verticle_pid_pitch(5.0, 0.0, 50.0, CONTROLS_BAND_MIN, CONTROLS_BAND_MAX, 6.0);
PID<float> verticle_pid_yaw(5.0, 0.0, 0.0, CONTROLS_BAND_MIN, CONTROLS_BAND_MAX, 6.0);

void vertical_mode(struct IbusCannels* ch_struct_ptr) {
    float target_pitch = fband2band(RC_BAND_MIN, RC_BAND_MAX,
            90.0 - (VERTICLE_PITCH_FULL_BAND / 200.0) * VERTICLE_PITCH_BAND_PT,
            90.0 + (VERTICLE_PITCH_FULL_BAND / 200.0) * VERTICLE_PITCH_BAND_PT,
            RC_BAND_MIDDLE * 2 - ch_struct_ptr->ch2);

    float target_yaw = fband2band(RC_BAND_MIN, RC_BAND_MAX,
            0.0 - (VERTICLE_YAW_FULL_BAND / 200.0) * VERTICLE_YAW_BAND_PT,
            0.0 + (VERTICLE_YAW_FULL_BAND / 200.0) * VERTICLE_YAW_BAND_PT,
            ch_struct_ptr->ch4);

    verticle_pid_pitch.setTarget(target_pitch);
    verticle_pid_yaw.setTarget(target_yaw);

    /* Get gyro rates */
    float yaw_rate, roll_rate, pitch_rate;
    if (get_gyro(&yaw_rate, &roll_rate, &pitch_rate) != true) {
        return;
    }

    /* Get accel angles */
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    if (imu_get_acc_mg(&x, &y, &z) != true) {
        return;
    }

    float g_module = sqrtf(x*x + y*y + z*z);
    float accel_yaw = rad2grad(acosf(z/g_module)) - 90.0;
    float accel_pitch = rad2grad(acosf(-x/g_module)) + 6.7;

    float pitch = get_fused_pitch(pitch_rate, accel_pitch);
    float yaw = get_fused_roll(roll_rate, accel_yaw);

    float new_pitch = -verticle_pid_pitch.calculate(pitch);
    float new_yaw = verticle_pid_yaw.calculate(yaw);
    float new_throttle = fband2band(RC_BAND_MIN, RC_BAND_MAX, CONTROLS_BAND_MIN,
            CONTROLS_BAND_MAX, ch_struct_ptr->ch3);
    float new_roll = fband2band(RC_BAND_MIN, RC_BAND_MAX, CONTROLS_BAND_MIN,
            CONTROLS_BAND_MAX, ch_struct_ptr->ch1) * VERTICLE_ROLL_BAND_PT / 100.0;

    set_yaw_throttle(new_yaw, new_throttle);
    set_roll_pitch(new_roll, new_pitch);

    static uint32_t last_tick = HAL_GetTick();

    uint32_t current_tick = HAL_GetTick();
    printf("accel_pitch: %.2f, pitch: %.2f, new_pitch: %.2f, period %.2f\n", accel_pitch, pitch, new_pitch, (float)(current_tick - last_tick));
    last_tick = current_tick;
}

void horizontal_mode(struct IbusCannels* ch_struct_ptr) {
    /* Get gyro rates */
    float yaw_rate, roll_rate, pitch_rate;
    if (get_gyro(&yaw_rate, &roll_rate, &pitch_rate) != true) {
        return;
    }

    /* Get accel angles */
    float accel_pitch, accel_roll;
    if (imu_get_pitch_roll(&accel_pitch, &accel_roll) != true) {
        return;
    }

    float pitch = get_fused_pitch(pitch_rate, accel_pitch);
    float roll = get_fused_roll(roll_rate, accel_roll);

//    printf("%.2f, %.2f\n", pitch, roll);

//    set_roll_pitch(current_yaw, ch_struct_ptr->ch2);
}




//    int elevon_diff = ((int)ch_struct_ptr->ch1 - 1500);
//    int propeller_diff = ((int)ch_struct_ptr->ch4 - 1500)/2;
//    uint32_t elevon_right_val = crop_data(1010, 1950, ch_struct_ptr->ch2 + elevon_diff);
//    uint32_t elevon_left_val = crop_data(1010, 1950, 3000 - ch_struct_ptr->ch2 + elevon_diff);
//    uint32_t propeller_right_val = ch_struct_ptr->ch3;
//    uint32_t propeller_left_val = ch_struct_ptr->ch3;
//    if (propeller_diff > 0) {
//        propeller_right_val -= propeller_diff/2;
//    } else if (propeller_diff < 0) {
//        propeller_left_val += propeller_diff/2;
//    }
//
////    printf("ch1: %d, ch2: %d, ch3: %d, ch4: %d, elevon_diff: %d, r_val: %lu, l_val: %lu\n", ch_struct_ptr->ch1, ch_struct_ptr->ch2, ch_struct_ptr->ch3, ch_struct_ptr->ch4, propeller_diff, propeller_right_val, propeller_left_val);
////    printf("ch1: %d, ch2: %d, ch3: %d, ch4: %d, ch5: %d, ch6: %d, ch7: %d, ch8: %d, ch9: %d, ch10: %d, ch11: %d, ch12: %d, ch13: %d, ch14: %d\n",
////            ch_struct_ptr->ch1, ch_struct_ptr->ch2, ch_struct_ptr->ch3, ch_struct_ptr->ch4, ch_struct_ptr->ch5, ch_struct_ptr->ch6, ch_struct_ptr->ch7,
////            ch_struct_ptr->ch8, ch_struct_ptr->ch9, ch_struct_ptr->ch10, ch_struct_ptr->ch11, ch_struct_ptr->ch12, ch_struct_ptr->ch13, ch_struct_ptr->ch14);
//
//
//
////    printf("pitch: %.2f, yaw: %.2f\n", pitch, yaw);
//
//    /* Right elevon */
//    TIM2->CCR2 = elevon_right_val;
//
//    /* Left elevon */
//    TIM2->CCR1 = elevon_left_val;
//
//    /* Check disarm flag */
//    if (disarm_flag == true) {
//        TIM4->CCR1 = 0;
//        TIM4->CCR2 = 0;
//        return;
//    }
//
//    uint32_t prop_max_val = 1980;
//
//    /* Check turtle mode flag */
//    if (turtle_mode_flag == true) {
//        prop_max_val = 1350;
//    }
//
//    /* Right propeller */
//    TIM4->CCR1 = band2band(1000, 2000, 20, 100, crop_data(1010, prop_max_val, propeller_right_val));
//
//    /* Left propeller */
//    TIM4->CCR2 = band2band(1000, 2000, 20, 100, crop_data(1010, prop_max_val, propeller_left_val));
