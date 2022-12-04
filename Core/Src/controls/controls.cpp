#include <stdbool.h>
#include "controls.h"
#include "utils.h"
#include "stm32f1xx_hal.h"
#include "filter_low_pass.h"

/* Start angles */
#define ANGLE_INIT_PITCH                  90.0
#define ANGLE_INIT_YAW                    0.0
#define ANGLE_INIT_ROLL                   0.0

#define ELEVON_OFFSET_MIN                 10
#define ELEVON_OFFSET_MAX                 50
#define PROPELLER_OFFSET_MIN              10

#define PROPELLER_MAX_VAL_NORMAL          2000
#define PROPELLER_MAX_VAL_TURTLE          1350

#define PROPELLER_TIM_VAL_MIN             20
#define PROPELLER_TIM_VAL_MAX             100

extern bool disarm_flag;
extern bool turtle_mode_flag;

FilterLowPass turtle_trans_filt(10.0, 4000.0);

/* -100 to +100 */
void set_roll_pitch(float roll, float pitch) {
    int elevon_diff = (int)fband2band(CONTROLS_BAND_MIN, CONTROLS_BAND_MAX,
            (float)RC_BAND_MIN, (float)RC_BAND_MAX, roll) - RC_BAND_MIDDLE;
    uint32_t elevon_right_val = crop_data(RC_BAND_MIN + ELEVON_OFFSET_MIN,
            RC_BAND_MAX - ELEVON_OFFSET_MAX, (int)fband2band(CONTROLS_BAND_MIN,
                    CONTROLS_BAND_MAX, (float)RC_BAND_MIN, (float)RC_BAND_MAX, pitch) + elevon_diff);
    uint32_t elevon_left_val = crop_data(RC_BAND_MIN + ELEVON_OFFSET_MIN,
            RC_BAND_MAX - ELEVON_OFFSET_MAX, (RC_BAND_MIDDLE * 2) -
            (int)fband2band(CONTROLS_BAND_MIN, CONTROLS_BAND_MAX, (float)RC_BAND_MIN,
                    (float)RC_BAND_MAX, pitch) + elevon_diff);

    /* Set right elevon */
    TIM2->CCR2 = elevon_right_val;

    /* Set left elevon */
    TIM2->CCR1 = elevon_left_val;
}

/* -100 to +100 */
void set_yaw_throttle(float yaw, float throttle) {

    int propeller_diff = (int)fband2band(CONTROLS_BAND_MIN, CONTROLS_BAND_MAX,
            (float)RC_BAND_MIN, (float)RC_BAND_MAX, yaw) - RC_BAND_MIDDLE;
    uint32_t propeller_right_val = (int)fband2band(CONTROLS_BAND_MIN, CONTROLS_BAND_MAX,
            (float)RC_BAND_MIN, (float)RC_BAND_MAX, throttle);
    uint32_t propeller_left_val = propeller_right_val;
    if (propeller_diff > 0) {
        propeller_right_val -= propeller_diff / 2;
    } else if (propeller_diff < 0) {
        propeller_left_val += propeller_diff / 2;
    }

    /* Check disarm flag */
    if (disarm_flag == true) {
        TIM4->CCR1 = 0;
        TIM4->CCR2 = 0;
        return;
    }

    uint32_t prop_max_val = turtle_trans_filt.process(PROPELLER_MAX_VAL_NORMAL);

    /* Check turtle mode flag */
    if (turtle_mode_flag == true) {
        prop_max_val = turtle_trans_filt.process(PROPELLER_MAX_VAL_TURTLE);
    }

    /* Right propeller */
    TIM4->CCR1 = band2band(RC_BAND_MIN, RC_BAND_MAX, PROPELLER_TIM_VAL_MIN, PROPELLER_TIM_VAL_MAX,
            crop_data(RC_BAND_MIN + PROPELLER_OFFSET_MIN, prop_max_val, propeller_right_val));

    /* Left propeller */
    TIM4->CCR2 = band2band(RC_BAND_MIN, RC_BAND_MAX, PROPELLER_TIM_VAL_MIN, PROPELLER_TIM_VAL_MAX,
            crop_data(RC_BAND_MIN + PROPELLER_OFFSET_MIN, prop_max_val, propeller_left_val));
}

float get_fused_pitch(float pitch_rate, float accel_pitch) {
    static float current_pitch = ANGLE_INIT_PITCH;
    static uint32_t last_tick = 0;

    /* Initial run */
    if (last_tick == 0) {
        last_tick = HAL_GetTick();
        return 0.0;
    }

    uint32_t current_tick = HAL_GetTick();
    float dt = (float)(current_tick - last_tick)/1000.0;
    float accel_k = 0.015;
    current_pitch = (current_pitch + (pitch_rate/1000.0) * dt) * (1 - accel_k) + accel_pitch*accel_k;
    last_tick = current_tick;

    return current_pitch;
}

float get_fused_roll(float roll_rate, float accel_roll) {
    static float current_pitch = ANGLE_INIT_ROLL;
    static uint32_t last_tick = 0;

    /* Initial run */
    if (last_tick == 0) {
        last_tick = HAL_GetTick();
        return 0.0;
    }

    uint32_t current_tick = HAL_GetTick();
    float dt = (float)(current_tick - last_tick)/1000.0;
    float accel_k = 0.015;
    current_pitch = (current_pitch + (roll_rate/1000.0) * dt) * (1 - accel_k) + accel_roll*accel_k;
    last_tick = current_tick;

    return current_pitch;
}
