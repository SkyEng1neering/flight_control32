#include "utils.h"

uint32_t crop_data(uint32_t min, uint32_t max, uint32_t val) {
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

uint32_t band2band(uint32_t b1_min, uint32_t b1_max, uint32_t b2_min,
        uint32_t b2_max, uint32_t val_from_b1) {
    uint32_t retval = ((((val_from_b1 - b1_min) * 100) /
            (b1_max - b1_min)) * (b2_max - b2_min)) / 100 + b2_min;
    return retval;
}

float fband2band(float b1_min, float b1_max, float b2_min,
        float b2_max, float val_from_b1) {
    float retval = ((((val_from_b1 - b1_min) * 100.0) /
            (b1_max - b1_min)) * (b2_max - b2_min)) / 100.0 + b2_min;
    return retval;
}

