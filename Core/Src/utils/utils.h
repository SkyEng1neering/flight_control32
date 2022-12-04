#ifndef SRC_UTILS_UTILS_H_
#define SRC_UTILS_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t crop_data(uint32_t min, uint32_t max, uint32_t val);
uint32_t band2band(uint32_t b1_min, uint32_t b1_max, uint32_t b2_min,
        uint32_t b2_max, uint32_t val_from_b1);
float fband2band(float b1_min, float b1_max, float b2_min,
        float b2_max, float val_from_b1);

#ifdef __cplusplus
}
#endif

#endif /* SRC_UTILS_UTILS_H_ */
