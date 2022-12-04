#ifndef SRC_CONTROLS_FLIGHT_MODES_H_
#define SRC_CONTROLS_FLIGHT_MODES_H_

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif

#include "ibus.h"

EXTERN_C void vertical_mode(struct IbusCannels* ch_struct_ptr);
EXTERN_C void horizontal_mode(struct IbusCannels* ch_struct_ptr);

#undef EXTERN_C

#endif /* SRC_CONTROLS_FLIGHT_MODES_H_ */
