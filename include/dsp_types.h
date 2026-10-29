/**
 * @file dsp_types.h
 * @brief Common DSP types, constants, and return codes for embedded execution.
 */

#ifndef DSP_TYPES_H
#define DSP_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Real floating-point precision (can be float on single-precision FPUs or double) */
typedef double dsp_real_t;

#define MAX_HARMONICS 8

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TWO_PI (2.0 * M_PI)

typedef enum {
    DSP_OK = 0,
    DSP_ERROR_INVALID_PARAM = -1,
    DSP_ERROR_BUFFER_OVERFLOW = -2,
    DSP_ERROR_NOT_INITIALIZED = -3
} dsp_status_t;

#ifdef __cplusplus
}
#endif

#endif /* DSP_TYPES_H */
