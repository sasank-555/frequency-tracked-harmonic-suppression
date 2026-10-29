/**
 * @file fir_notch.h
 * @brief High-attenuation Linear-Phase FIR Harmonic Notch Filters for embedded DSP.
 * 
 * Provides:
 * 1. Cascaded Linear-Phase Zero-Placement FIR Notch (Zero MIPS overhead, exact theoretical -inf dB depth)
 * 2. Windowed-Sinc Multi-Bandstop Linear-Phase FIR Filter (Customizable tap length and notch width)
 */

#ifndef FIR_NOTCH_H
#define FIR_NOTCH_H

#include "dsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_FIR_TAPS         255

/**
 * Single 2nd-order symmetric FIR notch section:
 * H(z) = (b0 + b1*z^-1 + b2*z^-2) / norm
 * where b0 = 1, b1 = -2*cos(omega_k), b2 = 1
 * Strictly linear phase with group delay = 1 sample.
 */
typedef struct {
    dsp_real_t b0;        /* 1.0 / norm */
    dsp_real_t b1;        /* -2 * cos(omega_k) / norm */
    dsp_real_t b2;        /* 1.0 / norm */
    dsp_real_t x1;        /* State: x[n-1] */
    dsp_real_t x2;        /* State: x[n-2] */
    dsp_real_t center_freq;
} fir_biquad_notch_t;

/**
 * Cascaded Zero-Placement FIR Multi-Notch Filter.
 * Extremely efficient for Cortex-M0/M3/M4/DSPs.
 * Total group delay = num_harmonics samples.
 */
typedef struct {
    fir_biquad_notch_t stages[MAX_HARMONICS];
    uint8_t            num_stages;
    dsp_real_t         sample_rate;
    dsp_real_t         current_f0;
} fir_cascade_notch_t;

/**
 * Windowed Sinc Multi-Bandstop Linear-Phase FIR Filter.
 * Constant group delay = (num_taps - 1) / 2 samples.
 */
typedef struct {
    dsp_real_t coeffs[MAX_FIR_TAPS];
    dsp_real_t state_buffer[MAX_FIR_TAPS];
    uint16_t   num_taps;
    uint16_t   buffer_index;
    dsp_real_t sample_rate;
    dsp_real_t notch_bandwidth;
    uint8_t    num_harmonics;
    dsp_real_t current_f0;
} fir_windowed_notch_t;

/* --- API Functions for Cascaded FIR Notch --- */

dsp_status_t fir_cascade_init(
    fir_cascade_notch_t *filter,
    dsp_real_t sample_rate,
    dsp_real_t initial_f0,
    uint8_t num_harmonics
);

void fir_cascade_update_frequency(
    fir_cascade_notch_t *filter,
    dsp_real_t new_f0
);

dsp_real_t fir_cascade_process_sample(
    fir_cascade_notch_t *filter,
    dsp_real_t x
);

void fir_cascade_process_block(
    fir_cascade_notch_t *filter,
    const dsp_real_t *input,
    dsp_real_t *output,
    uint16_t length
);

void fir_cascade_reset(fir_cascade_notch_t *filter);

/* --- API Functions for Windowed-Sinc FIR Notch --- */

dsp_status_t fir_windowed_init(
    fir_windowed_notch_t *filter,
    dsp_real_t sample_rate,
    uint16_t num_taps,
    dsp_real_t notch_bandwidth,
    dsp_real_t initial_f0,
    uint8_t num_harmonics
);

void fir_windowed_update_frequency(
    fir_windowed_notch_t *filter,
    dsp_real_t new_f0
);

dsp_real_t fir_windowed_process_sample(
    fir_windowed_notch_t *filter,
    dsp_real_t x
);

void fir_windowed_process_block(
    fir_windowed_notch_t *filter,
    const dsp_real_t *input,
    dsp_real_t *output,
    uint16_t length
);

void fir_windowed_reset(fir_windowed_notch_t *filter);

/**
 * Calculates theoretical attenuation (in dB) of filter at target frequency.
 */
dsp_real_t fir_calculate_attenuation_db(
    dsp_real_t sample_rate,
    const dsp_real_t *coeffs,
    uint16_t num_taps,
    dsp_real_t target_freq
);

#ifdef __cplusplus
}
#endif

#endif /* FIR_NOTCH_H */
