/**
 * @file goertzel.h
 * @brief High-efficiency Goertzel analysis and precision frequency tracking.
 * 
 * Optimized for embedded microcontrollers (ARM Cortex-M, DSPs, etc.).
 * Uses second-order IIR recursion requiring only 1 multiply and 2 additions per sample.
 */

#ifndef GOERTZEL_H
#define GOERTZEL_H

#include "dsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GOERTZEL_MAX_CANDIDATE_BINS 64
#define GOERTZEL_MAX_BLOCK_SIZE     1024

/**
 * State structure for single-bin Goertzel filter.
 */
typedef struct {
    dsp_real_t coeff;     /* 2 * cos(2*pi*f/fs) */
    dsp_real_t s_prev1;   /* s[n-1] */
    dsp_real_t s_prev2;   /* s[n-2] */
    dsp_real_t omega;     /* 2*pi*f/fs */
} goertzel_bin_t;

/**
 * Configuration for the frequency tracker.
 */
typedef struct {
    dsp_real_t sample_rate;         /* Sampling frequency in Hz */
    dsp_real_t nominal_freq;        /* Nominal interference frequency (e.g., 50.0 Hz) */
    dsp_real_t search_span_hz;      /* Search window +/- span (e.g. 4.0 Hz) */
    dsp_real_t step_hz;             /* Candidate frequency step size (e.g. 0.2 Hz) */
    dsp_real_t smoothing_alpha;     /* EMA filter factor for tracked f0 [0.0, 1.0] */
    uint16_t   block_size;          /* Number of samples per analysis block */
} goertzel_tracker_config_t;

/**
 * Goertzel Frequency Tracker Context.
 */
typedef struct {
    goertzel_tracker_config_t config;
    dsp_real_t candidate_freqs[GOERTZEL_MAX_CANDIDATE_BINS];
    dsp_real_t candidate_coeffs[GOERTZEL_MAX_CANDIDATE_BINS];
    uint16_t   num_candidates;
    
    dsp_real_t current_f0;          /* Currently tracked fundamental frequency */
    dsp_real_t sample_buffer[GOERTZEL_MAX_BLOCK_SIZE];
    uint16_t   buffer_index;
    bool       is_initialized;
} goertzel_tracker_t;

/* --- API Functions --- */

/**
 * Initializes a single-frequency Goertzel bin evaluator.
 */
void goertzel_bin_init(goertzel_bin_t *bin, dsp_real_t target_freq, dsp_real_t sample_rate);

/**
 * Resets state of single-frequency bin.
 */
void goertzel_bin_reset(goertzel_bin_t *bin);

/**
 * Processes a single input sample through the 2nd-order Goertzel recursion.
 */
static inline void goertzel_bin_update(goertzel_bin_t *bin, dsp_real_t sample) {
    dsp_real_t s0 = sample + bin->coeff * bin->s_prev1 - bin->s_prev2;
    bin->s_prev2 = bin->s_prev1;
    bin->s_prev1 = s0;
}

/**
 * Computes power at target frequency after feeding N samples.
 */
dsp_real_t goertzel_bin_compute_power(const goertzel_bin_t *bin);

/**
 * Computes block power for an array of samples.
 */
dsp_real_t goertzel_compute_power_block(
    const dsp_real_t *samples,
    uint16_t num_samples,
    dsp_real_t target_freq,
    dsp_real_t sample_rate
);

/**
 * Initializes the real-time Goertzel frequency tracker.
 */
dsp_status_t goertzel_tracker_init(
    goertzel_tracker_t *tracker,
    const goertzel_tracker_config_t *config
);

/**
 * Feeds a new input sample into tracker buffer.
 * When block is full, automatically updates tracked frequency.
 * 
 * @param tracker Pointer to tracker context.
 * @param sample New audio/sensor input sample.
 * @param f0_updated Output flag set to true if f0 was re-estimated on this sample.
 * @return Latest estimated fundamental frequency (Hz).
 */
dsp_real_t goertzel_tracker_feed_sample(
    goertzel_tracker_t *tracker,
    dsp_real_t sample,
    bool *f0_updated
);

/**
 * Directly analyzes a block of samples and estimates instantaneous fundamental frequency
 * using Goertzel spectral scan and 3-point parabolic peak interpolation.
 */
dsp_real_t goertzel_tracker_analyze_block(
    goertzel_tracker_t *tracker,
    const dsp_real_t *block,
    uint16_t num_samples
);

/**
 * Resets tracker context.
 */
void goertzel_tracker_reset(goertzel_tracker_t *tracker);

#ifdef __cplusplus
}
#endif

#endif /* GOERTZEL_H */
