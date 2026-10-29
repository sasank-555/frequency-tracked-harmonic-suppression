/**
 * @file harmonic_suppressor.h
 * @brief Top-level Frequency-Tracked Harmonic Interference Suppression System.
 */

#ifndef HARMONIC_SUPPRESSOR_H
#define HARMONIC_SUPPRESSOR_H

#include "goertzel.h"
#include "fir_notch.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FILTER_MODE_CASCADE_FIR = 0, /* Cascaded linear-phase zeros (minimal latency & MIPS, extreme attenuation) */
    FILTER_MODE_WINDOWED_FIR     /* Windowed-sinc multi-notch FIR (tapered response) */
} filter_mode_t;

typedef struct {
    dsp_real_t    sample_rate;
    dsp_real_t    nominal_f0;
    uint8_t       num_harmonics;
    filter_mode_t filter_mode;
    uint16_t      windowed_taps;
    dsp_real_t    notch_bandwidth_hz;
    dsp_real_t    retune_threshold_hz; /* Re-tune filter when |f0_est - f0_active| > threshold */
    dsp_real_t    search_span_hz;
    dsp_real_t    step_hz;
    dsp_real_t    smoothing_alpha;
    uint16_t      analysis_block_size;
} suppressor_config_t;

typedef struct {
    suppressor_config_t  config;
    goertzel_tracker_t   tracker;
    fir_cascade_notch_t  cascade_filter;
    fir_windowed_notch_t windowed_filter;
    
    dsp_real_t           active_f0;
    uint16_t             group_delay;
    bool                 is_initialized;
} harmonic_suppressor_t;

dsp_status_t suppressor_init(
    harmonic_suppressor_t *sys,
    const suppressor_config_t *config
);

/**
 * Real-time streaming sample processor.
 * 
 * @param sys Pointer to suppressor instance.
 * @param sample_in Raw incoming sample (signal + drifting interference + noise).
 * @param tracked_f0_out Optional pointer to output current estimated f0.
 * @return Cleaned, filtered output sample.
 */
dsp_real_t suppressor_process_sample(
    harmonic_suppressor_t *sys,
    dsp_real_t sample_in,
    dsp_real_t *tracked_f0_out
);

void suppressor_process_block(
    harmonic_suppressor_t *sys,
    const dsp_real_t *samples_in,
    dsp_real_t *samples_out,
    dsp_real_t *tracked_f0_out,
    uint16_t length
);

void suppressor_reset(harmonic_suppressor_t *sys);

#ifdef __cplusplus
}
#endif

#endif /* HARMONIC_SUPPRESSOR_H */
