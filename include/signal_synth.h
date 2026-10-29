/**
 * @file signal_synth.h
 * @brief Embedded signal generator for test bench, verification, and hardware loop.
 */

#ifndef SIGNAL_SYNTH_H
#define SIGNAL_SYNTH_H

#include "dsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SIGNAL_TYPE_MULTITONE = 0,
    SIGNAL_TYPE_ECG,
    SIGNAL_TYPE_CHIRP
} signal_type_t;

typedef struct {
    dsp_real_t sample_rate;
    signal_type_t desired_type;
    dsp_real_t nominal_f0;
    dsp_real_t drift_amp_hz;
    dsp_real_t drift_rate_hz;
    uint8_t    num_harmonics;
    dsp_real_t interference_amp;
    dsp_real_t noise_stddev;
} signal_synth_config_t;

typedef struct {
    signal_synth_config_t config;
    dsp_real_t current_time;
    dsp_real_t fundamental_phase;
    dsp_real_t harmonic_phases[MAX_HARMONICS];
    uint32_t   sample_count;
    uint32_t   rng_state;
} signal_synth_t;

typedef struct {
    dsp_real_t clean_sample;
    dsp_real_t interference_sample;
    dsp_real_t noise_sample;
    dsp_real_t composite_sample;
    dsp_real_t instantaneous_f0;
} synth_sample_output_t;

void signal_synth_init(signal_synth_t *synth, const signal_synth_config_t *config);

synth_sample_output_t signal_synth_step(signal_synth_t *synth);

void signal_synth_reset(signal_synth_t *synth);

#ifdef __cplusplus
}
#endif

#endif /* SIGNAL_SYNTH_H */
