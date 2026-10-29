/**
 * @file harmonic_suppressor.c
 * @brief Implementation of integrated frequency-tracked harmonic suppressor.
 */

#include "harmonic_suppressor.h"
#include <string.h>
#include <math.h>

dsp_status_t suppressor_init(
    harmonic_suppressor_t *sys,
    const suppressor_config_t *config
) {
    if (!sys || !config) return DSP_ERROR_INVALID_PARAM;
    
    sys->config = *config;
    sys->active_f0 = config->nominal_f0;
    
    /* 1. Initialize Goertzel Tracker */
    goertzel_tracker_config_t trk_cfg;
    trk_cfg.sample_rate = config->sample_rate;
    trk_cfg.nominal_freq = config->nominal_f0;
    trk_cfg.search_span_hz = config->search_span_hz;
    trk_cfg.step_hz = config->step_hz;
    trk_cfg.smoothing_alpha = config->smoothing_alpha;
    trk_cfg.block_size = config->analysis_block_size;
    
    dsp_status_t st = goertzel_tracker_init(&sys->tracker, &trk_cfg);
    if (st != DSP_OK) return st;
    
    /* 2. Initialize FIR Filter */
    if (config->filter_mode == FILTER_MODE_CASCADE_FIR) {
        st = fir_cascade_init(
            &sys->cascade_filter,
            config->sample_rate,
            config->nominal_f0,
            config->num_harmonics
        );
        if (st != DSP_OK) return st;
        /* Linear phase group delay for cascaded biquads: 1 sample per stage */
        sys->group_delay = config->num_harmonics;
    } else {
        st = fir_windowed_init(
            &sys->windowed_filter,
            config->sample_rate,
            config->windowed_taps,
            config->notch_bandwidth_hz,
            config->nominal_f0,
            config->num_harmonics
        );
        if (st != DSP_OK) return st;
        sys->group_delay = (config->windowed_taps - 1) / 2;
    }
    
    sys->is_initialized = true;
    return DSP_OK;
}

dsp_real_t suppressor_process_sample(
    harmonic_suppressor_t *sys,
    dsp_real_t sample_in,
    dsp_real_t *tracked_f0_out
) {
    if (!sys || !sys->is_initialized) return sample_in;
    
    bool f0_updated = false;
    dsp_real_t est_f0 = goertzel_tracker_feed_sample(&sys->tracker, sample_in, &f0_updated);
    
    if (tracked_f0_out) {
        *tracked_f0_out = est_f0;
    }
    
    /* Retune filter if frequency drift exceeds threshold */
    if (f0_updated && fabs(est_f0 - sys->active_f0) >= sys->config.retune_threshold_hz) {
        sys->active_f0 = est_f0;
        if (sys->config.filter_mode == FILTER_MODE_CASCADE_FIR) {
            fir_cascade_update_frequency(&sys->cascade_filter, sys->active_f0);
        } else {
            fir_windowed_update_frequency(&sys->windowed_filter, sys->active_f0);
        }
    }
    
    /* Apply Linear-Phase FIR Filtering */
    dsp_real_t sample_out;
    if (sys->config.filter_mode == FILTER_MODE_CASCADE_FIR) {
        sample_out = fir_cascade_process_sample(&sys->cascade_filter, sample_in);
    } else {
        sample_out = fir_windowed_process_sample(&sys->windowed_filter, sample_in);
    }
    
    return sample_out;
}

void suppressor_process_block(
    harmonic_suppressor_t *sys,
    const dsp_real_t *samples_in,
    dsp_real_t *samples_out,
    dsp_real_t *tracked_f0_out,
    uint16_t length
) {
    if (!sys || !samples_in || !samples_out) return;
    
    for (uint16_t i = 0; i < length; i++) {
        dsp_real_t f0 = 0.0;
        samples_out[i] = suppressor_process_sample(sys, samples_in[i], &f0);
        if (tracked_f0_out) {
            tracked_f0_out[i] = f0;
        }
    }
}

void suppressor_reset(harmonic_suppressor_t *sys) {
    if (!sys) return;
    goertzel_tracker_reset(&sys->tracker);
    sys->active_f0 = sys->config.nominal_f0;
    if (sys->config.filter_mode == FILTER_MODE_CASCADE_FIR) {
        fir_cascade_reset(&sys->cascade_filter);
    } else {
        fir_windowed_reset(&sys->windowed_filter);
    }
}
