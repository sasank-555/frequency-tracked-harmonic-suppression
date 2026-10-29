/**
 * @file goertzel.c
 * @brief Implementation of Goertzel algorithm and precision frequency tracker.
 */

#include "goertzel.h"
#include <string.h>
#include <math.h>

void goertzel_bin_init(goertzel_bin_t *bin, dsp_real_t target_freq, dsp_real_t sample_rate) {
    if (!bin || sample_rate <= 0.0) return;
    
    bin->omega = TWO_PI * target_freq / sample_rate;
    bin->coeff = 2.0 * cos(bin->omega);
    bin->s_prev1 = 0.0;
    bin->s_prev2 = 0.0;
}

void goertzel_bin_reset(goertzel_bin_t *bin) {
    if (!bin) return;
    bin->s_prev1 = 0.0;
    bin->s_prev2 = 0.0;
}

dsp_real_t goertzel_bin_compute_power(const goertzel_bin_t *bin) {
    if (!bin) return 0.0;
    dsp_real_t p = (bin->s_prev1 * bin->s_prev1) + 
                   (bin->s_prev2 * bin->s_prev2) - 
                   (bin->coeff * bin->s_prev1 * bin->s_prev2);
    return (p > 0.0) ? p : 0.0;
}

dsp_real_t goertzel_compute_power_block(
    const dsp_real_t *samples,
    uint16_t num_samples,
    dsp_real_t target_freq,
    dsp_real_t sample_rate
) {
    if (!samples || num_samples == 0 || sample_rate <= 0.0) return 0.0;
    
    dsp_real_t omega = TWO_PI * target_freq / sample_rate;
    dsp_real_t coeff = 2.0 * cos(omega);
    dsp_real_t s_prev1 = 0.0;
    dsp_real_t s_prev2 = 0.0;
    
    for (uint16_t i = 0; i < num_samples; i++) {
        dsp_real_t s0 = samples[i] + coeff * s_prev1 - s_prev2;
        s_prev2 = s_prev1;
        s_prev1 = s0;
    }
    
    dsp_real_t p = (s_prev1 * s_prev1) + (s_prev2 * s_prev2) - (coeff * s_prev1 * s_prev2);
    return (p > 0.0) ? p : 0.0;
}

dsp_status_t goertzel_tracker_init(
    goertzel_tracker_t *tracker,
    const goertzel_tracker_config_t *config
) {
    if (!tracker || !config) return DSP_ERROR_INVALID_PARAM;
    if (config->sample_rate <= 0.0 || config->step_hz <= 0.0 || config->block_size == 0) {
        return DSP_ERROR_INVALID_PARAM;
    }
    if (config->block_size > GOERTZEL_MAX_BLOCK_SIZE) {
        return DSP_ERROR_BUFFER_OVERFLOW;
    }
    
    tracker->config = *config;
    tracker->current_f0 = config->nominal_freq;
    tracker->buffer_index = 0;
    tracker->is_initialized = true;
    
    /* Precompute candidate frequency grid */
    dsp_real_t f_min = config->nominal_freq - config->search_span_hz;
    dsp_real_t f_max = config->nominal_freq + config->search_span_hz;
    if (f_min < 1.0) f_min = 1.0;
    if (f_max > config->sample_rate / 2.0 - 1.0) f_max = config->sample_rate / 2.0 - 1.0;
    
    uint16_t count = 0;
    for (dsp_real_t f = f_min; f <= f_max + 1e-6; f += config->step_hz) {
        if (count >= GOERTZEL_MAX_CANDIDATE_BINS) break;
        tracker->candidate_freqs[count] = f;
        dsp_real_t omega = TWO_PI * f / config->sample_rate;
        tracker->candidate_coeffs[count] = 2.0 * cos(omega);
        count++;
    }
    tracker->num_candidates = count;
    
    memset(tracker->sample_buffer, 0, sizeof(tracker->sample_buffer));
    return DSP_OK;
}

dsp_real_t goertzel_tracker_analyze_block(
    goertzel_tracker_t *tracker,
    const dsp_real_t *block,
    uint16_t num_samples
) {
    if (!tracker || !block || num_samples == 0 || tracker->num_candidates == 0) {
        return tracker ? tracker->current_f0 : 0.0;
    }
    
    dsp_real_t powers[GOERTZEL_MAX_CANDIDATE_BINS];
    dsp_real_t max_p = -1.0;
    int best_idx = 0;
    
    /* Scan candidate bins using Goertzel recurrence */
    for (uint16_t c = 0; c < tracker->num_candidates; c++) {
        dsp_real_t coeff = tracker->candidate_coeffs[c];
        dsp_real_t s_prev1 = 0.0;
        dsp_real_t s_prev2 = 0.0;
        
        for (uint16_t i = 0; i < num_samples; i++) {
            /* Optional Hamming window weighting on the fly */
            dsp_real_t w = 0.54 - 0.46 * cos(TWO_PI * i / (num_samples - 1));
            dsp_real_t s0 = (block[i] * w) + coeff * s_prev1 - s_prev2;
            s_prev2 = s_prev1;
            s_prev1 = s0;
        }
        
        dsp_real_t p = (s_prev1 * s_prev1) + (s_prev2 * s_prev2) - (coeff * s_prev1 * s_prev2);
        if (p < 0.0) p = 0.0;
        powers[c] = p;
        
        if (p > max_p) {
            max_p = p;
            best_idx = (int)c;
        }
    }
    
    /* 3-Point Parabolic Peak Interpolation for Sub-Bin Frequency Accuracy */
    dsp_real_t raw_f0 = tracker->candidate_freqs[best_idx];
    if (best_idx > 0 && best_idx < (int)tracker->num_candidates - 1) {
        dsp_real_t p_left = powers[best_idx - 1];
        dsp_real_t p_mid  = powers[best_idx];
        dsp_real_t p_right= powers[best_idx + 1];
        
        dsp_real_t denom = p_left - 2.0 * p_mid + p_right;
        if (fabs(denom) > 1e-12) {
            dsp_real_t delta = 0.5 * (p_left - p_right) / denom;
            if (delta < -1.0) delta = -1.0;
            if (delta > 1.0) delta = 1.0;
            raw_f0 += delta * tracker->config.step_hz;
        }
    }
    
    /* Exponential Moving Average (EMA) smoothing to eliminate single-sample jitter */
    dsp_real_t alpha = tracker->config.smoothing_alpha;
    tracker->current_f0 = alpha * raw_f0 + (1.0 - alpha) * tracker->current_f0;
    
    return tracker->current_f0;
}

dsp_real_t goertzel_tracker_feed_sample(
    goertzel_tracker_t *tracker,
    dsp_real_t sample,
    bool *f0_updated
) {
    if (f0_updated) *f0_updated = false;
    if (!tracker || !tracker->is_initialized) return 0.0;
    
    tracker->sample_buffer[tracker->buffer_index++] = sample;
    
    if (tracker->buffer_index >= tracker->config.block_size) {
        /* Analysis block is full: run tracking scan */
        goertzel_tracker_analyze_block(
            tracker,
            tracker->sample_buffer,
            tracker->config.block_size
        );
        
        /* Fast update: shift buffer by 12.5% (hop size) for high tracking responsiveness */
        uint16_t shift = tracker->config.block_size / 8;
        if (shift < 1) shift = 1;
        uint16_t remaining = tracker->config.block_size - shift;
        
        memmove(
            tracker->sample_buffer,
            &tracker->sample_buffer[shift],
            remaining * sizeof(dsp_real_t)
        );
        tracker->buffer_index = remaining;
        
        if (f0_updated) *f0_updated = true;
    }
    
    return tracker->current_f0;
}

void goertzel_tracker_reset(goertzel_tracker_t *tracker) {
    if (!tracker) return;
    tracker->current_f0 = tracker->config.nominal_freq;
    tracker->buffer_index = 0;
    memset(tracker->sample_buffer, 0, sizeof(tracker->sample_buffer));
}
