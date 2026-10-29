/**
 * @file fir_notch.c
 * @brief Implementation of Linear-Phase FIR Harmonic Notch Filters.
 */

#include "fir_notch.h"
#include <string.h>
#include <math.h>

/* --- Cascaded Zero-Placement FIR Multi-Notch --- */

dsp_status_t fir_cascade_init(
    fir_cascade_notch_t *filter,
    dsp_real_t sample_rate,
    dsp_real_t initial_f0,
    uint8_t num_harmonics
) {
    if (!filter || sample_rate <= 0.0 || initial_f0 <= 0.0 || num_harmonics == 0) {
        return DSP_ERROR_INVALID_PARAM;
    }
    if (num_harmonics > MAX_HARMONICS) {
        num_harmonics = MAX_HARMONICS;
    }
    
    filter->sample_rate = sample_rate;
    filter->current_f0 = initial_f0;
    filter->num_stages = num_harmonics;
    
    fir_cascade_update_frequency(filter, initial_f0);
    fir_cascade_reset(filter);
    
    return DSP_OK;
}

void fir_cascade_update_frequency(
    fir_cascade_notch_t *filter,
    dsp_real_t new_f0
) {
    if (!filter || new_f0 <= 0.0) return;
    
    filter->current_f0 = new_f0;
    dsp_real_t nyquist = filter->sample_rate / 2.0;
    
    for (uint8_t k = 0; k < filter->num_stages; k++) {
        dsp_real_t fk = (k + 1) * new_f0;
        fir_biquad_notch_t *st = &filter->stages[k];
        st->center_freq = fk;
        
        if (fk >= nyquist - 2.0) {
            /* Pass-through if beyond Nyquist */
            st->b0 = 1.0;
            st->b1 = 0.0;
            st->b2 = 0.0;
            continue;
        }
        
        dsp_real_t omega = TWO_PI * fk / filter->sample_rate;
        dsp_real_t cos_w = cos(omega);
        
        /* Normalization factor so DC gain is exactly 1.0: (2 - 2*cos(omega)) */
        dsp_real_t norm = 2.0 - 2.0 * cos_w;
        if (fabs(norm) < 1e-12) norm = 1.0;
        
        st->b0 = 1.0 / norm;
        st->b1 = -2.0 * cos_w / norm;
        st->b2 = 1.0 / norm;
    }
}

dsp_real_t fir_cascade_process_sample(
    fir_cascade_notch_t *filter,
    dsp_real_t x
) {
    if (!filter) return x;
    
    dsp_real_t current_in = x;
    
    /* Cascaded processing through each 2nd-order FIR notch stage */
    for (uint8_t k = 0; k < filter->num_stages; k++) {
        fir_biquad_notch_t *st = &filter->stages[k];
        
        /* Direct Form FIR: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] */
        dsp_real_t y = st->b0 * current_in + st->b1 * st->x1 + st->b2 * st->x2;
        
        /* Update delay line */
        st->x2 = st->x1;
        st->x1 = current_in;
        
        current_in = y;
    }
    
    return current_in;
}

void fir_cascade_process_block(
    fir_cascade_notch_t *filter,
    const dsp_real_t *input,
    dsp_real_t *output,
    uint16_t length
) {
    if (!filter || !input || !output) return;
    for (uint16_t i = 0; i < length; i++) {
        output[i] = fir_cascade_process_sample(filter, input[i]);
    }
}

void fir_cascade_reset(fir_cascade_notch_t *filter) {
    if (!filter) return;
    for (uint8_t k = 0; k < filter->num_stages; k++) {
        filter->stages[k].x1 = 0.0;
        filter->stages[k].x2 = 0.0;
    }
}

/* --- Windowed Sinc Multi-Bandstop Linear-Phase FIR Filter --- */

static dsp_real_t sinc(dsp_real_t val) {
    if (fabs(val) < 1e-12) return 1.0;
    dsp_real_t pix = M_PI * val;
    return sin(pix) / pix;
}

static dsp_real_t blackman_window(int n, int M) {
    if (M <= 1) return 1.0;
    dsp_real_t a0 = 0.42;
    dsp_real_t a1 = 0.50;
    dsp_real_t a2 = 0.08;
    dsp_real_t phi = TWO_PI * n / (M - 1);
    return a0 - a1 * cos(phi) + a2 * cos(2.0 * phi);
}

dsp_status_t fir_windowed_init(
    fir_windowed_notch_t *filter,
    dsp_real_t sample_rate,
    uint16_t num_taps,
    dsp_real_t notch_bandwidth,
    dsp_real_t initial_f0,
    uint8_t num_harmonics
) {
    if (!filter || sample_rate <= 0.0 || num_taps == 0 || initial_f0 <= 0.0) {
        return DSP_ERROR_INVALID_PARAM;
    }
    if (num_taps > MAX_FIR_TAPS) num_taps = MAX_FIR_TAPS;
    /* Force odd number of taps for Type-I linear phase */
    if (num_taps % 2 == 0) num_taps++;
    
    filter->sample_rate = sample_rate;
    filter->num_taps = num_taps;
    filter->notch_bandwidth = notch_bandwidth;
    filter->num_harmonics = num_harmonics;
    filter->current_f0 = initial_f0;
    filter->buffer_index = 0;
    
    fir_windowed_update_frequency(filter, initial_f0);
    fir_windowed_reset(filter);
    
    return DSP_OK;
}

void fir_windowed_update_frequency(
    fir_windowed_notch_t *filter,
    dsp_real_t new_f0
) {
    if (!filter || new_f0 <= 0.0) return;
    filter->current_f0 = new_f0;
    
    uint16_t M = filter->num_taps;
    dsp_real_t tau = (M - 1) / 2.0;
    dsp_real_t fs = filter->sample_rate;
    dsp_real_t B = filter->notch_bandwidth;
    dsp_real_t fc = B / (2.0 * fs);
    dsp_real_t nyquist = fs / 2.0;
    
    /* Temporary buffers for bandpass calculations */
    dsp_real_t h_bp_total[MAX_FIR_TAPS];
    memset(h_bp_total, 0, sizeof(h_bp_total));
    
    for (uint8_t k = 1; k <= filter->num_harmonics; k++) {
        dsp_real_t fk = k * new_f0;
        if (fk >= nyquist - 2.0) break;
        
        dsp_real_t h_bp[MAX_FIR_TAPS];
        dsp_real_t g_k = 0.0;
        
        for (uint16_t n = 0; n < M; n++) {
            dsp_real_t m = (dsp_real_t)n - tau;
            dsp_real_t h_lp = 2.0 * fc * sinc(2.0 * fc * m);
            dsp_real_t cos_mod = 2.0 * cos(TWO_PI * fk * m / fs);
            dsp_real_t w = blackman_window(n, M);
            h_bp[n] = h_lp * cos_mod * w;
            g_k += h_bp[n] * cos(TWO_PI * fk * m / fs);
        }
        
        /* Normalize peak bandpass gain to exactly 1.000 */
        if (fabs(g_k) > 1e-6) {
            dsp_real_t inv_g = 1.0 / g_k;
            for (uint16_t n = 0; n < M; n++) {
                h_bp_total[n] += (h_bp[n] * inv_g);
            }
        }
    }
    
    /* Subtract from delta: H_notch = 1 - sum(H_BP,k) */
    for (uint16_t n = 0; n < M; n++) {
        dsp_real_t m = (dsp_real_t)n - tau;
        dsp_real_t delta = (fabs(m) < 1e-6) ? 1.0 : 0.0;
        filter->coeffs[n] = delta - h_bp_total[n];
    }
}

dsp_real_t fir_windowed_process_sample(
    fir_windowed_notch_t *filter,
    dsp_real_t x
) {
    if (!filter) return x;
    
    filter->state_buffer[filter->buffer_index] = x;
    
    dsp_real_t acc = 0.0;
    uint16_t idx = filter->buffer_index;
    uint16_t M = filter->num_taps;
    
    for (uint16_t i = 0; i < M; i++) {
        acc += filter->coeffs[i] * filter->state_buffer[idx];
        if (idx == 0) {
            idx = M - 1;
        } else {
            idx--;
        }
    }
    
    filter->buffer_index++;
    if (filter->buffer_index >= M) {
        filter->buffer_index = 0;
    }
    
    return acc;
}

void fir_windowed_process_block(
    fir_windowed_notch_t *filter,
    const dsp_real_t *input,
    dsp_real_t *output,
    uint16_t length
) {
    if (!filter || !input || !output) return;
    for (uint16_t i = 0; i < length; i++) {
        output[i] = fir_windowed_process_sample(filter, input[i]);
    }
}

void fir_windowed_reset(fir_windowed_notch_t *filter) {
    if (!filter) return;
    filter->buffer_index = 0;
    memset(filter->state_buffer, 0, sizeof(filter->state_buffer));
}

dsp_real_t fir_calculate_attenuation_db(
    dsp_real_t sample_rate,
    const dsp_real_t *coeffs,
    uint16_t num_taps,
    dsp_real_t target_freq
) {
    if (!coeffs || num_taps == 0 || sample_rate <= 0.0) return 0.0;
    
    dsp_real_t omega = TWO_PI * target_freq / sample_rate;
    dsp_real_t re = 0.0;
    dsp_real_t im = 0.0;
    
    for (uint16_t n = 0; n < num_taps; n++) {
        re += coeffs[n] * cos(omega * n);
        im -= coeffs[n] * sin(omega * n);
    }
    
    dsp_real_t mag_sq = re * re + im * im;
    if (mag_sq < 1e-16) mag_sq = 1e-16;
    
    dsp_real_t gain_db = 10.0 * log10(mag_sq);
    return -gain_db; /* Attenuation in positive dB */
}
