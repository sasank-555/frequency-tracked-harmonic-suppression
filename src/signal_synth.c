/**
 * @file signal_synth.c
 * @brief Implementation of embedded test signal synthesizer.
 */

#include "signal_synth.h"
#include <math.h>

/* Simple, fast linear congruential generator for embedded PRNG */
static uint32_t lcg_rand(uint32_t *state) {
    *state = (*state * 1664525u + 1013904223u);
    return *state;
}

static dsp_real_t rand_uniform(uint32_t *state) {
    return (dsp_real_t)lcg_rand(state) / 4294967296.0;
}

static dsp_real_t rand_gaussian(uint32_t *state, dsp_real_t stddev) {
    dsp_real_t u1 = rand_uniform(state);
    dsp_real_t u2 = rand_uniform(state);
    if (u1 < 1e-10) u1 = 1e-10;
    dsp_real_t z0 = sqrt(-2.0 * log(u1)) * cos(TWO_PI * u2);
    return z0 * stddev;
}

void signal_synth_init(signal_synth_t *synth, const signal_synth_config_t *config) {
    if (!synth || !config) return;
    synth->config = *config;
    synth->current_time = 0.0;
    synth->fundamental_phase = 0.0;
    synth->sample_count = 0;
    synth->rng_state = 123456789u;
    
    for (uint8_t k = 0; k < MAX_HARMONICS; k++) {
        synth->harmonic_phases[k] = (dsp_real_t)k * 0.785398; /* arbitrary fixed phases */
    }
}

static dsp_real_t synth_clean_multitone(dsp_real_t t) {
    return 1.00 * sin(TWO_PI * 22.0  * t + 0.1) +
           0.70 * sin(TWO_PI * 78.0  * t + 0.9) +
           0.50 * sin(TWO_PI * 128.0 * t + 1.7) +
           0.35 * sin(TWO_PI * 185.0 * t + 2.5);
}

static dsp_real_t synth_clean_ecg(dsp_real_t t) {
    /* 72 bpm -> 0.833 sec period */
    dsp_real_t period = 0.833333;
    dsp_real_t t_mod = fmod(t, period);
    dsp_real_t ecg = 0.0;
    
    /* Gaussian approximations for P, Q, R, S, T waves */
    struct { dsp_real_t center, amp, width; } waves[5] = {
        { 0.15,  0.18, 0.035 }, /* P */
        { 0.26, -0.20, 0.015 }, /* Q */
        { 0.31,  1.30, 0.012 }, /* R */
        { 0.36, -0.35, 0.018 }, /* S */
        { 0.52,  0.35, 0.065 }  /* T */
    };
    
    for (int i = 0; i < 5; i++) {
        dsp_real_t dt = t_mod - waves[i].center;
        ecg += waves[i].amp * exp(-0.5 * (dt * dt) / (waves[i].width * waves[i].width));
    }
    return ecg;
}

synth_sample_output_t signal_synth_step(signal_synth_t *synth) {
    synth_sample_output_t out;
    if (!synth) {
        out.clean_sample = 0.0;
        out.interference_sample = 0.0;
        out.noise_sample = 0.0;
        out.composite_sample = 0.0;
        out.instantaneous_f0 = 0.0;
        return out;
    }
    
    dsp_real_t t = synth->current_time;
    dsp_real_t dt = 1.0 / synth->config.sample_rate;
    
    /* 1. Instantaneous drifting fundamental frequency f0(t) */
    dsp_real_t drift = synth->config.drift_amp_hz * sin(TWO_PI * synth->config.drift_rate_hz * t);
    dsp_real_t f0_inst = synth->config.nominal_f0 + drift;
    out.instantaneous_f0 = f0_inst;
    
    /* Accumulate continuous phase: theta += 2*pi*f0*dt */
    synth->fundamental_phase += TWO_PI * f0_inst * dt;
    if (synth->fundamental_phase > TWO_PI) {
        synth->fundamental_phase -= TWO_PI;
    }
    
    /* 2. Generate Clean Signal */
    if (synth->config.desired_type == SIGNAL_TYPE_ECG) {
        out.clean_sample = synth_clean_ecg(t);
    } else {
        out.clean_sample = synth_clean_multitone(t);
    }
    
    /* 3. Generate Drifting Multi-Harmonic Interference */
    dsp_real_t interf = 0.0;
    uint8_t K = synth->config.num_harmonics;
    if (K > MAX_HARMONICS) K = MAX_HARMONICS;
    
    for (uint8_t k = 1; k <= K; k++) {
        /* Harmonic amplitude decay: A_k = A0 / (k^0.75) */
        dsp_real_t amp_k = synth->config.interference_amp / pow((dsp_real_t)k, 0.75);
        dsp_real_t phase_k = (dsp_real_t)k * synth->fundamental_phase + synth->harmonic_phases[k - 1];
        interf += amp_k * sin(phase_k);
    }
    out.interference_sample = interf;
    
    /* 4. Additive Gaussian Noise */
    out.noise_sample = rand_gaussian(&synth->rng_state, synth->config.noise_stddev);
    
    /* 5. Composite Input */
    out.composite_sample = out.clean_sample + out.interference_sample + out.noise_sample;
    
    /* Advance time */
    synth->current_time += dt;
    synth->sample_count++;
    
    return out;
}

void signal_synth_reset(signal_synth_t *synth) {
    if (!synth) return;
    synth->current_time = 0.0;
    synth->fundamental_phase = 0.0;
    synth->sample_count = 0;
    synth->rng_state = 123456789u;
}
