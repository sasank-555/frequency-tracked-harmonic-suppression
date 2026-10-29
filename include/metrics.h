/**
 * @file metrics.h
 * @brief Performance evaluation metrics in C: Attenuation, SINAD, and Tracking Errors.
 */

#ifndef METRICS_H
#define METRICS_H

#include "dsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dsp_real_t harmonic_attenuation_db[MAX_HARMONICS];
    uint8_t    num_harmonics_evaluated;
    dsp_real_t avg_harmonic_attenuation_db;
    
    dsp_real_t sinad_input_db;
    dsp_real_t sinad_output_db;
    dsp_real_t sinad_improvement_db;
    
    dsp_real_t clean_signal_rmse;
    dsp_real_t prd_percent;           /* Percent Root-Mean-Square Difference */
    
    dsp_real_t tracking_mafe_hz;      /* Mean absolute frequency error */
    dsp_real_t tracking_rmse_hz;
    dsp_real_t max_tracking_error_hz;
} system_performance_report_t;

/**
 * Computes power at target frequency over an array using Goertzel algorithm.
 */
dsp_real_t metrics_measure_power_at_freq(
    const dsp_real_t *signal,
    uint32_t num_samples,
    dsp_real_t sample_rate,
    dsp_real_t target_freq
);

/**
 * Computes attenuation (dB) between input and output at a given frequency:
 * Attenuation_dB = 10 * log10( P_in / P_out )
 */
dsp_real_t metrics_calculate_attenuation_db(
    dsp_real_t power_in,
    dsp_real_t power_out
);

/**
 * Calculates SINAD (Signal-to-Interference-and-Noise Ratio) in dB.
 */
dsp_real_t metrics_calculate_sinad_db(
    const dsp_real_t *noisy_signal,
    const dsp_real_t *clean_reference,
    uint32_t num_samples
);

/**
 * Prints a formatted ASCII performance report to stdout/UART.
 */
void metrics_print_report(const system_performance_report_t *report);

#ifdef __cplusplus
}
#endif

#endif /* METRICS_H */
