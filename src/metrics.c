/**
 * @file metrics.c
 * @brief Performance evaluation metrics and formatted reporting.
 */

#include "metrics.h"
#include "goertzel.h"
#include <stdio.h>
#include <math.h>

dsp_real_t metrics_measure_power_at_freq(
    const dsp_real_t *signal,
    uint32_t num_samples,
    dsp_real_t sample_rate,
    dsp_real_t target_freq
) {
    if (!signal || num_samples == 0 || sample_rate <= 0.0) return 0.0;
    
    dsp_real_t p_raw = goertzel_compute_power_block(
        signal,
        (uint16_t)(num_samples > 65535 ? 65535 : num_samples),
        target_freq,
        sample_rate
    );
    
    /* Normalized power */
    dsp_real_t p_norm = p_raw / ((dsp_real_t)num_samples * (dsp_real_t)num_samples);
    return (p_norm > 1e-18) ? p_norm : 1e-18;
}

dsp_real_t metrics_calculate_attenuation_db(
    dsp_real_t power_in,
    dsp_real_t power_out
) {
    if (power_out < 1e-18) power_out = 1e-18;
    if (power_in < 1e-18) power_in = 1e-18;
    
    dsp_real_t ratio = power_in / power_out;
    return 10.0 * log10(ratio);
}

dsp_real_t metrics_calculate_sinad_db(
    const dsp_real_t *noisy_signal,
    const dsp_real_t *clean_reference,
    uint32_t num_samples
) {
    if (!noisy_signal || !clean_reference || num_samples == 0) return 0.0;
    
    dsp_real_t p_tot = 0.0;
    dsp_real_t p_noise = 0.0;
    
    for (uint32_t i = 0; i < num_samples; i++) {
        p_tot += noisy_signal[i] * noisy_signal[i];
        dsp_real_t diff = noisy_signal[i] - clean_reference[i];
        p_noise += diff * diff;
    }
    
    p_tot /= (dsp_real_t)num_samples;
    p_noise /= (dsp_real_t)num_samples;
    
    if (p_noise < 1e-16) return 100.0;
    if (p_tot < 1e-16) return 0.0;
    
    return 10.0 * log10(p_tot / p_noise);
}

void metrics_print_report(const system_performance_report_t *report) {
    if (!report) return;
    
    printf("\n================================================================================\n");
    printf("   FREQUENCY-TRACKED HARMONIC INTERFERENCE SUPPRESSION - PERFORMANCE REPORT    \n");
    printf("================================================================================\n");
    
    printf("\n--- HARMONIC INTERFERENCE ATTENUATION METRICS ---\n");
    printf("+----------+-----------------+---------------------------------+\n");
    printf("| Harmonic | Harmonic Index  | Frequency Attenuation Depth(dB) |\n");
    printf("+----------+-----------------+---------------------------------+\n");
    for (uint8_t k = 0; k < report->num_harmonics_evaluated; k++) {
        printf("|    k=%u   |   %2u-th Harmonic |            %7.2f dB             |\n",
               k + 1, k + 1, report->harmonic_attenuation_db[k]);
    }
    printf("+----------+-----------------+---------------------------------+\n");
    printf("| AVERAGE HARMONIC ATTENUATION:             %7.2f dB             |\n",
           report->avg_harmonic_attenuation_db);
    printf("+--------------------------------------------------------------+\n");
    
    printf("\n--- SIGNAL QUALITY & SINAD METRICS ---\n");
    printf("  * Input SINAD:                   %6.2f dB\n", report->sinad_input_db);
    printf("  * Output SINAD (Post-Filtering): %6.2f dB\n", report->sinad_output_db);
    printf("  * Net SINAD Improvement:         %6.2f dB\n", report->sinad_improvement_db);
    printf("  * Clean Signal RMSE:             %8.4f\n", report->clean_signal_rmse);
    printf("  * Percent RMS Difference (PRD):  %6.2f %%\n", report->prd_percent);
    
    printf("\n--- GOERTZEL FREQUENCY TRACKING PERFORMANCE ---\n");
    printf("  * Mean Absolute Freq Error:      %6.4f Hz\n", report->tracking_mafe_hz);
    printf("  * Tracking RMSE:                 %6.4f Hz\n", report->tracking_rmse_hz);
    printf("  * Peak Tracking Error:           %6.4f Hz\n", report->max_tracking_error_hz);
    printf("================================================================================\n\n");
}
