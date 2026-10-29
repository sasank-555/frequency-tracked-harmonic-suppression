/**
 * @file main.c
 * @brief Embedded DSP Simulation & Benchmark Runner for Frequency-Tracked Harmonic Suppression.
 * 
 * Title: "Design and Performance Optimization of a Frequency-Tracked Harmonic
 *        Interference Suppression System Using Goertzel Analysis and FIR Filtering"
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "harmonic_suppressor.h"
#include "signal_synth.h"
#include "metrics.h"
#include "tms320c6748_sim.h"

#define SIM_SAMPLE_RATE     1000.0   /* 1 kHz sampling rate */
#define SIM_DURATION_SEC    5.0      /* 5 seconds */
#define TOTAL_SAMPLES       ((uint32_t)(SIM_SAMPLE_RATE * SIM_DURATION_SEC))
#define NOMINAL_F0          50.0     /* 50 Hz powerline hum */
#define NUM_HARMONICS       4        /* 50, 100, 150, 200 Hz */
#define DRIFT_AMP_HZ        1.5      /* +/- 1.5 Hz dynamic frequency drift */
#define DRIFT_RATE_HZ       0.4      /* Drift variation cycle */

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    printf("================================================================================\n");
    printf("  DESIGN AND PERFORMANCE OPTIMIZATION OF A FREQUENCY-TRACKED HARMONIC\n");
    printf("  INTERFERENCE SUPPRESSION SYSTEM USING GOERTZEL ANALYSIS AND FIR FILTERING    \n");
    printf("================================================================================\n");
    printf("Target Hardware Platform  : Texas Instruments TMS320C6748 Floating-Point DSP\n");
    printf("DSP Clock Frequency       : 456.0 MHz (C674x VLIW Core)\n");
    printf("Audio / Sensor Interface  : McASP Serial Port with EDMA3 Ping-Pong Buffers\n");
    printf("Sampling Rate             : %.1f Hz\n", SIM_SAMPLE_RATE);
    printf("Duration                  : %.1f s (%u samples)\n", SIM_DURATION_SEC, TOTAL_SAMPLES);
    printf("Nominal Fundamental (f0)  : %.1f Hz (Dynamic Drift: +/-%.2f Hz)\n", NOMINAL_F0, DRIFT_AMP_HZ);
    printf("Suppressed Harmonics      : %u (Fundamental + %u overtones: 50, 100, 150, 200 Hz)\n",
           NUM_HARMONICS, NUM_HARMONICS - 1);
    printf("Filter Architecture       : Linear-Phase FIR (151 Taps, Blackman Windowed)\n");
    printf("--------------------------------------------------------------------------------\n\n");

    /* Allocate simulation memory buffers */
    dsp_real_t *t_vec          = (dsp_real_t *)malloc(TOTAL_SAMPLES * sizeof(dsp_real_t));
    dsp_real_t *clean_sig      = (dsp_real_t *)malloc(TOTAL_SAMPLES * sizeof(dsp_real_t));
    dsp_real_t *noisy_input    = (dsp_real_t *)malloc(TOTAL_SAMPLES * sizeof(dsp_real_t));
    dsp_real_t *true_f0        = (dsp_real_t *)malloc(TOTAL_SAMPLES * sizeof(dsp_real_t));
    dsp_real_t *out_tracked    = (dsp_real_t *)malloc(TOTAL_SAMPLES * sizeof(dsp_real_t));
    dsp_real_t *est_f0_vec     = (dsp_real_t *)malloc(TOTAL_SAMPLES * sizeof(dsp_real_t));
    dsp_real_t *out_static     = (dsp_real_t *)malloc(TOTAL_SAMPLES * sizeof(dsp_real_t));

    if (!t_vec || !clean_sig || !noisy_input || !true_f0 || !out_tracked || !est_f0_vec || !out_static) {
        fprintf(stderr, "Error: Memory allocation failed!\n");
        return -1;
    }

    /* 1. Initialize Signal Synthesizer */
    signal_synth_t synth;
    signal_synth_config_t synth_cfg;
    synth_cfg.sample_rate = SIM_SAMPLE_RATE;
    synth_cfg.desired_type = SIGNAL_TYPE_MULTITONE;
    synth_cfg.nominal_f0 = NOMINAL_F0;
    synth_cfg.drift_amp_hz = DRIFT_AMP_HZ;
    synth_cfg.drift_rate_hz = DRIFT_RATE_HZ;
    synth_cfg.num_harmonics = NUM_HARMONICS;
    synth_cfg.interference_amp = 2.5;   /* Strong interference */
    synth_cfg.noise_stddev = 0.05;      /* AWGN */
    signal_synth_init(&synth, &synth_cfg);

    /* 2. Initialize Frequency-Tracked Suppressor */
    harmonic_suppressor_t suppressor;
    suppressor_config_t supp_cfg;
    supp_cfg.sample_rate = SIM_SAMPLE_RATE;
    supp_cfg.nominal_f0 = NOMINAL_F0;
    supp_cfg.num_harmonics = NUM_HARMONICS;
    supp_cfg.filter_mode = FILTER_MODE_WINDOWED_FIR;
    supp_cfg.windowed_taps = 151;
    supp_cfg.notch_bandwidth_hz = 8.0;
    supp_cfg.retune_threshold_hz = 0.04;
    supp_cfg.search_span_hz = 3.5;
    supp_cfg.step_hz = 0.15;
    supp_cfg.smoothing_alpha = 0.45;
    supp_cfg.analysis_block_size = 250; /* 250 ms analysis window */
    suppressor_init(&suppressor, &supp_cfg);

    /* 3. Initialize Static (Non-Tracking) Benchmark Filter */
    fir_windowed_notch_t static_filter;
    fir_windowed_init(&static_filter, SIM_SAMPLE_RATE, 151, 8.0, NOMINAL_F0, NUM_HARMONICS);

    /* 4. Initialize TMS320C6748 Hardware Profiler */
    c6748_profile_t c6748_prof;
    c6748_sim_init(&c6748_prof);

    printf(">> Executing real-time streaming processing loop (TMS320C6748 McASP/EDMA Emulation)...\n");

    /* 5. Real-Time Streaming Processing Loop */
    for (uint32_t i = 0; i < TOTAL_SAMPLES; i++) {
        t_vec[i] = (dsp_real_t)i / SIM_SAMPLE_RATE;
        
        /* Synthesize incoming hardware sample */
        synth_sample_output_t s = signal_synth_step(&synth);
        clean_sig[i]   = s.clean_sample;
        noisy_input[i] = s.composite_sample;
        true_f0[i]     = s.instantaneous_f0;

        /* Process with proposed Frequency-Tracked System */
        dsp_real_t f0_tracked = 0.0;
        out_tracked[i] = suppressor_process_sample(&suppressor, noisy_input[i], &f0_tracked);
        est_f0_vec[i]  = f0_tracked;

        /* Process with baseline Static (Non-Tracking) Filter */
        out_static[i]  = fir_windowed_process_sample(&static_filter, noisy_input[i]);

        /* Simulate EDMA3 Ping-Pong DMA buffer interrupt on TMS320C6748 */
        if ((i + 1) % C6748_EDMA_PING_PONG_SIZE == 0) {
            c6748_sim_record_block(&c6748_prof, C6748_EDMA_PING_PONG_SIZE, SIM_SAMPLE_RATE);
        }
    }
    printf(">> Real-time processing complete (%u samples processed).\n\n", TOTAL_SAMPLES);

    /* 5. Quantitative Metric Calculations */
    system_performance_report_t report_tracked;
    memset(&report_tracked, 0, sizeof(report_tracked));
    report_tracked.num_harmonics_evaluated = NUM_HARMONICS;

    system_performance_report_t report_static;
    memset(&report_static, 0, sizeof(report_static));
    report_static.num_harmonics_evaluated = NUM_HARMONICS;

    dsp_real_t sum_att_tracked = 0.0;
    dsp_real_t sum_att_static  = 0.0;

    /* Measure spectral power and attenuation at each harmonic */
    for (uint8_t k = 0; k < NUM_HARMONICS; k++) {
        dsp_real_t fk = (k + 1) * NOMINAL_F0;
        dsp_real_t p_in   = metrics_measure_power_at_freq(noisy_input, TOTAL_SAMPLES, SIM_SAMPLE_RATE, fk);
        dsp_real_t p_trkd = metrics_measure_power_at_freq(out_tracked, TOTAL_SAMPLES, SIM_SAMPLE_RATE, fk);
        dsp_real_t p_stat = metrics_measure_power_at_freq(out_static,  TOTAL_SAMPLES, SIM_SAMPLE_RATE, fk);

        dsp_real_t att_trkd = metrics_calculate_attenuation_db(p_in, p_trkd);
        dsp_real_t att_stat = metrics_calculate_attenuation_db(p_in, p_stat);

        report_tracked.harmonic_attenuation_db[k] = att_trkd;
        report_static.harmonic_attenuation_db[k]  = att_stat;

        sum_att_tracked += att_trkd;
        sum_att_static  += att_stat;
    }
    report_tracked.avg_harmonic_attenuation_db = sum_att_tracked / NUM_HARMONICS;
    report_static.avg_harmonic_attenuation_db  = sum_att_static / NUM_HARMONICS;

    /* SINAD and Interference Suppression Metrics (Delay Compensated) */
    uint16_t gd = suppressor.group_delay;
    uint32_t valid_n = TOTAL_SAMPLES - gd;

    report_tracked.sinad_input_db  = metrics_calculate_sinad_db(noisy_input, clean_sig, valid_n);
    report_tracked.sinad_output_db = metrics_calculate_sinad_db(&out_tracked[gd], clean_sig, valid_n);
    report_tracked.sinad_improvement_db = report_tracked.sinad_output_db - report_tracked.sinad_input_db;

    report_static.sinad_input_db  = report_tracked.sinad_input_db;
    report_static.sinad_output_db = metrics_calculate_sinad_db(&out_static[gd], clean_sig, valid_n);
    report_static.sinad_improvement_db = report_static.sinad_output_db - report_static.sinad_input_db;

    /* Total Harmonic Interference Power Attenuation */
    dsp_real_t p_interf_in = 0.0, p_res_tracked = 0.0, p_res_static = 0.0;
    for (uint32_t i = 0; i < valid_n; i++) {
        dsp_real_t in_err = noisy_input[i] - clean_sig[i];
        dsp_real_t trkd_err = out_tracked[i + gd] - clean_sig[i];
        dsp_real_t stat_err = out_static[i + gd] - clean_sig[i];
        p_interf_in += in_err * in_err;
        p_res_tracked += trkd_err * trkd_err;
        p_res_static += stat_err * stat_err;
    }
    dsp_real_t broadband_att_tracked = 10.0 * log10(p_interf_in / (p_res_tracked > 1e-12 ? p_res_tracked : 1e-12));
    dsp_real_t broadband_att_static  = 10.0 * log10(p_interf_in / (p_res_static > 1e-12 ? p_res_static : 1e-12));

    /* Tracking Accuracy */
    dsp_real_t sum_err = 0.0, sum_sq_err = 0.0, max_err = 0.0;
    /* Skip initial convergence transients (first 500 samples) */
    uint32_t start_eval = 500;
    for (uint32_t i = start_eval; i < TOTAL_SAMPLES; i++) {
        dsp_real_t err = fabs(est_f0_vec[i] - true_f0[i]);
        sum_err += err;
        sum_sq_err += err * err;
        if (err > max_err) max_err = err;
    }
    uint32_t n_eval = TOTAL_SAMPLES - start_eval;
    report_tracked.tracking_mafe_hz = sum_err / n_eval;
    report_tracked.tracking_rmse_hz = sqrt(sum_sq_err / n_eval);
    report_tracked.max_tracking_error_hz = max_err;

    /* RMSE with group delay compensation */
    dsp_real_t diff_sq = 0.0, clean_energy = 0.0;
    for (uint32_t i = 0; i < TOTAL_SAMPLES - gd; i++) {
        dsp_real_t diff = clean_sig[i] - out_tracked[i + gd];
        diff_sq += diff * diff;
        clean_energy += clean_sig[i] * clean_sig[i];
    }
    report_tracked.clean_signal_rmse = sqrt(diff_sq / (TOTAL_SAMPLES - gd));
    report_tracked.prd_percent = sqrt(diff_sq / clean_energy) * 100.0;

    /* 6. Display Benchmark Comparison */
    printf("\n================================================================================\n");
    printf("   COMPARATIVE PERFORMANCE EVALUATION: TRACKED FIR NOTCH vs STATIC NOTCH        \n");
    printf("================================================================================\n");
    printf("+----------+-------------+-----------------------+----------------------+-------+\n");
    printf("| Harmonic | Freq (Nom.) | Static Attenuation(dB)| Tracked Attenuat.(dB)| Gain  |\n");
    printf("+----------+-------------+-----------------------+----------------------+-------+\n");
    for (uint8_t k = 0; k < NUM_HARMONICS; k++) {
        dsp_real_t fk = (k + 1) * NOMINAL_F0;
        dsp_real_t att_s = report_static.harmonic_attenuation_db[k];
        dsp_real_t att_t = report_tracked.harmonic_attenuation_db[k];
        printf("|   k=%u    |  %5.1f Hz   |       %6.2f dB        |      %7.2f dB        | +%.1fdB|\n",
               k + 1, fk, att_s, att_t, att_t - att_s);
    }
    printf("+----------+-------------+-----------------------+----------------------+-------+\n");
    printf("| AVERAGE ATTENUATION    |       %6.2f dB        |      %7.2f dB        | +%.1fdB|\n",
           report_static.avg_harmonic_attenuation_db,
           report_tracked.avg_harmonic_attenuation_db,
           report_tracked.avg_harmonic_attenuation_db - report_static.avg_harmonic_attenuation_db);
    printf("+------------------------+-----------------------+----------------------+-------+\n");
    printf("| TOTAL INTERF. SUPPRESS.|       %6.2f dB        |      %7.2f dB        | +%.1fdB|\n",
           broadband_att_static,
           broadband_att_tracked,
           broadband_att_tracked - broadband_att_static);
    printf("+------------------------+-----------------------+----------------------+-------+\n");
    printf("| SINAD Improvement      |       %6.2f dB        |      %7.2f dB        | +%.1fdB|\n",
           report_static.sinad_improvement_db,
           report_tracked.sinad_improvement_db,
           report_tracked.sinad_improvement_db - report_static.sinad_improvement_db);
    printf("+------------------------+-----------------------+----------------------+-------+\n");

    metrics_print_report(&report_tracked);
    c6748_print_profile_report(&c6748_prof);

    /* 7. Save Simulation Data to CSV for Plotting */
    const char *csv_path = "build/simulation_results.csv";
    FILE *fp = fopen(csv_path, "w");
    if (fp) {
        fprintf(fp, "time,clean,noisy_input,out_tracked,out_static,true_f0,est_f0\n");
        for (uint32_t i = 0; i < TOTAL_SAMPLES; i++) {
            fprintf(fp, "%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f\n",
                    t_vec[i], clean_sig[i], noisy_input[i],
                    out_tracked[i], out_static[i],
                    true_f0[i], est_f0_vec[i]);
        }
        fclose(fp);
        printf(">> Saved simulation data to '%s'\n", csv_path);
    } else {
        printf("Note: Could not open '%s' for writing.\n", csv_path);
    }

    /* Save summary metrics CSV */
    const char *summary_csv = "build/attenuation_summary.csv";
    FILE *fps = fopen(summary_csv, "w");
    if (fps) {
        fprintf(fps, "harmonic_index,nominal_freq,static_attenuation_db,tracked_attenuation_db\n");
        for (uint8_t k = 0; k < NUM_HARMONICS; k++) {
            fprintf(fps, "%u,%.1f,%.2f,%.2f\n",
                    k + 1, (k + 1) * NOMINAL_F0,
                    report_static.harmonic_attenuation_db[k],
                    report_tracked.harmonic_attenuation_db[k]);
        }
        fclose(fps);
        printf(">> Saved attenuation summary to '%s'\n", summary_csv);
    }

    /* Free memory */
    free(t_vec);
    free(clean_sig);
    free(noisy_input);
    free(true_f0);
    free(out_tracked);
    free(est_f0_vec);
    free(out_static);

    printf("\n>> Simulation completed successfully.\n");
    return 0;
}
