/**
 * @file tms320c6748_sim.h
 * @brief Hardware Emulation Layer for Texas Instruments TMS320C6748 DSP.
 * 
 * Emulates:
 * - C674x VLIW Floating-Point Core @ 456 MHz
 * - McASP (Multichannel Audio Serial Port) / TLV320AIC3106 Codec
 * - EDMA3 Ping-Pong Double-Buffering Mechanism
 * - TSCL / TSCH Hardware Cycle Counter for Profiling CPU Load
 */

#ifndef TMS320C6748_SIM_H
#define TMS320C6748_SIM_H

#include "dsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define C6748_CPU_FREQ_HZ         456000000.0   /* 456 MHz Core Clock */
#define C6748_EDMA_PING_PONG_SIZE 256           /* Frame size for EDMA ping-pong buffer */

typedef struct {
    uint64_t total_cycles;
    uint64_t max_block_cycles;
    uint32_t total_blocks_processed;
    dsp_real_t avg_cycles_per_sample;
    dsp_real_t cpu_utilization_percent;
} c6748_profile_t;

/**
 * Initializes TMS320C6748 hardware profiling emulation.
 */
void c6748_sim_init(c6748_profile_t *profile);

/**
 * Simulates EDMA3 ping-pong DMA transfer interrupt service routine (ISR).
 * Records cycles, MIPS consumption, and hardware load.
 */
void c6748_sim_record_block(
    c6748_profile_t *profile,
    uint32_t block_size,
    dsp_real_t sample_rate
);

/**
 * Prints TMS320C6748 Hardware Profile Report.
 */
void c6748_print_profile_report(const c6748_profile_t *profile);

#ifdef __cplusplus
}
#endif

#endif /* TMS320C6748_SIM_H */
