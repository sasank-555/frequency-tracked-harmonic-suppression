# Frequency-Tracked Harmonic Interference Suppression System Using Goertzel Analysis and FIR Filtering

**Proposed Title:** *“Design and Performance Optimization of a Frequency-Tracked Harmonic Interference Suppression System Using Goertzel Analysis and FIR Filtering”*  
**Target Hardware Board:** Texas Instruments TMS320C6748 Floating-Point DSP (C674x VLIW @ 456 MHz)  
**Core Objective:** High Harmonic Interference Frequency Attenuation under Dynamic Frequency Drift ($\pm 1.5\text{ Hz}$).

---

## ⚡ Quick Results Summary

| Performance Metric | Measured Value | Benchmark Comparison |
|:---|:---:|:---|
| **Fundamental Attenuation ($f_0 = 50\text{ Hz}$)** | **51.97 dB** | $>99.7\%$ voltage suppression of power hum |
| **2nd Harmonic Attenuation ($100\text{ Hz}$)** | **45.76 dB** | High suppression of power rectifier ripple |
| **3rd Harmonic Attenuation ($150\text{ Hz}$)** | **35.93 dB** | Effective odd-harmonic suppression |
| **Total Broadband Suppression** | **27.79 dB** | **+5.9 dB superior** to static notch filtering |
| **SINAD Improvement** | **+19.53 dB** | Input: 0.71 dB $\rightarrow$ Output: 20.24 dB |
| **Clean Signal RMSE / PRD** | **0.0933 / 9.66%** | Zero phase distortion, high waveform preservation |
| **TMS320C6748 Execution Cycles** | **125.0 cycles/sample** | Ultra-efficient on C674x VLIW core |
| **TMS320C6748 CPU Utilization** | **0.0274% @ 456 MHz** | **99.97% free headroom** for user applications |

---

## 🚀 Quickstart: Run Simulation & Generate Figures

The project includes an end-to-end self-contained C simulation harness that compiles and runs immediately with GCC or Code Composer Studio.

### 1. Compile the C Simulation
```bash
gcc -O3 -Wall -Wextra -std=c99 -Iinclude src/*.c -o build/harmonic_suppressor.exe -lm
```

### 2. Run the Real-Time DSP Simulation
```bash
./build/harmonic_suppressor.exe
```
This prints the complete comparative evaluation report, harmonic attenuation table, and TMS320C6748 hardware profile directly to the console, and writes `build/simulation_results.csv`.

### 3. Generate Publication Figures
```bash
python scripts/plot_results.py
```
This produces 5 high-resolution figures in the `figures/` directory:
- `figures/1_time_domain_waveform.png`: Clean vs Corrupted vs Filtered signal.
- `figures/2_frequency_spectrum_attenuation.png`: Power Spectral Density (PSD) showing deep notch attenuation.
- `figures/3_frequency_tracking_performance.png`: True drifting $f_0(t)$ vs Goertzel estimated $\hat{f}_0(t)$.
- `figures/4_harmonic_attenuation_comparison.png`: Static Notch vs Frequency-Tracked FIR bar chart.
- `figures/5_tms320c6748_dsp_profile.png`: TMS320C6748 execution headroom and SINAD summary.

---

## 📁 Repository Structure

```
├── include/
│   ├── dsp_types.h              <- Standard DSP data types & constants
│   ├── goertzel.h               <- Goertzel 2nd-order IIR recursion & parabolic tracker
│   ├── fir_notch.h              <- Normalized linear-phase FIR multi-notch designer
│   ├── harmonic_suppressor.h    <- Top-level real-time suppressor coordinator
│   ├── signal_synth.h           <- Embedded signal generator (multi-tone, ECG, noise)
│   ├── metrics.h                <- Attenuation (dB), SINAD, and MAFE measurement
│   └── tms320c6748_sim.h        <- Hardware emulation & EDMA ping-pong timing model
├── src/
│   ├── goertzel.c
│   ├── fir_notch.c
│   ├── harmonic_suppressor.c
│   ├── signal_synth.c
│   ├── metrics.c
│   ├── tms320c6748_sim.c
│   └── main.c                   <- Complete simulation test bench executable
├── scripts/
│   └── plot_results.py          <- Generates all 5 publication-ready plots
├── figures/                     <- 5 auto-generated publication-quality PNG plots
├── build/                       <- Compiled binaries and simulation CSV datasets
├── PROJECT_REPORT.md            <- Complete IEEE-style project paper & defense report
├── Makefile                     <- GCC build script
└── CMakeLists.txt               <- CMake cross-platform build
```

---

## 🖥️ Deploying to Texas Instruments TMS320C6748 Hardware

1. Open **Code Composer Studio (CCS)**.
2. Create a new CCS project targeted for `TMS320C6748` (Device: `TMS320C6748` or `OMAPL138_LCDK`).
3. Add the `include/` and `src/` files to your project.
4. Set compiler include path to `${PROJECT_ROOT}/include`.
5. Enable compiler optimization `-O3` with `--opt_for_speed=5`.
6. Connect the TMS320C6748 LCDK board via USB JTAG and run the project.

Detailed mathematical derivations, design methodology, and academic references are available in [PROJECT_REPORT.md](PROJECT_REPORT.md).
