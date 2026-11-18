# Auto-EDCL: Artifact and Evaluation Repository

> **Double-Blind Review Artifact for DAC 2027 Submission**  
> *Track AI5.3: Edge & Embedded AI Hardware / DES1.3*

This repository provides the complete open-source synthesizable Verilog RTL, simulation testbenches, FPGA implementation reports, TSMC 22nm signoff logs, and machine learning evaluation data supporting the paper:

**"Auto-EDCL: An Event-Driven Edge AI Accelerator With In-Situ Continual Learning and Hardware-Aware Architecture Search for Multimodal Sparse Streams"**

---

## Repository Directory Structure

```
Auto-EDCL-DAC_2027/
├── rtl/                               # Synthesizable RTL for all 13 evaluated architectures
│   ├── auto_edcl_core/                # Auto-EDCL standalone 4x4 DPW array & update logic
│   ├── auto_edcl_soc/                 # Auto-EDCL full SoC (RV32I + EMRS + UADT + Coprocessor)
│   ├── baseline_static_core/          # Static INT8 systolic array baseline
│   ├── sota_bols_core/                # BOLS (ISCAS'24) core
│   ├── sota_fsl_hdnn_core/            # FSL-HDnn (JSSC'26) core
│   ├── sota_fsl_hdnn_full/            # FSL-HDnn (JSSC'26) full system
│   ├── sota_hyblearn_core/            # Hyb-Learn (DAC'24) RRAM/SRAM core
│   ├── sota_hybmed_core/              # HybMED (TBioCAS'24) CDFA core
│   ├── sota_hybmed_full/              # HybMED (TBioCAS'24) full system
│   ├── sota_isscc25_core/             # KWS'25 (ISSCC'25) activation-replay core
│   ├── sota_isscc25_full/             # KWS'25 (ISSCC'25) full system
│   ├── sota_liu_full/                 # Liu et al. (JSSC'25) event-driven SoC
│   └── sota_snn_bols_full/            # BOLS (ISCAS'24) SNN full system
│
├── tb/                                # Comprehensive 100k-cycle verification testbenches
│   ├── tb_auto_edcl_core.v            # Standalone core multi-modal benchmark
│   ├── tb_auto_edcl_soc.v             # Full heterogeneous SoC testbench
│   └── tb_sota_*.v                    # Testbenches for all comparator architectures
│
├── sim/                               # Simulation vectors & execution scripts
│   ├── run_all_simulations.sh         # Open-source Icarus Verilog execution script
│   └── test_vectors/                  # Multi-modal clinical & sensor stimulus streams (HEX)
│
├── fpga_evidence/                     # FPGA Implementation Signoff (Tables III & V)
│   └── fpga_summary_table.csv         # Artix-7 (100 MHz) & ZCU106 (250/375 MHz) metrics
│
├── asic_signoff_tsmc22nm/             # Cadence Innovus 23.13 Signoff at 1.0 GHz (Tables IV & VI)
│   ├── ppa_inventory.csv              # Post-route PPA inventory for all 13 designs
│   ├── ledger.csv                     # Detailed signoff metrics ledger
│   ├── tables.json                    # Automated table verification extraction
│   └── reports/                       # Timing, power (VCD-driven), area, DRC (0 viol.) reports
│
├── continual_learning/                # In-Situ Continual Learning Benchmarks (Table I & Fig 4)
│   ├── insitu_continual_learning_summary.csv # Head-to-head metrics vs 5 SOTA chips
│   ├── uci_har_drift_ablation_summary.csv    # Dual-loop drift ablation data (Fig 4a)
│   └── uci_har_paired_deltas.csv             # 5-seed statistical drift recovery data
│
├── hwnas_codesign/                    # Hardware-Aware NAS Artifacts (Table II & Fig 5)
│   ├── hwnas_summary.csv              # 8-query HAR ablation & 7-modality SOTA NAS comparison
│   ├── multimodality_sota_nas_comparison.csv # Comparison vs SENNA, DANCE++, MixQ
│   └── nas_sota_pareto_benchmark.csv  # Runtime, correlation, and memory footprint
│
└── multimodal_characterization/       # Dynamic Precision Wavefront Characterization (Fig 3)
    └── multimodal_power_accuracy.csv  # 7-modality precision, power, and accuracy sweeps
```

---

## Mapping Paper Evidence to Repository Artifacts

| Paper Item | Evidence Description | Repository Location |
| :--- | :--- | :--- |
| **Table I** | In-situ continual learning accuracy & 0 KB SRAM overhead | `continual_learning/insitu_continual_learning_summary.csv` |
| **Table II** | HW-NAS 8-query ablation & SOTA search comparison | `hwnas_codesign/hwnas_summary.csv` |
| **Table III** | Standalone core FPGA signoff (Artix-7 & ZCU106) | `fpga_evidence/fpga_summary_table.csv` (Rows 1–14) |
| **Table IV** | Standalone core TSMC 22nm Innovus signoff at 1.0 GHz | `asic_signoff_tsmc22nm/ppa_inventory.csv` (Rows 1–8) |
| **Table V** | Full SoC FPGA signoff (Artix-7 & ZCU106) | `fpga_evidence/fpga_summary_table.csv` (Rows 15–26) |
| **Table VI** | Full SoC TSMC 22nm Innovus signoff at 1.0 GHz | `asic_signoff_tsmc22nm/ppa_inventory.csv` (Rows 9–14) |
| **Fig. 1** | System RTL architecture (EMRS, UADT, DPW, updates) | `rtl/auto_edcl_soc/` & `rtl/auto_edcl_core/` |
| **Fig. 3** | Multi-modal power savings vs accuracy trade-offs | `multimodal_characterization/multimodal_power_accuracy.csv` |
| **Fig. 4** | UCI HAR dual-loop drift ablation and recovery | `continual_learning/uci_har_drift_ablation_summary.csv` |
| **Fig. 5** | HW-NAS Pareto frontiers, speedup, and rank correlation | `hwnas_codesign/` |

---

## Quickstart: Running Functional Verification

The RTL models and testbenches are compatible with any standard IEEE 1364-2005 / SystemVerilog simulator, including open-source **Icarus Verilog (`iverilog`)**:

```bash
# 1. Run Auto-EDCL Core Standalone Testbench (100,000 cycles)
iverilog -g2012 -o sim_core.out \
    rtl/auto_edcl_core/auto_edcl_core.v \
    tb/tb_auto_edcl_core.v
vvp sim_core.out

# 2. Run Auto-EDCL Full SoC System Testbench
iverilog -g2012 -o sim_soc.out \
    rtl/auto_edcl_soc/auto_edcl_soc.v \
    tb/tb_auto_edcl_soc.v
vvp sim_soc.out
```

Alternatively, execute the bundled automation script:
```bash
chmod +x sim/run_all_simulations.sh
./sim/run_all_simulations.sh
```

---

## ASIC Signoff Methodology Summary

* **Foundry Process:** TSMC 22nm Ultra-Low Leakage (ULL) CMOS Logic Library.
* **Corner:** Slow-Slow-Global (SSG), 0.63 V, 125 °C.
* **Synthesis:** Cadence Genus 23.1 with cycle-accurate VCD switching activity (`read_vcd`).
* **Equivalence Checking:** Cadence Conformal LEC (clean comparison).
* **Place & Route:** Cadence Innovus Implementation System 23.13 with CCOpt clock tree synthesis (M3–M6), NanoRoute, and DRC verification (0 violations across all 13 designs).
* **Parasitics Extraction:** Quantus SPEF signoff RC extraction.
* **Timing & Power:** Static timing analysis and post-route SPEF back-annotated power calculation.
