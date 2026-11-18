#!/usr/bin/env bash
# ==============================================================================
# Auto-EDCL DAC 2027: Multi-Modal Benchmark Simulation Runner
# Verified with Icarus Verilog (iverilog v12+)
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR"

mkdir -p sim/build

echo "=========================================================="
echo " Running Auto-EDCL Standalone Core Multi-Modal Testbench   "
echo "=========================================================="
iverilog -g2012 -o sim/build/tb_auto_edcl_core.out \
    rtl/auto_edcl_core/auto_edcl_core.v \
    tb/tb_auto_edcl_core.v

vvp sim/build/tb_auto_edcl_core.out

echo ""
echo "=========================================================="
echo " Running Auto-EDCL Full SoC System Testbench               "
echo "=========================================================="
iverilog -g2012 -o sim/build/tb_auto_edcl_soc.out \
    rtl/auto_edcl_soc/auto_edcl_soc.v \
    tb/tb_auto_edcl_soc.v

vvp sim/build/tb_auto_edcl_soc.out

echo ""
echo "All simulations completed successfully."
