#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  Build Script for Self-Balancing Robot Project
# ═══════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_TYPE="${1:-Release}"

echo "╔═══════════════════════════════════════════════════════╗"
echo "║  Self-Balancing Robot — Build System                ║"
echo "╚═══════════════════════════════════════════════════════╝"
echo ""

# ── Build Raspberry Pi 5 Balance Controller ──────────────────
echo "━━━ Building Robot (RPi5) ━━━"

mkdir -p "${SCRIPT_DIR}/build_rpi"
cd "${SCRIPT_DIR}/build_rpi"

cmake "${SCRIPT_DIR}/robot" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX=/usr/local

make -j$(nproc)

echo ""
echo "✓ Robot built: build_rpi/robot"
echo ""

# ── Build MSI Claw Controller App ────────────────────────────
echo "━━━ Building Controller App (MSI Claw 8 / Arch Linux) ━━━"

mkdir -p "${SCRIPT_DIR}/build_controller"
cd "${SCRIPT_DIR}/build_controller"

cmake "${SCRIPT_DIR}/controller" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX=/usr/local

make -j$(nproc)

echo ""
echo "✓ Controller app built: build_controller/robot_controller"
echo ""

echo "╔═══════════════════════════════════════════════════════╗"
echo "║  Build Complete!                                      ║"
echo "║                                                       ║"
echo "║  RPi5:  ./build_rpi/robot --help         ║"
echo "║  Claw:  ./build_controller/robot_controller --help  ║"
echo "╚═══════════════════════════════════════════════════════╝"
