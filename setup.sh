#!/bin/bash
# Setup script for MIC-733 (Ubuntu/Linux)
set -e

echo "=== Arm Controller Setup ==="

echo "[1/2] Creating venv..."
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
else
    echo "  .venv exists, skipping."
fi

echo "[2/2] Installing Python packages..."
.venv/bin/pip install --upgrade pip -q
.venv/bin/pip install -r requirements.txt -q

echo ""
echo "=== Done ==="
echo "Run: .venv/bin/python3 -u arm_ui.py"
