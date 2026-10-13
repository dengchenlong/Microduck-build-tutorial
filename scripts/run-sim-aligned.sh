#!/bin/bash
set -eu
export PATH="$HOME/.local/bin:$PATH"
export DISPLAY="${DISPLAY:-:0}"
export PYTHONPATH=/mnt/g/Microduck-build-tutorial/microduck/src
VENV=$HOME/fange-sim-venv
PREFIX=$VENV/lib/python3.12/site-packages/cmeel.prefix
export LD_LIBRARY_PATH="$PREFIX/lib:${LD_LIBRARY_PATH:-}"
cd /mnt/g/Microduck-build-tutorial/microduck
rm -f /tmp/microduck_scheduler.stop
echo "DISPLAY=$DISPLAY starting aligned sim"
exec "$VENV/bin/python" -u /mnt/g/Microduck-build-tutorial/microduck/src/sim/sim_aligned.py
