#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_env
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd "$SCRIPT_DIR"
python3 movement_pkg/movement.py
