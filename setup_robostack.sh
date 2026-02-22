#!/usr/bin/env bash
set -euo pipefail

echo "=== RoboStack ROS2 Humble Setup for macOS ==="
echo ""

# Step 1: Check for Miniforge / conda
if ! command -v conda &>/dev/null; then
    MINIFORGE_DIR="$HOME/miniforge3"
    if [ -d "$MINIFORGE_DIR" ]; then
        echo "Found Miniforge at $MINIFORGE_DIR, activating..."
        eval "$("$MINIFORGE_DIR/bin/conda" shell.bash hook)"
    else
        echo "Miniforge not found. Installing..."
        ARCH=$(uname -m)
        URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-${ARCH}.sh"
        curl -fsSL "$URL" -o /tmp/miniforge.sh
        bash /tmp/miniforge.sh -b -p "$MINIFORGE_DIR"
        rm /tmp/miniforge.sh
        eval "$("$MINIFORGE_DIR/bin/conda" shell.bash hook)"
        conda init bash zsh 2>/dev/null || true
        echo ""
        echo "Miniforge installed at $MINIFORGE_DIR"
    fi
else
    echo "conda found: $(which conda)"
fi

# Step 2: Create ros_env if it doesn't exist
if conda env list | grep -q "ros_env"; then
    echo "Environment 'ros_env' already exists. Skipping creation."
else
    echo ""
    echo "Creating 'ros_env' with ROS2 Humble Desktop + CycloneDDS + Gazebo..."
    echo "  (This may take 10-20 minutes on first install)"
    echo ""
    conda create -n ros_env -y \
        -c conda-forge \
        -c robostack-humble \
        ros-humble-desktop \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-xacro \
        ros-humble-robot-state-publisher
fi

# Step 3: Configure channels in the environment
eval "$(conda shell.bash hook)"
conda activate ros_env
conda config --env --add channels robostack-humble 2>/dev/null || true

echo ""
echo "=== Setup complete ==="
echo ""
echo "To verify, run:"
echo "  conda activate ros_env"
echo "  rviz2"
echo ""
echo "You can now use the RViz and Gazebo buttons in the Robot Control app."
