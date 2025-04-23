#!/usr/bin/env bash

set -e  # Exit on error

# Absolute path to current script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/mediapipe-env"

# Create virtual environment if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
  echo "Creating virtual environment at $VENV_DIR..."
  python3 -m venv "$VENV_DIR"
fi

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Install dependencies
REQUIRED_PACKAGES=(opencv-python mediapipe numpy)
echo "Installing Python packages: ${REQUIRED_PACKAGES[*]}"
pip install --upgrade pip
pip install "${REQUIRED_PACKAGES[@]}"

# Detect Windows host IP (WSL -> Windows bridge)
HOST_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
echo "Detected Windows host IP: $HOST_IP"

# Export for client.py (if it uses this)
export HOST_IP

# Run your client script
echo "Starting client.py..."
python3 "$SCRIPT_DIR/client.py"
