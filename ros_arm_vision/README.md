# FFmpeg UDP Webcam Stream for WSL2 (Updated)

## Overview

This package contains scripts to stream your Windows webcam into WSL2 using FFmpeg and UDP:

- **server.bat**: Windows batch script to list available webcams, select one, check the UDP port, and start streaming.
- **client.sh**: WSL2 shell script to detect the Windows host IP and launch the Python client.
- **client.py**: Python script using OpenCV to capture and display the UDP stream.
- **README.md**: Documentation.

## Requirements

- **Windows**:
  - FFmpeg installed and added to PATH.
  - Webcam accessible via DirectShow.

- **WSL2**:
  - Python3 with OpenCV (`pip install opencv-python`).

## Usage

1. **On Windows**:
   ```batch
   server.bat
   ```
   - The script will list your DirectShow devices.
   - Enter the exact name of your webcam when prompted.
   - It will check if port 1234 is free and open a firewall rule if needed.

2. **In WSL2**:
   ```bash
   chmod +x client.sh
   ./client.sh
   ```
   - Press `q` in the video window to quit.

## Notes

- If you need a different port, edit the `PORT` variable at the top of `server.bat` and in `client.py`.
- Ensure Windows Firewall allows the chosen UDP port.
