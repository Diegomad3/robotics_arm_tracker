# robotics_arm_tracker  
Part of the `robotics_cluster` project  

This system streams video from a Windows-connected webcam into the WSL2 environment, where it is processed for robotic arm tracking.

---

## 1. On the Windows Host: Stream Webcam Feed via FFmpeg

### Step 1: Install FFmpeg

Download a static Windows build of FFmpeg from:

[https://www.gyan.dev/ffmpeg/builds/](https://www.gyan.dev/ffmpeg/builds/)

- Choose the "essentials" ZIP archive
- Extract it to a known folder (e.g., `C:\ffmpeg`)
- Add the `bin` folder (e.g., `C:\ffmpeg\bin`) to your System `PATH` environment variable

To confirm installation, open a new Command Prompt and run:

```bash
ffmpeg -version
```

### Step 2: Find Your Webcam Name

Run this in Command Prompt or PowerShell:

```bash
ffmpeg -list_devices true -f dshow -i dummy 2>&1
```

Look for your video device in the output:

```
[dshow @ ...] "Logi C270 HD WebCam"
```

Copy the exact name shown in quotes.

### Step 3: Get the WSL2 IP Address

Run the following in Command Prompt or PowerShell:

```bash
wsl hostname -I
```

You’ll get an IP like:

```
172.23.246.5
```

### Step 4: Launch the FFmpeg UDP Streamer

Run the following command from Command Prompt or PowerShell, **replacing the camera name and WSL IP** if needed:

```bash
ffmpeg -f dshow -video_size 1280x720 -framerate 30 -vcodec mjpeg -i video="Logitech Webcam C270" -vcodec copy -fflags nobuffer -flags low_delay -f mjpeg udp://172.23.246.5:8080?pkt_size=1316
```

This streams MJPEG video with low latency to your WSL-based vision system.

---

## 2. On the WSL2 (Linux) Side: Launch the Vision Client

### Step 1: Navigate to the `ros_arm_vision` folder

```bash
cd ~/ros_arm_vision
```

### Step 2: Make the script executable

```bash
chmod +x client.sh
```

### Step 3: Launch the client

```bash
./client.sh
```

This script will:

- Create a Python virtual environment in the current directory
- Install required Python packages (`opencv-python`, `mediapipe`, `numpy`)
- Detect the Windows host IP
- Launch `client.py` to receive and process the video stream

---

## Notes

- The video stream is sent to UDP port 8080 on the WSL machine.
- Ensure that any firewall software allows communication between Windows and WSL.
- You may change the resolution and frame rate in the FFmpeg command if needed.
