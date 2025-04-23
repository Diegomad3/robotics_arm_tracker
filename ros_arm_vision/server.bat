@echo off
REM Get WSL IP address
FOR /F "delims=" %%i IN ('wsl hostname -I') DO SET WSL_IP=%%i
echo Detected WSL IP: %WSL_IP%

REM Start FFmpeg stream
ffmpeg -f dshow -video_size 1280x720 -framerate 30 -vcodec mjpeg -i video="Logi C270 HD WebCam" -vcodec copy -fflags nobuffer -flags low_delay -f mjpeg udp://172.23.246.5:8080?pkt_size=1316
