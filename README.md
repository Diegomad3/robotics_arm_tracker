# robotics_arm_tracker
robotics_cluster


### 1. On windows machine use launch the video server for the webcam.
#### Run : ffmpeg -list_devices true -f dshow -i dummy 2>&1
Output will look like this:
[dshow @ 000001750e02eb40] "Logitech Webcam C270" (video)
[dshow @ 000001750e02eb40]   Alternative name "@device_pnp_\\?\usb#vid_1532&pid_0e05&mi_00#9&37c21d98&1&0000#{65e8773d-8f56-11d0-a3b9-00a0c9223196}\global"
[dshow @ 000001750e02eb40] "Microphone (Steam Streaming Microphone)" (audio)
[dshow @ 000001750e02eb40]   Alternative name "@device_cm_{33D9A762-90C8-11D0-BD43-00A0C911CE86}\wave_{3AB60300-35F8-495D-B231-E8D744537DAB}"
[dshow @ 000001750e02eb40] "Microphone (Razer Kiyo Pro)" (audio)
[dshow @ 000001750e02eb40]   Alternative name "@device_cm_{33D9A762-90C8-11D0-BD43-00A0C911CE86}\wave_{060734FB-1932-47E8-9CFD-57728D9C4301}"
Copy (including quotes): "USB Video Device" Or whatever name is there.

#### Next Run: 
wsl hostname -I

You will get a an IP address like: 172.23.246.5 

Finally run:
Replace <WSL_IP> with your IP address
ffmpeg -f dshow -video_size 1280x720 -framerate 30 -vcodec mjpeg -i video="Logitech Webcam C270" -vcodec copy -fflags nobuffer -flags low_delay -f mjpeg udp://<WSL_IP>:8080?pkt_size=1316
->
ffmpeg -f dshow -video_size 640x480 -framerate 30 -vcodec mjpeg -i video="Logi C270 HD WebCam" -vcodec copy -fflags nobuffer -flags low_delay -f mjpeg udp://172.23.246.5:8080?pkt_size=1316

### 2. On wsl subsystem machine.

Open the ros_arm_vison folder. 
Run 
chmod +x client.sh

Then run 
./client.sh

This will make the environment and download all the dependencies to run the program.

