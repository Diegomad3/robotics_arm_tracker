import re
import subprocess
import cv2
import mediapipe as mp
import time
import math

# === Calibration state ===
calibration = {
    "x": 0.0, "y": 0.0, "z": 0.0,
    "depth_base_wrist_index": 0.0,
    "depth_base_knuckle_knuckle": 0.0
}
is_calibrated = False

# === UDP Stream URL Builder ===
def get_udp_stream_url(port=8080):
    res = subprocess.check_output("grep nameserver /etc/resolv.conf", shell=True).decode()
    host_ip = re.search(r'nameserver\s+([\d\.]+)', res).group(1)
    return (
        f"udp://{host_ip}:{port}"
        "?fifo_size=3000000"
        "&overrun_nonfatal=1"
        "&fflags=nobuffer"
        "&flags=low_delay"
        "&probesize=32"
        "&analyzeduration=0"
    )

# === Angle Calculation ===
def calculate_angle(a, b, c):
    def vector(p1, p2):
        return [p1.x - p2.x, p1.y - p2.y, p1.z - p2.z]
    ab = vector(a, b)
    cb = vector(c, b)
    dot = sum(i * j for i, j in zip(ab, cb))
    mag_ab = math.sqrt(sum(i ** 2 for i in ab))
    mag_cb = math.sqrt(sum(i ** 2 for i in cb))
    if mag_ab == 0 or mag_cb == 0:
        return 0
    cos_angle = dot / (mag_ab * mag_cb)
    angle_rad = math.acos(max(min(cos_angle, 1), -1))
    return math.degrees(angle_rad)

# === Start Hand Tracking from Stream ===
def start_hand_tracking_from_udp():
    global calibration, is_calibrated

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    mp_draw = mp.solutions.drawing_utils

    stream_url = get_udp_stream_url()
    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"❌ Failed to open stream: {stream_url}")
        return

    print("Press 'c' to calibrate")
    print("Press 'q' to quit")

    frame_count = 0
    total_latency = 0

    while True:
        t0 = time.time()
        ret, frame = cap.read()
        t1 = time.time()

        if not ret:
            print("⚠️ Frame not received. Exiting.")
            break

        latency_ms = (t1 - t0) * 1000
        frame_count += 1
        total_latency += latency_ms
        avg_latency = total_latency / frame_count

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                h, w, _ = frame.shape

                wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
                index_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
                pinky_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]
                index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]

                angle_at_wrist = calculate_angle(index_mcp, wrist, pinky_mcp)
                use_knuckle_distance = angle_at_wrist > 45

                base_dist = math.sqrt(
                    (index_mcp.x - pinky_mcp.x) ** 2 +
                    (index_mcp.y - pinky_mcp.y) ** 2 +
                    ((-index_mcp.z) - (-pinky_mcp.z)) ** 2
                ) if use_knuckle_distance else math.sqrt(
                    (index_mcp.x - wrist.x) ** 2 +
                    (index_mcp.y - wrist.y) ** 2 +
                    ((-index_mcp.z) - (-wrist.z)) ** 2
                )

                wrist_z_raw = wrist.z - calibration["z"] if is_calibrated else wrist.z
                wrist_z_scaled = -wrist_z_raw * 100
                raw_score = wrist_z_scaled + base_dist * 300

                if is_calibrated:
                    base = calibration["depth_base_knuckle_knuckle"] if use_knuckle_distance else calibration["depth_base_wrist_index"]
                    depth_score = 50 + (raw_score - base)
                else:
                    depth_score = raw_score

                dist_thumb_index = math.sqrt(
                    (thumb_tip.x - index_tip.x) ** 2 +
                    (thumb_tip.y - index_tip.y) ** 2 +
                    (thumb_tip.z - index_tip.z) ** 2
                )
                dist_pinky_thumb = math.sqrt(
                    (thumb_tip.x - pinky_tip.x) ** 2 +
                    (thumb_tip.y - pinky_tip.y) ** 2 +
                    (thumb_tip.z - pinky_tip.z) ** 2
                )

                if dist_pinky_thumb < 0.05:
                    wrist_z_scaled = -wrist.z * 100
                    dist_wrist_index = math.sqrt(
                        (index_mcp.x - wrist.x) ** 2 +
                        (index_mcp.y - wrist.y) ** 2 +
                        ((-index_mcp.z) - (-wrist.z)) ** 2
                    )
                    dist_knuckle_knuckle = math.sqrt(
                        (index_mcp.x - pinky_mcp.x) ** 2 +
                        (index_mcp.y - pinky_mcp.y) ** 2 +
                        ((-index_mcp.z) - (-pinky_mcp.z)) ** 2
                    )
                    calibration.update({
                        "depth_base_wrist_index": wrist_z_scaled + dist_wrist_index * 300,
                        "depth_base_knuckle_knuckle": wrist_z_scaled + dist_knuckle_knuckle * 300,
                        "x": wrist.x,
                        "y": wrist.y,
                        "z": wrist.z
                    })
                    is_calibrated = True
                    print("Auto-Calibrated! Pinky touched thumb.")

                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                wx, wy = int(wrist.x * w), int(wrist.y * h)
                wrist_x = wrist.x - calibration["x"] if is_calibrated else wrist.x
                wrist_y = wrist.y - calibration["y"] if is_calibrated else wrist.y

                cv2.putText(frame, f"x: {wrist_x:.3f}", (wx + 10, wy - 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, f"y: {wrist_y:.3f}", (wx + 10, wy - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, f"z: {wrist_z_scaled:.1f}", (wx + 10, wy - 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, f"Wrist Angle: {angle_at_wrist:.1f}", (wx + 10, wy - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)
                cv2.putText(frame, f"Depth Score: {depth_score:.1f}", (wx + 10, wy - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

                if use_knuckle_distance:
                    cv2.putText(frame, "Using MCP→MCP", (wx + 10, wy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                else:
                    cv2.putText(frame, "Using Wrist→MCP", (wx + 10, wy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                ix, iy = int(index_tip.x * w), int(index_tip.y * h)
                cv2.putText(frame, f"T↔I Dist: {dist_thumb_index:.3f}", (ix + 10, iy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 1)

        cv2.putText(frame, f"Latency: {latency_ms:.2f} ms", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Avg: {avg_latency:.2f} ms", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Hand Tracker (UDP)", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('c') and result.multi_hand_landmarks:
            print("Manual calibration not needed; use pinky-to-thumb gesture.")

    cap.release()
    cv2.destroyAllWindows()

# === RUN ===
if __name__ == "__main__":
    start_hand_tracking_from_udp()
