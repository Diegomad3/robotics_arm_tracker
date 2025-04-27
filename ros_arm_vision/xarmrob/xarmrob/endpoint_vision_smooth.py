#!/usr/bin/env python3

# ROS2 node to drive a HiWonder xArm 1S endpoint from hand‐tracking
# Uses smooth_interpolation to generate minimum-jerk trajectories
# Peter Adamczyk, UW–Madison (2024-11-12)
# Integrated & extended for Mediapipe + RTTI, callback groups, MT executor

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import threading
import subprocess, re, time, math
import numpy as np
import cv2
import mediapipe as mp

from xarmrob_interfaces.msg import ME439PointXYZ
import xarmrob.smooth_interpolation as smoo


# simple colored‐text helper
coloredtext = lambda r, g, b, text: f'\033[38;2;{r};{g};{b}m{text}\033[38;2;255;255;255m'


class HandEndpointNode(Node):
    def __init__(self):
        super().__init__('hand_endpoint')

        # ── parameters ──────────────────────────────────────────────────────────
        self.command_frequency = float(self.declare_parameter('command_frequency', 10).value)
        self.endpoint_speed    = float(self.declare_parameter('endpoint_speed', 0.05).value)
        self.movement_time_ms  = round(1000.0 / self.command_frequency)

        # ── trajectory state ───────────────────────────────────────────────────
        self.old_xyz_goal = np.array([0.165, 0.0, 0.155])  # neutral start
        self.disp_traj    = [self.old_xyz_goal.copy()]
        self.idx          = 0
        self.new_goal_tol = 0.02  # only re-traj if hand moved >2cm

        # ── publisher + timer ─────────────────────────────────────────────────
        cbg = ReentrantCallbackGroup()
        self.pub = self.create_publisher(
            ME439PointXYZ,
            '/endpoint_desired',
            1,
            callback_group=cbg
        )
        self.timer = self.create_timer(
            self.movement_time_ms / 1000.0,
            self.send_endpoint_desired,
            callback_group=cbg
        )

        # ── Mediapipe & video stream ──────────────────────────────────────────
        self.mp_hands = mp.solutions.hands
        self.hands    = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        stream_url = self.get_udp_stream_url()
        self.cap = cv2.VideoCapture(stream_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open UDP stream: {stream_url}')
            raise RuntimeError('Couldn’t open video stream')

        # ── start background detection thread ──────────────────────────────────
        t = threading.Thread(target=self.detection_loop, daemon=True)
        t.start()


    def get_udp_stream_url(self, port=8080):
        res = subprocess.check_output("grep nameserver /etc/resolv.conf", shell=True).decode()
        host_ip = re.search(r'nameserver\s+([\d\.]+)', res).group(1)
        return (
            f"udp://{host_ip}:{port}"
            "?fifo_size=3000000&overrun_nonfatal=1"
            "&fflags=nobuffer&flags=low_delay"
            "&probesize=32&analyzeduration=0"
        )


    def calculate_angle(self, a, b, c):
        """Angle at b formed by (a–b) and (c–b) in degrees."""
        def vector(p1, p2):
            return np.array([p1.x - p2.x, p1.y - p2.y, p1.z - p2.z])
        ab = vector(a, b)
        cb = vector(c, b)
        mag_ab = np.linalg.norm(ab)
        mag_cb = np.linalg.norm(cb)
        if mag_ab * mag_cb == 0:
            return 0.0
        cosang = np.dot(ab, cb) / (mag_ab * mag_cb)
        return math.degrees(math.acos(np.clip(cosang, -1, 1)))


    def detection_loop(self):
        """Continuously grab frames, compute hand‐pose → raw endpoint,
           and whenever it moves enough, regen a new smooth trajectory."""
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = self.hands.process(rgb)
            if not res.multi_hand_landmarks:
                continue

            lm = res.multi_hand_landmarks[0]
            w, h, _ = frame.shape

            # choose coordinates & depth‐score per your original logic
            wrist     = lm.landmark[self.mp_hands.HandLandmark.WRIST]
            index_mcp = lm.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
            pinky_mcp = lm.landmark[self.mp_hands.HandLandmark.PINKY_MCP]

            angle_w = self.calculate_angle(index_mcp, wrist, pinky_mcp)
            use_knuckle = angle_w > 45
            if use_knuckle:
                base_dist = np.linalg.norm(
                    np.array([index_mcp.x, index_mcp.y, -index_mcp.z]) -
                    np.array([pinky_mcp.x, pinky_mcp.y, -pinky_mcp.z])
                )
            else:
                base_dist = np.linalg.norm(
                    np.array([index_mcp.x, index_mcp.y, -index_mcp.z]) -
                    np.array([wrist.x,    wrist.y,    -wrist.z])
                )

            # simple depth‐score → meters
            wrist_z_scaled = -wrist.z * 100
            raw_score      = wrist_z_scaled + base_dist * 300
            z_m = raw_score * 0.001

            # x,y from normalized image coords → meters
            x_m = (wrist.x * w -  w/2) * 0.001
            y_m = (h/2 - wrist.y * h) * 0.001

            raw_goal = np.array([x_m, y_m, z_m])

            # if moved > tol, rebuild disp_traj
            if np.linalg.norm(raw_goal - self.old_xyz_goal) > self.new_goal_tol:
                self.get_logger().info(
                    coloredtext(50,255,50,
                        f' New hand‐goal: [{raw_goal[0]:.3f}, {raw_goal[1]:.3f}, {raw_goal[2]:.3f}]'
                    )
                )
                # minimum-jerk from old → new
                _, traj = smoo.minimum_jerk_interpolation(
                    self.old_xyz_goal,
                    raw_goal,
                    self.endpoint_speed,
                    self.command_frequency
                )
                self.disp_traj = [pt.tolist() for pt in traj]
                self.idx        = 0
                self.old_xyz_goal = raw_goal.copy()

            # tiny sleep so this loop doesn't spin at 100%
            time.sleep(1.0 / (self.command_frequency * 2))


    def send_endpoint_desired(self):
        """Called at command_frequency: pops next point off disp_traj and publishes it."""
        if self.idx >= len(self.disp_traj):
            # hold at final
            self.idx = len(self.disp_traj) - 1

        xyz = self.disp_traj[self.idx]
        self.idx += 1

        msg = ME439PointXYZ()
        msg.xyz = xyz
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandEndpointNode()

    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
