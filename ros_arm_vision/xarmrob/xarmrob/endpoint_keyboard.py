#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import traceback
import subprocess, re, time, math
import cv2
import mediapipe as mp
from sensor_msgs.msg import JointState
from xarmrob_interfaces.srv import ME439XArmInverseKinematics

class HandEndpointNode(Node):
    def __init__(self):
        super().__init__('hand_endpoint')

        # --- ROS pubs & clients ---
        self.pub_joint = self.create_publisher(JointState, '/joint_angles_desired', 1)
        self.cli_ik   = self.create_client(ME439XArmInverseKinematics, 'xarm_inverse_kinematics')
        while not self.cli_ik.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service…')

        self.req_ik = ME439XArmInverseKinematics.Request()

        # --- Mediapipe + video capture ---
        self.mp_hands = mp.solutions.hands
        self.hands    = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        stream = self.get_udp_stream_url()
        self.cap = cv2.VideoCapture(stream)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error(f'Couldn’t open stream {stream}')
            raise RuntimeError

        # --- calibration storage ---
        self.calibration = {
            'x':0., 'y':0., 'z':0.,
            'depth_base_wrist_index':0.,
            'depth_base_knuckle_knuckle':0.
        }
        self.is_calibrated = False

        # --- joint‐angle storage (for publishing) ---
        self.ang_all = [0., -np.pi/2., np.pi/2., 0., 0., 0., np.pi/2.]

        # --- timer to run at ~10 Hz ---
        self.timer = self.create_timer(0.1, self.timer_callback)

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
        def vec(p1, p2): return [p1.x-p2.x, p1.y-p2.y, p1.z-p2.z]
        ab, cb = vec(a,b), vec(c,b)
        dot = sum(x*y for x,y in zip(ab,cb))
        mag = lambda v: math.sqrt(sum(i*i for i in v))
        m1, m2 = mag(ab), mag(cb)
        if m1*m2 == 0: return 0.0
        cos_a = max(min(dot/(m1*m2),1),-1)
        return math.degrees(math.acos(cos_a))

    def timer_callback(self):
        # grab frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Frame drop')
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        if not res.multi_hand_landmarks:
            return

        lm = res.multi_hand_landmarks[0]
        w,h,_ = frame.shape
        wrist       = lm.landmark[self.mp_hands.HandLandmark.WRIST]
        idx_mcp     = lm.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        pinky_mcp   = lm.landmark[self.mp_hands.HandLandmark.PINKY_MCP]
        idx_tip     = lm.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        thumb_tip   = lm.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        pinky_tip   = lm.landmark[self.mp_hands.HandLandmark.PINKY_TIP]

        # auto‐calibrate gesture
        if math.dist([thumb_tip.x,thumb_tip.y,thumb_tip.z],
                     [pinky_tip.x,pinky_tip.y,pinky_tip.z]) < 0.05 and not self.is_calibrated:
            # same math you had for calibration
            wrist_z_scaled = -wrist.z*100
            di = math.dist([idx_mcp.x,idx_mcp.y,-idx_mcp.z],
                           [wrist.x,wrist.y,-wrist.z])
            dk = math.dist([idx_mcp.x,idx_mcp.y,-idx_mcp.z],
                           [pinky_mcp.x,pinky_mcp.y,-pinky_mcp.z])
            self.calibration.update({
                'depth_base_wrist_index': wrist_z_scaled + di*300,
                'depth_base_knuckle_knuckle': wrist_z_scaled + dk*300,
                'x': wrist.x, 'y': wrist.y, 'z': wrist.z
            })
            self.is_calibrated = True
            self.get_logger().info('✳️  Auto-calibrated')

        # compute depth score & decide endpoint
        angle_wrist = self.calculate_angle(idx_mcp, wrist, pinky_mcp)
        use_knuckle = angle_wrist > 45
        base_dist = (
            math.dist([idx_mcp.x,idx_mcp.y,-idx_mcp.z],
                      [pinky_mcp.x,pinky_mcp.y,-pinky_mcp.z])
            if use_knuckle
            else math.dist([idx_mcp.x,idx_mcp.y,-idx_mcp.z],
                           [wrist.x,wrist.y,-wrist.z])
        )
        wrist_z_raw    = wrist.z - self.calibration['z'] if self.is_calibrated else wrist.z
        wrist_z_scaled = -wrist_z_raw * 100
        raw_score      = wrist_z_scaled + base_dist * 300
        base           = (self.calibration['depth_base_knuckle_knuckle']
                          if use_knuckle
                          else self.calibration['depth_base_wrist_index'])
        depth_score    = 50 + (raw_score - base) if self.is_calibrated else raw_score

        # map depth_score & image coords → real-world x,y,z (tweak these gains)
        x = (wrist.x - self.calibration['x']) * 0.5   # meters per norm-pixel
        y = (wrist.y - self.calibration['y']) * -0.5
        z = depth_score * 0.001                       # depth_score→meters

        xyz_goal = [x, y, z]
        self.call_ik_and_publish(xyz_goal)

    def call_ik_and_publish(self, xyz_goal):
        # set service request
        self.req_ik.endpoint = xyz_goal
        fut = self.cli_ik.call_async(self.req_ik)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
        if fut.result() is None:
            self.get_logger().warn('IK service failed')
            return
        resp = fut.result()
        if resp.modified:
            self.get_logger().warn('Unreachable → moved to nearest')
        self.ang_all = resp.joint_angles

        # publish
        msg = JointState()
        msg.name     = ['base_joint','shoulder_joint','elbow_joint',
                        'forearm_joint','wrist_joint','fingers_joint','gripper']
        msg.position = np.append(self.ang_all, 0.0)  # gripper open
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = HandEndpointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
