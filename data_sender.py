#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import openvr
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import socket
import json
import sys

def rotation_matrix_to_euler(rot, seq='yxz'):
    """Convert rotation matrix to Euler angles (deg)."""
    r = R.from_matrix(rot)
    return r.as_euler(seq, degrees=True)

# ---------------- TCP CONNECTION SETUP ----------------
UBUNTU_IP = "192.168.1.2"     # change this to your Ubuntu machine IP
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"🔌 Connecting to Ubuntu at {UBUNTU_IP}:{PORT} ...")
try:
    sock.connect((UBUNTU_IP, PORT))
    print("✅ Connected to Ubuntu receiver.")
except Exception as e:
    print(f"❌ Connection failed: {e}")
    sys.exit(1)

# ---------------- OPENVR INIT ----------------
openvr.init(openvr.VRApplication_Scene)
poses = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
game_poses = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
vr_system = openvr.VRSystem()

R_correction = np.array([
    [1, 0, 0],
    [0, 0, 1],
    [0, -1, 0]
])

ref_pos = None
streaming = False
BACK_BUTTON = 1 << 33   # bit mask for back button
back_press_start = None
hold_threshold = 2.0    # seconds

print("✅ VR Controller Streaming at 100 Hz")
print("🟦 Hold BACK button 2 s → toggle live output ON/OFF.")
sys.stdout.flush()

# ---------------- MAIN LOOP ----------------
try:
    while True:
        openvr.VRCompositor().waitGetPoses(poses, game_poses)

        for device_index in range(openvr.k_unMaxTrackedDeviceCount):
            if not vr_system.isTrackedDeviceConnected(device_index):
                continue

            pose = poses[device_index]
            if not pose.bPoseIsValid:
                continue

            if vr_system.getTrackedDeviceClass(device_index) != openvr.TrackedDeviceClass_Controller:
                continue
            if vr_system.getControllerRoleForTrackedDeviceIndex(device_index) != openvr.TrackedControllerRole_RightHand:
                continue

            # --- Position and rotation ---
            m = pose.mDeviceToAbsoluteTracking
            pos = np.array([-m[2][3], -m[0][3], m[1][3]])  # coordinate correction
            rot = np.array([[m[i][j] for j in range(3)] for i in range(3)])
            rot_new = R_correction @ rot

            # --- Button handling ---
            result, state = vr_system.getControllerState(device_index)
            button_pressed = state.ulButtonPressed

            now = time.time()
            if button_pressed & BACK_BUTTON:
                if back_press_start is None:
                    back_press_start = now
                elif now - back_press_start >= hold_threshold:
                    streaming = not streaming
                    ref_pos = pos.copy()
                    back_press_start = None
                    print("🟩 Streaming STARTED." if streaming else "🟥 Streaming STOPPED.", flush=True)
            else:
                back_press_start = None

            # --- Compute relative position ---
            rel_pos = (pos - ref_pos) * 1000.0 if ref_pos is not None else np.zeros(3)  # mm
            euler = -rotation_matrix_to_euler(rot_new, seq='yxz')  # deg

            if streaming:
                # Build JSON payload
                pose_dict = {
                    "x": float(rel_pos[0] / 1000.0),  # convert back to meters for ROS
                    "y": float(rel_pos[1] / 1000.0),
                    "z": float(rel_pos[2] / 1000.0),
                    "rx": float(np.radians(euler[0])),
                    "ry": float(np.radians(euler[1])),
                    "rz": float(np.radians(euler[2]))
                }

                try:
                    sock.sendall((json.dumps(pose_dict) + "\n").encode())
                except Exception as e:
                    print(f"⚠️ Send error: {e}")
                    sock.close()
                    sys.exit(1)

                # Optional local print
                print(f"Δpos [mm]: {np.round(rel_pos,2)}, rot [deg]: {np.round(euler,2)}", flush=True)

        time.sleep(0.01)  # 100 Hz loop

except KeyboardInterrupt:
    print("\n🛑 Exiting.")
finally:
    sock.close()
    openvr.shutdown()


# run python -u data_sender.py