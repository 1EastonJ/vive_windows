#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import socket, time
import openvr, numpy as np
import orjson   # ⚡ 更快的 JSON 库
from scipy.spatial.transform import Rotation as R

# === UDP Config ===
UBUNTU_IP = "192.168.1.2"
PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1048576)  # 增大发送缓冲
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# === Transform functions ===
def rotation_matrix_to_quat(rot):
    r = R.from_matrix(rot)
    q = r.as_quat()  # [x, y, z, w]
    return [q[3], q[0], q[1], q[2]]  # → [w, x, y, z]

def retarget_pos(pos):
    """把真实控制器坐标缩放到 G1 身高比例"""
    scale_xyz = np.array([0.8, 1.0, 0.9 / 1.4])  # X/Y 稍缩小一点，Z 高度匹配
    pos *= scale_xyz
    return pos.tolist()

# === VR System Init ===
openvr.init(openvr.VRApplication_Scene)
poses = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
vr_system = openvr.VRSystem()

# === 坐标修正矩阵 ===
R1 = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
R2 = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])

print(f"✅ Sending controller data to Ubuntu {UBUNTU_IP}:{PORT}")

# === 高精度定时控制 ===
target_rate = 120.0
dt = 1.0 / target_rate
next_t = time.perf_counter()

try:
    while True:
        openvr.VRCompositor().waitGetPoses(poses, None)

        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if not vr_system.isTrackedDeviceConnected(i):
                continue
            pose = poses[i]
            if not pose.bPoseIsValid:
                continue
            if vr_system.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_Controller:
                continue

            role = vr_system.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_RightHand:
                role_name = "right"
            elif role == openvr.TrackedControllerRole_LeftHand:
                role_name = "left"
            else:
                continue

            # --- 提取位姿 ---
            m = pose.mDeviceToAbsoluteTracking
            pos = np.array([-m[2][3], -m[0][3], m[1][3]])
            rot = np.array([[m[r][c] for c in range(3)] for r in range(3)])
            rot_final = R2 @ (R1 @ rot) @ R2.T

            quat = rotation_matrix_to_quat(rot_final)
            pos = retarget_pos(pos)

            msg = orjson.dumps({
                "role": role_name,
                "pos": [float(x) for x in pos],
                "quat": [float(x) for x in quat]
            })
            sock.sendto(msg, (UBUNTU_IP, PORT))


        # === 精准时间控制，无额外延迟 ===
        next_t += dt
        sleep = next_t - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.perf_counter()  # 掉帧则重新校准时间基准

except KeyboardInterrupt:
    print("🛑 Stopping...")
finally:
    openvr.shutdown()



