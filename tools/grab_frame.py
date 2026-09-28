#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
grab_frame.py —— 从 ROS 2 图像话题抓一帧存成 PNG。

合并后的 ELF3 仿真里有两路相机（都在机器人自己身上）：
  /simulation/body_depth_camera/color/image_raw    胸口 D435i（朝前）
  /simulation/head_depth_camera/color/image_raw    头部（朝前）

相机话题是 sensor_msgs/Image，通常用 BEST_EFFORT QoS（qos_profile_sensor_data）。

用法：
    python3 grab_frame.py <话题> <输出.png> [超时秒]
"""

import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Grabber(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("grab_frame")
        self.bridge = CvBridge()
        self.img = None
        self.enc = None
        self.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Image) -> None:
        if self.img is None:
            self.enc = msg.encoding
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    topic, out = sys.argv[1], sys.argv[2]
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0

    rclpy.init()
    n = Grabber(topic)
    t0 = time.time()
    while time.time() - t0 < timeout and n.img is None:
        rclpy.spin_once(n, timeout_sec=0.05)

    if n.img is None:
        print(f"[失败] {timeout:.0f}s 内没收到 {topic} 的图像")
        rclpy.shutdown()
        return 1

    bgr = cv2.cvtColor(np.asarray(n.img), cv2.COLOR_RGB2BGR)
    cv2.imwrite(out, bgr)
    print(f"  已保存 {out}  ({n.img.shape[1]}x{n.img.shape[0]}, 原始编码 {n.enc})")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
