import json
from pathlib import Path

import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def depth_images(message, minimum, maximum):
    depth = np.frombuffer(message.data, dtype=">f4" if message.is_bigendian else "<f4").reshape(message.height, message.step//4)[:, :message.width]
    valid = np.isfinite(depth) & (depth >= minimum) & (depth <= maximum)
    millimetres = np.where(valid, np.rint(depth*1000), 0).astype("<u2")
    aligned = Image(header=message.header, height=message.height, width=message.width,
                    encoding="16UC1", is_bigendian=0, step=message.width*2, data=millimetres.tobytes())
    return aligned, depth, valid


class CameraBridge(Node):
    def __init__(self):
        super().__init__("camera_bridge")
        scene = json.loads(Path(self.declare_parameter("scene_description", "").value).read_text())
        self.subscriptions_ = []
        for camera in scene["cameras"]:
            prefix = camera["prefix"]
            info = [self.create_publisher(CameraInfo, prefix+suffix, qos_profile_sensor_data)
                    for suffix in ("/color/camera_info", "/aligned_depth_to_color/camera_info")]
            if camera["name"] == "external":
                info.append(self.create_publisher(CameraInfo, prefix+"/camera_info", 1))
            aligned = self.create_publisher(Image, prefix+"/aligned_depth_to_color/image_raw", qos_profile_sensor_data)
            legacy = self.create_publisher(Image, prefix+"/depth/image_raw", 1) if camera["name"] == "external" else None
            self.subscriptions_.append(self.create_subscription(CameraInfo, prefix+"/_native_camera_info",
                lambda msg, spec=camera, pubs=info: self.info(msg, spec, pubs), 1))
            self.subscriptions_.append(self.create_subscription(Image, prefix+"/_native_depth",
                lambda msg, spec=camera, pub=aligned, old=legacy: self.depth(msg, spec, pub, old), 1))

    def info(self, message, camera, publishers):
        fx, fy, cx, cy = camera["fx"], camera["fy"], camera["width"]/2, camera["height"]/2
        message.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
        message.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
        message.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        for publisher in publishers:
            publisher.publish(message)

    def depth(self, message, camera, publisher, legacy):
        aligned, depth, valid = depth_images(message, camera["min_depth_m"], camera["max_depth_m"])
        publisher.publish(aligned)
        if legacy is not None and legacy.get_subscription_count():
            message.data = np.where(valid, depth, np.nan).astype("<f4").tobytes()
            message.is_bigendian, message.step = 0, message.width*4
            legacy.publish(message)


def main():
    from panda_bringup.node_lifecycle import spin_node
    spin_node(CameraBridge)
