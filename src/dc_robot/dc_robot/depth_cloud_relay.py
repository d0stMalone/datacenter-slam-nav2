"""Shrink the depth camera's point cloud before it goes to the Nav2 costmaps.

Gazebo's depth camera publishes an organised cloud with colour: 32 bytes per pixel, NaN for
pixels that see nothing. Even at 160x120 that is ~0.6 MB per message, and the costmaps
(subscribing best-effort inside the Nav2 container) silently received almost none of them:
the voxel layer kept a single cloud from startup and never updated. Real robots also thin
their depth data before the costmap (depth_image_proc, PCL voxel filters, ...).

This node keeps only x, y, z (12 bytes), drops NaN / out-of-range points and, optionally,
every n-th row/column. Result: a few 100 kB at most, delivered reliably.

    /camera/points  (PointCloud2, xyz + rgb, organised)  ->  /camera/points_xyz  (xyz only)
"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class DepthCloudRelay(Node):

    def __init__(self):
        super().__init__('depth_cloud_relay')
        self.step = self.declare_parameter('decimation', 1).value         # keep every n-th pixel
        self.max_range = self.declare_parameter('max_range', 4.0).value   # m; costmaps use <= 3
        self.pub = self.create_publisher(PointCloud2, 'camera/points_xyz', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'camera/points', self.callback,
                                 qos_profile_sensor_data)
        self.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                       for i, n in enumerate('xyz')]

    def callback(self, msg):
        offsets = {f.name: f.offset for f in msg.fields}
        if msg.is_bigendian or [offsets.get(k) for k in 'xyz'] != [0, 4, 8]:
            self.get_logger().error('expected little-endian float x, y, z at offsets 0, 4, 8',
                                    throttle_duration_sec=10.0)
            return
        raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, msg.point_step)
        raw = raw[::self.step, ::self.step, :12]
        xyz = np.ascontiguousarray(raw).view(np.float32).reshape(-1, 3)
        dist2 = np.einsum('ij,ij->i', xyz, xyz)
        keep = np.isfinite(xyz).all(axis=1) & (dist2 < self.max_range ** 2)
        xyz = xyz[keep]
        out = PointCloud2(header=msg.header, height=1, width=len(xyz), fields=self.fields,
                          is_bigendian=False, point_step=12, row_step=12 * len(xyz),
                          is_dense=True, data=xyz.tobytes())
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthCloudRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl-C / launch shutdown
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
