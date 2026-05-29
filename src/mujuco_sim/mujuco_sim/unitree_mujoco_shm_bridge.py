#!/usr/bin/env python3

import math
import mmap
import os
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


SHM_PATH = "/dev/shm/unitree_mujoco_nav"
MAGIC = 0x564E4A4D
VERSION = 1
VERSION_WITH_CLOUD = 2
VERSION_WITH_IMU = 3
MAX_RANGES = 360
MAX_POINTS = 24000
STRUCT_FORMAT_V1 = "<IIIQd7d6d360f"
STRUCT_FORMAT_V2 = "<IIIIQd7d6d3d3d360f72000f"
STRUCT_FORMAT_V3 = "<IIIIQd7d6d3d3d4d3d3d360f72000f"
STRUCT_FORMAT = STRUCT_FORMAT_V3
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)


class UnitreeMujocoShmBridge(Node):
    def __init__(self):
        super().__init__("unitree_mujoco_shm_bridge")

        self.declare_parameter("shm_path", SHM_PATH)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "laser_frame")
        self.declare_parameter("livox_frame", "livox_frame")
        self.declare_parameter("laser_xyz", "0.0,0.0,0.95")
        self.declare_parameter("livox_rpy", "0.0,-0.04014257,0.0")
        self.declare_parameter("scan_hz", 10.0)
        self.declare_parameter("odom_hz", 50.0)
        self.declare_parameter("publish_robot_odom", True)
        self.declare_parameter("publish_sensor_tf_static", False)
        self.declare_parameter("range_min", 0.10)
        self.declare_parameter("range_max", 8.0)

        self.shm_path = self.get_parameter("shm_path").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.livox_frame = self.get_parameter("livox_frame").value
        self.laser_xyz = self._parse_xyz(self.get_parameter("laser_xyz").value)
        self.livox_rpy = self._parse_xyz(self.get_parameter("livox_rpy").value)
        self.publish_robot_odom = bool(self.get_parameter("publish_robot_odom").value)
        self.publish_sensor_tf_static = bool(
            self.get_parameter("publish_sensor_tf_static").value
        )
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)

        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter("scan_topic").value, 10
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter("cloud_topic").value, 10
        )
        self.imu_pub = self.create_publisher(
            Imu, self.get_parameter("imu_topic").value, 50
        )
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 10
        )
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)

        self.fd = None
        self.mm = None
        self.last_scan_seq = None
        self.static_tf_sent = False

        self.create_timer(1.0 / float(self.get_parameter("odom_hz").value), self._odom_timer)
        self.create_timer(1.0 / float(self.get_parameter("scan_hz").value), self._scan_timer)

        self.get_logger().info(f"Reading unitree_mujoco nav shared memory: {self.shm_path}")

    def _parse_xyz(self, text):
        values = [float(item.strip()) for item in text.split(",") if item.strip()]
        if len(values) != 3:
            raise ValueError("laser_xyz must contain 3 comma-separated values")
        return values

    def _open_if_needed(self):
        if self.mm is not None:
            return True
        if not os.path.exists(self.shm_path):
            return False
        if os.path.getsize(self.shm_path) < STRUCT_SIZE:
            return False
        self.fd = os.open(self.shm_path, os.O_RDONLY)
        self.mm = mmap.mmap(self.fd, STRUCT_SIZE, access=mmap.ACCESS_READ)
        return True

    def _read(self):
        if not self._open_if_needed():
            return None
        self.mm.seek(0)
        header = struct.unpack("<IIII", self.mm.read(16))
        magic, version, num_ranges, num_points = header
        if magic != MAGIC:
            return None
        self.mm.seek(0)
        if version == VERSION_WITH_IMU:
            data = struct.unpack(STRUCT_FORMAT_V3, self.mm.read(struct.calcsize(STRUCT_FORMAT_V3)))
            _magic, _version, num_ranges, num_points, seq, sim_time = data[:6]
            num_ranges = min(int(num_ranges), MAX_RANGES)
            num_points = min(int(num_points), MAX_POINTS)
            pose = data[6:13]
            qvel = data[13:19]
            livox_xyz = data[19:22]
            livox_rpy = data[22:25]
            imu_quat = data[25:29]
            imu_gyro = data[29:32]
            imu_acc = data[32:35]
            ranges = list(data[35:35 + num_ranges])
            point_offset = 35 + MAX_RANGES
            points = data[point_offset:point_offset + num_points * 3]
            return seq, sim_time, pose, qvel, ranges, livox_xyz, livox_rpy, points, imu_quat, imu_gyro, imu_acc
        if version == VERSION_WITH_CLOUD:
            data = struct.unpack(STRUCT_FORMAT_V2, self.mm.read(struct.calcsize(STRUCT_FORMAT_V2)))
            _magic, _version, num_ranges, num_points, seq, sim_time = data[:6]
            num_ranges = min(int(num_ranges), MAX_RANGES)
            num_points = min(int(num_points), MAX_POINTS)
            pose = data[6:13]
            qvel = data[13:19]
            livox_xyz = data[19:22]
            livox_rpy = data[22:25]
            ranges = list(data[25:25 + num_ranges])
            point_offset = 25 + MAX_RANGES
            points = data[point_offset:point_offset + num_points * 3]
            imu_quat = (pose[3], pose[4], pose[5], pose[6])
            imu_gyro = (0.0, 0.0, 0.0)
            imu_acc = (0.0, 0.0, 0.0)
            return seq, sim_time, pose, qvel, ranges, livox_xyz, livox_rpy, points, imu_quat, imu_gyro, imu_acc
        if version == VERSION:
            data = struct.unpack(STRUCT_FORMAT_V1, self.mm.read(struct.calcsize(STRUCT_FORMAT_V1)))
            _magic, _version, num_ranges, seq, sim_time = data[:5]
            num_ranges = min(int(num_ranges), MAX_RANGES)
            pose = data[5:12]
            qvel = data[12:18]
            ranges = list(data[18:18 + num_ranges])
            imu_quat = (pose[3], pose[4], pose[5], pose[6])
            imu_gyro = (0.0, 0.0, 0.0)
            imu_acc = (0.0, 0.0, 0.0)
            return seq, sim_time, pose, qvel, ranges, self.laser_xyz, self.livox_rpy, [], imu_quat, imu_gyro, imu_acc
        return None

    def _quat_from_rpy(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _make_sensor_transforms(self, stamp, livox_xyz=None, livox_rpy=None):
        if livox_xyz is None:
            livox_xyz = self.laser_xyz
        if livox_rpy is None:
            livox_rpy = self.livox_rpy

        scan_tf = TransformStamped()
        scan_tf.header.stamp = stamp
        scan_tf.header.frame_id = self.base_frame
        scan_tf.child_frame_id = self.laser_frame
        scan_tf.transform.translation.x = livox_xyz[0]
        scan_tf.transform.translation.y = livox_xyz[1]
        scan_tf.transform.translation.z = livox_xyz[2]
        scan_tf.transform.rotation.w = 1.0

        qx, qy, qz, qw = self._quat_from_rpy(*livox_rpy)
        livox_tf = TransformStamped()
        livox_tf.header.stamp = stamp
        livox_tf.header.frame_id = self.base_frame
        livox_tf.child_frame_id = self.livox_frame
        livox_tf.transform.translation.x = livox_xyz[0]
        livox_tf.transform.translation.y = livox_xyz[1]
        livox_tf.transform.translation.z = livox_xyz[2]
        livox_tf.transform.rotation.x = qx
        livox_tf.transform.rotation.y = qy
        livox_tf.transform.rotation.z = qz
        livox_tf.transform.rotation.w = qw
        return scan_tf, livox_tf

    def _publish_sensor_tf(self, stamp, livox_xyz=None, livox_rpy=None):
        if self.publish_sensor_tf_static and self.static_tf_sent:
            return
        tf_stamp = stamp
        if self.publish_sensor_tf_static:
            tf_stamp = rclpy.time.Time().to_msg()
        transforms = self._make_sensor_transforms(tf_stamp, livox_xyz, livox_rpy)
        if self.publish_sensor_tf_static:
            self.static_tf_pub.sendTransform(list(transforms))
            self.static_tf_sent = True
            return
        self.tf_pub.sendTransform(list(transforms))

    def _publish_odom_tf(self, stamp, pose, qvel, livox_xyz=None, livox_rpy=None):
        self._publish_sensor_tf(stamp, livox_xyz, livox_rpy)
        if not self.publish_robot_odom:
            return

        x, y, _z, qw, qx, qy, qz = pose

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = qw
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        self.tf_pub.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x = qvel[0]
        odom.twist.twist.linear.y = qvel[1]
        odom.twist.twist.angular.z = qvel[5]
        self.odom_pub.publish(odom)

    def _odom_timer(self):
        sample = self._read()
        if sample is None:
            return
        _seq, _sim_time, pose, qvel, _ranges, livox_xyz, livox_rpy, _points, imu_quat, imu_gyro, imu_acc = sample
        stamp = self.get_clock().now().to_msg()
        self._publish_odom_tf(stamp, pose, qvel, livox_xyz, livox_rpy)
        self._publish_imu(stamp, imu_quat, imu_gyro, imu_acc)

    def _scan_timer(self):
        sample = self._read()
        if sample is None:
            return
        seq, _sim_time, pose, qvel, ranges, livox_xyz, livox_rpy, points, imu_quat, imu_gyro, imu_acc = sample
        if self.last_scan_seq == seq:
            return
        self.last_scan_seq = seq

        stamp = self.get_clock().now().to_msg()
        self._publish_odom_tf(stamp, pose, qvel, livox_xyz, livox_rpy)
        self._publish_imu(stamp, imu_quat, imu_gyro, imu_acc)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.laser_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / float(len(ranges))
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / float(self.get_parameter("scan_hz").value)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)
        self._publish_cloud(stamp, points)

    def _publish_imu(self, stamp, imu_quat, imu_gyro, imu_acc):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.orientation.w = float(imu_quat[0])
        msg.orientation.x = float(imu_quat[1])
        msg.orientation.y = float(imu_quat[2])
        msg.orientation.z = float(imu_quat[3])
        msg.angular_velocity.x = float(imu_gyro[0])
        msg.angular_velocity.y = float(imu_gyro[1])
        msg.angular_velocity.z = float(imu_gyro[2])
        msg.linear_acceleration.x = float(imu_acc[0])
        msg.linear_acceleration.y = float(imu_acc[1])
        msg.linear_acceleration.z = float(imu_acc[2])
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1
        self.imu_pub.publish(msg)

    def _publish_cloud(self, stamp, points):
        point_count = len(points) // 3
        if point_count <= 0:
            return

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.livox_frame
        msg.height = 1
        msg.width = point_count
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * point_count
        msg.is_dense = True

        data = bytearray(msg.row_step)
        for i in range(point_count):
            struct.pack_into(
                "<ffff",
                data,
                i * msg.point_step,
                float(points[3 * i + 0]),
                float(points[3 * i + 1]),
                float(points[3 * i + 2]),
                1.0,
            )
        msg.data = bytes(data)
        self.cloud_pub.publish(msg)

    def destroy_node(self):
        if self.mm is not None:
            self.mm.close()
        if self.fd is not None:
            os.close(self.fd)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeMujocoShmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
