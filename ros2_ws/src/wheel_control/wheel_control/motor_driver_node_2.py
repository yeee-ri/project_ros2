#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from dynamixel_sdk import *

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_ACCELERATION = 108

LEN_GOAL_VELOCITY = 4

BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DEVICENAME = "/dev/ttyUSB0"

DXL_IDS = [1, 2, 3]

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_CONTROL_MODE = 1
PROFILE_ACCELERATION = 10

WHEEL_RADIUS = 0.05
ROBOT_RADIUS = 0.17
ENCODER_RES = 4096
GEAR_RATIO = 1.0

DXL_VEL_UNIT_RPM = 0.229


def int32_from_uint32(value):
    if value > 2147483647:
        value -= 4294967296
    return value


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            ADDR_GOAL_VELOCITY,
            LEN_GOAL_VELOCITY
        )

        if not self.portHandler.openPort():
            raise RuntimeError("Failed to open port")

        if not self.portHandler.setBaudRate(BAUDRATE):
            raise RuntimeError("Failed to set baudrate")

        for dxl_id in DXL_IDS:
            self.set_operating_mode(dxl_id, VELOCITY_CONTROL_MODE)
            self.write4(dxl_id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)
            self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info("Dynamixel motors connected")

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_positions = self.read_positions()
        self.prev_time = self.get_clock().now()

        self.timer = self.create_timer(0.02, self.update_odom)

    def write1(self, dxl_id, addr, value):
        result, error = self.packetHandler.write1ByteTxRx(
            self.portHandler, dxl_id, addr, value
        )
        if result != COMM_SUCCESS:
            self.get_logger().error(self.packetHandler.getTxRxResult(result))
        elif error != 0:
            self.get_logger().error(self.packetHandler.getRxPacketError(error))

    def write4(self, dxl_id, addr, value):
        result, error = self.packetHandler.write4ByteTxRx(
            self.portHandler, dxl_id, addr, value
        )
        if result != COMM_SUCCESS:
            self.get_logger().error(self.packetHandler.getTxRxResult(result))
        elif error != 0:
            self.get_logger().error(self.packetHandler.getRxPacketError(error))

    def set_operating_mode(self, dxl_id, mode):
        self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.write1(dxl_id, ADDR_OPERATING_MODE, mode)

    def velocity_to_param(self, velocity):
        return list(int(velocity).to_bytes(4, byteorder="little", signed=True))

    def send_goal_velocity(self, v1, v2, v3):
        velocities = [v1, v2, v3]

        for dxl_id, vel in zip(DXL_IDS, velocities):
            param = self.velocity_to_param(vel)
            self.groupSyncWrite.addParam(dxl_id, param)

        result = self.groupSyncWrite.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(self.packetHandler.getTxRxResult(result))

        self.groupSyncWrite.clearParam()

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        theta1 = math.radians(150)
        theta2 = math.radians(270)
        theta3 = math.radians(30)

        w1 = (-math.sin(theta1) * vx + math.cos(theta1) * vy + ROBOT_RADIUS * wz) / WHEEL_RADIUS
        w2 = (-math.sin(theta2) * vx + math.cos(theta2) * vy + ROBOT_RADIUS * wz) / WHEEL_RADIUS
        w3 = (-math.sin(theta3) * vx + math.cos(theta3) * vy + ROBOT_RADIUS * wz) / WHEEL_RADIUS

        dxl_v1 = self.rad_s_to_dxl_velocity(w1)
        dxl_v2 = self.rad_s_to_dxl_velocity(w2)
        dxl_v3 = self.rad_s_to_dxl_velocity(w3)

        self.send_goal_velocity(dxl_v1, dxl_v2, dxl_v3)

    def rad_s_to_dxl_velocity(self, rad_s):
        rpm = rad_s * 60.0 / (2.0 * math.pi)
        return int(rpm / DXL_VEL_UNIT_RPM)

    def read_positions(self):
        positions = []

        for dxl_id in DXL_IDS:
            pos, result, error = self.packetHandler.read4ByteTxRx(
                self.portHandler,
                dxl_id,
                ADDR_PRESENT_POSITION
            )

            if result != COMM_SUCCESS:
                self.get_logger().error(self.packetHandler.getTxRxResult(result))
                positions.append(0)
            elif error != 0:
                self.get_logger().error(self.packetHandler.getRxPacketError(error))
                positions.append(0)
            else:
                positions.append(int32_from_uint32(pos))

        return positions

    def ticks_to_rad(self, ticks):
        return (ticks / ENCODER_RES) * 2.0 * math.pi / GEAR_RATIO

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9

        if dt <= 0.0:
            return

        current_positions = self.read_positions()

        d_ticks = [
            current_positions[i] - self.prev_positions[i]
            for i in range(3)
        ]

        d_rad = [self.ticks_to_rad(tick) for tick in d_ticks]

        v1 = d_rad[0] * WHEEL_RADIUS / dt
        v2 = d_rad[1] * WHEEL_RADIUS / dt
        v3 = d_rad[2] * WHEEL_RADIUS / dt

        vx = (2.0 / 3.0) * v2 - (1.0 / 3.0) * v1 - (1.0 / 3.0) * v3
        vy = (1.0 / math.sqrt(3.0)) * (v3 - v1)
        wz = (v1 + v2 + v3) / (3.0 * ROBOT_RADIUS)

        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

        self.prev_positions = current_positions
        self.prev_time = now

    def destroy_node(self):
        self.send_goal_velocity(0, 0, 0)

        for dxl_id in DXL_IDS:
            self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        self.portHandler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()