#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys
    import tty
    import termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *
from wheel_cal.omni import cal_vel

ADDR_OPERATING_MODE       = 11
ADDR_TORQUE_ENABLE        = 64
ADDR_GOAL_VELOCITY        = 104
LEN_GOAL_VELOCITY         = 4
ADDR_PROFILE_ACCELERATION = 108

BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

DXL1_ID = 1
DXL2_ID = 2
DXL3_ID = 3

DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

VELOCITY_CONTROL_MODE = 1

PROFILE_ACCELERATION = 10


def write1(packetHandler, portHandler, dxl_id, addr, value):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, addr, value
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
        quit()
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))
        quit()


def write4(packetHandler, portHandler, dxl_id, addr, value):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, dxl_id, addr, value
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
        quit()
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))
        quit()


def enable_torque(packetHandler, portHandler, dxl_id):
    write1(packetHandler, portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    print("Dynamixel#%d has been successfully connected" % dxl_id)


def disable_torque(packetHandler, portHandler, dxl_id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))


def set_operating_mode(packetHandler, portHandler, dxl_id, mode):
    disable_torque(packetHandler, portHandler, dxl_id)
    write1(packetHandler, portHandler, dxl_id, ADDR_OPERATING_MODE, mode)


def set_profile(packetHandler, portHandler, dxl_id, accel):
    write4(packetHandler, portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, accel)


def velocity_to_param(velocity):
    return list(int(velocity).to_bytes(4, byteorder="little", signed=True))


def send_goal(groupSyncWrite, goal_0, goal_1, goal_2):
    param_goal_velocity_0 = velocity_to_param(goal_0)
    param_goal_velocity_1 = velocity_to_param(goal_1)
    param_goal_velocity_2 = velocity_to_param(goal_2)

    if not groupSyncWrite.addParam(DXL1_ID, param_goal_velocity_0):
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()
    if not groupSyncWrite.addParam(DXL2_ID, param_goal_velocity_1):
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
        quit()
    if not groupSyncWrite.addParam(DXL3_ID, param_goal_velocity_2):
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
        quit()

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))

    groupSyncWrite.clearParam()

    return goal_0, goal_1, goal_2


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncWrite = GroupSyncWrite(
    portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY
)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    getch()
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    getch()
    quit()

set_operating_mode(packetHandler, portHandler, DXL1_ID, VELOCITY_CONTROL_MODE)
set_operating_mode(packetHandler, portHandler, DXL2_ID, VELOCITY_CONTROL_MODE)
set_operating_mode(packetHandler, portHandler, DXL3_ID, VELOCITY_CONTROL_MODE)

set_profile(packetHandler, portHandler, DXL1_ID, PROFILE_ACCELERATION)
set_profile(packetHandler, portHandler, DXL2_ID, PROFILE_ACCELERATION)
set_profile(packetHandler, portHandler, DXL3_ID, PROFILE_ACCELERATION)

enable_torque(packetHandler, portHandler, DXL1_ID)
enable_torque(packetHandler, portHandler, DXL2_ID)
enable_torque(packetHandler, portHandler, DXL3_ID)

while True:
    print("w: forward, s: backward, a: left, d: right, q: rotate left, e: rotate right, x: stop, ESC: quit")
    ch = getch()

    if ch == chr(0x1b):
        break

    if ch in ['w', 'a', 's', 'd', 'q', 'e', 'x']:
        goal_0, goal_1, goal_2 = cal_vel(ch)
        goal_0, goal_1, goal_2 = send_goal(groupSyncWrite, goal_0, goal_1, goal_2)
        print("[ID:%03d] Vel:%d  [ID:%03d] Vel:%d  [ID:%03d] Vel:%d"
              % (DXL1_ID, goal_0, DXL2_ID, goal_1, DXL3_ID, goal_2))

send_goal(groupSyncWrite, 0, 0, 0)

disable_torque(packetHandler, portHandler, DXL1_ID)
disable_torque(packetHandler, portHandler, DXL2_ID)
disable_torque(packetHandler, portHandler, DXL3_ID)

portHandler.closePort()
