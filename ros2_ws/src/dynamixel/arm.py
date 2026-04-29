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
from ik.ik2 import ik_cal

ADDR_TORQUE_ENABLE        = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_GOAL_POSITION        = 116
LEN_GOAL_POSITION         = 4
ADDR_PRESENT_POSITION     = 132
LEN_PRESENT_POSITION      = 4

BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

DXL12_ID = 12
DXL13_ID = 13
DXL14_ID = 14
DXL15_ID = 15

DEVICENAME = '/dev/ttyUSB1'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20

DEG_PER_TICK = 0.087890625

PROFILE_ACCELERATION = 10
PROFILE_VELOCITY = 30

PROFILE_ACCELERATION_gripper = 50
PROFILE_VELOCITY_gripper = 60


def deg_to_tick(deg):
    return int(round(deg / DEG_PER_TICK))

def clamp_goal(goal_0, goal_1, goal_2, goal_3):
    if goal_0 < 740:
        goal_0 = 745
    if goal_0 > 3000:
        goal_0 = 3010

    if goal_1 < 745:
        goal_1 = 745
    if goal_1 > 3345:
        goal_1 = 3350

    if goal_2 < 855:
        goal_2 = 855
    if goal_2 > 3410:
        goal_2 = 3410

    if goal_3 < 3:
        goal_3 = 5
    if goal_3 > 1590:
        goal_3 = 1590

    return goal_0, goal_1, goal_2, goal_3

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

def set_profile(packetHandler, portHandler, dxl_id, accel, velocity):
    write4(packetHandler, portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, accel)
    write4(packetHandler, portHandler, dxl_id, ADDR_PROFILE_VELOCITY, velocity)

def set_profile_gripper(packetHandler, portHandler, dxl_id, accel, velocity):
    write4(packetHandler, portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, accel)
    write4(packetHandler, portHandler, dxl_id, ADDR_PROFILE_VELOCITY, velocity)

def read_present_positions(groupSyncRead):
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
        return None

    if not groupSyncRead.isAvailable(DXL12_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
        print("[ID:%03d] groupSyncRead getdata failed" % DXL12_ID)
        return None
    if not groupSyncRead.isAvailable(DXL13_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
        print("[ID:%03d] groupSyncRead getdata failed" % DXL13_ID)
        return None
    if not groupSyncRead.isAvailable(DXL14_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
        print("[ID:%03d] groupSyncRead getdata failed" % DXL14_ID)
        return None
    if not groupSyncRead.isAvailable(DXL15_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
        print("[ID:%03d] groupSyncRead getdata failed" % DXL15_ID)
        return None

    goal_0 = groupSyncRead.getData(DXL12_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    goal_1 = groupSyncRead.getData(DXL13_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    goal_2 = groupSyncRead.getData(DXL14_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    goal_3 = groupSyncRead.getData(DXL15_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    return goal_0, goal_1, goal_2, goal_3

def send_goal(groupSyncWrite, goal_0, goal_1, goal_2, goal_3):
    goal_0, goal_1, goal_2, goal_3 = clamp_goal(goal_0, goal_1, goal_2, goal_3)

    param_goal_position_0 = list(goal_0.to_bytes(4, byteorder="little", signed=False))
    param_goal_position_1 = list(goal_1.to_bytes(4, byteorder="little", signed=False))
    param_goal_position_2 = list(goal_2.to_bytes(4, byteorder="little", signed=False))
    param_goal_position_3 = list(goal_3.to_bytes(4, byteorder="little", signed=False))

    if not groupSyncWrite.addParam(DXL12_ID, param_goal_position_0):
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL12_ID)
        quit()
    if not groupSyncWrite.addParam(DXL13_ID, param_goal_position_1):
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL13_ID)
        quit()
    if not groupSyncWrite.addParam(DXL14_ID, param_goal_position_2):
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL14_ID)
        quit()
    if not groupSyncWrite.addParam(DXL15_ID, param_goal_position_3):
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL15_ID)
        quit()

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))

    groupSyncWrite.clearParam()

    return goal_0, goal_1, goal_2, goal_3

def wait_until_arrived(groupSyncRead, goal_0, goal_1, goal_2, goal_3, ignore_gripper_goal=False):
    prev_dxl15 = None

    while True:
        positions = read_present_positions(groupSyncRead)
        if positions is None:
            continue

        dxl12_present_position, dxl13_present_position, dxl14_present_position, dxl15_present_position = positions

        print(
            "[ID:%03d] Goal:%d Pres:%d\t[ID:%03d] Goal:%d Pres:%d\t[ID:%03d] Goal:%d Pres:%d\t[ID:%03d] Goal:%d Pres:%d"
            % (
                DXL12_ID, goal_0, dxl12_present_position,
                DXL13_ID, goal_1, dxl13_present_position,
                DXL14_ID, goal_2, dxl14_present_position,
                DXL15_ID, goal_3, dxl15_present_position
            )
        )

        arm_arrived = (
            abs(goal_0 - dxl12_present_position) <= DXL_MOVING_STATUS_THRESHOLD and
            abs(goal_1 - dxl13_present_position) <= DXL_MOVING_STATUS_THRESHOLD and
            abs(goal_2 - dxl14_present_position) <= DXL_MOVING_STATUS_THRESHOLD
        )

        if ignore_gripper_goal:
            grip_arrived = True
            if prev_dxl15 is not None and abs(dxl15_present_position - prev_dxl15) < 3:
                grip_arrived = True
        else:
            grip_arrived = abs(goal_3 - dxl15_present_position) <= DXL_MOVING_STATUS_THRESHOLD

        prev_dxl15 = dxl15_present_position

        if arm_arrived and grip_arrived:
            break

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncWrite = GroupSyncWrite(
    portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION
)
groupSyncRead = GroupSyncRead(
    portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
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

set_profile(packetHandler, portHandler, DXL12_ID, PROFILE_ACCELERATION, PROFILE_VELOCITY)
set_profile(packetHandler, portHandler, DXL13_ID, PROFILE_ACCELERATION, PROFILE_VELOCITY)
set_profile(packetHandler, portHandler, DXL14_ID, PROFILE_ACCELERATION, PROFILE_VELOCITY)
set_profile_gripper(packetHandler, portHandler, DXL15_ID, PROFILE_ACCELERATION_gripper, PROFILE_VELOCITY_gripper)

enable_torque(packetHandler, portHandler, DXL12_ID)
enable_torque(packetHandler, portHandler, DXL13_ID)
enable_torque(packetHandler, portHandler, DXL14_ID)
enable_torque(packetHandler, portHandler, DXL15_ID)

if not groupSyncRead.addParam(DXL12_ID):
    print("[ID:%03d] groupSyncRead addparam failed" % DXL12_ID)
    quit()
if not groupSyncRead.addParam(DXL13_ID):
    print("[ID:%03d] groupSyncRead addparam failed" % DXL13_ID)
    quit()
if not groupSyncRead.addParam(DXL14_ID):
    print("[ID:%03d] groupSyncRead addparam failed" % DXL14_ID)
    quit()
if not groupSyncRead.addParam(DXL15_ID):
    print("[ID:%03d] groupSyncRead addparam failed" % DXL15_ID)
    quit()

while True:
    print("q: close, w: open, m: move, r: read, ESC: quit")
    ch = getch()

    if ch == chr(0x1b):
        break

    if ch == 'q' or ch == 'w':
        positions = read_present_positions(groupSyncRead)
        if positions is None:
            continue

        goal_0, goal_1, goal_2, goal_3 = positions

        if ch == 'q':
            goal_3 = deg_to_tick(0)
            print("Gripper Close")
        else:
            goal_3 = deg_to_tick(140)
            print("Gripper Open")

        goal_0, goal_1, goal_2, goal_3 = send_goal(groupSyncWrite, goal_0, goal_1, goal_2, goal_3)
        wait_until_arrived(groupSyncRead, goal_0, goal_1, goal_2, goal_3, ignore_gripper_goal=True)
        continue

    if ch == 'm':
        try:
            x, y = input("Enter input x, y: ").split()
            d = int(x)
            h = int(y)
        except ValueError:
            print("Again")
            continue

        deg_0, deg_1, deg_2 = ik_cal(d, h)

        if deg_0 is None or deg_1 is None or deg_2 is None:
            print("Calculation Error")
            continue

        positions = read_present_positions(groupSyncRead)
        if positions is None:
            continue

        _, _, _, goal_3 = positions

        goal_0 = deg_to_tick(deg_0)
        goal_1 = deg_to_tick(deg_1)
        goal_2 = deg_to_tick(deg_2)

        goal_0, goal_1, goal_2, goal_3 = send_goal(groupSyncWrite, goal_0, goal_1, goal_2, goal_3)
        wait_until_arrived(groupSyncRead, goal_0, goal_1, goal_2, goal_3, ignore_gripper_goal=False)
        continue

    if ch == 'r':
        positions = read_present_positions(groupSyncRead)
        if positions is None:
            continue

        dxl12_present_position, dxl13_present_position, dxl14_present_position, dxl15_present_position = positions

        dxl12_present_position_deg = dxl12_present_position*360/4095
        dxl13_present_position_deg = dxl13_present_position*360/4095
        dxl14_present_position_deg = dxl14_present_position*360/4095
        dxl15_present_position_deg = dxl15_present_position*360/4095
        

        print(
            "[ID:%03d] Pres:%d (%d)  [ID:%03d] Pres:%d (%d)  [ID:%03d] Pres:%d (%d)  [ID:%03d]  Pres:%d (%d)"
            % (
                DXL12_ID, dxl12_present_position, dxl12_present_position_deg,
                DXL13_ID, dxl13_present_position, dxl13_present_position_deg,
                DXL14_ID, dxl14_present_position, dxl14_present_position_deg,
                DXL15_ID, dxl15_present_position, dxl15_present_position_deg
            )
        )

disable_torque(packetHandler, portHandler, DXL12_ID)
disable_torque(packetHandler, portHandler, DXL13_ID)
disable_torque(packetHandler, portHandler, DXL14_ID)
disable_torque(packetHandler, portHandler, DXL15_ID)

groupSyncRead.clearParam()
portHandler.closePort()