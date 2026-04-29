#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library


MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430


# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = 0     # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4090     # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000

PROTOCOL_VERSION            = 2.0

DXL0_ID                     = 15            # Dynamixel#1 ID : 0
# DXL1_ID                     = 13                # Dynamixel#1 ID : 1
# DXL2_ID                     = 14             # Dynamixel#1 ID :2
# DXL3_ID                     = 15            # Dynamixel#1 ID :2
DEVICENAME                  = '/dev/ttyUSB1'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20         # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


# Enable Dynamixel#0 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL0_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL0_ID)


# Add parameter storage for Dynamixel#0 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL0_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL0_ID)
    quit()


while 1:
    print("Press a(0,0 deg) / s(60,120 deg) / d(120,180 deg)  / (ESC to quit)")
    ch = getch()


    
################################################################
    if ch == chr(0x1b):  # ESC
        break

    if ch == '1':  # 1
        deg_0 = 0
        print("60deg")

    elif ch == '2':  # 2
        deg_0 = 140
        print("120deg")
    elif ch == '3':  # 3
        deg_0 = 180
        print("180 deg")

    elif ch == '0':  # 0
        deg_0 = 0
        print("0 is pressed!")



    else:
        continue

    goal_0 = int(round(deg_0 / 0.087890625))



    if goal_0 < 5: goal_0 = 5
    if goal_0 > 4080: goal_0 = 4090

    # if goal_0 < 740: goal_0 = 745
    # if goal_0 > 3000: goal_0 = 3010
    param_goal_position_0 = list(goal_0.to_bytes(4, byteorder="little", signed=False))

    # Add Dynamixel#0 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL0_ID, param_goal_position_0)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL0_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    while 1:
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        dxl_getdata_result = groupSyncRead.isAvailable(DXL0_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL0_ID)
            quit()

        dxl0_present_position = groupSyncRead.getData(DXL0_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        dxl0_present_deg = dxl0_present_position*0.087890625
        # l2_present_deg = dxl2_present_position*0.087890625

        print("[ID:%03d] Goal:%d  Pres:%d" % (DXL0_ID, goal_0, dxl0_present_position))
    
        if (abs(goal_0 - dxl0_present_position) <= DXL_MOVING_STATUS_THRESHOLD ):
            break



    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Clear syncread parameter storage
groupSyncRead.clearParam()

# Disable Dynamixel#0 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL0_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))


# Close port
portHandler.closePort()
