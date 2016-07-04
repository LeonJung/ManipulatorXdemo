#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# sync_read_write.py
#
#  Created on: 2016. 6. 16.
#      Author: Ryu Woon Jung (Leon)
#

#
# *********     Sync Read and Sync Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is designed for using two Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
# To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below variables yourself.
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000 [1M])
#

import os, ctypes, time

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

os.sys.path.append('../dynamixel_functions_py')             # Path setting

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

# Control table address
ADDR_XM430_ACCELERATION_LIMIT = 40                          # Control table address is different in Dynamixel model
ADDR_XM430_VELOCITY_LIMIT     = 44
ADDR_XM430_TORQUE_ENABLE      = 64
ADDR_XM430_POSITION_P_GAIN    = 84
ADDR_XM430_PROF_ACCELERATION  = 108
ADDR_XM430_PROF_VELOCITY      = 112
ADDR_XM430_GOAL_POSITION      = 116
ADDR_XM430_PRESENT_POSITION   = 132


# Data Byte Length
LEN_XM430_GOAL_POSITION       = 4
LEN_XM430_PRESENT_POSITION    = 4

LEN_XM430_GOAL_VELOCITY       = 2

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                             # Dynamixel ID: 1
DXL2_ID                     = 2                             # Dynamixel ID: 2
DXL3_ID                     = 3                             # Dynamixel ID: 3
DXL4_ID                     = 4                             # Dynamixel ID: 4
DXL5_ID                     = 5                             # Dynamixel ID: 5
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')        # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 30                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = dynamixel.portHandler(DEVICENAME)

# Initialize PacketHandler Structs
dynamixel.packetHandler()

# Initialize Groupsyncwrite instance
groupwrite_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XM430_GOAL_POSITION, LEN_XM430_GOAL_POSITION)

# Initialize Groupsyncread Structs for Present Position
groupread_num = dynamixel.groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)

index = 0
dxl_comm_result = COMM_TX_FAIL                              # Communication result
dxl_addparam_result = 0                                     # AddParam result
dxl_getdata_result = 0                                      # GetParam result

dxl_goal_position = [
    [2048, 2048, 2048, 2048, 2700],                         # Init
    [1920, 2048, 750, 2400, 2400],                          # Bow
    [1920, 1350, 1350, 2530, 2400],                         # Move
    [1920, 1350, 1350, 2530, 2560],                         # Grab
    [2048, 2048, 2048, 2048, 2560],                         # Center
    [2176, 1350, 1350, 2530, 2560],                         # Move
    [2176, 1350, 1350, 2530, 2400],                         # Loose
    [2176, 2048, 750, 2400, 2400],                          # Bow
    [2048, 2048, 2048, 2048, 2700]]                         # Center

dxl_position_p_gain = 700

dxl_acc_limit = 4
dxl_vel_limit = 40
dxl_prof_acc = 4
dxl_prof_vel = 40

dxl_error = 0                                               # Dynamixel error

# Open port
if dynamixel.openPort(port_num):
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if dynamixel.setBaudRate(port_num, BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    print("Press any key to terminate...")
    getch()
    quit()




# Acceleration Limit #1
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_XM430_ACCELERATION_LIMIT, dxl_acc_limit)
# Acceleration Limit #2
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_XM430_ACCELERATION_LIMIT, dxl_acc_limit)
# Acceleration Limit #3
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_XM430_ACCELERATION_LIMIT, dxl_acc_limit)
# Acceleration Limit #4
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_XM430_ACCELERATION_LIMIT, dxl_acc_limit)
# Acceleration Limit #5
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_XM430_ACCELERATION_LIMIT, dxl_acc_limit)


# Velocity Limit #1
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_XM430_VELOCITY_LIMIT, dxl_vel_limit)
# Velocity Limit #2
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_XM430_VELOCITY_LIMIT, dxl_vel_limit)
# Velocity Limit #3
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_XM430_VELOCITY_LIMIT, dxl_vel_limit)
# Velocity Limit #4
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_XM430_VELOCITY_LIMIT, dxl_vel_limit)
# Velocity Limit #5
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_XM430_VELOCITY_LIMIT, dxl_vel_limit)


# Enable Dynamixel#1 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_ENABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
else:
    print("Dynamixel#1 has been successfully connected")

# Enable Dynamixel#2 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_ENABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
else:
    print("Dynamixel#2 has been successfully connected")

# Enable Dynamixel#3 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_ENABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
else:
    print("Dynamixel#3 has been successfully connected")

# Enable Dynamixel#4 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_ENABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
else:
    print("Dynamixel#4 has been successfully connected")

# Enable Dynamixel#5 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_ENABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
else:
    print("Dynamixel#5 has been successfully connected")





# Profile Acceleration Limit #1
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_XM430_PROF_ACCELERATION, dxl_prof_acc)
# Profile Acceleration Limit #2
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_XM430_PROF_ACCELERATION, dxl_prof_acc)
# Profile Acceleration Limit #3
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_XM430_PROF_ACCELERATION, dxl_prof_acc)
# Profile Acceleration Limit #4
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_XM430_PROF_ACCELERATION, dxl_prof_acc)
# Profile Acceleration Limit #5
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_XM430_PROF_ACCELERATION, dxl_prof_acc)


# Profile Velocity Limit #1
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_XM430_PROF_VELOCITY, dxl_prof_vel)
# Profile Velocity Limit #2
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_XM430_PROF_VELOCITY, dxl_prof_vel)
# Profile Velocity Limit #3
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_XM430_PROF_VELOCITY, dxl_prof_vel)
# Profile Velocity Limit #4
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_XM430_PROF_VELOCITY, dxl_prof_vel)
# Profile Velocity Limit #5
dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_XM430_PROF_VELOCITY, dxl_prof_vel)





# Enable position p gain #1
dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_XM430_POSITION_P_GAIN, dxl_position_p_gain)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Enable position p gain #2
dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_XM430_POSITION_P_GAIN, dxl_position_p_gain)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Enable position p gain #3
dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_XM430_POSITION_P_GAIN, dxl_position_p_gain)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Enable position p gain #4
dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_XM430_POSITION_P_GAIN, dxl_position_p_gain)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Enable position p gain #5
dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_XM430_POSITION_P_GAIN, dxl_position_p_gain)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))


# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL1_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL1_ID))
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL2_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL2_ID))
    quit()

# Add parameter storage for Dynamixel#3 present position value
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL3_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL3_ID))
    quit()

# Add parameter storage for Dynamixel#4 present position value
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL4_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL4_ID))
    quit()

# Add parameter storage for Dynamixel#5 present position value
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL5_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL5_ID))
    quit()


while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(ESC_ASCII_VALUE):
        break

    for pose in range(0, 9):
        # Add Dynamixel#1 goal position value to the Syncwrite storage
        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_num, DXL1_ID, dxl_goal_position[pose][DXL1_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_num, DXL2_ID, dxl_goal_position[pose][DXL2_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_num, DXL3_ID, dxl_goal_position[pose][DXL3_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_num, DXL4_ID, dxl_goal_position[pose][DXL4_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL4_ID))
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_num, DXL5_ID, dxl_goal_position[pose][DXL5_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL5_ID))
            quit()

        # Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(groupwrite_num)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(groupwrite_num)

        while 1:
            # Syncread present position
            dynamixel.groupSyncReadTxRxPacket(groupread_num)
            if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL1_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL1_ID))
                quit()

            # Check if groupsyncread data of Dynamixel#2 is available
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL2_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL2_ID))
                quit()

            # Check if groupsyncread data of Dynamixel#3 is available
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL3_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL3_ID))
                quit()

            # Check if groupsyncread data of Dynamixel#4 is available
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL4_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL4_ID))
                quit()

            # Check if groupsyncread data of Dynamixel#5 is available
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL5_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL5_ID))
                quit()

            # Get Dynamixel#1 present position value
            dxl1_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL1_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)

            # Get Dynamixel#2 present position value
            dxl2_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL2_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)

            # Get Dynamixel#3 present position value
            dxl3_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL3_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)

            # Get Dynamixel#4 present position value
            dxl4_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL4_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)

            # Get Dynamixel#5 present position value
            dxl5_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL5_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d"
                   % (DXL1_ID, dxl_goal_position[pose][DXL1_ID - 1], dxl1_present_position, DXL2_ID, dxl_goal_position[pose][DXL2_ID - 1], dxl2_present_position, DXL3_ID, dxl_goal_position[pose][DXL3_ID - 1], dxl3_present_position, DXL4_ID, dxl_goal_position[pose][DXL4_ID - 1], dxl4_present_position, DXL5_ID, dxl_goal_position[pose][DXL5_ID - 1], dxl5_present_position))

            if not ((abs(dxl_goal_position[pose][DXL1_ID - 1] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) or (abs(dxl_goal_position[pose][DXL2_ID - 1] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break

        time.sleep(2.5)

# Disable Dynamixel#1 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_DISABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Disable Dynamixel#2 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_DISABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Disable Dynamixel#3 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_DISABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Disable Dynamixel#4 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_DISABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Disable Dynamixel#5 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_DISABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Close port
dynamixel.closePort(port_num)
