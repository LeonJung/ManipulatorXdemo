#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# manipulatorx_4dof.py
#
#  Created on: 2016. 7. 5.
#      Author: Ryu Woon Jung (Leon), Tae Hoon Lim (Darby)
#

#
# *********     ManipulatorX-4DOF Example      *********
#
#
# This example uses "SyncReadWrite example" using Dynamixel XM-430.
# For the details of the Dynamixel, visit E-Manual(support.robotis.com).
# Be sure that Dynamixel XM-430 properties are already set as %% ID : 1 ~ 5 from the bottom / Baudnum : 3 (Baudrate : 1000000 [1M])
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
ADDR_XM430_TORQUE_DISABLE     = 64


# Data Byte Length
LEN_XM430_GOAL_POSITION           = 4
LEN_XM430_PRESENT_POSITION        = 4
LEN_XM430_ACCELERATION_LIMIT      = 4
LEN_XM430_VELOCITY_LIMIT          = 4
LEN_XM430_TORQUE_ENABLE           = 1
LEN_XM430_POSITION_P_GAIN         = 2
LEN_XM430_PROFLIE_ACCELERATION    = 4
LEN_XM430_PROFILE_VELOCITY        = 4
LEN_XM430_TORQUE_DISABLE          = 1

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                             # Dynamixel ID: 1
DXL2_ID                     = 2                             # Dynamixel ID: 2
DXL3_ID                     = 3                             # Dynamixel ID: 3
DXL4_ID                     = 4                             # Dynamixel ID: 4
DXL5_ID                     = 5                             # Dynamixel ID: 5
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
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

# Initialize Groupsyncread Structs for Present Position
groupread_num = dynamixel.groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)

index = 0
dxl_comm_result = COMM_TX_FAIL                              # Communication result
dxl_addparam_result = 0                                     # AddParam result
dxl_getdata_result = 0                                      # GetParam result

dxl_goal_position = [
    [2048, 2320, 1049, 1770, 2700],                         # Init
    [1920, 2048,  750, 2400, 2400],                         # Bow
    [1920, 1350, 1350, 2530, 2400],                         # Move
    [1920, 1350, 1350, 2530, 2560],                         # Grab
    [2048, 2320, 1049, 1770, 2560],                         # Center
    [2176, 1350, 1350, 2530, 2560],                         # Move
    [2176, 1350, 1350, 2530, 2400],                         # Loose
    [2176, 2048,  750, 2400, 2400],                         # Bow
    [2048, 2320, 1049, 1770, 2700]]

dxl_position_p_gain = 1000

dxl_acc_limit = 1000
dxl_vel_limit = 100
dxl_prof_acc = [4, 4, 4, 9, 4]
dxl_prof_vel = 40
dxl_prof_vel_array = [dxl_prof_vel] * 5

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


# Initialize Groupsyncwrite instance
groupwrite_acceleration_limit = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XM430_ACCELERATION_LIMIT, LEN_XM430_ACCELERATION_LIMIT)

# Add Dynamixel #1~#5 acceleration limit value to the Syncwrite storage
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_acceleration_limit, DXL1_ID, dxl_acc_limit, LEN_XM430_ACCELERATION_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_acceleration_limit, DXL2_ID, dxl_acc_limit, LEN_XM430_ACCELERATION_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_acceleration_limit, DXL3_ID, dxl_acc_limit, LEN_XM430_ACCELERATION_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_acceleration_limit, DXL4_ID, dxl_acc_limit, LEN_XM430_ACCELERATION_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL4_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_acceleration_limit, DXL5_ID, dxl_acc_limit, LEN_XM430_ACCELERATION_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL5_ID))
    quit()

# Syncwrite acceleration limit
dynamixel.groupSyncWriteTxPacket(groupwrite_acceleration_limit)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

# Clear syncwrite parameter storage
dynamixel.groupSyncWriteClearParam(groupwrite_acceleration_limit)


#--------------------------------------------------------------------------------------------------------------#

# Initialize Groupsyncwrite instance
groupwrite_velocity_limit = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XM430_VELOCITY_LIMIT, LEN_XM430_VELOCITY_LIMIT)

# Add Dynamixel #1~#5 velocity limit value to the Syncwrite storage
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_velocity_limit, DXL1_ID, dxl_vel_limit, LEN_XM430_VELOCITY_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_velocity_limit, DXL2_ID, dxl_vel_limit, LEN_XM430_VELOCITY_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_velocity_limit, DXL3_ID, dxl_vel_limit, LEN_XM430_VELOCITY_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_velocity_limit, DXL4_ID, dxl_vel_limit, LEN_XM430_VELOCITY_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL4_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_velocity_limit, DXL5_ID, dxl_vel_limit, LEN_XM430_VELOCITY_LIMIT)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL5_ID))
    quit()

# Syncwrite velocity limit
dynamixel.groupSyncWriteTxPacket(groupwrite_velocity_limit)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

# Clear syncwrite parameter storage
dynamixel.groupSyncWriteClearParam(groupwrite_velocity_limit)

#--------------------------------------------------------------------------------------------------------------#
# Initialize Groupsyncwrite instance
groupwrite_torque_enable = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XM430_TORQUE_ENABLE, LEN_XM430_TORQUE_ENABLE)

# Add Dynamixel #1~#5 torque enable value to the Syncwrite storage
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_enable, DXL1_ID, TORQUE_ENABLE, LEN_XM430_TORQUE_ENABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_enable, DXL2_ID, TORQUE_ENABLE, LEN_XM430_TORQUE_ENABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_enable, DXL3_ID, TORQUE_ENABLE, LEN_XM430_TORQUE_ENABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_enable, DXL4_ID, TORQUE_ENABLE, LEN_XM430_TORQUE_ENABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL4_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_enable, DXL5_ID, TORQUE_ENABLE, LEN_XM430_TORQUE_ENABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL5_ID))
    quit()

# Syncwrite torque enable
dynamixel.groupSyncWriteTxPacket(groupwrite_torque_enable)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

# Clear syncwrite parameter storage
dynamixel.groupSyncWriteClearParam(groupwrite_torque_enable)


#--------------------------------------------------------------------------------------------------------------#

# Initialize Groupsyncwrite instance
groupwrite_profile_acceleration      = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XM430_PROF_ACCELERATION, LEN_XM430_PROFLIE_ACCELERATION)

# Add Dynamixel #1~#5 profile acceleration value to the Syncwrite storage
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_acceleration, DXL1_ID, dxl_prof_acc[DXL1_ID - 1], LEN_XM430_PROFLIE_ACCELERATION)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_acceleration, DXL2_ID, dxl_prof_acc[DXL2_ID - 1], LEN_XM430_PROFLIE_ACCELERATION)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_acceleration, DXL3_ID, dxl_prof_acc[DXL3_ID - 1], LEN_XM430_PROFLIE_ACCELERATION)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_acceleration, DXL4_ID, dxl_prof_acc[DXL4_ID - 1], LEN_XM430_PROFLIE_ACCELERATION)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL4_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_acceleration, DXL5_ID, dxl_prof_acc[DXL5_ID - 1], LEN_XM430_PROFLIE_ACCELERATION)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL5_ID))
    quit()

# Syncwrite profile acceleration
dynamixel.groupSyncWriteTxPacket(groupwrite_profile_acceleration)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

# Clear syncwrite parameter storage
dynamixel.groupSyncWriteClearParam(groupwrite_profile_acceleration)

#--------------------------------------------------------------------------------------------------------------#


# Initialize Groupsyncwrite instance
groupwrite_position_p_gain       = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XM430_POSITION_P_GAIN, LEN_XM430_POSITION_P_GAIN)

# Add Dynamixel #1~#5 position p gain value to the Syncwrite storage
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_position_p_gain, DXL1_ID, dxl_position_p_gain, LEN_XM430_POSITION_P_GAIN)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_position_p_gain, DXL2_ID, dxl_position_p_gain, LEN_XM430_POSITION_P_GAIN)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_position_p_gain, DXL3_ID, dxl_position_p_gain, LEN_XM430_POSITION_P_GAIN)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_position_p_gain, DXL4_ID, dxl_position_p_gain, LEN_XM430_POSITION_P_GAIN)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL4_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_position_p_gain, DXL5_ID, dxl_position_p_gain, LEN_XM430_POSITION_P_GAIN)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL5_ID))
    quit()

# Syncwrite position p gain
dynamixel.groupSyncWriteTxPacket(groupwrite_position_p_gain)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

# Clear syncwrite parameter storage
dynamixel.groupSyncWriteClearParam(groupwrite_position_p_gain)

#--------------------------------------------------------------------------------------------------------------#


# Add parameter storage for Dynamixel #1~#5 present position values
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL1_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL1_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL2_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL2_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL3_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL3_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL4_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL4_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL5_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL5_ID))
    quit()


# Syncread present position
dynamixel.groupSyncReadTxRxPacket(groupread_num)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))


# Check if groupsyncread data of Dynamixel #1~#5 is available
dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL1_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
if dxl_getdata_result != 1:
    print("[ID:%03d] groupSyncRead getdata failed" % (DXL1_ID))
    quit()

dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL2_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
if dxl_getdata_result != 1:
    print("[ID:%03d] groupSyncRead getdata failed" % (DXL2_ID))
    quit()

dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL3_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
if dxl_getdata_result != 1:
    print("[ID:%03d] groupSyncRead getdata failed" % (DXL3_ID))
    quit()

dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL4_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
if dxl_getdata_result != 1:
    print("[ID:%03d] groupSyncRead getdata failed" % (DXL4_ID))
    quit()

dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL5_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
if dxl_getdata_result != 1:
    print("[ID:%03d] groupSyncRead getdata failed" % (DXL5_ID))
    quit()

# Get Dynamixel #1~#5 present position values
dxl1_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL1_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)
dxl2_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL2_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)
dxl3_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL3_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)
dxl4_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL4_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)
dxl5_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL5_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)


while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(ESC_ASCII_VALUE):
        break

    # Run 9 poses for manipulation
    for pose in range(0, 9):
        dxl_position = [
            abs(dxl1_present_position - dxl_goal_position[pose][DXL1_ID - 1]),
            abs(dxl2_present_position - dxl_goal_position[pose][DXL2_ID - 1]),
            abs(dxl3_present_position - dxl_goal_position[pose][DXL3_ID - 1]),
            abs(dxl4_present_position - dxl_goal_position[pose][DXL4_ID - 1]),
            abs(dxl5_present_position - dxl_goal_position[pose][DXL5_ID - 1])]

        dxl_max_position = max(dxl_position)
        if dxl_max_position == 0:
            dxl_max_position = 1

        dxl_prof_vel_array = [dxl_prof_vel] * 5
        dxl_prof_vel_array[DXL1_ID - 1] = dxl_prof_vel_array[DXL1_ID - 1] * (abs(dxl1_present_position - dxl_goal_position[pose][DXL1_ID - 1]) / dxl_max_position)
        dxl_prof_vel_array[DXL2_ID - 1] = dxl_prof_vel_array[DXL2_ID - 1] * (abs(dxl2_present_position - dxl_goal_position[pose][DXL2_ID - 1]) / dxl_max_position)
        dxl_prof_vel_array[DXL3_ID - 1] = dxl_prof_vel_array[DXL3_ID - 1] * (abs(dxl3_present_position - dxl_goal_position[pose][DXL3_ID - 1]) / dxl_max_position)
        dxl_prof_vel_array[DXL4_ID - 1] = dxl_prof_vel_array[DXL4_ID - 1] * (abs(dxl4_present_position - dxl_goal_position[pose][DXL4_ID - 1]) / dxl_max_position)
        dxl_prof_vel_array[DXL5_ID - 1] = dxl_prof_vel_array[DXL5_ID - 1] * (abs(dxl5_present_position - dxl_goal_position[pose][DXL5_ID - 1]) / dxl_max_position)

        # Initialize Groupsyncwrite instance
        groupwrite_profile_velocity = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XM430_PROF_VELOCITY, LEN_XM430_PROFILE_VELOCITY)

        # Add Dynamixel #1~5 profile velocity value to the Syncwrite storage
        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_velocity, DXL1_ID, int(dxl_prof_vel_array[DXL1_ID-1]), LEN_XM430_PROFILE_VELOCITY)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
            quit()

        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_velocity, DXL2_ID, int(dxl_prof_vel_array[DXL2_ID-1]), LEN_XM430_PROFILE_VELOCITY)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
            quit()

        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_velocity, DXL3_ID, int(dxl_prof_vel_array[DXL3_ID-1]), LEN_XM430_PROFILE_VELOCITY)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
            quit()

        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_velocity, DXL4_ID, int(dxl_prof_vel_array[DXL4_ID-1]), LEN_XM430_PROFILE_VELOCITY)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL4_ID))
            quit()

        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_profile_velocity, DXL5_ID, int(dxl_prof_vel_array[DXL5_ID-1]), LEN_XM430_PROFILE_VELOCITY)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL5_ID))
            quit()

        # Syncwrite profile velocity
        dynamixel.groupSyncWriteTxPacket(groupwrite_profile_velocity)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(groupwrite_profile_velocity)

        # Initialize Groupsyncwrite instance
        groupwrite_goal_position = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XM430_GOAL_POSITION, LEN_XM430_GOAL_POSITION)

        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_goal_position, DXL1_ID, dxl_goal_position[pose][DXL1_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
            quit()

        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_goal_position, DXL2_ID, dxl_goal_position[pose][DXL2_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
            quit()

        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_goal_position, DXL3_ID, dxl_goal_position[pose][DXL3_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
            quit()

        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_goal_position, DXL4_ID, dxl_goal_position[pose][DXL4_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL4_ID))
            quit()

        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_goal_position, DXL5_ID, dxl_goal_position[pose][DXL5_ID - 1], LEN_XM430_GOAL_POSITION)).value
        if dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncWrite addparam failed" % (DXL5_ID))
            quit()

        # Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(groupwrite_goal_position)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(groupwrite_goal_position)

        while 1:
            # Syncread present position
            dynamixel.groupSyncReadTxRxPacket(groupread_num)
            if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

            # Check if groupsyncread data of Dynamixel #1~#5 is available
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL1_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL1_ID))
                quit()

            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL2_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL2_ID))
                quit()

            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL3_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL3_ID))
                quit()

            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL4_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL4_ID))
                quit()

            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL5_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (DXL5_ID))
                quit()

            # Get Dynamixel #1~#5 present position value
            dxl1_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL1_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)
            dxl2_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL2_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)
            dxl3_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL3_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)
            dxl4_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL4_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)
            dxl5_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL5_ID, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d"
                   % (DXL1_ID, dxl_goal_position[pose][DXL1_ID - 1], dxl1_present_position, DXL2_ID, dxl_goal_position[pose][DXL2_ID - 1], dxl2_present_position, DXL3_ID, dxl_goal_position[pose][DXL3_ID - 1], dxl3_present_position, DXL4_ID, dxl_goal_position[pose][DXL4_ID - 1], dxl4_present_position, DXL5_ID, dxl_goal_position[pose][DXL5_ID - 1], dxl5_present_position))

            if not ((abs(dxl_goal_position[pose][DXL1_ID - 1] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) or (abs(dxl_goal_position[pose][DXL2_ID - 1] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break

        time.sleep(2.5)

# Initialize Groupsyncwrite instance
groupwrite_torque_disable = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XM430_TORQUE_DISABLE, LEN_XM430_TORQUE_DISABLE)

# Add Dynamixel #1~#5 torque enable value to the Syncwrite storage
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_disable, DXL1_ID, TORQUE_DISABLE, LEN_XM430_TORQUE_DISABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_disable, DXL2_ID, TORQUE_DISABLE, LEN_XM430_TORQUE_DISABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_disable, DXL3_ID, TORQUE_DISABLE, LEN_XM430_TORQUE_DISABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_disable, DXL4_ID, TORQUE_DISABLE, LEN_XM430_TORQUE_DISABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL4_ID))
    quit()

dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_torque_disable, DXL5_ID, TORQUE_DISABLE, LEN_XM430_TORQUE_DISABLE)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncWrite addparam failed" % (DXL5_ID))
    quit()

# Syncwrite torque enable
dynamixel.groupSyncWriteTxPacket(groupwrite_torque_disable)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

# Clear syncwrite parameter storage
dynamixel.groupSyncWriteClearParam(groupwrite_torque_disable)

# Close port
dynamixel.closePort(port_num)
