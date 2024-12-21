#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os
from scipy.signal import find_peaks, peak_prominences
import scipy.fftpack
import numpy as np
import psocScanner as psoc
import math

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


def get_to_position(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, goal_position, DXL_MOVING_STATUS_THRESHOLD):
    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, goal_position, dxl_present_position))

        if not abs(goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break






# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = "Com4" #'/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

#===============================================================================
# Motor position
DXL_MINIMUM_POSITION_VALUE  = 385 #acrylic 425           # Dynamixel will rotate between this value ... RANGE is [0,4095]
DXL_MIDDLE_POSITION_VALUE  = 386 #acrylic 429           # Dynamixel will rotate between this value ... RANGE is [0,4095]
DXL_MAXIMUM_POSITION_VALUE  = 500            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MAXIMUM_POSITION_VALUE, DXL_MIDDLE_POSITION_VALUE, DXL_MINIMUM_POSITION_VALUE, ]         # Goal position
#===============================================================================


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# P-gain (address:26, default:32, max:254)
# I-gain (address:27, default:0, max:254)
# D-gain (address:28, default:0, max:254)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 27, 100) #I
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 26, 200) #P
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 28, 200) #D

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


#===============================================================================
################### Close Motor Until Touch is Detected#########################
# Open Hand
p = 420
while 1:
    print("Openning hand... Press any key to continue! (or press ESC to quit!)")
    got_key = getch()
    if got_key == chr(0x1b):
        break
    get_to_position(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, p, DXL_MOVING_STATUS_THRESHOLD)
    p -= 1
    while 1:
        print("Closing hand... Press any key to continue, or R/r to reset, or backspace to go back! (or press ESC to quit!)")
        got_key = getch()
        if got_key == chr(0x1b):
            break
        elif got_key == chr(0x72) or got_key == chr(0x52):
            p = 420
        elif got_key == chr(0x08):
            p += 15
        else:
            p -= 1

        get_to_position(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, p, DXL_MOVING_STATUS_THRESHOLD)
        #if p == 380:
        #    break
        #p -= 1
    break

# Close port
portHandler.closePort()
