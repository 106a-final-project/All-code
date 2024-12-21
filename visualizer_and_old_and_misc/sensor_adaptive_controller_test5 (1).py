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
import time
import matplotlib.pyplot as plt
import datetime
import keyboard # import the keyboard module
from visualizer import vectorFieldPixelVisualizer as VisualizerPixel

plt.ion()
visualizer = VisualizerPixel()
visualizer.redraw()

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

def get_sensor_deets(sensingMode,SensorNum):
    MODE_ONE_PAD = 0    # measures pressure, "super fast" (~kHz), 1 channel
    MODE_FOUR_PAD = 1   # measures linear in 4 directions
    MODE_NINE_PAD = 2   # measures pressure across "nibs" (each is a cluster of 4 pips)
    MODE_INDIVIDUAL = 3 # measures each electrode (hardwired across 4 pips)
    MODE_LINEAR_ROTATIONAL = 4  # measures linear (up/down) and rotational (cw/ccw)
    MODE_CAPSENSE = 17
    MODE_ONE_AND_FOUR =32
    MODE_NINE_AND_INDIV= 33
    MODE_ONE_AND_NINE = 34
    MODE_FOUR_AND_INDIV = 35
    MODE_TOR_NS_AND_INDIV = 36

    num_sensors_2 = 0
    IsTwoModeMerged = False
    if sensingMode == MODE_ONE_PAD:
        SamplingFreq = 2e3
    elif sensingMode == MODE_FOUR_PAD:
        SamplingFreq = 600
    elif sensingMode == MODE_NINE_PAD:
        SamplingFreq = 150
    elif sensingMode == MODE_INDIVIDUAL:
        SamplingFreq = 60
    elif sensingMode == MODE_LINEAR_ROTATIONAL:
        SamplingFreq = 600
    elif sensingMode == MODE_ONE_AND_FOUR:
        SamplingFreq = 1e3
        num_sensors_2 = 4
        IsTwoModeMerged = True
    elif sensingMode == MODE_ONE_AND_NINE:
        SamplingFreq = 1e3
        num_sensors_2 = 9
        IsTwoModeMerged = True
    elif sensingMode == MODE_NINE_AND_INDIV:
        SamplingFreq = 110
        num_sensors_2 = 36
        IsTwoModeMerged = True
    elif sensingMode == MODE_FOUR_AND_INDIV:
        SamplingFreq = 250
        num_sensors_2 = 36
        IsTwoModeMerged = True
    elif sensingMode == MODE_TOR_NS_AND_INDIV:
        SamplingFreq = 250
        num_sensors_2 = 36
        IsTwoModeMerged = True
    SamplingPeriod = 1.0/SamplingFreq
    KitprogTimerClockFreq = 100e3
    tempPeriodInput = divmod(math.floor(SamplingPeriod * KitprogTimerClockFreq - 1) , 2**8)
    periodInput = np.array([tempPeriodInput[1], tempPeriodInput[0]])
    ts.ser.flushInput()
    thisInputArray = np.array([ord('a'), SensorNum, SensorAddress[0], SensorAddress[1]])
    ts.sendNum(thisInputArray)
    time.sleep(0.1)
    thisInputArray = np.array([ord('p'), periodInput[0], periodInput[1]])
    ts.sendNum(thisInputArray)
    thisInputArray = np.array([ord('m'), sensingMode])
    ts.sendNum(thisInputArray)
    time.sleep(0.1)
    ts.sendChar("q")
    time.sleep(0.1)
    ts.packet_size = ord(ts.ser.read(1))-1
    num_sensors_1= (ts.packet_size - 1) / 2
    #if sensingMode == MODE_INDIVIDUAL:
    #    num_sensors_1 = 4
    ts.num_sensors= (ts.packet_size - 1) / 2
    ts.unpackFormat = '<'
    for i in range(0,ts.packet_size):
        ts.unpackFormat = ts.unpackFormat + 'B'
    if IsTwoModeMerged: #% Deal with Merging Techniq
        num_sensors_1 = num_sensors_1/2
        sensorIndexInData_1 = list(range(0,(ts.packet_size - 1) / 2, 2) )
        sensorIndexInData_2 = list(range(1,(ts.packet_size - 1) / 2, 2) )
        groupIndexMax = num_sensors_2/ num_sensors_1 -1; #% Follows C convention
    else:
        num_sensors_2 = 0   # % Dummy
    print("num_sensor 1=")
    num_sensors_1 = int(num_sensors_1)
    print(num_sensors_1)
    print("num_sensor 2=")
    num_sensors_2 = int(num_sensors_2)
    print(num_sensors_2)
    return num_sensors_1, num_sensors_2, SamplingFreq

def convert_ind2UDtor(tempSampledData):
    wbtt = tempSampledData[0][[2, 3, 6, 9, 10, 15, 21, 24, 27, 28, 32, 33]] #ccw
    wbtt0 = (wbtt.sum(axis=0) - individual_baseline_g[0]) / scale_factor[0] + lin_rot_baseline[0]
    wbtt = tempSampledData[0][[1,5,9,13,17,21,25,29,33]] #up
    wbtt1 = (wbtt.sum(axis=0) - individual_baseline_g[1]) / scale_factor[1] + lin_rot_baseline[1]
    wbtt = tempSampledData[0][[0, 1, 4, 8, 11, 13, 23, 25, 26, 30, 34, 35]] #cw
    wbtt2 = (wbtt.sum(axis=0) - individual_baseline_g[2]) / scale_factor[2] + lin_rot_baseline[2]
    wbtt = tempSampledData[0][[3,7,11,15,19,23,27,31,35]] #down
    wbtt3 = (wbtt.sum(axis=0) - individual_baseline_g[3]) / scale_factor[3] + lin_rot_baseline[3]
    return np.array([[wbtt0,wbtt1,wbtt2,wbtt3]])
    
def convert_ind2Pressure(tempSampledData):
    wbtt = tempSampledData[0].sum(axis=0)#[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35]] #pressure
    return wbtt
    
def convert_ind2PartialPressure(tempSampledData):
    wbtt0 = tempSampledData[0].sum(axis=0) #Pressure at ALL
    wbtt1 = tempSampledData[0][[0,1,2,3]].sum(axis=0) #Pressure at 1
    wbtt2 = tempSampledData[0][[4,5,6,7]].sum(axis=0) #Pressure at 2
    wbtt3 = tempSampledData[0][[8,9,10,11]].sum(axis=0) #Pressure at 3
    wbtt4 = tempSampledData[0][[12,13,14,15]].sum(axis=0) #Pressure at 4
    wbtt5 = tempSampledData[0][[16,17,18,19]].sum(axis=0) #Pressure at 5
    wbtt6 = tempSampledData[0][[20,21,22,23]].sum(axis=0) #Pressure at 6
    wbtt7 = tempSampledData[0][[24,25,26,27]].sum(axis=0) #Pressure at 7
    wbtt8 = tempSampledData[0][[28,29,30,31]].sum(axis=0) #Pressure at 8
    wbtt9 = tempSampledData[0][[32,33,34,35]].sum(axis=0) #Pressure at 9
    return [wbtt0,wbtt1,wbtt2,wbtt3,wbtt4,wbtt5,wbtt6,wbtt7,wbtt8,wbtt9]

def pd_controller(sense_target,sense_current):
    Kp = 1 # POPORTIONAL You can adjust this value as needed
    Kd = 0.1 # DERIVATIVE You can adjust this value as needed
    Ki = 0.01 # INTEGRAL You can adjust this value as needed
    
    # Calculate the error and its derivative
    error = sense_target - sense_current
    print(error)
    derivative = (error - pd_controller.prev_error) / pd_controller.dt # Assume a constant time step
    
    # Update the previous error
    pd_controller.prev_error = error
    pd_controller.integral += error * pd_controller.dt

    # Calculate the output and return
    sense_output = Kp * error + Kd * derivative + Ki * pd_controller.integral
    return sense_output


def sense2imp(sense_output):
    min_sense = 156000
    max_sense = 167000
    min_imp = 60
    max_imp = 600
    
    #target_imp = (sense_output-min_sense) / (max_sense-min_sense) * (max_imp-min_imp) + min_imp
    target_imp = sense_output/(max_sense-min_sense) * (max_imp-min_imp)
    return target_imp

#===============================================================================
# Sensor initializations
howManyToMean = 3 # peaks and stuff
ts = psoc.TactileSensor(port="COM3")
SensorNum = 1
SensorAddress = np.array([8, 9]) # if SensorNum == 1, second address will be ignored.
slopeCompensateOn = True

MODE_ONE_PAD = 0    # measures pressure, "super fast" (~kHz), 1 channel
MODE_FOUR_PAD = 1   # measures linear in 4 directions
MODE_NINE_PAD = 2   # measures pressure across "nibs" (each is a cluster of 4 pips)
MODE_INDIVIDUAL = 3 # measures each electrode (hardwired across 4 pips)
MODE_LINEAR_ROTATIONAL = 4  # measures linear (up/down) and rotational (cw/ccw)
MODE_CAPSENSE = 17
MODE_ONE_AND_FOUR =32
MODE_NINE_AND_INDIV= 33
MODE_ONE_AND_NINE = 34
MODE_FOUR_AND_INDIV = 35
MODE_TOR_NS_AND_INDIV = 36

sensingMode = MODE_INDIVIDUAL
[num_sensors_1, num_sensors_2, SamplingFreq] = get_sensor_deets(sensingMode,SensorNum)
#===============================================================================

#===============================================================================
print(sensingMode)
ts.sendChar("s")

plt.ion() # turn on interactive mode
plt.rcParams["font.family"] = "Times New Roman"
plt.style.use('classic')
fig, ax = plt.subplots(nrows=3, ncols=1) # create a figure and an axis
tempSampledData_hist = np.array([])
target_all = np.array([])
sense_current_all = np.array([])
sense_output_all = np.array([])
sense_target = 160000

# Initialize the previous error and time step
pd_controller.prev_error = 0
pd_controller.dt = 0.01 # You can adjust this value as needed
pd_controller.integral = 0

tic = time.time()

tempSampledData = ts.readRestData()
tempSampledData = ts.readRestData()
tempSampledData = ts.readRestData()
while 1:
    while ord(ts.ser.read(1))!= ts.STX:
        print('pop')
        continue
    tempSampledData = ts.readRestData()
    tempSampledData = convert_ind2PartialPressure(tempSampledData)
    tempSampledData_hist = np.append(tempSampledData_hist, [tempSampledData], axis=0) if tempSampledData_hist.size else np.array([tempSampledData])
    if len(tempSampledData_hist) > 300:
        tempSampledData_hist = tempSampledData_hist[len(tempSampledData_hist)-300:]
    n_cols = np.shape(tempSampledData_hist)[1]
    
    if len(tempSampledData_hist)<6:
        sense_current = np.mean(tempSampledData_hist[:,0])
    else:
        sense_current = np.mean(tempSampledData_hist[len(tempSampledData_hist)-6:,0])
    sense_current_all = np.append(sense_current_all,sense_current)
    sense_output = pd_controller(sense_target,sense_current)
    sense_output_all = np.append(sense_output_all,sense_output)
    target_imp = sense2imp(sense_output)
    target_all = np.append(target_all, target_imp)
    if len(target_all) > 300:
        target_all = target_all[len(target_all)-300:]
    if len(sense_current_all) > 300:
        sense_current_all = sense_current_all[len(sense_current_all)-300:]
    if len(sense_output_all) > 300:
        sense_output_all = sense_output_all[len(sense_output_all)-300:]
    
    
    if time.time() - tic > 2:
        visualizer.update_all(tempSampledData)  # /np.max(forimage))
        visualizer.redraw()

        ax[0].clear() # clear the previous plot
        ax[1].clear() # clear the previous plot
        ax[2].clear() # clear the previous plot
        
        ax[0].plot(tempSampledData_hist[:, 0])
        ax[0].set_title("Pressure")
        #for i in range(1,n_cols):
        #    ax[1].plot(tempSampledData_hist[:, i], label=f"Sensor Pressure {i+1}")
        #ax[1].set_title("Partial Pain")
        #ax[1].legend()
        
        ax[1].plot(sense_current_all) #shows an average of the sensing
        ax[1].set_title("sense_current")
        #ax[2].plot(sense_output_all)
        #ax[2].set_title("sense_output")
        ax[2].plot(target_all)
        ax[2].set_title("Target impedance")
        
        plt.draw() # update the figure
        plt.pause(0.01) # pause for a short time
        tic = time.time()
    if keyboard.is_pressed('q'): break # break the loop if 'q' is pressed

plt.close()
#-----------------------------------------------------
ts.ser.flushInput()
ts.sendChar("i") #End the communication.
ts.closePort()
time.sleep(1)