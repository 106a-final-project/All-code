#!/usr/bin/env python

import serial
import serial.tools.list_ports
import struct
import numpy as np
from time import sleep, time

#from tactile_calibration_new import LoadCalibration, CalculateForce

''' searches for sensors as they show up in different systems '''
def findSensors():
    port_list = []
    search = ""

    # change the search term depending on OS:
    import platform
    os_name = platform.system()

    if os_name == "Darwin":
        search = "serial"
    elif os_name == "Windows":
        search = "COM"
    elif os_name == "Linux":
        search = "USB"
    else:
        raise Exception("Unsupported OS: " + os_name)

    for p in serial.tools.list_ports.grep(search):
        # print p[0] + " is available"
        port_list.append(p[0])

    if len(port_list) < 1:
        raise Exception("No sensors found!")

    return port_list

class TactileSensor():

    STX = 0x02
    ETX = 0x03

    # Commands to the sensors (simple, 1-byte transmissions)
    STREAM = 0x80       # Start streaming data from sensors
    SAMPLE = 0x81       # Send a single data sample
    IDLE = 0x82         # Idle mode
    STATUS_REQ = 0x83   # Report current status

    STATUS_TYPE = 0x17 # not sure why it's not 0x11
    # Data types
    DATA_TYPE = 0x10

    # Statuses
    INITIALIZING = 0x00
    IDLING = 0x01
    STREAMING = 0x02
    ERROR = 0x03
    
    num_byte_per_sensor = 2 #The default assuems 2byte data, but it can be changed.
    packet_size = 0 # Length after STX
    num_sensors = 0 # Number of sensor = (packet_size - 1) / 2
    unpackFormat = ''

    groupIndex = 0 # For Two mode scanning

    ''' Constructor loads calibration and opens serial communication '''
    def __init__(self, port):        
        self.port = port
        self.baud = 115200
        self.status = ""
#        self.runtime = runtime
        #self.data_init = [0.0] * 6
        self.data_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


        # set this to True to stop reading at any time
        self.stop = False

        try:
            self.ser = serial.Serial(port, baudrate=self.baud, timeout=1, writeTimeout=1)            
            print("Connected to Comport")

        except serial.SerialException:
            raise Exception("Unable to open serial port. Make sure the device is connected to {port}".format(port=port))
    
    def sendChar(self, cmd):
        txMsg = struct.pack('<B', ord(cmd))
        self.ser.write(txMsg)    
        
    def sendNum(self, inputNumpyArray):
        inputNumpyArray=np.uint8(inputNumpyArray)
        if inputNumpyArray.size<1:
            sys.exit("Wrong Input to SendNum")
        else:
            txMsg = struct.pack('<B', inputNumpyArray[0])
            for i in range(1,inputNumpyArray.size):
                txMsg += struct.pack('<B', inputNumpyArray[i])
            self.ser.write(txMsg)

        
        
    def readRestData(self):
        thisPacket = struct.unpack(self.unpackFormat, self.ser.read(self.packet_size) )
        if (thisPacket[self.packet_size-1] % 8) == self.ETX:
            
            self.groupIndex = (thisPacket[self.packet_size-1] - self.ETX) / 8 # This is particularly for Dynamic clustering paper
            
            processedPacket = np.zeros((1,int(self.num_sensors)))
                        
            #Pull the sensor data from the data stream
            for i in range(0,int(self.num_sensors)):
                for j in range(0,int(self.num_byte_per_sensor)):
                    processedPacket[0,i] += thisPacket[self.num_byte_per_sensor*i+j] * (2**8)**j #Assumes LSB to MSB
                                  
                      
            return processedPacket
            
        else:
            return 0
        
        
     
    ''' verifies communication, then reads streaming data
        and writes it to the output file '''
    def closePort(self):
        self.ser.close();
        

