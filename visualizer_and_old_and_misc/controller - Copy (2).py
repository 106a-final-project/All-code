import serial
import time
import os
from multiprocessing import Process, Pipe, Queue
import numpy as np


class ControllerInterface:
    def __init__(self, port="COM5", rod_len=67.5):
        self.queue = Queue()
        self.controller = Controller(self.queue, port, rod_len)

class Controller:
    def __init__(self, queue, port="COM5", rod_len=67.5):
        self.queue = queue

        self.port = port
        ## Create & Configure Port ##
        self.motorPort = serial.Serial(
            port=port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.05  # timeout is required, no timeout will cause hang up
        )  # timeout is in terms of seconds, and will be the polling speed

        ## Open Port ##
        print("Opening port...", end='', flush=True)
        if self.motorPort.isOpen():
            self.motorPort.close()

        self.motorPort.open()
        print("complete.")

        ## turn echo feature off for easier reporting, method a ##
        self.motorPort.write("ECHO_OFF ".encode())
        time.sleep(1)
        self.motorPort.reset_input_buffer()

        ## Disable limits & Clear Errors, method a ##			#THIS IS REQUIRED FOR MOTION W/O LIMIT SWITCHES, SEE DEV GUIDE
        self.motorPort.write("EIGN(2) ".encode())  # disable positive limit switch
        self.motorPort.write("EIGN(2) ".encode())  # disable negative limit switch
        self.motorPort.write("ZS ".encode())  # clear errors

        print("starting position move...", end='', flush=True)
        self.motorPort.reset_input_buffer()
        message = [0x52, 0x50, 0x41, 0x20]  # command sent is RPA, R=0x52, P=0x50, A=0x41, ' '=0x20
        self.motorPort.write(message)
        self.home = int(self.motorPort.readline())

        self.vel = "VT=1000 "
        self.accel = "ADT=10"
        self.rod_len = rod_len
        self.die = False

        self.rbt = [0x52, 0x42, 0x74, 0x20]  # command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20


def separate_mainloop(port, queue):
    motorPort = serial.Serial(
        port=port,
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.05
    )
    rbt = [0x52, 0x42, 0x74, 0x20]
    while True:
        msg = queue.get(block=True)
        if msg == "c":
            pass
        else:
            print("logic fuckup 2 continue")
        while True:
            motorPort.write(rbt)
            time.sleep(0.01)
            response = motorPort.readline().decode().strip()
            if not queue.empty():
                msg = queue.get()
                if msg == "s":
                    queue.put("o")
                    break
                elif msg == "d":
                    break
                else:
                    print("logic fuckup 3 stop die")
            if response == '0':
                #print("hogging")
                time.sleep(0.03)
                continue
            time.sleep(0.03)
        if msg == "d":
            return