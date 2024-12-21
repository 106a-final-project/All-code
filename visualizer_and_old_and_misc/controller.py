import serial
import time
import os
from multiprocessing import Process, Pipe, Queue
import numpy as np


class ControllerInterface:
    def __init__(self, port="COM5", rod_len=67.5):
        self.queue = Queue()
        self.controller_process = Process(target=launch_controller, args=(self.queue, port, rod_len))
        self.updateAngle(0)
        self.controller_process.start()

    def updateAngle(self, angle):
        self.queue.put("u")
        self.queue.put(f"{angle}")

    def stop(self):
        self.queue.put("d")
        self.controller_process.join()

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
        self.rbt = [0x52, 0x42, 0x74, 0x20]  # command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20

    def toPT(self, len):
        return int(len / 0.0026)

    def sendGo(self, angle):
        rposPT = self.toPT(np.sin(angle * np.pi/180)*self.rod_len)
        self.motorPort.write("RPA ".encode())
        #print("getting current position ")
        response = int(self.motorPort.readline())
        #print("Current Position = ", response)
        self.motorPort.write("MP ".encode())
        self.motorPort.write(self.accel.encode())
        self.motorPort.write(self.vel.encode())
        #print("updating relative pos to ", rposPT)
        self.motorPort.write(f"PRT={rposPT} ".encode())
        self.motorPort.write("G ".encode())

    # "d" = die
    # "u", "angle" = update, angle in degrees in the next message
    def mainloop(self):
        msg = None
        while True:
            self.motorPort.write(self.rbt)
            time.sleep(0.01)
            response = self.motorPort.readline().decode().strip()
            while not self.queue.empty():
                msg = self.queue.get()
                if msg == "d":
                    return
                angle = float(self.queue.get())
            if msg == "u":
                msg = None
                self.sendGo(angle)
            if response == '0':
                # print("hogging")
                #time.sleep(0.03)
                pass
            time.sleep(0.03)

def launch_controller(queue, port="COM5", rod_len=67.5):
    controller = Controller(queue, port, rod_len)
    controller.mainloop()