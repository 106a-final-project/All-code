import serial
import time
import os
from multiprocessing import Process, Lock
import numpy as np

mainLock = Lock()

class Controller:
    def __init__(self, port="COM5", rod_len=67.5):
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


    def toPT(self, len):
        return int(len / 0.0026)

    def updateAngle(self, angle):
        self.updatePos(np.sin(angle) * self.rod_len)

    def updatePos(self, dist):
        self.sendGo(self.toPT(dist))

    def sendGo(self, rposPT):
        with mainLock:
            self.motorPort.write("RPA ".encode())
            time.sleep(0.02)
            print("getting current position ")
            response = int(self.motorPort.readline())
            print("Current Position = ", response)
            self.motorPort.write("MP ".encode())
            self.motorPort.write(self.accel.encode())
            self.motorPort.write(self.vel.encode())

            print("updating relative pos to ", rposPT)
            self.motorPort.write(f"PRT={rposPT} ".encode())
            self.motorPort.write("G ".encode())
            time.sleep(0.01)

    def mainloop(self):
        while True:
            with mainLock:
                self.motorPort.write(self.rbt)
                time.sleep(0.01)
                response = self.motorPort.readline().decode().strip()
                if self.die:
                    print("ending controller movement")
                    return
                if response == '0':
                    #print("hogging")
                    continue
                time.sleep(0.03)

    def stop(self):
        with mainLock:
            self.die = True
        self.thread.join()

    def run(self):
        self.sendGo(0)
        self.thread = Process(target=self.mainloop, args=())
        self.thread.start()


def separate_mainloop(self, port, lock, queue):
    self.motorPort = serial.Serial(
        port=port,
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.05
    )


    while True:
        self.motorPort.write(self.rbt)
        time.sleep(0.01)
        response = self.motorPort.readline().decode().strip()
        if self.die:
            print("ending controller movement")
            return
        if response == '0':
            #print("hogging")
            time.sleep(0.03)
            continue
        time.sleep(0.03)
