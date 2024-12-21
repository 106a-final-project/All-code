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

    def updateTorque(self, t):
        self.queue.put("t")
        self.queue.put(f"{t}")

    def stop(self):
        self.queue.put("d")
        self.controller_process.join()

class Controller:
    def __init__(self, queue, port="COM5", rod_len=67.5):
        self.queue = queue
        self.cnt = 0

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

        self.vel = "VT=10000000 "
        self.accel = "ADT=150 "
        self.rod_len = rod_len
        self.rbt = [0x52, 0x42, 0x74, 0x20]  # command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20

    def toPT(self, len):
        return int(len / 0.0026)

    def sendTorque(self, torque):
        print("MT GO ")
        self.motorPort.write("MT ".encode())
        self.motorPort.write(f"T={torque} ".encode())
        self.motorPort.write("G ".encode())

    def sendGo(self, angle):
        rposPT = self.toPT(np.sin(angle * np.pi/180)*self.rod_len)
        # self.motorPort.write("RPA ".encode())
        #print("getting current position ")
        # response = int(self.motorPort.readline())
        #print("Current Position = ", response)
        self.motorPort.write("MP ".encode())
        self.motorPort.write(self.accel.encode())
        self.motorPort.write(self.vel.encode())
        #print("updating relative pos to ", rposPT)
        self.motorPort.write(f"PRT={rposPT} ".encode())
        self.motorPort.write("G ".encode())

    def die(self):
        self.sendTorque(0)
        self.motorPort.close()

    # "d" = die
    # "u", "angle" = update, angle in degrees in the next message
    def mainloop(self):
        msg = None
        try:
            while True:
                self.motorPort.write("RPA ".encode())
                #s = time.time()
                response = int(self.motorPort.readline())

                if response > 100000 or response < -100000:
                    print("dying because of boundary ")
                    self.die()
                    return

                #print("responce took ", time.time() - s)
                while not self.queue.empty():
                    msg = self.queue.get()
                    if msg == "d":
                        print("dying due to msg")
                        self.die()
                        return
                    angle = self.queue.get()
                    if angle == "d":
                        print("dying due to angle")
                        self.die()
                        return
                    angle = float(angle)
                if msg == "u":
                    self.cnt += 1
                    msg = None
                    self.sendGo(angle)
                elif msg == "t":
                    self.cnt += 1
                    msg = None
                    self.sendTorque(angle)
                time.sleep(0.03)
        except KeyboardInterrupt as e:
            print("process 2 dying to keyboard")
            self.die()
            

def launch_controller(queue, port="COM5", rod_len=67.5):
    controller = Controller(queue, port, rod_len)
    controller.mainloop()
    print(controller.cnt)