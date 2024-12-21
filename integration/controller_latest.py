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

    def updateAbsolute(self, pt):
        self.queue.put("a")
        self.queue.put(f"{pt}")

    def toPT(self, len):
        return int(len / 0.0026)

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
        self.accel = "ADT=1200 "
        self.rod_len = rod_len
        self.rbt = [0x52, 0x42, 0x74, 0x20]  # command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20

    def toPT(self, len):
        return int(len / 0.0026)

    def sendTorque(self, torque):
        print("MT GO ")
        self.motorPort.write("MT ".encode())
        self.motorPort.write(f"T={torque} ".encode())
        self.motorPort.write("G ".encode())

    def sendGo(self, rposPT):
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

    def sendAbsoluteGo(self, abposPT):
        # self.motorPort.write("RPA ".encode())
        #print("getting current position ")
        # response = int(self.motorPort.readline())
        #print("Current Position = ", response)
        self.motorPort.write("MP ".encode())
        self.motorPort.write(self.accel.encode())
        self.motorPort.write(self.vel.encode())
        #print("updating relative pos to ", rposPT)
        self.motorPort.write(f"PT={abposPT} ".encode())
        self.motorPort.write("G ".encode())

    def die(self):
        self.motorPort.write("MP ".encode())
        self.motorPort.write(self.accel.encode())
        self.motorPort.write(self.vel.encode())
        self.motorPort.write(f"PT={0} ".encode())
        self.motorPort.write("G ".encode())
        message = [0x52, 0x42, 0x74, 0x20]			#command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20
        self.motorPort.write(message)
        while True:
            response = self.motorPort.readline().decode().strip()  # Decode bytes to string and strip whitespace/newline characters
            if response == '0':  # Check if the decoded and stripped string is '0'
                break  # Exit the loop if the motor is stopped
            self.motorPort.write(message)  # If not '0', send the command again to poll the motor status
            time.sleep(0.05)  # It's often good practice to have a small delay in such loops to avoid overwhelming the motor controller
        self.sendTorque(0)
        self.motorPort.close()

    # "d" = die
    # "u", "dist" = update, dist in mm in the next message
    def mainloop(self):
        msg = None
        try:
            while True:
                # self.motorPort.write("RPA ".encode())
                # #s = time.time()
                # response = int(self.motorPort.readline())
                #
                # if response > 110000 or response < -170000:
                #     print("dying because of boundary ")
                #     self.die()
                #     return
                # if response == -150000:
                #     self.die()
                #     return

                #print("responce took ", time.time() - s)
                while not self.queue.empty():
                    msg = self.queue.get()
                    if msg == "d":
                        print("dying due to msg")
                        self.die()
                        return
                    val = self.queue.get()
                    if val == "d":
                        print("dying due to angle")
                        self.die()
                        return
                    val = float(val)
                if msg == "a":
                    self.cnt += 1
                    msg = None
                    self.sendAbsoluteGo(int(val))
                elif msg == "u":
                    self.cnt += 1
                    msg = None
                    self.sendGo(self.toPT(val))
                elif msg == "t":
                    self.cnt += 1
                    msg = None
                    self.sendTorque(val)
                #time.sleep(0.03)
        except KeyboardInterrupt as e:
            print("process 2 dying to keyboard")
            self.die()
            

    def mainloop_oscillate(self):

        left_fin = -70000
        left_bound = -60000
        right_bound = -10000


        cur = left_fin
        self.sendAbsoluteGo(cur)
        cur = 0 
        start = time.time()
        try:
            while True:
                self.motorPort.write("RPA ".encode())
                #s = time.time()
                response = int(self.motorPort.readline())
                print(response)
                if response > right_bound:
                    print("time taken ", time.time() - start)
                    start = time.time()
                    self.sendAbsoluteGo(left_fin)
                    cur = left_fin
                elif response < left_bound:
                    print("time taken ", time.time() - start)
                    start = time.time()
                    self.sendAbsoluteGo(0)
                    cur = 0

                while not self.queue.empty():
                    msg = self.queue.get()
                    if msg == "d":
                        self.die()
                        return
                    
                time.sleep(0.03)
        except KeyboardInterrupt as e:
            print("process 2 dying to keyboard")
            self.die()                 

def launch_controller(queue, port="COM5", rod_len=67.5):
    controller = Controller(queue, port, rod_len)
    controller.mainloop()
    print(controller.cnt)

def launch_oscillator(queue, port="COM5", rod_len=67.5):
    controller = Controller(queue, port, rod_len)
    controller.mainloop_oscillate()
    print(controller.cnt)