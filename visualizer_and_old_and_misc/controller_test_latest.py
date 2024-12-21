from controller_latest import ControllerInterface
import numpy as np
import time

if __name__ == '__main__':
    rlist = np.random.uniform(-10, 10, 100)
    #theta = -45

    motor = ControllerInterface(rod_len=100)
    time.sleep(2)
    try:
        for i in range(1000):
            time.sleep(0.1)
            motor.updateAngle(10)
        time.sleep(0.1)
        motor.stop()

        #motor.updateAngle(theta)

    except KeyboardInterrupt as e:
        print("stopping due to exception ", e)
        time.sleep(0.1)
        motor.stop()

    time.sleep(5)
    motor.stop()