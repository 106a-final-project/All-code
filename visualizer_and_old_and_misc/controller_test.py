from controller import ControllerInterface
import numpy as np
import time

if __name__ == '__main__':
    rlist = np.random.uniform(-10, 10, 100)

    motor = ControllerInterface()
    try:
        for angle in rlist:
            motor.updateAngle(angle * np.pi/180)

    except Exception as e:
        print("stopping due to exception ", e)
        motor.stop()

    motor.stop()