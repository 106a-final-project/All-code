from controller_latest import ControllerInterface
from collections import deque
import csv
import time
75
if __name__ == '__main__':
    motor = ControllerInterface(rod_len=280)
    with open('trajectory.csv', 'r') as file:
        reader = csv.reader(file)
        rows = list(reader)
    motor.updateAbsolute(0)
    time.sleep(3)
    initial_time = time.time() 
    for i in range(1, len(rows) - 1):
        cur_time, pos = map(lambda x: float(x), rows[i])
        cur_time += initial_time
        next_time, next_pos = map(lambda x: float(x), rows[i + 1])
        next_time += initial_time
        motor.updateAbsolute(-motor.toPT(next_pos))
        print("sleep ", next_time - time.time())
        time.sleep(abs(next_time - time.time()))

            