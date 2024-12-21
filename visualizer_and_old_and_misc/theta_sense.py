import numpy as np

class AngleSensor():
    def __init__(self):
        self.w = 0
        self.theta = 0
        self.center_pos = (-1, -1)
        self.prev_t = ...
