import csv
import visualizer
import time
import numpy as np
import matplotlib.pyplot as plt

plt.ion()
visu = visualizer.vectorFieldPixelVisualizer(3, 3)
all_data = np.zeros((3, 3))
cur_i = 0
with open('flfr.csv', 'r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        all_data[(cur_i//3)%3, cur_i%3] = float(row[0]) - float(row[1])
        cur_i += 1
        if (cur_i % 9) == 0:
            visu.update_all(abs(all_data) / (np.max(abs(all_data)) + 0.0001))
            visu.redraw()
            time.sleep(0.05)
