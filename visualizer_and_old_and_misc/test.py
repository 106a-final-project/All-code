import matplotlib.pyplot as plt
import numpy as np
import random
import time

class vectorFieldVisualiser():
    def __init__(self, nx, ny):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.arrows = [[self.ax.arrow(j, i, 0.5, 0.5) for j in range(nx)] for i in range(ny)]
        self.fig.canvas.draw()

    def update_point(self, x, y, dx, dy):
        self.arrows[y][x].remove()
        self.arrows[y][x] = self.ax.arrow(x, y, dx, dy)

    def update_all(self, arrdxdy):
        for i in range(len(self.arrows)):
            for j in range(len(self.arrows[0])):
                self.arrows[i][j].remove()
                self.arrows[i][j] = self.ax.arrow(j, i, arrdxdy[i][j][0], arrdxdy[i][j][1])

    def redraw(self):
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()

class vectorFieldVisualiserQuiver():
    def __init__(self, nx, ny, scale=1):
        self.xarr, self.yarr = np.meshgrid(np.arange(0, nx, 1), np.arange(0, ny, 1))
        self.dxarr = [[j for j in range(nx)] for i in range(ny)]
        self.dyarr = [[i for j in range(nx)] for i in range(ny)]

        self.scale = scale

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.quiver = self.ax.quiver(self.xarr, self.yarr, self.dxarr, self.dyarr)
        self.fig.canvas.draw()

    def update_point(self, x, y, dx, dy):
        self.dxarr[y][x] = dx * self.scale
        self.dyarr[y][x] = dy * self.scale
        self.quiver.set_UVC(self.dxarr, self.dyarr)

    def update_all(self, arrdxdy):
        self.dxarr = [[k[0] * self.scale for k in row] for row in arrdxdy]
        self.dyarr = [[k[1] * self.scale for k in row] for row in arrdxdy]
        self.quiver.set_UVC(self.dxarr, self.dyarr)

    def redraw(self):
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()

x = np.linspace(0, 6*np.pi, 100)
y = np.sin(x)

# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

dxdy = [[[i, j] for j in range(36)] for i in range(36)]
vecvis = vectorFieldVisualiserQuiver(36, 36, 40)

for phase in np.linspace(0, 10*np.pi, 500):
    t = time.time()
    for x in range(36):
        for y in range(36):
            dxdy[y][x] = [random.random(), random.random()]
    vecvis.update_all(dxdy)
    vecvis.redraw()
    print("time taken", t - time.time())

