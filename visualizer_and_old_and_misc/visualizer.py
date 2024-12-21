import matplotlib.pyplot as plt
import numpy as np

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


class vectorFieldPixelVisualizer():
    def __init__(self, height=6, width=6, cmap='grey', vmin=0, vmax=1):
        self.height = height
        self.width = width
        self.image = np.zeros((height, width))
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.canvas = self.ax.imshow(self.image, interpolation="nearest", cmap=cmap, vmin=vmin, vmax=vmax)
        plt.xticks(np.arange(0.0, 2.5, 1), np.arange(0.5, 2, 0.5))
        plt.yticks(np.arange(2, -0.5, -1), np.arange(0.5, 2, 0.5))
        self.fig.canvas.draw()

    def update_all(self, sensdata):
        self.image = self.convert_to_img(sensdata)
        self.image = np.reshape(self.image, (self.height, self.width))
        self.canvas.set_data(self.image)

    def convert_to_img(self, sensdata):
        return sensdata

    def redraw(self):
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()


class vectorFieldPixelVisualizerBoth():
    def __init__(self, cmap='grey', vmin=0, vmax=1):
        self.visu1 = vectorFieldPixelVisualizer(6, 6, cmap, vmin, vmax)
        self.visu2 = vectorFieldPixelVisualizer(3, 3, cmap, vmin, vmax)

    def update_all(self, sensdata):
        self.image = self.convert_to_img(sensdata)
        self.image = np.reshape(self.image, (6, 6))
        wbtt0 = sensdata.sum(axis=0)  # Pressure at ALL
        wbtt1 = sensdata.flatten()[[0, 1, 2, 3]].sum(axis=0)  # Pressure at 1
        wbtt2 = sensdata.flatten()[[4, 5, 6, 7]].sum(axis=0)  # Pressure at 2
        wbtt3 = sensdata.flatten()[[8, 9, 10, 11]].sum(axis=0)  # Pressure at 3
        wbtt4 = sensdata.flatten()[[12, 13, 14, 15]].sum(axis=0)  # Pressure at 4
        wbtt5 = sensdata.flatten()[[16, 17, 18, 19]].sum(axis=0)  # Pressure at 5
        wbtt6 = sensdata.flatten()[[20, 21, 22, 23]].sum(axis=0)  # Pressure at 6
        wbtt7 = sensdata.flatten()[[24, 25, 26, 27]].sum(axis=0)  # Pressure at 7
        wbtt8 = sensdata.flatten()[[28, 29, 30, 31]].sum(axis=0)  # Pressure at 8
        wbtt9 = sensdata.flatten()[[32, 33, 34, 35]].sum(axis=0)  # Pressure at 9
        self.image2 = np.array([wbtt1, wbtt2, wbtt3, wbtt4, wbtt5, wbtt6, wbtt7, wbtt8, wbtt9])
        self.image2 /= np.max(self.image2)
        self.visu1.update_all(self.image)
        self.visu2.update_all(self.image2)

    def convert_to_img(self, sensdata):
        return sensdata

    def redraw(self):
        self.visu1.redraw()
        self.visu2.redraw()

# plt.ion()
# image = np.zeros((6, 6))
# visu = vectorFieldPixelVisualizer()
# visu.redraw()
# for i in range(100):
#     image[2, 2] = i
#     image[3, 3] = i
#     visu.update_all(image)
#     visu.redraw()
