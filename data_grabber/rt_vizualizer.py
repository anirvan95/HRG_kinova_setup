import numpy as np
from pyqtgraph.Qt import QtGui
import pyqtgraph as pg
import platform
from collections import deque

class RT_Visualiser:

    def __init__(self):
        # create QT Application
        self.app = QtGui.QApplication([])

        # configure window
        self.win = pg.GraphicsLayoutWidget(show=True, )
        self.win.resize(720, 480)
        self.win.setWindowTitle('Visualisation')
        self.win.setBackground('w')
        # enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        # create plots and curves for forces
        self.pl_fx = self.win.addPlot(title="Packet Number")
        self.pl_fy = self.win.addPlot(title="Acc (X)")
        #self.pl_fz = self.win.addPlot(title="Acc (Z)")

        # Add plotters
        self.fx = self.pl_fx.plot(pen='r')
        self.fy = self.pl_fy.plot(pen='r')
        #self.fz = self.pl_fz.plot(pen='r')

        # buffers for plotted data
        self.b_fx = deque(maxlen=30)
        self.b_fy = deque(maxlen=1000)
        #self.b_fz = deque(maxlen=10000)

    def reset(self):
        self.win.close()

    def update_plot(self, data):
        data = list(data)
        # store new data
        self.b_fx.extend([data[0]])
        self.b_fy.extend(data[1:])
        #self.b_fz.extend(data[1:]*0)

        # plot new data
        self.fx.setData(self.b_fx)
        self.fy.setData(self.b_fy)
        #self.fz.setData(self.b_fz)

        # on macOS, calling processEvents() is unnecessary
        # and even results in an error. only do so on Linux and Linux
        if platform.system() == 'Linux' or platform.system() == 'Windows':
            self.app.processEvents()