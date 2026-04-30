#!/usr/bin/env python

import sys
import datetime
import os
from threading import Thread
import serial
import time
import collections
import struct
import copy
import pandas as pd

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QLabel
from PySide6.QtCore import QTimer, Qt
import pyqtgraph as pg

# ================= SERIAL =================
class serialPlot:
    def __init__(self, port, baud, plotLength):

        self.plotMaxLength = plotLength

        self.rawData = bytearray(56)  # NUEVO

        # [R,X1,Y,U] x 4 = 16 buffers
        self.data = [collections.deque([0]*plotLength, maxlen=plotLength) for _ in range(16)]

        self.packetQueue = collections.deque()
        self.csvPackets = []

        self.isRun = True
        self.thread = None

        self.serialConnection = serial.Serial(port, baud, timeout=4)

    def readSerialStart(self):
        self.thread = Thread(target=self.backgroundThread)
        self.thread.start()

    def backgroundThread(self):
        time.sleep(1)
        self.serialConnection.reset_input_buffer()

        while self.isRun:
            if self.serialConnection.in_waiting >= 56:
                self.serialConnection.readinto(self.rawData)
                packet = copy.deepcopy(self.rawData)
                self.packetQueue.append(packet)
                self.csvPackets.append(packet)

    def close(self, saveCSV):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()

        if saveCSV:
            self.saveCSV()

    def saveCSV(self):

        data = []

        for packet in self.csvPackets:
            values = []
            for i in range(14):
                value, = struct.unpack('f', packet[i*4:(i+1)*4])
                values.append(value)

            data.append(values)

        df = pd.DataFrame(data, columns=[
            't','R',
            'X1_1','Y1','U1',
            'X1_2','Y2','U2',
            'X1_3','Y3','U3',
            'X1_4','Y4','U4'
        ])

        os.makedirs("datos", exist_ok=True)
        filename = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.abspath(os.path.join("datos", f"datos_{filename}.csv"))

        df.to_csv(path, index=False)

        print("\nCSV guardado en:")
        print(path)

        self.calcular_J(df)

    def calcular_J(self, df):

        Q = [[1,0],[0,5]]
        R = 1

        dt = df['t'].diff().fillna(0)

        resultados = []

        for i in range(1,5):
            x1 = df[f'X1_{i}']
            x2 = df[f'Y{i}']
            u  = df[f'U{i}']

            J = ((x1**2 + 5*x2**2 + u**2) * dt).sum()

            resultados.append(J)

        nombres = ["AdP","Int-AdP","LQR","LQI"]

        print("\n===== COSTO J =====")
        for i in range(4):
            print(f"{nombres[i]}: {resultados[i]:.4f}")

        best = resultados.index(min(resultados))
        print(f"\nMEJOR SEGÚN J: {nombres[best]}")


# ================= GUI =================
class Monitor(QMainWindow):
    def __init__(self, serialObj, interval, ymin, ymax):
        super().__init__()

        self.s = serialObj

        central = QWidget()
        self.setCentralWidget(central)

        layout = QGridLayout()
        central.setLayout(layout)

        self.plots = []
        self.curves = []

        self.names = ["Control por Asignación de Polos", "Control Integral por Asignación de Polos", "Control LQR", "Control LQI"]
        for i in range(4):
            p = pg.PlotWidget(title=self.names[i])
            p.setYRange(ymin, ymax)
            p.setBackground('w')

            r = p.plot(pen='r')
            y = p.plot(pen='k')
            u = p.plot(pen='b')

            self.plots.append(p)
            self.curves.append((r,y,u))

            layout.addWidget(p, i//2, i%2)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(interval)

    def update(self):

        if len(self.s.packetQueue) == 0:
            return

        packet = self.s.packetQueue[-1]
        self.s.packetQueue.clear()

        values = [struct.unpack('f', packet[i*4:(i+1)*4])[0] for i in range(14)]

        R = values[1]

        idx = 2

        for i in range(4):
            X1 = values[idx]
            Y  = values[idx+1]
            U  = values[idx+2]
            idx += 3

            self.s.data[i*4].append(R)
            self.s.data[i*4+1].append(X1)
            self.s.data[i*4+2].append(Y)
            self.s.data[i*4+3].append(U)

            r,y,u = self.curves[i]

            r.setData(list(self.s.data[i*4]))
            y.setData(list(self.s.data[i*4+2]))
            u.setData(list(self.s.data[i*4+3]))


# ================= MAIN =================
def main():

    port = input("Puerto: ")
    baud = int(input("Baudrate: "))
    plotLength = int(input("Samples: "))
    interval = int(input("Intervalo (ms): "))
    ymin = float(input("Y min: "))
    ymax = float(input("Y max: "))
    saveCSV = int(input("Guardar CSV? (1/0): "))

    s = serialPlot(port, baud, plotLength)
    s.readSerialStart()

    app = QApplication(sys.argv)
    win = Monitor(s, interval, ymin, ymax)
    win.show()

    app.exec()

    s.close(saveCSV)

if __name__ == "__main__":
    main()