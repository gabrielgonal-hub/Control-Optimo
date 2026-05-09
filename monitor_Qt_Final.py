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
import matplotlib.pyplot as plt
import platform
import subprocess

from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QGridLayout,
    QLabel,
    QPushButton,
    QLineEdit,
    QVBoxLayout,
    QHBoxLayout,
    QCheckBox,
    QMessageBox
)
from PySide6.QtCore import QTimer, Qt
import pyqtgraph as pg

# ================= CONFIG =================
def load_config():

    if getattr(sys, 'frozen', False):
        base_dir = os.path.dirname(sys.executable)
    else:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    
    config_dir = os.path.join(base_dir, "config")
    config_path = os.path.join(config_dir, "config.ini")

    if not os.path.exists(config_path):
        return None

    try:
        df = pd.read_csv(config_path)

        config = {
            "port": df["port"][0],
            "baud": int(df["baud"][0]),
            "plotLength": int(df["plotLength"][0]),
            "interval": int(df["interval"][0]),
            "ymin": float(df["ymin"][0]),
            "ymax": float(df["ymax"][0]),
            "saveCSV": str(df["saveCSV"][0]).lower() in ["1", "true", "yes"]
        }

        QMessageBox.information(
            None,
            "Configuración",
            "Configuración cargada desde config.ini"
        )

        return config

    except Exception as e:

        QMessageBox.critical(
            None,
            "Error",
            f"Error leyendo config.ini:\n\n{e}"
        )

        return None

def save_config(port, baud, plotLength, interval, ymin, ymax, saveCSV):

    if getattr(sys, 'frozen', False):
        base_dir = os.path.dirname(sys.executable)
    else:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    
    config_dir = os.path.join(base_dir, "config")
    os.makedirs(config_dir, exist_ok=True)
    
    config_path = os.path.join(config_dir, "config.ini")

    df = pd.DataFrame([{
        "port": port,
        "baud": baud,
        "plotLength": plotLength,
        "interval": interval,
        "ymin": ymin,
        "ymax": ymax,
        "saveCSV": saveCSV
    }])

    df.to_csv(config_path, index=False)

    QMessageBox.information(
        None,
        "Configuración guardada",
        f"Configuración guardada en:\n\n{config_path}"
    )

def clear(): 
    if os.name == "posix":
        os.system ("clear")
    elif os.name in ("ce", "nt", "dos"):
        os.system ("cls")

# ================= SERIAL =================
class serialPlot:
    def __init__(self, port, baud, plotLength):

        self.plotMaxLength = plotLength

        self.rawData = bytearray(56)  # NUEVO

        self.buffer = bytearray()

        # [R,X1,Y,U] x 4 = 16 buffers
        self.data = [collections.deque([0]*plotLength, maxlen=plotLength) for _ in range(16)]

        self.packetQueue = collections.deque()
        self.csvPackets = []

        self.isRun = True
        self.thread = None

        self.serialConnection = serial.Serial(port, baud, timeout=4)

    def readSerialStart(self):
        self.thread = Thread(target=self.backgroundThread)
        self.thread.daemon = True
        self.thread.start()

    def backgroundThread(self):
        time.sleep(1)
        self.serialConnection.reset_input_buffer()
    
        while self.isRun:

            try:
        
                data = self.serialConnection.read(
                    self.serialConnection.in_waiting or 1
                )
        
                if data:
                    self.buffer.extend(data)
        
            except serial.SerialException as e:
        
                print("\nError serial:")
                print(e)
        
                self.isRun = False
                break
    
            #sincronización de paquetes
            while True:
                start = self.buffer.find(b'\xAA\x55')
    
                if start == -1:
                    self.buffer = self.buffer[-2:]
                    break
    
                if len(self.buffer) < start + 2 + 56:
                    break  # esperar más datos
    
                packet = self.buffer[start+2:start+2+56]
                self.buffer = self.buffer[start+2+56:]
    
                self.packetQueue.append(packet)
                self.csvPackets.append(packet)
            
    def close(self, saveCSV):
        self.isRun = False
        if self.thread is not None:
            self.thread.join(timeout=1)
        
        if self.serialConnection.is_open:
            self.serialConnection.close()

        if saveCSV:
            self.saveCSV()

    def saveCSV(self):

        if not self.csvPackets:
        
            QMessageBox.warning(
                None,
                "Sin datos",
                "No hay datos para guardar."
            )
        
            return
            
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

        # Ruta base compatible con .py y .exe
        if getattr(sys, 'frozen', False):
            base_dir = os.path.dirname(sys.executable)
        else:
            base_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Carpeta /data
        data_dir = os.path.join(base_dir, "data")
        os.makedirs(data_dir, exist_ok=True)
        
        filename = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Ruta completa del CSV
        path = os.path.join(data_dir, f"datos_{filename}.csv")

        df.to_csv(path, index=False)

        QMessageBox.information(
            None,
            "CSV guardado",
            f"CSV guardado en:\n\n{path}"
        )

        resultados, nombres = self.calcular_J(df)
        self.generar_reporte(df, resultados, nombres, filename, path)

    def calcular_J(self, df):

        dt = df['t'].diff().fillna(0)
    
        resultados = []
    
        for i in range(1,5):
            x1 = df[f'X1_{i}']
            x2 = df[f'Y{i}']
            u  = df[f'U{i}']
    
            J = ((x1**2 + 5*x2**2 + u**2) * dt).sum()
            resultados.append(J)
    
        nombres = ["AdP","Int-AdP","LQR","LQI"]
    
        return resultados, nombres

    def generar_reporte(self, df, resultados, nombres, filename, path):

        t = df['t']
        R = df['R']
    
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        axs = axs.flatten()
    
        for i in range(4):
            ax = axs[i]
        
            Y = df[f'Y{i+1}']
            U = df[f'U{i+1}']
        
            ax.plot(t, R, 'r', label='Referencia')
            ax.plot(t, Y, 'k', label='Salida')
            ax.plot(t, U, 'b--', label='Control')
        
            ax.set_title(nombres[i])
            ax.grid()
            ax.legend()
    
        # Mejor índice
        best = resultados.index(min(resultados))
        
        # Título general
        fig.suptitle(filename, fontsize=14)
        
        # === NUEVO BLOQUE DE TEXTO ===
        
        # Título del índice
        fig.text(0.5, 0.14, "Índice de desempeño", ha='center', fontsize=12, weight='bold')
        
        # Fórmula (LaTeX)
        fig.text(0.5, 0.11, r"$J = \int (x^T Q x + u^T R u)\, dt$", ha='center', fontsize=11)
        
        # Nombres y valores alineados por columnas
        for i in range(4):
            fig.text(0.2 + i*0.2, 0.08, nombres[i], ha='center', fontsize=10)
            fig.text(0.2 + i*0.2, 0.05, f"J = {resultados[i]:.2f}", ha='center', fontsize=10)
        
        # Mejor controlador
        mejor_texto = f"MEJOR: {nombres[best]}"
        fig.text(0.5, 0.02, mejor_texto, ha='center', fontsize=12, weight='bold')
    
        plt.tight_layout(rect=[0, 0.16, 1, 0.95])
    
        # Guardar imagen
        img_path = path.replace(".csv", ".png")
        plt.savefig(img_path, dpi=300)
        
        manager = plt.get_current_fig_manager()

        try:
            manager.window.showMaximized()
        except:
            pass
        
        plt.show(block=False)
        plt.pause(0.1)
        
        QMessageBox.information(
            None,
            "Reporte generado",
            f"Imagen guardada en:\n\n{img_path}"
        )

# ================= SETUP WINDOW =================
class SetupWindow(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Monitor Serial v2.1 - FIME UANL")
        self.setMinimumWidth(350)

        layout = QVBoxLayout()

        title = QLabel(
            "Monitor de Puerto Serial\n"
            "v2.1 - 7 de mayo del 2026\n"
            "Departamento de Electrónica y Automatización\n"
            "FIME - UANL"
        )

        title.setStyleSheet("""
            font-size: 14pt;
            font-weight: bold;
        """)
        
        title.setAlignment(Qt.AlignCenter)
        
        layout.addWidget(title)

        # ===== Inputs =====

        self.portEdit = QLineEdit("COM4")
        self.baudEdit = QLineEdit("115200")
        self.samplesEdit = QLineEdit("50")
        self.intervalEdit = QLineEdit("50")
        self.yminEdit = QLineEdit("-0.1")
        self.ymaxEdit = QLineEdit("3.3")

        layout.addWidget(QLabel("Puerto"))
        layout.addWidget(self.portEdit)

        layout.addWidget(QLabel("Baudrate"))
        layout.addWidget(self.baudEdit)

        layout.addWidget(QLabel("Samples"))
        layout.addWidget(self.samplesEdit)

        layout.addWidget(QLabel("Intervalo (ms)"))
        layout.addWidget(self.intervalEdit)

        layout.addWidget(QLabel("Y min"))
        layout.addWidget(self.yminEdit)

        layout.addWidget(QLabel("Y max"))
        layout.addWidget(self.ymaxEdit)

        # ===== Checkboxes =====

        self.saveCSVCheck = QCheckBox("Guardar CSV")
        self.saveConfigCheck = QCheckBox("Guardar configuración")
        self.useConfigCheck = QCheckBox("Usar configuración guardada")

        layout.addWidget(self.saveCSVCheck)
        layout.addWidget(self.saveConfigCheck)
        layout.addWidget(self.useConfigCheck)

        # ===== Botón =====

        self.startButton = QPushButton("INICIAR")
        self.startButton.clicked.connect(self.startMonitor)

        layout.addWidget(self.startButton)

        self.setLayout(layout)

        # ===== Cargar config automáticamente =====

        self.config = load_config()

        if self.config:
            self.useConfigCheck.setChecked(True)
            self.saveCSVCheck.setChecked(self.config["saveCSV"])
            self.portEdit.setText(str(self.config["port"]))
            self.baudEdit.setText(str(self.config["baud"]))
            self.samplesEdit.setText(str(self.config["plotLength"]))
            self.intervalEdit.setText(str(self.config["interval"]))
            self.yminEdit.setText(str(self.config["ymin"]))
            self.ymaxEdit.setText(str(self.config["ymax"]))

    def startMonitor(self):

        try:

            # ===== Config guardada =====

            if self.useConfigCheck.isChecked() and self.config:

                port = self.config["port"]
                baud = self.config["baud"]
                plotLength = self.config["plotLength"]
                interval = self.config["interval"]
                ymin = self.config["ymin"]
                ymax = self.config["ymax"]

            else:

                port = self.portEdit.text()
                baud = int(self.baudEdit.text())
                plotLength = int(self.samplesEdit.text())
                interval = int(self.intervalEdit.text())
                ymin = float(self.yminEdit.text())
                ymax = float(self.ymaxEdit.text())

            saveCSV = self.saveCSVCheck.isChecked()

            # ===== Guardar config =====

            if self.saveConfigCheck.isChecked():

                save_config(
                    port,
                    baud,
                    plotLength,
                    interval,
                    ymin,
                    ymax,
                    saveCSV
                )

            # ===== Serial =====

            self.serialObj = serialPlot(
                port,
                baud,
                plotLength
            )

            self.serialObj.readSerialStart()

            # ===== Monitor =====

            self.monitor = Monitor(
                self.serialObj,
                interval,
                ymin,
                ymax,
                saveCSV
            )

            self.monitor.showMaximized()

            self.hide()

            # guardar para closeEvent
            self.saveCSV = saveCSV

        except Exception as e:

            QMessageBox.critical(
                self,
                "Error",
                str(e)
            )

# ================= GUI =================
class Monitor(QMainWindow):      
    def __init__(self, serialObj, interval, ymin, ymax, saveCSV):
        
        super().__init__()

        self.setWindowTitle("Monitor de Control")

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
            p.showGrid(x=True, y=True)
            p.addLegend()

            r = p.plot(
                pen=pg.mkPen('r', width=2),
                name='Referencia'
            )
            
            y = p.plot(
                pen=pg.mkPen('k', width=2),
                name='Salida'
            )
            
            u = p.plot(
                pen=pg.mkPen(color='b', width=2, style=Qt.DashLine),
                name='Control'
            )

            self.plots.append(p)
            self.curves.append((r,y,u))

            layout.addWidget(p, i//2, i%2)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(interval)
        self.saveCSV = saveCSV

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

    def closeEvent(self, event):

        try:
            self.s.close(self.saveCSV)
        except:
            pass
    
        event.accept()
        
# ================= MAIN =================
def main():

    app = QApplication(sys.argv)

    setup = SetupWindow()
    setup.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
