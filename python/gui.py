from python.storage import *
from python.impl import *
from python.plot import f_plot
from python.log import logging

import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtWidgets import QRadioButton, QPushButton, QCheckBox, QLabel, QComboBox
mode = 0
rt = False

def set_elas():
    global mode
    mode = 0

def set_sgbm():
    global mode
    mode = 1

def set_rt(set):
    global rt
    rt = bool(set)

def show_gui():
    app = QApplication([])
    window = QWidget()
    window.setWindowTitle("Wizard")
    window.resize(425,600)
    tool_label = QLabel("ToolBox ____________________",window)
    tool_label.move(40,20)
    list_btn = QPushButton("List",window)
    list_btn.setGeometry(40,60,160,50)
    list_btn.clicked.connect(f_list)

    plot_btn = QPushButton("Plot",window)
    plot_btn.setGeometry(220,60,160,50)
    plot_btn.clicked.connect(f_plot)
    
    clear_btn = QPushButton("Clear",window)
    clear_btn.setGeometry(40,130,160,50)
    clear_btn.clicked.connect(clear_storage)

    save_btn = QPushButton("Save",window)
    save_btn.setGeometry(220,130,160,50)
    save_btn.clicked.connect(store_data)

    calib_label = QLabel("Calibration ________________",window)
    calib_label.move(40,200)

    cbox = QComboBox(window)
    cbox.setGeometry(40,240,160,50)
    cbox.addItems(["   init","   next","   free","   input"])

    run_btn = QPushButton("Calibrate",window)
    run_btn.setGeometry(220,240,160,50)
    run_btn.clicked.connect(lambda x:f_select(cbox.currentText().strip()))

    matlab_label = QLabel("Matlab _____________________",window)
    matlab_label.move(40,310)
    matlab_btn = QPushButton("Run Matlab",window)
    matlab_btn.setGeometry(40,350,340,50)
    matlab_btn.clicked.connect(f_matlab)

    measure_label = QLabel("Measurement ________________", window)
    measure_label.move(40,420)
    measure_btn = QPushButton("Measure",window)
    measure_btn.setGeometry(40,460,160,50)
    global mode
    global rt
    measure_btn.clicked.connect(lambda:f_measure(mode,rt))

    c_btn = QCheckBox("RealTime",window)
    c_btn.move(230,475)
    c_btn.stateChanged.connect(lambda x : set_rt(x))

    r_btn1 = QRadioButton(" ELAS", window)
    r_btn1.move(50, 530)    
    r_btn1.setChecked(True)
    r_btn1.toggled.connect(lambda x : set_elas() if x else None)

    r_btn2 = QRadioButton(" SGBM", window)
    r_btn2.move(230, 530)
    r_btn2.toggled.connect(lambda x : set_sgbm() if x else None)

    # r_btn3 = QRadioButton("AD-Census", window)
    # r_btn3.move(220, 530)
    # # r_btn3.toggled.connect(lambda x : set_adcensus() if x else None) 

    
    
    window.show()
    app.exec_()


def show_wait(func,msg):
    app = QApplication([])
    window = QWidget()
    window.setWindowTitle("Wizard")
    window.resize(425,100)
    tool_label = QLabel(str(msg),window)
    tool_label.move(40,30)
    window.show()
    logging.info('%s',msg)

    func()
    
    app.quit()

