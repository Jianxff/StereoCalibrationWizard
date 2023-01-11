from python.storage import *
from python.impl import *
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtWidgets import QRadioButton, QPushButton, QCheckBox, QLabel
mode = 'elas'
rt = False

def set_elas():
    global mode
    mode = 'elas'

def set_sgbm():
    global mode
    mode = 'sgbm'

def set_rt(set):
    global rt
    rt = bool(set)

def show_gui():
    app = QApplication([])
    window = QWidget()
    window.setWindowTitle("Wizard")
    window.resize(425,630)
    tool_label = QLabel("ToolBox ____________________",window)
    tool_label.move(40,20)
    list_btn = QPushButton("List",window)
    list_btn.setGeometry(40,60,160,50)
    list_btn.clicked.connect(f_list)

    plot_btn = QPushButton("Plot",window)
    plot_btn.setGeometry(220,60,160,50)
    
    clear_btn = QPushButton("Clear",window)
    clear_btn.setGeometry(40,130,160,50)
    clear_btn.clicked.connect(clear_storage)

    save_btn = QPushButton("Save",window)
    save_btn.setGeometry(220,130,160,50)
    save_btn.clicked.connect(store_data)

    calib_label = QLabel("Calibration ________________",window)
    calib_label.move(40,200)

    init_btn = QPushButton("Init",window)
    init_btn.setGeometry(40,240,100,50)
    init_btn.clicked.connect(f_init)

    next_btn = QPushButton("Next", window)
    next_btn.setGeometry(160,240,100,50)
    next_btn.clicked.connect(f_next)

    free_btn = QPushButton("Free",window)
    free_btn.setGeometry(280,240,100,50)
    free_btn.clicked.connect(f_free)

    matlab_label = QLabel("Matlab _____________________",window)
    matlab_label.move(40,310)
    matlab_btn = QPushButton("Run Matlab",window)
    matlab_btn.setGeometry(40,350,340,50)
    matlab_btn.clicked.connect(f_matlab)

    measure_label = QLabel("Measure ____________________", window)
    measure_label.move(40,420)
    measure_btn = QPushButton("Measurement",window)
    measure_btn.setGeometry(40,460,340,50)
    global mode
    global rt
    measure_btn.clicked.connect(lambda:f_measure(mode,rt))

    r_btn1 = QRadioButton("ELAS", window)
    r_btn1.move(50, 530)    
    r_btn1.setChecked(True)
    r_btn1.toggled.connect(lambda x : set_elas() if x else None)

    r_btn2 = QRadioButton("SGBM", window)
    r_btn2.move(50, 570)
    r_btn2.toggled.connect(lambda x : set_sgbm() if x else None)

    c_btn = QCheckBox("RealTime",window)
    c_btn.move(230,530)
    c_btn.stateChanged.connect(lambda x : set_rt(x))
    
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
    print(msg)

    func()

    app.exit()

