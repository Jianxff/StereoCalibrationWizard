import os
import matlab
import matlab.engine
eng = None

def f_list():
    os.system('cd bin && StereoCalibrationWizard.exe list')

def f_init():
    res = os.system('cd bin && StereoCalibrationWizard.exe init')
    if res == 0:
        f_matlab()

def f_next():
    res = os.system('cd bin && StereoCalibrationWizard.exe next')
    if res == 0:
        f_matlab()

def f_free():
    os.system('cd bin && StereoCalibrationWizard.exe free')

def f_measure(mode,rt):
    os.system('cd bin && StereoCalibrationWizard.exe measure ' + str(mode) + (' rt' if rt else ''))

def f_matlab_init():
    global eng
    eng = matlab.engine.start_matlab()
    eng.cd(os.getcwd() + './matlab', nargout = 0)

def f_matlab():
    global eng
    eng.main(nargout = 0)