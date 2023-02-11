import os
import matlab
import matlab.engine
from python.plot import f_plot
from python.log import logging
eng = None

def f_select(function):
    res = os.system(f'cd bin && StereoCalibrationWizard.exe -f {function}')
    logging.info(f'{function} calibrate return [{res}]')
    if function == "next" or function == "free":
        f_plot()
    if function == "next" or function == "init":
        if res == 0:
            f_matlab() 

def f_list():
    os.system('cd bin && StereoCalibrationWizard.exe -f list')


def f_measure(mode,rt):
    res = os.system(f'cd bin && StereoCalibrationWizard.exe -f measure -m {mode}' + (' -rt' if rt else ''))
    logging.info('measurement return [%d]',res)

def f_matlab_init():
    global eng
    eng = matlab.engine.start_matlab()
    eng.cd(os.getcwd() + './matlab', nargout = 0)

def f_matlab():
    global eng
    eng.main(nargout = 0)