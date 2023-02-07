import os
import matlab
import matlab.engine
from python.plot import f_plot
from python.log import logging
eng = None

def f_list():
    os.system('cd bin && StereoCalibrationWizard.exe -f list')

def f_init():
    res = os.system('cd bin && StereoCalibrationWizard.exe -f init')
    logging.info('initialize return [%d]',res)
    if res == 0:
        f_matlab()

def f_next():
    res = os.system('cd bin && StereoCalibrationWizard.exe -f next')
    logging.info('next capture return [%d]',res)
    if res == 0:
        f_plot(show=False)
        f_matlab()

def f_free():
    res = os.system('cd bin && StereoCalibrationWizard.exe -f free')
    logging.info('free capture return [%d]',res)
    f_plot()


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