from python.gui import show_gui, show_wait
from python.impl import f_matlab_init

if __name__ == '__main__':
    show_wait(f_matlab_init,'connecting to matlab ...')
    show_gui()
    