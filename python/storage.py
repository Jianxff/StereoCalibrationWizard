import os
import shutil
import time

def clear_storage():
    path = ['data/images','data/output', 'data/next']
    for p in path:
        if os.path.exists(p):
            shutil.rmtree(p)
        os.makedirs(p)
    print('finish')

def store_data():
    target = os.path.abspath('scripts/history/' + time.strftime('%Y%m%d%H%M%S',time.localtime(time.time())))
    if os.path.exists(target):
        shutil.rmtree(target)
    shutil.copytree(os.path.abspath('data'), target)
    print('finish')