from bs4 import BeautifulSoup as BS
import matplotlib.pyplot as plt
from python.log import logging

def f_plot(show = True):
    plot_data(read_data(),show)

def read_data():
    try:
        with open('data/calib_data.xml','r') as f:
            doc = BS(f,features='xml').find('record')
            f.close()
    except Exception:
        logging.error('calib-data not found')
        return None

    try:
        f = open('data/images/count.txt','r')
    except Exception:
        f = None

    record = {}
    record['count'] = int(f.readline().strip()) if f != None else 0

    if doc == None:
        logging.critical('calib-data damaged or lost')
        return None
    
    record['std'] = list(map(float,doc.find('std').text.split()))
    record['rms'] = list(map(float,doc.find('rms').text.split()))
    record['epi'] = list(map(float,doc.find('epi').text.split())) if doc.find('epi') != None else None
    
    for key in ['primary','secondary']:
        node = doc.find(key)
        if node == None:
            record[key] = None
            continue
        dat = {}
        dat['F'] = list(map(float,node.find('F').text.split()))
        dat['K1'] = list(map(float,node.find('K1').text.split()))
        dat['K2'] = list(map(float,node.find('K2').text.split()))
        
        record[key] = dat

    return record


def plot_data(data = None,show = True):
    if data == None:
        return

    fig, axes = plt.subplots(figsize=(10,8),nrows=2,ncols=2)
    plt.subplots_adjust(wspace=0.5)

    x = range(data['count'] - len(data['rms']) + 1, data['count'] + 1)
    # rms/epi
    axes[0,0].set(title='mean/std - rms')
    axes[0,0].plot(x,data['rms'],label='rms',color='red')
    # if data['epi'] != None:
    #     ax = axes[0,0].twinx()
    #     ax.plot(x,data['epi'],label='epi',color='green',linestyle=':')
    ax = axes[0,0].twinx()
    ax.plot(x,data['std'],label='std',color='green')
    axes[0,0].legend(loc='best')

    for pic,key in [(axes[0,1],'F'),(axes[1,0],'K1'),(axes[1,1],'K2')]:
        pic.set(title='mean - '+key)
        pic.plot(x,data['primary'][key],label='main',color='blue')
        if data['secondary'] != None:
            pic.plot(x,data['secondary'][key],label='sec',color='orange')
        pic.legend(loc='best')
    
    plt.savefig('data/calib_record.jpg',dpi= 300)
    if show:
        plt.show()
    plt.close('all')