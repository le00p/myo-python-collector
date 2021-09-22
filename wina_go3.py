import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import myo
import sys

class Lis(myo.DeviceListener):

    def __init__ (self, n):
        self.n= n
        self.emg= None
        self.pose= None

    def on_emg (self, event):
        self.emg= event.emg
        self.output()

    def output (self):
        outline=[]
        if self.emg:
            [outline.append(int(i)) for i in self.emg]
        return outline
        sys.stdout.flush()
    
    def on_pose (self, event):
        self.pose= event.pose
        event.device.stream_emg(True)
        self.emg_enabled= True
        self.emg= None
        self.output()

    def on_connected (self, event):
        print ('Device is connected\nWelcome {}'.format(event.device_name))
        event.device.request_battery_level()
        
    def on_battery_level (self, event):
        print("Your battery level is:", event.battery_level)

def gen(t=0):
    while hub.run(l.on_event, 5):
        out= []
        out=l.output()
        if len(out) == 0:
            yy= np.zeros(8)
        else:
            yy= l.output()
        t += 0.1
        yield t, float(yy[0]), float(yy[1]), float(yy[2]), float(yy[3]),\
                 float(yy[4]), float(yy[5]), float(yy[6]), float(yy[7])

def init():
    for i in range(0,4):
        for j in range(0,2):
            ax[i,j].set_ylim(-ymax, ymax)
            ax[i,j].set_xlim(0, xmax)

    del xdata[:]
    
    for i in range(1,9):
        del ymat['ydata'+str(i)][:]
        linemat['line'+str(i)].set_data(xdata, ymat['ydata'+str(i)])

    return linemat['line1'], linemat['line2'], linemat['line3'], linemat['line4'], \
           linemat['line5'], linemat['line6'], linemat['line7'], linemat['line8']

myo.init()
hub= myo.Hub()
l= Lis(1)

plt.rcParams['axes.grid'] = True
fig, ax= plt.subplots(4,2, figsize=(13,13))

linemat={}
for i in range(0,4):
    for j in range(0,2):
        k=i*2+j*1+1
        linemat['line'+str(k)], = ax[i,j].plot([], [])

xmax=10
ymax=150
xdata = []
ymat={}
for i in range(1,9):
    ymat['ydata'+str(i)]=[]

def upd(data):
    t= data[0]
    yvalue={}
    for i in range(1,9):
        yvalue['y'+str(i)]= data[i]
    
    xdata.append(t)
    [ymat['ydata'+str(i)].append(yvalue['y'+str(i)]) for i in range(1,9)]

    [linemat['line'+str(i)].set_data(xdata, ymat['ydata'+str(i)]) for i in range(1,9)]

    if t >= xmax-0.1:
        for i in range(0,4):
            [ax[i,j].set_xlim(t-xmax+0.1, t+0.1) for j in range(0,2)]
                # ax[i,j].figure.canvas.blit()
    
    return linemat['line1'], linemat['line2'], linemat['line3'], linemat['line4'], \
           linemat['line5'], linemat['line6'], linemat['line7'], linemat['line8'],

ani= animation.FuncAnimation(fig, upd, gen, blit=True, interval=5, repeat=False, init_func=init)

plt.show()