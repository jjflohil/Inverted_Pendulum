import socket
import time
import sys

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
import pandas as pd
import numpy as np
from decimal import *

from tkinter import *
from tkinter import messagebox

ControlParameters = Tk(screenName= "Control tuning parameters")
name = ControlParameters.title('Tuning parameters')
ControlParameters.geometry("300x300")


datalen = 500
#Set decimal precision
getcontext().prec = 3

# Save up to 60000 samples =  1 minute with ts = 1ms
TimeVec = [None for y in range( datalen )]
RefPosCartVec = [None for y in range( datalen )]
PosCartVec = [None for y in range( datalen )]
RefVelCartVec = [None for y in range( datalen )]
VelCartVec = [None for y in range( datalen )]
AccCartVec = [None for y in range( datalen )]
PosPendVec = [None for y in range( datalen )]
VelPendVec = [None for y in range( datalen )]
CtrlVec = [None for y in range( datalen )]
Mode  = [None for y in range( datalen )]

data = [None for y in range( 255 )]
Time = 0
RefPosCart = 0
PosCart = 0
RefVelCart = 0
VelCart = 0
AccCart = 0
PosPend = 0
VelPend = 0
Ctrl = 0
Pgain = 0
Igain = 0
Dgain = 0
Xgain = 0

i_c = 0 #current sample
t_c = 0.0 #current time
t_0 = 0.0 #start time
t_print = 0
ts_print = 0.1

FirstSample = 1

UDP_IP = ""

try:
	UDP_PORT = 4210
except:
	print("Example: python UDPRecv.py 3333")
	sys.exit()

sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

x_len = datalen

# Figure 1  cart
fig0 = plt.figure()
plt.subplots_adjust( wspace=0.4, hspace=0.4)
ax0 = fig0.add_subplot(3,1,1)
ax0.set_ylim([-0.2,0.2])
ax0.grid()
plt.title('Cart Position')
plt.ylabel('Position [m]')
ax1 = fig0.add_subplot(3,1,2)
ax1.set_ylim([-0.1,0.1])
ax1.grid()
plt.title('Cart Velocity')
plt.ylabel('Velocity [m]')
ax2 = fig0.add_subplot(3,1,3)
ax2.set_ylim([-255,255])
ax2.grid()
plt.title('Control value')
plt.xlabel("time [s]")
plt.ylabel('Ctrl [-]')
y_range = [-0.2, 0.2]
x_value = np.linspace(0,10,num=datalen) #list(range(0, x_len))
y_value = [0] * x_len
ax0.set_ylim(y_range)
line0, =ax0.plot(x_value, y_value)
line1, =ax0.plot(x_value, y_value)
line2, =ax1.plot(x_value, y_value)
line3, =ax1.plot(x_value, y_value)
line4, =ax2.plot(x_value, y_value)

# Figure 2  pendulum
fig1 = plt.figure()
plt.subplots_adjust( wspace=0.4, hspace=0.4)
ax3 = fig1.add_subplot(2,1,1)
ax3.set_ylim([-10.0,10.0])
ax3.grid()
plt.title('Pendulum Position')
plt.ylabel('Position [m]')
ax4 = fig1.add_subplot(2,1,2)
ax4.set_ylim([-3.5,3.5])
ax4.grid()
plt.title('Pendulum Velocity')
plt.ylabel('Velocity [m]')
line5, =ax3.plot(x_value, y_value)
line6, =ax4.plot(x_value, y_value)


def helloCallBack():
   ControlParameters.destroy

B0 = Button(ControlParameters, text ="P gain = 0", height= 20, width=30, command = helloCallBack)
B0.place(x=20,y=0)
B1 = Button(ControlParameters, text ="I gain = 0", height= 20, width=30, command = helloCallBack)
B1.place(x=20,y=50)
B2 = Button(ControlParameters, text ="D gain = 0", height= 20, width=30, command = helloCallBack)
B2.place(x=20,y=100)

def DataToFloat(data, i_start):
    output = round((data[i_start+6]-48 + 10*(data[i_start+5]-48) + 100*(data[i_start+4]-48) + 1000*(data[i_start+3]-48) + 10000*(data[i_start+2]-48) + 100000*(data[i_start+1]-48) + 1000000*(data[i_start]-48))-5000000)/1000
    return output

def animate0(i, y_value):
    data, addr = sock.recvfrom(512)
    #Get float from data bytes
    Time =  DataToFloat(data, 0)
    RefPosCart = DataToFloat(data, 8)
    PosCart = DataToFloat(data, 16)
    RefVelCart = DataToFloat(data, 24)
    VelCart = DataToFloat(data, 32)
    AccCart = DataToFloat(data, 40)
    PosPend = DataToFloat(data, 48)
    VelPend = DataToFloat(data, 56)
    Ctrl = DataToFloat(data, 64)
    Mode = DataToFloat(data, 72)
    Pgain = DataToFloat(data, 80)
    Igain = DataToFloat(data, 88)
    Dgain = DataToFloat(data, 96)
    Xgain = DataToFloat(data, 104)

    for ii in range(1,x_len-1):
        TimeVec[ii]=TimeVec[ii+1]
        RefPosCartVec[ii]=RefPosCartVec[ii+1]
        PosCartVec[ii]=PosCartVec[ii+1]
        RefVelCartVec[ii]=RefVelCartVec[ii+1]
        VelCartVec[ii]=VelCartVec[ii+1]
        AccCartVec[ii]=AccCartVec[ii+1]
        PosPendVec[ii]=PosPendVec[ii+1]
        VelPendVec[ii]=VelPendVec[ii+1]
        CtrlVec[ii]=CtrlVec[ii+1]
    TimeVec[x_len-1] = Time
    RefPosCartVec[x_len-1] = RefPosCart
    PosCartVec[x_len-1] = PosCart
    RefVelCartVec[x_len-1] = RefVelCart
    VelCartVec[x_len-1] = VelCart
    AccCartVec[x_len-1] = AccCart
    PosPendVec[x_len-1] = PosPend
    VelPendVec[x_len-1] = VelPend
    CtrlVec[x_len-1] = Ctrl
     
    line0.set_ydata(RefPosCartVec)
    line1.set_ydata(PosCartVec)
    line2.set_ydata(RefVelCartVec)
    line3.set_ydata(VelCartVec)
    line4.set_ydata(CtrlVec)
    
    if Mode==6:
        with open('csvFolder/plotdata.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'PosCart', 'VelCart','AccCart', 'PosPend', 'VelPend', 'CtrlVec'])
            for ii in range(0,datalen):
                writer.writerow([TimeVec[ii], PosCartVec[ii], VelCartVec[ii], AccCartVec[ii], PosPendVec[ii], VelPendVec[ii], CtrlVec[ii]])

    B0["text"] = "P gain = " + str(Pgain)
    B1["text"] = "I gain = " + str(Igain)
    B2["text"] = "D gain = " + str(Dgain)
    
    return line0,line1,line2,line3,line4

def animate1(i, y_value):
    line5.set_ydata(PosPendVec)
    line6.set_ydata(VelPendVec)
        
    return line5,line6

ani0 = animation.FuncAnimation(fig0,animate0,fargs=(y_value,),interval=10,blit=True)
ani1 = animation.FuncAnimation(fig1,animate1,fargs=(y_value,),interval=10,blit=True)


   


plt.show()
ControlParameters.mainloop()
