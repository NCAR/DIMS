import numpy as np
import matplotlib.pyplot as pl
from numpy import loadtxt
import time
import datetime



def format_time(DateTime):
    TimeNew = []
    for i in range(len(DateTime)):
        dateTime = DateTime[i].lstrip('00')
        dateTime = f'20{dateTime}'
        t = datetime.datetime.strptime(str(dateTime), '%Y-%m-%d %H:%M:%S')
        newtime = t + datetime.timedelta(days=1652, hours=19, minutes=3, seconds=11)
        TimeNew.append(newtime)
    return TimeNew


###added offset for correct day and time (8/20 1800 - 8/21 0908)
def import_data(file):
    lines = loadtxt(file, comments='#', delimiter=' ', dtype='str', unpack=True)
    D = lines[1]  ## time that data was recorded in format H:M:S need HMS
    V_EPS = np.array([float(i) for i in lines[6]])  ## voltage of EPS in mv
    I_EPS = np.array([float(i) for i in lines[10]])  ## current of EPS in mA
    I_LUP5 = np.array([float(i) for i in lines[14]])  ## curent of LUP 5V bus in mA
    I_LUP3 = np.array([float(i) for i in lines[18]])
    ON_LUP5 = np.array([int(i) for i in lines[21]])  ## LUP5 ON/OFF >4000 ON <4000 OFF
    ON_LUP3 = np.array([int(i) for i in lines[24]])
    I_BCR = np.array([float(i) for i in lines[28]])  ## current of Battery Charge Regulator in mA
    V_BCR = np.array([float(i) for i in lines[32]])  ## voltage of BCR in mV

    ## Change mV to V
    VEPS = V_EPS / 1000.00
    VBCR = V_BCR / 1000.00

    DateTime = []
    for i in range(len(D)):
        day = lines[0][i]
        time = lines[1][i]
        dt = day + " " + time
        DateTime.append(dt)

    return DateTime, VEPS, I_EPS, I_LUP5, I_LUP3, ON_LUP5, ON_LUP3, I_BCR, VBCR


def EPS_Voltage_Graph(time, VEPS):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, VEPS, color='k')
    pl.xlabel("\n Date (mm-dd) Time (hr)")
    pl.ylabel("Voltage (V) \n")
    pl.title("EPS Voltage \n" + "30 Min IS1 Test")
    pl.show()


def EPS_Current_Graph(time, I_EPS):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, I_EPS)
    pl.xlabel("\n Date/Time")
    pl.ylabel("Current (mA) \n")
    pl.title("EPS Current \n" + "30 Min IS1 Test")
    pl.show()


def LUP_5_Current(time, I_LUP5):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, I_LUP5)
    pl.xlabel("\n Date/Time")
    pl.ylabel("Current (mA) \n")
    pl.title("5V Latch Up Current \n" + "30 Min IS1 Test")
    pl.show()


def LUP_3_Current(time, I_LUP3):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, I_LUP3)
    pl.xlabel("\n Date/Time")
    pl.ylabel("Current (mA) \n")
    pl.title("3V Latch Up Current \n" + "30 Min IS1 Test")
    pl.show()


def V_5_LUP_ON_OFF(time, ON_LUP5):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, ON_LUP5)
    pl.xlabel("\n Date/Time")
    pl.ylabel("ON/OFF State \n")
    pl.title("5V Latch Up ON/OFF \n" + "30 Min IS1 Test")
    pl.show()


def V_3_LUP_ON_OFF(time, ON_LUP3):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, ON_LUP3)
    pl.xlabel("\n Date/Time")
    pl.ylabel("ON/OFF State \n")
    pl.title("3V Latch Up ON/OFF \n" + "30 Min IS1 Test")
    pl.show()


def BCR_Current(time, I_BCR):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, I_BCR)
    pl.xlabel("\n Date/Time")
    pl.ylabel("Current (mA) \n")
    pl.title("BCR Current \n" + "30 Min IS1 Test")
    pl.show()


def BCR_Voltage(time, VBCR):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, VBCR)
    pl.xlabel("\n Date/Time")
    pl.ylabel("Voltage (V) \n")
    pl.title("BCR Voltage \n" + "30 Min IS1 Test")
    pl.show()

FileName = "BATT.txt"
DateTime, VEPS, I_EPS, I_LUP5, I_LUP3, ON_LUP5, ON_LUP3, I_BCR, VBCR = import_data(FileName)
time = format_time(DateTime)
EPS_Voltage_Graph(time, VEPS)
EPS_Current_Graph(time, I_EPS)
LUP_5_Current(time, I_LUP5)
LUP_3_Current(time, I_LUP3)
V_5_LUP_ON_OFF(time, ON_LUP5)
V_3_LUP_ON_OFF(time, ON_LUP3)
BCR_Voltage(time, VBCR)
BCR_Current(time, I_BCR)
