import numpy as np
import matplotlib.pyplot as pl
from numpy import loadtxt
import time
import datetime



#Read through the datetimes and offset them to start at zero
def offset_time(time):
    time_offset = []
    for i in range(len(time)):
        time_offset.append(convert_time(datetime.datetime.strptime(str(time[i]), '%Y-%m-%d %H:%M:%S') - datetime.datetime.strptime(str(time[0]), '%Y-%m-%d %H:%M:%S')))
    return time_offset


def format_time(DateTime):
    #Create a new list of datetimes
    TimeNew = []
    #Format the datetimes
    for i in range(len(DateTime)):
        #Strip out the leading zeros
        dateTime = DateTime[i].lstrip('00')
        #Append 20 to the dateTime
        dateTime = f'20{dateTime}'
        #Get the Date Time Format
        t = datetime.datetime.strptime(str(dateTime), '%Y-%m-%d %H:%M:%S')
        #Off set the Time to the Correct time
        newtime = t + datetime.timedelta(days=1652, hours=19, minutes=3, seconds=11)
        TimeNew.append(newtime)
    return TimeNew

#Convert a timedelta value to hours
def convert_time(time):
    return time.total_seconds() / 3600


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


def EPS_Voltage_Graph(time, VEPS, title):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, VEPS, color='k')
    pl.xlabel("Hours OBC is ON")
    pl.ylabel("Voltage (V) \n")
    pl.title("EPS Voltage \n" + title)
    pl.savefig(title + "_EPS_Voltage.png")


def EPS_Current_Graph(time, I_EPS, title):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, I_EPS)
    pl.xlabel("Hours OBC is ON")
    pl.ylabel("Current (mA) \n")
    pl.title("EPS Current \n" + title)
    pl.savefig(title + "_EPS_Current.png")


def LUP_5_Current(time, I_LUP5, title):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, I_LUP5)
    pl.xlabel("Hours OBC is ON")
    pl.ylabel("Current (mA) \n")
    pl.title("5V Latch Up Current \n" + title)
    pl.savefig(title + "_LUP_5_Current.png")


def LUP_3_Current(time, I_LUP3, title):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, I_LUP3)
    pl.xlabel("Hours OBC is ON")
    pl.ylabel("Current (mA) \n")
    pl.title("3V Latch Up Current \n" + title)
    pl.savefig(title + "_LUP_3_Current.png")


def V_5_LUP_ON_OFF(time, ON_LUP5, title):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, ON_LUP5)
    pl.xlabel("Hours OBC is ON")
    pl.ylabel("ON/OFF State \n")
    pl.title("5V Latch Up ON/OFF \n" + title)
    pl.savefig(title + "_5V_LUP_ON_OFF.png")


def V_3_LUP_ON_OFF(time, ON_LUP3, title):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, ON_LUP3)
    pl.xlabel("Hours OBC is ON")
    pl.ylabel("ON/OFF State \n")
    pl.title("3V Latch Up ON/OFF \n" + title)
    pl.savefig(title + "_3V_LUP_ON_OFF.png")


def BCR_Current(time, I_BCR, title):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, I_BCR)
    pl.xlabel("Hours OBC is ON")
    pl.ylabel("Current (mA) \n")
    pl.title("BCR Current \n" + title)
    pl.savefig(title + "_BCR_Current.png")


def BCR_Voltage(time, VBCR, title):
    pl.figure(figsize=(10, 5))
    pl.gcf().autofmt_xdate()
    pl.plot(time, VBCR)
    pl.xlabel("Hours OBC is ON")
    pl.ylabel("Voltage (V) \n")
    pl.title("BCR Voltage \n" + title)
    pl.savefig(title + "_BCR_Voltage.png")


Parentpath = "C:\\Users\\mjeffers\\Desktop\\DIMS Ft.Sumner\\OBC\\"
Filenames = ["BATT1.txt","BATT2.txt", "BATT3.txt", "BATT4.txt"]
titles = ["DataSet1","DataSet2", "DataSet3","DataSet4"]
#Batttery Ran for a total of 16.68 hours at 775 mA at 5V
for i in range(len(Filenames)):
    DateTime, VEPS, I_EPS, I_LUP5, I_LUP3, ON_LUP5, ON_LUP3, I_BCR, VBCR = import_data(Parentpath + Filenames[i])
    zeroed_time = offset_time(DateTime)
    print(f"Time_On: {zeroed_time[-1]}")
    EPS_Voltage_Graph(zeroed_time, VEPS,title=titles[i])
    EPS_Current_Graph(zeroed_time, I_EPS,title=titles[i])
    LUP_5_Current(zeroed_time, I_LUP5,title=titles[i])
    LUP_3_Current(zeroed_time, I_LUP3,title=titles[i])
    V_5_LUP_ON_OFF(zeroed_time, ON_LUP5,title=titles[i])
    V_3_LUP_ON_OFF(zeroed_time, ON_LUP3,title=titles[i])
    print(f"The Average Current Draw is {np.average(I_BCR)}")
    BCR_Current(zeroed_time, I_BCR,title=titles[i])
    BCR_Voltage(zeroed_time, VBCR,title=titles[i])
