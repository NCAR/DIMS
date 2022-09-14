import os
import datetime
import matplotlib.pyplot as pl
import numpy as np
#Find every file in a path that is named enviro.txt check folders recursivley
def find_enviro_recursive(path):
    env_files = []
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.endswith('.txt'):
                if file.startswith('enviro'):
                    env_files.append(os.path.join(root, file))
    return env_files

#Gett the date time from the line
def parse_date_time(line):
    date_time = line.split()[0:2]
    #date_time = datetime.datetime.strptime(date_time, '%Y-%m-%d %H:%M:%S')
    return date_time


env_files = find_enviro_recursive('C:\\Users\\mjeffers\\Desktop\\DIMS Ft.Sumner\\IS1')
time_list = []
temp_list = []
pressure_list = []
for file in env_files:
    print(file)
    with open(file, 'r') as f:
        for line in f:
            date_time = parse_date_time(line)
            if date_time[0] != '20000000' and date_time[0] != '-':
                date_time = datetime.datetime.strptime(date_time[0]+date_time[1], '%Y%m%d%H%M%S')

                #Get the Temperature from the Same line
                temp = line.split()[4]
                #Get pressure from the same line
                pressure = line.split()[6]
                print(date_time, temp, pressure)
                #Make sure the Data is real
                if (float(temp) > -40) and (float(pressure) > 0):
                    time_list.append(date_time)
                    temp_list.append(float(temp))
                    pressure_list.append(float(pressure))
                

pl.figure(figsize=(10, 5))
# pl.gcf().autofmt_xdate()
pl.plot(time_list, temp_list, color='k')
pl.xlabel("Date Time")
pl.yticks(np.arange(min(temp_list), max(temp_list)+1, 1))
pl.ylabel("Temperature (C)")
pl.title("Temperature")
pl.show()

pl.figure(figsize=(10, 5))
# pl.gcf().autofmt_xdate()
pl.plot(time_list, pressure_list, color='k')
pl.yticks(np.arange(min(pressure_list), max(pressure_list), 1))
pl.xlabel("Date Time")
pl.ylabel("Pressure (mbarr)")
pl.title("Pressure")
pl.show()
#Go through each of the files and parse the data


