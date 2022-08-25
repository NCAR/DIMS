import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import csv
import struct
from pathlib import Path
from scipy.signal import savgol_filter

##
paths = Path('/users/mjeffers/Desktop/IS1/').glob('*')
window=5 # This is a window of 5
#go through Paths
for path in paths:

    #Find the Correct files starting with n00
    files = Path(path).glob('**/n00*.txt')
    #create an array of Zeros
    nsum = np.zeros(3648)
    matplotlib.use('Agg')
    for file in files:
        spc=[]
        blk=[]
        with open(file, 'rb') as f:
            print(file)

            f.read(9)
            data=f.read(4) # integration time
            integration_time=data[2]*256+data[3]+256*256*(data[0]*256+data[1])#Find the integration_time in us
            f.read(2) # pixel mode
            f.read(5*2) # not usable
            for i in range(13):
                data=f.read(2)
                blk.append(struct.unpack(">H", data)[0])
            f.read(3*2) # transition pixels
            #Read in the Data
            for i in range(3648):
                data=f.read(2)
                spc.append(struct.unpack(">H", data)[0])
            with open(f'{str(file).strip(".txt")}.csv', 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                # write multiple rows
                writer.writerow(data)

