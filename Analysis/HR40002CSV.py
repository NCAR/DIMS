import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import csv
import struct
from pathlib import Path
from scipy.signal import savgol_filter


########################EDIT PATH HERE###############################################################
paths = Path('C:\\HAO-IG\\DIMS\\Testing\\10 Minute Test 7-Dec-2022\\IS1').glob('*')
#go through Paths
for path in paths:

    #Find the Correct files starting with n00
    files = Path(path).glob('**/n00*.txt')
    for file in files:
        print(f"Processing File {file}")
        spc=[]
        #Open The File and Read in the Data
        with open(file, 'rb') as f:
            #Read in the First 14 Bytes
            data=f.read(14)
            while True:
                #Read int the next 2 Bytes
                data=f.read(2)
                if not data:
                    break
                #Unpack the Data
                spc.append(struct.unpack("H", data)[0])
            #Write to the CSV
            with open(f'{str(file).strip(".txt")}.csv', 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                # write multiple rows
                writer.writerow(spc)
            #Plot the Data
            plt.plot(spc)
            plt.title(file)
            plt.savefig(file.with_suffix('.png'))
            plt.clf()


