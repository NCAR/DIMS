import matplotlib.pyplot as plt
import numpy as np
import matplotlib

import struct
from pathlib import Path
from scipy.signal import savgol_filter

paths = Path('/users/mjeffers/Desktop/IS1/IS1').glob('*')
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
            for i in range(3648):
                data=f.read(2)
                spc.append(struct.unpack(">H", data)[0])
            black = np.mean(np.array(blk))
            ns= np.array(spc)-black
            nsum += ns

             # window must be odd
            window=29
            window=7
#            nss=np.empty(np.shape(ns))
            nss=savgol_filter(ns, window, 3)
            plt.plot(ns[1200:2000])
#            plt.plot(nss, '.', markersize=1)
            plt.ylim([-200,2**14-black])
#            plt.title("filename: {:s}\nIntegration Time: {:d},{:s}".format(str(file), integration_time, str(file)[-19:-4]))
            plt.title("{:s}\nIntegration Time: {:d} us, {:s}-{:s}-{:s}{:s}:{:s}:{:s}".format(str(file), integration_time,
                     str(file)[-19:-15], str(file)[-15:-13],str(file)[-13:-11], str(file)[-10:-8], str(file)[-8:-6],str(file)[-6:-4], ))
            plt.savefig(file.with_suffix('.png'))
            plt.clf()
    nsums=savgol_filter(nsum, window, 3)
    plt.plot(nsums)
    plt.title(path)
    plt.savefig(path/'sum.png')
    plt.clf()
