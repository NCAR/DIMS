import numpy as np #needed for working with arrays
import array
import os
import matplotlib.pyplot as plt #needed for plotting
import math
import bitstring
import datetime
from os import listdir
import time
import ntpath
import glob
import sys

xsz      = 1280
ysz      = 1024
txsz     = 120
tysz     = 100
imsz     = xsz*ysz
tsz      = txsz*tysz
rawsz    = 1368640

rawdata  = np.zeros((rawsz,),dtype=np.uint8 )
img_raw  = np.zeros((imsz,),dtype=np.uint8  )
thm_raw  = np.zeros((tsz,), dtype=np.uint8   )

plt.close('all')
fig, ax = plt.subplots()

files = glob.glob("c:/alice/DIMS/Data/*.raw")
for  filenm in files:
    print('Reading: ',filenm)
    with open(filenm, 'rb') as fid:
        fid.seek(0,0)
        rawdata = np.fromfile(fid,dtype=np.uint8,count=rawsz)
        fid.close()

        stepsz = 260
        imStep = 249
        imBeg  = 9
        IDspot = 8
        IDXspot = 5 ## Spot for the buffer index, which should start at 1
                    ## and keep increasing.
                    ## Although it is not used below, it should be
                    ## so that the images are reconstructed properly.
        ii = 0
        tt = 0
        limit = rawsz - stepsz -1

        ## Go through the raw data in 260 byte chunks,
        ## strip out the header and crc's and create the byte image.
        for tpay in range(0,limit,stepsz):

            if (tpay < (stepsz*6) ):
                ## print out a few values to insure sync
                print(rawdata[tpay:tpay+16])

            image_ID = rawdata[tpay + IDspot]
            if image_ID == 0 or image_ID == 1:
                img_raw[ii:ii+imStep] = rawdata[tpay+imBeg:tpay+imBeg+imStep]
                if (tpay < (stepsz*6) ):
                    ## print out a few values to insure sync
                    print('img_raw                             ',img_raw[ii:ii+7])
                ii = ii + imStep

            elif image_ID == 4 or image_ID == 5:
                thm_raw[tt:tt+imStep] = rawdata[imBeg+tpay:stepsz+tpay]
                if (tpay < (stepsz*6) ):
                    ## print out a few values to insure sync
                    print('tmm_raw                             ',tmm_raw[ii:ii+7])
                tt = tt + imStep


        pixels = img_raw[0:(ysz*xsz)]
        img    = np.reshape(pixels,(ysz,xsz))

        pixels_thumb = thm_raw[0:(txsz*tysz)]
        img_thumb    = np.reshape(pixels_thumb,(tysz,txsz))
        img_thumb    = np.fliplr(np.flipud(img_thumb))

        img_rev = np.fliplr(np.flipud(img))

        print('displaying: ',filenm)

        ## Display the image
        ax.imshow(img_rev, interpolation="nearest", cmap="gray")
        plt.draw()
        plt.pause(10)



