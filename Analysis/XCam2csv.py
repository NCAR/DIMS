import numpy as np #needed for working with arrays
import array
import os
import matplotlib.pyplot as plt #needed for plotting
import math
# import bitstring
import datetime
from os import listdir
import time
import ntpath
import glob
import csv
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
###EDIT THIS FOLLOWING LINE TO POINT TO YOUR FOLDER##### OUTPUT WILL LIVE HERE TOO
files = glob.glob("C:\\Users\mjeffers\Desktop\OBC\\*.RAW");
#files = glob.glob("D:\\011*.RAW")
for filenm in files:
    print('Reading: ',filenm)
    try:
        with open(filenm, 'rb') as fid:
            print(f"Opened File {filenm}")
            fid.seek(0,0)

            rawdata = np.fromfile(fid,dtype=np.uint8,count=rawsz)
            fid.close()

            stepsz = 260 ## Packet Size
            imStep = 249 # Image data in packet
            imBeg  = 9 #Where the image begins
            IDspot = 8 #The byte where the packet ID lives
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
                #print(f"Size of image {len(tpay)}")
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

    ##Takes Care of the Thumbnail
                # elif image_ID == 4 or image_ID == 5:
                #     thm_raw[tt:tt+imStep] = rawdata[imBeg+tpay:stepsz+tpay]
                #     if (tpay < (stepsz*6) ):
                #         ## print out a few values to insure sync
                #         print('tmm_raw                             ',tmm_raw[ii:ii+7])
                #     tt = tt + imStep


            pixels = img_raw[0:(ysz*xsz)]
            img    = np.reshape(pixels,(ysz,xsz))

            pixels_thumb = thm_raw[0:(txsz*tysz)]
            img_thumb    = np.reshape(pixels_thumb,(tysz,txsz))
            img_thumb    = np.fliplr(np.flipud(img_thumb))

            img_rev = np.fliplr(np.flipud(img))

            print('displaying: ',filenm)
            print(img_rev)
            ## Display the image
            img_rev
            with open(f'{filenm.strip(".RAW")}.csv', 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                # write multiple rows
                writer.writerows(img_rev)

            print(f'Image Written to following CSV: {filenm.strip(".RAW")}.csv')


    except:
        print(f"Couldnt Display {filenm}")
        pass


