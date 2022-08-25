# Onboard Computer Codebase
## What is this
This is the Code base for the OBC Made by EnduroSat
The OBC works in conjunction with the IS1 to capture data for DIMs
The OBC has control over the C3D XCam and the EnduroSat EPS.

### What does the OBC Do:
The OBC Currently does the Following:
  - Controls and Communicates with the EnduroSat EPS 
  - Controls and Communicates with the C3D XCam
  - Writes the resulting data (XCAM and EPS) to an SD card


## Before Getting Started:

### Software Needed to Program:
The priimary way to Write Software to the OBC is via the STMWorkbench Distributed by Endurosat 
found here:
Through this software you can:
  - Write Software for the OBC
  - program the OBC
  - Debug the OBC

### Hardware Needed:
All software will be Written to the STM Using a Software Debugger SWD
The one we are using in lab is a ST-Link V2. This is connected to a prebuilt break-out board that converts the 20 Pin on the SWD to a 6 PIN output. 
There is then a custom Fab Cable that connects to the 9Pin femal on the DIMS system labeled "OBC"
