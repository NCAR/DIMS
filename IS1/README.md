# Independant Subsystem 1 Codebase
## What is this
This is the Code base for the Independent Subsystem 1 (IS1)
The independant subsystem currently works in conjunction with the OBC to capture data for DIMs
The IS1 has control over the HR4000 ocean optics spectragraph.
Documentation found here:
https://www.oceaninsight.com/products/spectrometers/high-resolution/hr4000cg-uv-nir/?qty=1

### What does the IS1 Do:
The IS1 Currently does the Following:
  - Controls and Communicates with the HR4000 
  - Communicates with a GPS to get location and DateTime
  - Writes the resulting data (HR4000 and GPS information) to an SD card


## Before Getting Started:

### Software Needed to Program:
The priimary way to Write Software to the IS1 is via the STM32CUBE IDE
found here:
https://www.st.com/en/ecosystems/stm32cube.html
Through this software you can:
  - designate the hardware setting of the STM32 Chip on board ie (Pin 1 is SPI1 miso) etc
  - Write Software for the STM32
  - Configure hardware intterupts
  - program the STM32
  - Debug the STM32

### Hardware Needed:
All software will be Written to the STM Using a Software Debugger SWD
The one we are using in lab is a ST-Link V2. This is connected to a prebuilt break-out board that converts the 20 Pin on the SWD to a 6 PIN output. 
There is then a custom Fab Cable that connects to the 9Pin femal on the DIMS system labeled "IS1"

### Important Note
The IS1 is powered by the 5V and 3v3 rails on the EPS. The EPS is controlled by the OBC
Therefore inorder to program the IS1 you must have the OBC powered as well, AND the OBC must be in a state where it is holding the 5v and 3v3 Buses on.
