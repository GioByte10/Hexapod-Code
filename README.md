# TODO:


# Getting Started with USB CAN bus

To check if USB CAN bus is working, run

```
ifconfig -a
```

and verify that `can0` appears in the list.

# Install Code base

Do this by running the installation.sh file alternatively, you can manually run the below commands yourself

## Cloning Repo

```
sudo apt-get install git
git clone https://github.com/ucsdarclab/arcsnake_v2.git
```

## Installing Packages

Install can utils:

```
sudo apt-get install can-utils
```

Install pip packages:

```
pip install pyinstrument
pip install numpy
pip install matplotlib
pip install python-can
```

# Running test programs

cd inside of arcsnake directory 
run 
'''
python3 tests/test_sanity.py
'''

# Installing Ros (fix up)


# Notes:

## Add Python to ubuntu path

- type sudo gedit ~/.bashrc in /home directory 
- Insert this command near end of text file: 
  '''
    export PATH="$HOME/myeoh/Documents/GitHub/arcsnake_v2:$PATH"
  '''
- Run source ~/.bashrc
- save and verify by running echo $PATH in terminal 

## Otherwise, if issues with Path stuff insert this into top of code

from os.path import dirname, realpath  
import sys  
arcsnake_v2_path = dirname(dirname(realpath(__file__)))  
sys.path.append(arcsnake_v2_path)  

# Setting up the Longan Board
## Basic setup
1) Navigate to https://www.arduino.cc/en/software
2) Scroll down the page and download "Arduino IDE 2.0 RC". It is a much improved IDE than the old Arduino software
3) Open up the humiditytest.ino file in the Arduino IDE(arcsnake_v2/ArduinoSensors/humidtytest)
4) Make sure that the humidity sensor is wired up according to the pin declared in the Arduino sketch 
5) Locate drop down menu at the top of the IDE and ensure that your board is selected. The Longa board will show up as a Arduino Leonardo
6) Press upload
7) Open the Serial monitor with ctrl+shift+m
8) Make sure baudrate of serial monitor is 9600
9) If the correct readings are printing to the Serial monitor then everything is in order!

## Uploading and testing Arduino - Python CAN
1) Open the ArduinoSensors.ino file (arcsnake_v2/ArduinoSensors/ArduinoSensors)
2) Upload
3) Connect the CAN H and CAN L from the Longan board to the appropriate pins on the USB to CAN device 
4) Run the test_ArduinoSensors.py file
5) If the correct numbers are printing on Python's terminal as well as Arduino's terminal then everything is good
