#!/usr/bin/env python3

############################################################################
#              OpenMMS Real-Time Controller Software (VLP-16)              #
############################################################################
# Version: 1.3                                                             #
# Date:    June 2020                                                       #
# Author:  Ryan Brazeal                                                    #
# Email:   ryan.brazeal@ufl.edu                                            #
#                                                                          #
#    OPEN-SOURCE LICENSE INFO:                                             #
#                                                                          #
#    This file is part of OpenMMS_OSS.                                     #
#                                                                          #
#    OpenMMS_OSS is free software: you can redistribute it and/or modify   #
#    it under the terms of the GNU General Public License as published by  #
#    the Free Software Foundation, either version 3 of the License, or     #
#    (at your option) any later version.                                   #
#                                                                          #
#    OpenMMS_OSS is distributed in the hope that it will be useful,        #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#    GNU General Public License for more details.                          #
#                                                                          #
#    You should have received a copy of the GNU General Public License     #
#    along with OpenMMS_OSS. If not, see <https://www.gnu.org/licenses/>.  #
#                                                                          #
############################################################################

import RPi.GPIO as GPIO
import time
import os
import subprocess
import re
import os.path
from picamera import PiCamera
import serial

#change working directory to scans
os.chdir("/home/openmms/mms_data")

cameraInstalled = "1"
videoOnOff = "0"

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

lidar = 12
button = 16
redLED = 40
greenLED = 38
blueLED = 36
camera1 = 22
camera2 = 24
camera3 = 26

ON = 1
OFF = 0
ledON = 0
ledOFF = 1

GPIO.setup(lidar, GPIO.OUT)
GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(redLED, GPIO.OUT)
GPIO.setup(greenLED, GPIO.OUT)
GPIO.setup(blueLED, GPIO.OUT)
GPIO.setup(camera1, GPIO.OUT)
GPIO.setup(camera2, GPIO.OUT)
GPIO.setup(camera3, GPIO.OUT)

GPIO.output(lidar, OFF)
GPIO.output(redLED, ledOFF)
GPIO.output(greenLED, ledOFF)
GPIO.output(blueLED, ledOFF)
GPIO.output(camera1, ON)
GPIO.output(camera2, ON)
GPIO.output(camera3, ON)

dataCollecting = False
dataCollectType = 0

try:
    videoCamera = PiCamera()
    videoCamera.rotation = 180
    videoCamera.resolution = (1920,1080)
    videoCamera.framerate = 30
    video_quality = 20
except:
    cameraInstalled = "0"

def checkVideo():
    global videoOnOff
    if os.path.isfile("/home/openmms/mms_data/cameras/video.txt"):
        f = open("/home/openmms/mms_data/cameras/video.txt","r")
        videoOnOff = f.readline()
        videoOnOff = str(videoOnOff.strip('\n'))
        f.close()
    else:
        videoOnOff = "0"

    time.sleep(0.1)

def setUTCtime():
    #try to get time from the serial stream from the APX sensor
    try:
        ser = serial.Serial('/dev/serial0',115200,timeout=0.2)
        for t in range(0,50):
            line = ser.readline().decode('ascii')
            UTCdata = line.strip('\n').split(',')
            if UTCdata[0].upper() == "$GPZDA":
                utcHour = UTCdata[1][0:2]
                utcMin = UTCdata[1][2:4]
                utcSec = UTCdata[1][4:len(UTCdata[1])]
                utcDay = UTCdata[2]
                utcMonthNum = int(UTCdata[3])
                utcMonth = ""
                if utcMonthNum == 1:
                    utcMonth = "JANUARY"
                elif utcMonthNum == 2:
                    utcMonth = "FEBRUARY"
                elif utcMonthNum == 3:
                    utcMonth = "MARCH"
                elif utcMonthNum == 4:
                    utcMonth = "APRIL"
                elif utcMonthNum == 5:
                    utcMonth = "MAY"
                elif utcMonthNum == 6:
                    utcMonth = "JUNE"
                elif utcMonthNum == 7:
                    utcMonth = "JULY"
                elif utcMonthNum == 8:
                    utcMonth = "AUGUST"
                elif utcMonthNum == 9:
                    utcMonth = "SEPTEMBER"
                elif utcMonthNum == 10:
                    utcMonth = "OCTOBER"
                elif utcMonthNum == 11:
                    utcMonth = "NOVEMBER"
                elif utcMonthNum == 12:
                    utcMonth = "DECEMBER"
                utcYear = UTCdata[4]
                utcString = utcDay + " " + utcMonth + " " + utcYear + " " + utcHour + ":" + utcMin + ":" + utcSec
                command4 = "echo '$mmnep0' | sudo -S date -s \"" + utcString + "\""
                setUTCtime = subprocess.run(str(command4), shell=True)
                print("UTC time has been set to " + utcString)
                time.sleep(0.2)
        ser.close()
        time.sleep(0.2)
    except Exception as e:
        print("***** UTC time set error *****")
#        print(e)
        GPIO.output(redLED, ledOFF)
        GPIO.output(blueLED, ledOFF)
        GPIO.output(greenLED, ledOFF)
        for i in range(0,5):
            GPIO.output(redLED, ledON)
            time.sleep(0.2)
            GPIO.output(redLED, ledOFF)
            time.sleep(0.2)
        pass

time.sleep(1.0)

#check current camera interval setting
intervalCheck = ""
if os.path.isfile("/home/openmms/mms_data/cameras/interval.txt"):
    f = open("/home/openmms/mms_data/cameras/interval.txt","r")
    intervalCheck = f.readline()
    intervalCheck = intervalCheck.strip('\n')
    f.close()
    time.sleep(0.5)
else:
    #default camera interval = 2.0 seconds
    intervalCheck = "3"

#initial LED test
for i in range(0,13):
    GPIO.output(redLED, ledON)
    time.sleep(0.2)
    GPIO.output(redLED, ledOFF)
    time.sleep(0.2)
    GPIO.output(greenLED, ledON)
    time.sleep(0.2)
    GPIO.output(greenLED, ledOFF)
    time.sleep(0.2)
    GPIO.output(blueLED, ledON)
    time.sleep(0.2)
    GPIO.output(blueLED, ledOFF)
    time.sleep(0.2)

time.sleep(1.0)

#check to see if a RPi Nadir Video will be recorded
checkVideo()
if videoOnOff == "1" and cameraInstalled == "1":
    #flash the system light white
    for i in range(0,25):
        GPIO.output(redLED, ledON)
        GPIO.output(greenLED, ledON)
        GPIO.output(blueLED, ledON)
        time.sleep(0.1)
        GPIO.output(redLED, ledOFF)
        GPIO.output(greenLED, ledOFF)
        GPIO.output(blueLED, ledOFF)
        time.sleep(0.1)

GPIO.output(greenLED, ledON)

currentFileName = ""

while True:

    button_state = GPIO.input(button)

    if button_state == False:
        time.sleep(1)
        button_state2 = GPIO.input(button)
        if button_state2 == False:
            #quickly flash blue LED to show that 1 second has passed if data is NOT currently being collected
            if dataCollecting == False:
                GPIO.output(greenLED, ledOFF)
                GPIO.output(blueLED, ledON)
                time.sleep(0.2)
                GPIO.output(blueLED, ledOFF)
                GPIO.output(greenLED, ledON)
            time.sleep(1)
            button_state3 = GPIO.input(button)
            if button_state3 == False:
                #button held for more than 2 seconds so end data collection (if it has started)
                if dataCollecting == True:
                    dataCollecting = False
                    
                    try:
                        GPIO.output(blueLED, ledOFF)
                        GPIO.output(lidar, OFF)
                        GPIO.output(camera1, ON)
                        GPIO.output(camera2, ON)
                        GPIO.output(camera3, ON)

                        if dataCollectType == 1:
                            processIDS = subprocess.check_output(["pidof","tcpdump"])
                            tcpdumpSTOP = subprocess.run("echo '$mmnep0' | sudo -S kill " + str(int(processIDS)), shell=True)
                            time.sleep(0.5)

                        try:
                            processIDS2 = subprocess.check_output(["pidof","ttylog"])
                            ttylogSTOP = subprocess.run("echo '$mmnep0' | sudo -S kill " + str(int(processIDS2)), shell=True)
                            time.sleep(0.5)
                        except:
                            #ttylog process appears not to have started
                            for i in range(0,25):
                                GPIO.output(greeLED, ledOFF)
                                GPIO.output(blueLED, ledON)
                                time.sleep(0.2)
                                GPIO.output(blueLED, ledOFF)
                                GPIO.output(greenLED, ledON)
                                time.sleep(0.2)
                            GPIO.output(greenLED, ledOFF)
                            GPIO.output(redLED, ledON)

                        checkVideo()
                        if videoOnOff == "1" and cameraInstalled == "1":
                            videoCamera.stop_recording()
                            for i in range(0,5):
                                GPIO.output(greenLED, ledOFF)
                                GPIO.output(blueLED, ledOFF)
                                GPIO.output(redLED, ledOFF)
                                time.sleep(0.5)
                                GPIO.output(greenLED, ledON)
                                GPIO.output(blueLED, ledON)
                                GPIO.output(redLED, ledON)
                                time.sleep(0.5)
                            command3 = "echo '$mmnep0' | sudo -S MP4Box -fps 20 -add " + currentFileName + ".h264 " + currentFileName + ".mp4"
                            convertVideoSTART = subprocess.run(str(command3), shell=True)
                            time.sleep(0.5)
                            command4 = "echo '$mmnep0' | sudo -S rm " + currentFileName + ".h264"
                            deleteVideoSTART = subprocess.run(str(command4), shell=True)
                            GPIO.output(greenLED, ledOFF)
                            GPIO.output(blueLED, ledOFF)
                            GPIO.output(redLED, ledOFF)
                            time.sleep(0.5)

                        for i in range(0,5):
                            GPIO.output(greenLED, ledON)
                            time.sleep(0.5)
                            GPIO.output(greenLED, ledOFF)
                            time.sleep(0.5)
                        #status is back to green (no errors when stopping data collection)
                        GPIO.output(greenLED, ledON)
    
                    except:
                        #tcpdump process appears not to have started
                        for i in range(0,25):
                            GPIO.output(blueLED, ledOFF)
                            GPIO.output(redLED, ledON)
                            time.sleep(0.2)
                            GPIO.output(redLED, ledOFF)
                            GPIO.output(blueLED, ledON)
                            time.sleep(0.2)
                        GPIO.output(blueLED, ledOFF)
                        #status is red as an error occurred when trying to stop data collection (possibly it never started)
                        GPIO.output(redLED, ledON)
                    dataCollectType = 0
                else:
                    time.sleep(1)
                    GPIO.output(greenLED, ledOFF)
                    GPIO.output(blueLED, ledON)
                    time.sleep(0.2)
                    GPIO.output(blueLED, ledOFF)
                    GPIO.output(greenLED, ledON)
                    time.sleep(0.2)
                    GPIO.output(greenLED, ledOFF)
                    GPIO.output(blueLED, ledON)
                    time.sleep(0.2)
                    GPIO.output(blueLED, ledOFF)
                    GPIO.output(greenLED, ledON)
                    time.sleep(1)
                    button_state4 = GPIO.input(button)
                    
                    if button_state4 == True:
                        #button held for more than 2 seconds but not while data is being collected, and released after 2 blue blinks
                        #Images only mapping (no lidar)

                        if dataCollecting == False:
                            setUTCtime()
                            dataCollecting = True
                            dataCollectType = 2
                            filename = ""
                            if os.path.isfile("/home/openmms/firmware/fileNum.txt"):
                                f = open("/home/openmms/firmware/fileNum.txt","r")
                                filename = f.readline()
                                filename = filename.strip('\n')
                                f.close()
                                time.sleep(0.5)
                                lastnum = int(filename)
                                lastnum = lastnum + 1
                                f = open("/home/openmms/firmware/fileNum.txt","w")
                                f.write(str(lastnum))
                                f.close()
                            else:
                                filename = time.strftime("%d_%m_%Y_%H_%M_%S") + "_missing_filename"

                            interval = ""
                            if os.path.isfile("/home/openmms/mms_data/cameras/interval.txt"):
                                f = open("/home/openmms/mms_data/cameras/interval.txt","r")
                                interval = f.readline()
                                interval = interval.strip('\n')
                                f.close()
                                time.sleep(0.5)
                            else:
                                #default interval = 2.0 seconds
                                interval = "3"

                            #command = "echo '$mmnep0' | sudo -S tcpdump src 192.168.1.201 and udp and port 2368 or port 8308 -B 8192 -s 65535 -w "
                            #newCommand = command + "OpenMMS_" + filename + ".pcap" + " &"

                            command2 = "echo '$mmnep0' | sudo -S ttylog -b 115200 -d /dev/serial0 >"
                            newCommand2 = command2 + "OpenMMS_" + filename + ".traj" + " &"
                            #print(newCommand)

                            GPIO.output(redLED, ledOFF)
                            GPIO.output(greenLED, ledOFF)
                            GPIO.output(blueLED, ledOFF)
                            
                            #GPIO.output(lidar, ON)
                            
                            for i in range(0,3):
                                GPIO.output(greenLED, ledON)
                                time.sleep(0.5)
                                GPIO.output(greenLED, ledOFF)
                                time.sleep(0.5)

                            checkVideo()
                            if videoOnOff == "1" and cameraInstalled == "1":
                                currentFileName = "OpenMMS_" + filename
                                videoCamera.start_recording(currentFileName + ".h264", quality = video_quality, format = 'h264')
                                #videoCamera.start_recording(currentFileName + ".h264", format = 'h264')
                            
                            #tcpdumpSTART = subprocess.run(str(newCommand), shell=True)
                            #time.sleep(0.5)
                            
                            ttylogSTART = subprocess.run(str(newCommand2), shell=True)
                            time.sleep(0.5)
                            GPIO.output(blueLED, ledON)

                            if interval == "2":
                                #1.5 second camera interval
                                GPIO.output(camera1, OFF)
                                GPIO.output(camera2, OFF)
                                GPIO.output(camera3, OFF)
                            elif interval == "3":
                                #2.0 second camera interval
                                GPIO.output(camera1, ON)
                                GPIO.output(camera2, OFF)
                                GPIO.output(camera3, OFF)
                            elif interval == "4":
                                #2.5 second camera interval
                                GPIO.output(camera1, OFF)
                                GPIO.output(camera2, ON)
                                GPIO.output(camera3, OFF)
                            elif interval == "5":
                                #3.0 second camera interval
                                GPIO.output(camera1, ON)
                                GPIO.output(camera2, ON)
                                GPIO.output(camera3, OFF)
                            elif interval == "6":
                                #3.5 second camera interval
                                GPIO.output(camera1, OFF)
                                GPIO.output(camera2, OFF)
                                GPIO.output(camera3, ON)
                            elif interval == "7":
                                #4.0 second camera interval
                                GPIO.output(camera1, ON)
                                GPIO.output(camera2, OFF)
                                GPIO.output(camera3, ON)
                            elif interval == "8":
                                #5.0 second camera interval
                                GPIO.output(camera1, OFF)
                                GPIO.output(camera2, ON)
                                GPIO.output(camera3, ON)

                    else: #blink the blue LED 3 times to give option to safely shutdown the Raspberry Pi computer
                        time.sleep(1)
                        GPIO.output(greenLED, ledOFF)
                        GPIO.output(blueLED, ledON)
                        time.sleep(0.2)
                        GPIO.output(blueLED, ledOFF)
                        GPIO.output(greenLED, ledON)
                        time.sleep(0.2)
                        GPIO.output(greenLED, ledOFF)
                        GPIO.output(blueLED, ledON)
                        time.sleep(0.2)
                        GPIO.output(blueLED, ledOFF)
                        GPIO.output(greenLED, ledON)
                        time.sleep(0.2)
                        GPIO.output(greenLED, ledOFF)
                        GPIO.output(blueLED, ledON)
                        time.sleep(0.2)
                        GPIO.output(blueLED, ledOFF)
                        GPIO.output(greenLED, ledON)
                        time.sleep(1)
                        button_state5 = GPIO.input(button)
                        if button_state5 == True:
                            checkVideo()
                            if videoOnOff == "1" and cameraInstalled == "1":
                                videoCamera.close()
                            rpiShutDown = subprocess.run("echo '$mmnep0' | sudo shutdown -h 0 ", shell=True)
                        else:  # !!!!!!CAUTION!!!!!! button still held after 3 blue blinks, check to see if the user wants to delete the stored scan data
                            time.sleep(5)
                            GPIO.output(greenLED, ledOFF)
                            GPIO.output(blueLED, ledON)
                            time.sleep(0.2)
                            GPIO.output(blueLED, ledOFF)
                            GPIO.output(greenLED, ledON)
                            time.sleep(0.2)
                            GPIO.output(greenLED, ledOFF)
                            GPIO.output(blueLED, ledON)
                            time.sleep(0.2)
                            GPIO.output(blueLED, ledOFF)
                            GPIO.output(greenLED, ledON)
                            time.sleep(0.2)
                            GPIO.output(greenLED, ledOFF)
                            GPIO.output(blueLED, ledON)
                            time.sleep(0.2)
                            GPIO.output(blueLED, ledOFF)
                            GPIO.output(greenLED, ledON)
                            time.sleep(0.2)
                            GPIO.output(greenLED, ledOFF)
                            GPIO.output(blueLED, ledON)
                            time.sleep(0.2)
                            GPIO.output(blueLED, ledOFF)
                            GPIO.output(greenLED, ledON)
                            time.sleep(1)
                            button_state6 = GPIO.input(button)
                            if button_state6 == True:
                                try:
                                    clearDir = subprocess.run("echo '$mmnep0' | sudo rm -f /home/openmms/mms_data/* ", shell=True)
                                    GPIO.output(greenLED, ledOFF)
                                    time.sleep(0.5)
                                    GPIO.output(greenLED, ledON)
                                    time.sleep(0.5)
                                    GPIO.output(greenLED, ledOFF)
                                    time.sleep(0.5)
                                    GPIO.output(greenLED, ledON)
                                    time.sleep(0.5)
                                    GPIO.output(greenLED, ledOFF)
                                    time.sleep(0.5)
                                    GPIO.output(greenLED, ledON)
                                    time.sleep(0.5)
                                    GPIO.output(greenLED, ledOFF)
                                    time.sleep(0.5)
                                    GPIO.output(greenLED, ledON)
                                except:
                                    for i in range(0,10):
                                        GPIO.output(redLED, ledON)
                                        time.sleep(0.2)
                                        GPIO.output(redLED, ledOFF)
                                        time.sleep(0.2)
                                    GPIO.output(redLED, ledOFF)
                                    GPIO.output(blueLED, ledOFF)
                                    GPIO.output(greenLED, ledON)
                            else:
                                GPIO.output(greenLED, ledOFF)
                                GPIO.output(redLED, ledOFF)
                                GPIO.output(blueLED, ledOFF)
                                GPIO.output(redLED, ledON)
                                time.sleep(1.0)
                                GPIO.output(redLED, ledOFF)
                                GPIO.output(blueLED, ledON)
                                time.sleep(1.0)
                                GPIO.output(blueLED, ledOFF)
                                GPIO.output(greenLED, ledON)
            else:
                #button held more than 1 second but less than 2 seconds so start data collection (if it hasn't started)
                #Normal lidar and images mapping
                if dataCollecting == False:
                    setUTCtime()
                    dataCollecting = True
                    dataCollectType = 1
                    filename = ""
                    if os.path.isfile("/home/openmms/firmware/fileNum.txt"):
                        f = open("/home/openmms/firmware/fileNum.txt","r")
                        filename = f.readline()
                        filename = filename.strip('\n')
                        f.close()
                        time.sleep(0.5)
                        lastnum = int(filename)
                        lastnum = lastnum + 1
                        f = open("/home/openmms/firmware/fileNum.txt","w")
                        f.write(str(lastnum))
                        f.close()
                    else:
                        filename = time.strftime("%d_%m_%Y_%H_%M_%S") + "_missing_filename"

                    interval = ""
                    if os.path.isfile("/home/openmms/mms_data/cameras/interval.txt"):
                        f = open("/home/openmms/mms_data/cameras/interval.txt","r")
                        interval = f.readline()
                        interval = interval.strip('\n')
                        f.close()
                        time.sleep(0.5)
                    else:
                        interval = "3"

                    command = "echo '$mmnep0' | sudo -S tcpdump src 192.168.1.201 and udp and port 2368 or port 8308 -B 8192 -s 65535 -w "
                    newCommand = command + "OpenMMS_" + filename + ".pcap" + " &"

                    command2 = "echo '$mmnep0' | sudo -S ttylog -b 115200 -d /dev/serial0 >"
                    newCommand2 = command2 + "OpenMMS_" + filename + ".traj" + " &"
                    #print(newCommand)

                    GPIO.output(redLED, ledOFF)
                    GPIO.output(greenLED, ledOFF)
                    GPIO.output(blueLED, ledOFF)
                    
                    GPIO.output(lidar, ON)
                    
                    for i in range(0,20):
                        GPIO.output(greenLED, ledON)
                        time.sleep(0.5)
                        GPIO.output(greenLED, ledOFF)
                        time.sleep(0.5)

                    checkVideo()
                    if videoOnOff == "1" and cameraInstalled == "1":
                        currentFileName = "OpenMMS_" + filename
                        videoCamera.start_recording(currentFileName + ".h264", quality = video_quality, format = 'h264')
                        #videoCamera.start_recording(currentFileName + ".h264", format = 'h264')

                    tcpdumpSTART = subprocess.run(str(newCommand), shell=True)
                    time.sleep(0.5)
                    ttylogSTART = subprocess.run(str(newCommand2), shell=True)
                    time.sleep(0.5)
                    GPIO.output(blueLED, ledON)

                    if interval == "2":
                        #1.5 second camera interval
                        GPIO.output(camera1, OFF)
                        GPIO.output(camera2, OFF)
                        GPIO.output(camera3, OFF)
                    elif interval == "3":
                        #2.0 second camera interval
                        GPIO.output(camera1, ON)
                        GPIO.output(camera2, OFF)
                        GPIO.output(camera3, OFF)
                    elif interval == "4":
                        #2.5 second camera interval
                        GPIO.output(camera1, OFF)
                        GPIO.output(camera2, ON)
                        GPIO.output(camera3, OFF)
                    elif interval == "5":
                        #3.0 second camera interval
                        GPIO.output(camera1, ON)
                        GPIO.output(camera2, ON)
                        GPIO.output(camera3, OFF)
                    elif interval == "6":
                        #3.5 second camera interval
                        GPIO.output(camera1, OFF)
                        GPIO.output(camera2, OFF)
                        GPIO.output(camera3, ON)
                    elif interval == "7":
                        #4.0 second camera interval
                        GPIO.output(camera1, ON)
                        GPIO.output(camera2, OFF)
                        GPIO.output(camera3, ON)
                    elif interval == "8":
                        #5.0 second camera interval
                        GPIO.output(camera1, OFF)
                        GPIO.output(camera2, ON)
                        GPIO.output(camera3, ON)
