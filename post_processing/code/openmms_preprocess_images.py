# -*- coding: utf-8 -*-

############################################################################
#                         OpenMMS Preprocess Images                        #
############################################################################
# Version: 1.3.0                                                           #
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

import os
import time
import math
import numpy as np
import PIL.Image
import PIL.ExifTags
import cv2
import sys
import shutil
from tqdm import tqdm
from pathlib import Path
import multiprocessing as mp


def gpsFromUTC(year, month, day, hour, min, sec, leapSecs=18): 
#converts UTC to: gpsWeek, secsOfWeek, gpsDay, secsOfDay 
#a good reference is:  http://www.oc.nps.navy.mil/~jclynch/timsys.html 
#This is based on the following facts (see reference above): 
#GPS time is basically measured in (atomic) seconds since  
#January 6, 1980, 00:00:00.0  (the GPS Epoch) 
#The GPS week starts on Saturday midnight (Sunday morning), and runs 
#for 604800 seconds.   

#SOW = Seconds of Week 
#SOD = Seconds of Day 

#Note:  Python represents time in integer seconds, fractions are lost!!!
    gpsEpoch = (1980, 1, 6, 0, 0, 0)
    secsInWeek = 604800
    secsInDay = 86400
    secFract = sec % 1 
    epochTuple = gpsEpoch + (-1, -1, 0) 
    t0 = time.mktime(epochTuple) 
    t = time.mktime((year, month, day, hour, min, sec, -1, -1, 0))
    
    # Note: time.mktime strictly works in localtime and to yield UTC, it should be 
    #       corrected with time.timezone 
    #       However, since we use the difference, this correction is unnecessary. 
    # Warning:  trouble if daylight savings flag is set to -1 or 1 !!! 
    
    t = t + leapSecs + 6*60*60
    tdiff = t - t0 
    gpsSOW = (tdiff % secsInWeek)  + secFract 
    gpsWeek = int(math.floor(tdiff/secsInWeek))  
    gpsDay = int(math.floor(gpsSOW/secsInDay)) 
    gpsSOD = (gpsSOW % secsInDay)  
    return (gpsWeek, gpsSOW, gpsDay, gpsSOD) 


def imagesChunk(currentDir, statusD, lock, chunkID, images, rfFloat, doUndist, undist_type, camera_matrix, camera_dist_coeffs):
    
    descStr = "    Core " + str(chunkID+1)
    if chunkID+1 > 9:
        descStr += " "
    else:
        descStr += "  "
        
    lock.acquire()
    pbar = tqdm(total=len(images),unit=" images",desc=descStr,position=chunkID)
    lock.release()
    
    for imageName in images:
        lock.acquire()
        pbar.update()
        lock.release()
        suffix = ""
        pic = cv2.imread(str(currentDir / imageName), cv2.IMREAD_IGNORE_ORIENTATION | cv2.IMREAD_COLOR)
        h, w = pic.shape[:2]
        width = int(w * rfFloat)
        height = int(h * rfFloat)

        if doUndist:
            newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, camera_dist_coeffs, (w,h), 0)
            mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, camera_dist_coeffs, None, newcameramatrix, (w, h), 5)                
            pic = cv2.remap(pic, mapx, mapy, cv2.INTER_LINEAR)
            suffix += "_u" + undist_type
        
        if rfFloat != 1.0:
            pic = cv2.resize(pic, (width, height), interpolation = cv2.INTER_AREA)
            suffix += "_r" + str(rfFloat)
        
        cv2.imwrite(str(currentDir / ("resized_" + str(rfFloat)) / (imageName[:-4] + suffix + ".JPG")), pic)
            
    statusD[chunkID] = 1
    shouldStop = False
    numProcesses = len(statusD)
    
    while not shouldStop:
        shouldStop = True
        for i in range(0,numProcesses):
            if statusD[i] == 0:
                shouldStop = False
                break
        if shouldStop:
            shouldStop = False
            if chunkID == 0:
                statusD[chunkID] = 2
                shouldStop = True
            else:
                if statusD[chunkID-1] == 2:
                    statusD[chunkID] = 2
                    shouldStop = True
    
    lock.acquire()
    pbar.close()
    lock.release()


def iHeartLidar2():
    print("\n\n                        XXX     XXX                                        ")
    time.sleep(0.1)
    print("              IIIII    X   XX XX   X    L          DDDD     AAAA   RRRRR")
    time.sleep(0.1)
    print("                I     X      X      X   L       i  D   D   A    A  R    R")
    time.sleep(0.1)
    print("                I     X             X   L          D    D  A    A  R    R")
    time.sleep(0.1)
    print("                I      XX         XX    L       i  D    D  AAAAAA  RRRRR")
    time.sleep(0.1)
    print("                I        XX     XX      L       i  D    D  A    A  R  R")
    time.sleep(0.1)
    print("                I          XX XX        L       i  D   D   A    A  R   R")
    time.sleep(0.1)
    print("              IIIII         XXX         LLLLLL  i  DDDD    A    A  R    R")
    time.sleep(0.1)
    print("                             X                                               \n\n")


def OpenMMS1():
    print("\n _________                                 _______ _____  ______ ______  ________")
    time.sleep(0.1)
    print("|\\   ___  \\  ________  ________  ________ |\\   __ \\ __  \\|\\   __ \\ __  \\|\\   ____\\")
    time.sleep(0.1)
    print("\\|\\  \\_|\\  \\|\\   __  \\|\\   __  \\|\\   ___  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\___|_")
    time.sleep(0.1)
    print(" \\|\\  \\\\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\\\|\\  \\|\\  \\|\\__\\|\\  \\|\\  \\|\\__\\|\\  \\|\\_____  \\")
    time.sleep(0.1)
    print("  \\|\\  \\\\|\\  \\|\\   ____\\|\\   ____\\|\\  \\\\|\\  \\|\\  \\|__|\\|\\  \\|\\  \\|__|\\|\\  \\|____|\\  \\")
    time.sleep(0.1)
    print("   \\|\\  \\\\_\\  \\|\\  \\___|\\|\\  \\___|\\|\\  \\\\|\\  \\|\\  \\    \\|\\  \\|\\  \\    \\|\\  \\  __\\_\\  \\")
    time.sleep(0.1)
    print("    \\|\\________\\|\\__\\    \\|\\______\\\\|\\__\\\\|\\__\\|\\__\\    \\|\\__\\|\\__\\    \\|\\__\\|\\_______\\")
    time.sleep(0.1)
    print("     \\|________|\\|___|    \\|_______|\\|___|\\|__|\\|___|    \\|__|\\|___|    \\|__|\\|________|\n\n")
    
    

def main(currentDir):
    
    filesList = []
    foundEventFile = False
    eventFileName = ""
    eventData = []
    kappaData = np.zeros((36,1))
    cam_file = ""
    header_line = ""
    num_cam_files = 0
    
    for indfile in os.listdir(currentDir):
        fileInfo = Path(indfile)
        filename = fileInfo.name
        extension = (fileInfo.suffix).upper()
        
        if extension == ".JPG":
            filesList.append([filename])
            
        elif extension == ".TXT":
            prefix = filename[:10]
            if prefix == "event1_eo_":
                line_count = 0
                foundEventFile = True
                eventFileName = filename
                eventFile = open(currentDir / filename,'r')
                for record in eventFile:
                    line_count += 1
                    #read header line
                    if line_count > 1:
                        if len(record) > 1:
                            eventData.append(record)
                    else:
                        header_line = record
                eventFile.close()
            
        elif extension == ".CAM":
            cam_file = currentDir / filename
            num_cam_files += 1
    
    filesList.sort(key=lambda x: x[0])
    
    eventImageCount = len(eventData)
    
    numCores = mp.cpu_count() - 1
    
    designNum = 0
    
    foundStuff = False
    chunkIndex = [0]
    if eventImageCount == 0:
        print("\n\n****************************************************************")
        print("************ NO CAMERA EVENTS INFORMATION WAS FOUND ************")
        print("****************************************************************\n\n")
    else:
        foundStuff = True
        if numCores > 0:
            pass
        else:
            print("\n\n****************************************************************")
            print("************* COULD NOT READ CPU INFORMATION, ODD? *************")
            print("****************************************************************\n\n")
            foundStuff = False      
    
    if foundStuff:

        OpenMMS1()
        
        if foundEventFile:
            print("FOUND EVENTS FILE: " + eventFileName)
            print("  Recorded Events: " + str(eventImageCount))
        
        else:
            print("*** ERROR: NO EVENT EO FILE FOUND ***")
            return
            
        fileOut = open(currentDir / "images_exif_times_DZ.csv", 'w')
        fileOut.write("Image,EXIF,Event,Time Match,Digital Zoom\n")
        count = 0
        numImages = len(filesList)
        lastEXIFtime = 0.0
        lastFoundTimeStamp = 0.0
        timeIncrement = 0.0

        images_times_DZ = []
        
        badCount = 0
        badImages = []
        img_width = 0
        img_height = 0
        first_img_width = 0
        first_img_height = 0
        sizeStr = ""
        
        #ILCE-6000_16mm camera internal parameters (from Pix4D database in Pix4D/OpenCV camera model system)
        cameraPixelSize = 0.00391667                                    #millimetres
        cameraWidth = 6000.0                                            #pixels
        cameraHeight = 4000.0                                           #pixels
        cameraFocal = 15.803372857530 / cameraPixelSize                 #pixels
        cameraPPx = cameraWidth / 2.0 # - 29.0                            #pixels
        cameraPPy = cameraHeight / 2.0                                  #pixels
        
        #Radial Lens Distortion Coeffs
        cameraK1 = -0.068158781729                                      #unitless
        cameraK2 =  0.082860582820                                      #unitless
        cameraK3 =  0.010592147042                                      #unitless
        #Tangential Lens Distortion Coeffs
        cameraP1 = -0.000062573839                                      #unitless
        cameraP2 =  0.000779625408                                      #unitless
        
        # Define camera matrix K (OpenCV definition)
        camera_matrix = np.array([[cameraFocal, 0., cameraPPx],
                                  [0., cameraFocal, cameraPPy],
                                  [0., 0., 1.]])
        
        # Define distortion coefficients d [K1,K2,P1,P2,K3]
        camera_dist_coeffs = np.array([cameraK1,cameraK2,cameraP1,cameraP2,cameraK3])
        
        counter = 0
        for filenameList in filesList:
            filename = filenameList[0]
            imgdata = PIL.Image.open(currentDir / filename)
            
            foundEXIF = True
            try:
                exif = {
                    PIL.ExifTags.TAGS[k]: v
                    for k, v in imgdata._getexif().items()
                    if k in PIL.ExifTags.TAGS
                }
            except:
                foundEXIF = False
                
            img_width, img_height = imgdata.size
            imgdata.close()
            
            if count == 0:
                first_img_width = img_width
                first_img_height = img_height
                sizeStr = str(img_width) + "x" + str(img_height) + " pixels"
            else:
                if img_width != first_img_width or img_height != first_img_height:
                    first_img_width = -1
                    first_img_height = -1
                    sizeStr = "\n*** ERROR: VARYING IMAGE SIZES (PLEASE EXAMINE THE IMAGES BEFORE TRYING AGAIN) ***\n"
                    return
            
            time_stamp = 0.0
            digitalZoom = 1.0
            if foundEXIF:
                digitalZoomR = exif.get('DigitalZoomRatio', 'NONE')
                digitalZoom1 = str(digitalZoomR).replace("("," ")
                digitalZoom2 = digitalZoom1.replace(")"," ")
                digitalZoomNum = digitalZoom2.split(",")[0]
                digitalZoomDen = digitalZoom2.split(",")[1]
                digitalZoom = float(digitalZoomNum) / float(digitalZoomDen)
                timeHMS = exif.get('DateTimeOriginal', 'NONE')
                
                timeHMS1 = timeHMS.replace(" ", ":")
                timeHMS2 = timeHMS1.split(":")
                gpsw,gpsSOW,gpsday,gpsSOD = gpsFromUTC(int(timeHMS2[0]),int(timeHMS2[1]),int(timeHMS2[2]),int(timeHMS2[3]),int(timeHMS2[4]),int(timeHMS2[5]),1)
                time_stamp = gpsw * 604800.0 + gpsSOW
                lastFoundTimeStamp = time_stamp
                timeIncrement = 0.0
            else:
                timeIncrement += 1.0
                time_stamp = lastFoundTimeStamp + timeIncrement
        
            filesList[counter].append(time_stamp)
            counter += 1
            
            images_times_DZ.append([filename,time_stamp,digitalZoom])
        
        images_times_DZ.sort(key=lambda x: x[1])
        sorted_filesList = sorted(filesList, key=lambda x: x[1])
        
        filesList = []
        for i in range(0,len(sorted_filesList)):
            filesList.append(sorted_filesList[i][0])
        
        dups_str = ""
        for i in range(0,len(images_times_DZ)):
            filename = images_times_DZ[i][0]
            time_stamp = images_times_DZ[i][1]
            digitalZoom = images_times_DZ[i][2]
            
            if time_stamp == lastEXIFtime:
                badCount += 1
                badImages.append(filename)
                dups_str += ("             " + filename + " , " + str(time_stamp) + " , Digital Zoom: " + str(digitalZoom) + "\n")
                
            lastEXIFtime = time_stamp
            
            fileOut.write(str(filename) + ",")
            fileOut.write(str(time_stamp) + ",")
            eventTime = ""
            try:
                eventTime = str(round(float(eventData[count].split(",")[0])))
            except:
                eventTime = "?"
            fileOut.write(eventTime + ",")
            
            timesMatch = "NO"
            if str(gpsSOD) == eventTime:
                timesMatch = "YES"
            
            fileOut.write(timesMatch + ",")
            fileOut.write(str(digitalZoom))
            
            if count < numImages:
                fileOut.write("\n")
                
            count += 1
        fileOut.close()
        
        badCountStr = str(badCount)
        if badCount == 0:
            badCountStr = "NONE"
            
        print("\nIMAGES ANALYSIS")
        print("  Images Found in Current Directory:  " + str(numImages))
        print("         Size of All Images (W x H):  " + sizeStr)
        print("             Duplicate Images Found:  " + badCountStr + "\n")            
        
        if badCount > 0:
            print(dups_str)
            print("  Eliminating the duplicate images would leave " + str(numImages - badCount) + " images remaining")
            moveBad = input("\n  DO YOU WANT TO MOVE THESE IMAGES TO A \'DUPLICATES\' DIRECTORY? (y/N) : ")
            if str(moveBad) == "y" or str(moveBad) == "Y":
                if os.path.isdir(currentDir / "DUPLICATES") == False:
                    os.mkdir(currentDir / "DUPLICATES")
                    time.sleep(0.1) 
                for i in reversed(badImages):
                    shutil.move(currentDir / str(i),currentDir / "DUPLICATES" / str(i))
                    filesList.remove(str(i))
                    numImages -= 1
                time.sleep(0.1)
                print("  *** DUPLICATE IMAGES HAVE BEEN REMOVED! ***")
                
        run = True
        while run:
            designNum = numImages // numCores
            if designNum < 40: 
                numCores -= 1
                if numCores == 0:
                    numCores = 1
                    run = False
            else:
                run = False
                
        indexCount = designNum
        for i in range(1,numCores):
            chunkIndex.append(indexCount)
            indexCount += designNum
        chunkIndex.append(numImages)
        
        didResize = False
        
        resize = input("\nDO YOU WANT TO COPY AND RESIZE, AND/OR UNDISTORT, THE IMAGES TO A NEW DIRECTORY? (y/N) : ")
        suffix = ""
        if str(resize) == "y" or str(resize) == "Y":
            resizeFactor = input("\n    WHAT IS THE RESIZE FACTOR (RF)? [0.1 <= RF <= 1.0] : ")
            if float(resizeFactor) >= 0.1 and float(resizeFactor) <= 1.0:
                rfStr = str(np.round(float(resizeFactor),3))
                rfFloat = float(rfStr)
                doUndist = False
                undist_type = ""
                suffix = "_r" + str(rfFloat)
                good_cam = False
                new_cameraFocal = 0
                new_cameraPPx = 0
                new_cameraPPy = 0
                new_cameraK1 = 0
                new_cameraK2 = 0
                new_cameraK3 = 0
                new_cameraP1 = 0
                new_cameraP2 = 0
                if num_cam_files == 1:
                    cam_data = open(cam_file,'r')
                    record_count = 0
                    for record in cam_data:
                        record_count += 1
                        
                        if record_count == 1:
                            if "PIX4D CAMERA CALIBRATION FILE" in record.upper():
                                good_cam = True
                            else:
                                break
                        else:
                            line_elements = record.strip('\n').split(' ')
                            if len(line_elements) == 2:
                                if line_elements[0].upper() == "F":
                                    new_cameraFocal = float(line_elements[1]) / cameraPixelSize
                                elif line_elements[0].upper() == "PX":
                                    new_cameraPPx = float(line_elements[1]) / cameraPixelSize
                                elif line_elements[0].upper() == "PY":
                                    new_cameraPPy = float(line_elements[1]) / cameraPixelSize
                                elif line_elements[0].upper() == "K1":
                                    new_cameraK1 = float(line_elements[1])
                                elif line_elements[0].upper() == "K2":
                                    new_cameraK2 = float(line_elements[1])
                                elif line_elements[0].upper() == "K3":
                                    new_cameraK3 = float(line_elements[1])
                                elif line_elements[0].upper() == "T1":
                                    new_cameraP1 = float(line_elements[1])
                                elif line_elements[0].upper() == "T2":
                                    new_cameraP2 = float(line_elements[1])
                                        
                    cam_data.close()
                
                undist = ""
                if good_cam:
                    print("\n    FOUND .CAM FILE: " + (cam_file.name))
                    undist = input("    DO YOU WANT TO UNDISTORT THE IMAGES USING THE CAMERA MODEL FOUND IN THE .CAM FILE? (y/N) : ")
                    if str(undist) == "y" or str(undist) == "Y":
                        doUndist = True
                        undist_type = "CAM"
                        suffix = "_uCAM" + suffix
                        cameraFocal = new_cameraFocal
                        cameraPPx = new_cameraPPx
                        cameraPPy = new_cameraPPy
                        cameraK1 = new_cameraK1
                        cameraK2 = new_cameraK2
                        cameraK3 = new_cameraK3
                        cameraP1 = new_cameraP1
                        cameraP2 = new_cameraP2
                    else:
                        undist = ""
                    
                if undist == "":
                    undist = input("\n    DO YOU WANT TO UNDISTORT THE IMAGES USING THE GENERIC SONY A6000-16mm LENS MODEL? (y/N) : ")
                    if str(undist) == "y" or str(undist) == "Y":
                        doUndist = True
                        undist_type = "GEN"
                        suffix = "_uGEN" + suffix
                print()
                
                if os.path.isdir(currentDir / ("resized_" + rfStr)) == False:
                    os.mkdir(currentDir / ("resized_" + rfStr))
                    time.sleep(0.1)
                
                procs = []
                
                manager = mp.Manager()
                statusD = manager.dict()
            
                lock = mp.Lock()
                
                for i in range(0,len(chunkIndex)-1):
                    statusD[i] = 0
                    
                for i in range(0,len(chunkIndex)-1):
                    proc1 = mp.Process(target=imagesChunk,args=(currentDir, statusD, lock, i, filesList[chunkIndex[i]:chunkIndex[i+1]],rfFloat,doUndist,undist_type,camera_matrix,camera_dist_coeffs))                    
                    procs.append(proc1)
                    
                for iproc in procs:
                    iproc.start()
                
                for iproc in procs:
                    iproc.join()
                
                didResize = True
                if doUndist:
                    print("\n  *** IMAGES HAVE BEEN COPIED, RESIZED, AND UNDISTORTED! ***")
                else:
                    print("\n  *** IMAGES HAVE BEEN COPIED AND RESIZED! ***")
        
        
        if foundEventFile:
            createPix = input("\nDO YOU WANT TO CREATE AN IMAGES EO FILE FOR PIX4D? (y/N) : ")
            if str(createPix) == "y" or str(createPix) == "Y":
                filter1start = 1
                filter1end = 10000
                filter2start = 1
                filter2end = 10000
                filter3start = 1
                filter3end = 10000
                filterEvents = input("\n    DO YOU WANT TO FILTER THE EO FILE BASED ON EVENT NUMBERS? (y/N) : ")
                if str(filterEvents) == "y" or str(filterEvents) == "Y":
                    filter1start = input("\n      Enter first start Event number (integer) : ")
                    if filter1start == "":
                        filter1start = -1
                    filter1start = int(filter1start)
                    filter1end = input("      Enter first end Event number (integer) : ")
                    if filter1end == "":
                        filter1end = -1
                    filter1end = int(filter1end)
                    filter2start = input("\n      Enter second start Event number (integer) : ")
                    if filter2start == "":
                        filter2start = -1
                    filter2start = int(filter2start)
                    filter2end = input("      Enter second end Event number (integer) : ")
                    if filter2end == "":
                        filter2end = -1
                    filter2end = int(filter2end)
                    filter3start = input("\n      Enter third start Event number (integer) : ")
                    if filter3start == "":
                        filter3start = -1
                    filter3start = int(filter3start)
                    filter3end = input("      Enter third end Event number (integer) : ")
                    if filter3end == "":
                        filter3end = -1
                    filter3end = int(filter3end)
                    
                    eventFile2 = open(currentDir / eventFileName,'r')
                    eventCount = 0
                    for record in eventFile2:
                        eventCount += 1
                        if len(record) > 1:
                            if ((eventCount >= filter1start and eventCount <= filter1end) or (eventCount >= filter2start and eventCount <= filter2end) or (eventCount >= filter3start and eventCount <= filter3end)):
                                kappa = float(record.split(",")[6])
                                if kappa > -180 and kappa <= -170:
                                    kappaData[0,0] += 1
                                elif kappa > -170 and kappa <= -160:
                                    kappaData[1,0] += 1
                                elif kappa > -160 and kappa <= -150:
                                    kappaData[2,0] += 1
                                elif kappa > -150 and kappa <= -140:
                                    kappaData[3,0] += 1
                                elif kappa > -140 and kappa <= -130:
                                    kappaData[4,0] += 1
                                elif kappa > -130 and kappa <= -120:
                                    kappaData[5,0] += 1
                                elif kappa > -120 and kappa <= -110:
                                    kappaData[6,0] += 1
                                elif kappa > -110 and kappa <= -100:
                                    kappaData[7,0] += 1
                                elif kappa > -100 and kappa <= -90:
                                    kappaData[8,0] += 1
                                elif kappa > -90 and kappa <= -80:
                                    kappaData[9,0] += 1
                                elif kappa > -80 and kappa <= -70:
                                    kappaData[10,0] += 1
                                elif kappa > -70 and kappa <= -60:
                                    kappaData[11,0] += 1
                                elif kappa > -60 and kappa <= -50:
                                    kappaData[12,0] += 1
                                elif kappa > -50 and kappa <= -40:
                                    kappaData[13,0] += 1
                                elif kappa > -40 and kappa <= -30:
                                    kappaData[14,0] += 1
                                elif kappa > -30 and kappa <= -20:
                                    kappaData[15,0] += 1
                                elif kappa > -20 and kappa <= -10:
                                    kappaData[16,0] += 1
                                elif kappa > -10 and kappa <= 0:
                                    kappaData[17,0] += 1
                                elif kappa > 0 and kappa <= 10:
                                    kappaData[18,0] += 1
                                elif kappa > 10 and kappa <= 20:
                                    kappaData[19,0] += 1
                                elif kappa > 20 and kappa <= 30:
                                    kappaData[20,0] += 1
                                elif kappa > 30 and kappa <= 40:
                                    kappaData[21,0] += 1
                                elif kappa > 40 and kappa <= 50:
                                    kappaData[22,0] += 1
                                elif kappa > 50 and kappa <= 60:
                                    kappaData[23,0] += 1
                                elif kappa > 60 and kappa <= 70:
                                    kappaData[24,0] += 1
                                elif kappa > 70 and kappa <= 80:
                                    kappaData[25,0] += 1
                                elif kappa > 80 and kappa <= 90:
                                    kappaData[26,0] += 1
                                elif kappa > 90 and kappa <= 100:
                                    kappaData[27,0] += 1
                                elif kappa > 100 and kappa <= 110:
                                    kappaData[28,0] += 1
                                elif kappa > 110 and kappa <= 120:
                                    kappaData[29,0] += 1
                                elif kappa > 120 and kappa <= 130:
                                    kappaData[30,0] += 1
                                elif kappa > 130 and kappa <= 140:
                                    kappaData[31,0] += 1
                                elif kappa > 140 and kappa <= 150:
                                    kappaData[32,0] += 1
                                elif kappa > 150 and kappa <= 160:
                                    kappaData[33,0] += 1
                                elif kappa > 160 and kappa <= 170:
                                    kappaData[34,0] += 1
                                elif kappa > 170 and kappa <= 180:
                                    kappaData[35,0] += 1
                    eventFile2.close()
                    
                    filter1low = -180
                    filter1high = 180
                    filter2low = -180
                    filter2high = 180
                    filter3low = -180
                    filter3high = 180
                    filter4low = -180
                    filter4high = 180
                    filter5low = -180
                    filter5high = 180
                    filterKappa = input("\n    DO YOU WANT TO FILTER THE EO FILE BASED ON KAPPA ANGLES? (y/N) : ")
                    
                    if str(filterKappa) == "y" or str(filterKappa) == "Y":
                        print("\n      KAPPA VALUES SUMMARY FOR IMAGES:")
                        for i in range(-18,18):
                            print("        " + str(i*10) + " to " + str((i+1)*10) + ": " + str(int(kappaData[i+18,0])))
                        
                        filter1low = input("\n      Enter first lower Kappa bound (integer) : ")
                        if filter1low == "":
                            filter1low = -500
                        filter1low = int(filter1low)
                        filter1high = input("      Enter first upper Kappa bound (integer) : ")
                        if filter1high == "":
                            filter1high = -500
                        filter1high = int(filter1high)
                        filter2low = input("\n      Enter second lower Kappa bound (integer) : ")
                        if filter2low == "":
                            filter2low = -500
                        filter2low = int(filter2low)
                        filter2high = input("      Enter second upper Kappa bound (integer) : ")
                        if filter2high == "":
                            filter2high = -500
                        filter2high = int(filter2high)
                        filter3low = input("\n      Enter third lower Kappa bound (integer) : ")
                        if filter3low == "":
                            filter3low = -500
                        filter3low = int(filter3low)
                        filter3high = input("      Enter third upper Kappa bound (integer) : ")
                        if filter3high == "":
                            filter3high = -500
                        filter3high = int(filter3high)
                        filter4low = input("\n      Enter fourth lower Kappa bound (integer) : ")
                        if filter4low == "":
                            filter4low = -500
                        filter4low = int(filter4low)
                        filter4high = input("      Enter fourth upper Kappa bound (integer) : ")
                        if filter4high == "":
                            filter4high = -500
                        filter4high = int(filter4high)
                        filter5low = input("\n      Enter fifth lower Kappa bound (integer) : ")
                        if filter5low == "":
                            filter5low = -500
                        filter5low = int(filter5low)
                        filter5high = input("      Enter fifth upper Kappa bound (integer) : ")
                        if filter5high == "":
                            filter5high = -500
                        filter5high = int(filter5high)
                        
                    imageNum = 0
                    pix4dOut = open(currentDir / "images_pix4d.csv", 'w')
                    colorizeOut = open(currentDir / "images_colorize.csv", 'w') 
                    pix4dOut.write("#Image,X,Y,Z,Omega,Phi,Kappa,Hor.Acc,Vert.Acc\n")
                    colorizeOut.write("#Time,Image,X,Y,Z,Omega,Phi,Kappa\n")
                    for filename in filesList:
                        eventRecord = eventData[imageNum].split(",")
                        eventCount = imageNum + 1
                        if ((eventCount >= filter1start and eventCount <= filter1end) or (eventCount >= filter2start and eventCount <= filter2end) or (eventCount >= filter3start and eventCount <= filter3end)):
                            if ((float(eventRecord[6]) >= filter1low and float(eventRecord[6]) <= filter1high) or (float(eventRecord[6]) >= filter2low and float(eventRecord[6]) <= filter2high) or (float(eventRecord[6]) >= filter3low and float(eventRecord[6]) <= filter3high) or (float(eventRecord[6]) >= filter4low and float(eventRecord[6]) <= filter4high) or (float(eventRecord[6]) >= filter5low and float(eventRecord[6]) <= filter5high)):
                                pix4dOut.write(filename + "," + eventRecord[1] + "," + eventRecord[2] + "," + eventRecord[3] + "," + eventRecord[4] + "," + eventRecord[5] + "," + str(float(eventRecord[6])) + ",0.05,0.05\n")        
                                colorizeOut.write(eventRecord[0] + "," + filename + "," + eventRecord[1] + "," + eventRecord[2] + "," + eventRecord[3] + "," + eventRecord[4] + "," + eventRecord[5] + "," + str(float(eventRecord[6])) + "\n")        
                        imageNum += 1
                    print("\n*** PIX4D IMAGES FILE AND COLORIZATION INFO. FILE HAVE BEEN CREATED! ***")
                    pix4dOut.close()
                    colorizeOut.close()
                    
                else:
                    imageNum = 0
                    pix4dOut = open(currentDir / "images_pix4d.csv", 'w')
                    colorizeOut = open(currentDir / "images_colorize.csv", 'w') 
                    pix4dOut.write("#Image,X,Y,Z,Omega,Phi,Kappa,Hor.Acc,Vert.Acc\n")
                    colorizeOut.write("#Time,Image,X,Y,Z,Omega,Phi,Kappa\n")
                    for filename in filesList:
                        eventRecord = eventData[imageNum].split(",")
                        pix4dOut.write(filename + "," + eventRecord[1] + "," + eventRecord[2] + "," + eventRecord[3] + "," + eventRecord[4] + "," + eventRecord[5] + "," + str(float(eventRecord[6])) + ",0.05,0.05\n")        
                        colorizeOut.write(eventRecord[0] + "," + filename + "," + eventRecord[1] + "," + eventRecord[2] + "," + eventRecord[3] + "," + eventRecord[4] + "," + eventRecord[5] + "," + str(float(eventRecord[6])) + "\n")        
                        imageNum += 1
                    print("\n*** PIX4D IMAGES FILE AND COLORIZATION INFO. FILE HAVE BEEN CREATED! ***")
                    pix4dOut.close()
                    colorizeOut.close()
            
            camera_io = open(currentDir / "camera_io.csv", 'w')
            camera_io.write(str(cameraPixelSize) + ",")
            camera_io.write(str(cameraWidth) + ",")
            camera_io.write(str(cameraHeight) + ",")
            camera_io.write(str(cameraFocal) + ",")
            camera_io.write(str(cameraPPx) + ",")
            camera_io.write(str(cameraPPy) + ",")
            camera_io.write(str(cameraK1) + ",")
            camera_io.write(str(cameraK2) + ",")
            camera_io.write(str(cameraK3) + ",")
            camera_io.write(str(cameraP1) + ",")
            camera_io.write(str(cameraP2) + "\n")
            camera_io.close()
            
            if didResize:
                imageNum = 0
                colorizeOut = open(currentDir / ("resized_" + rfStr) / "images_colorize.csv", 'w') 
                colorizeOut.write("#Time,Image,X,Y,Z,Omega,Phi,Kappa\n")
                for filename in filesList:
                    fileInfo = Path(currentDir / filename)
                    extension = fileInfo.suffix
                    imageName = fileInfo.stem
                    eventRecord = eventData[imageNum].split(",")
                    colorizeOut.write(eventRecord[0] + "," + imageName + suffix + extension + "," + eventRecord[1] + "," + eventRecord[2] + "," + eventRecord[3] + "," + eventRecord[4] + "," + eventRecord[5] + "," + str(float(eventRecord[6])) + "\n")        
                    imageNum += 1
                print("\n*** COLORIZATION INFO. FILE FOR THE RESIZED IMAGES HAS BEEN CREATED! ***\n")
                colorizeOut.close()
                
                camera_io = open(currentDir / ("resized_" + rfStr) / "camera_io.csv", 'w')
                camera_io.write(str(cameraPixelSize) + ",")
                camera_io.write(str(cameraWidth) + ",")
                camera_io.write(str(cameraHeight) + ",")
                camera_io.write(str(cameraFocal) + ",")
                camera_io.write(str(cameraPPx) + ",")
                camera_io.write(str(cameraPPy) + ",")
                camera_io.write(str(cameraK1) + ",")
                camera_io.write(str(cameraK2) + ",")
                camera_io.write(str(cameraK3) + ",")
                camera_io.write(str(cameraP1) + ",")
                camera_io.write(str(cameraP2) + "\n")
                camera_io.close()
        
            #all events related to associated images file
            imageNum = 0
            allOut = open(currentDir / "images_matched_to_events.csv", 'w') 
            allOut.write("#LA_X,LA_Y,LA_Z,Bore_X,Bore_Y,Bore_Z\n")
            allOut.write(header_line)
            allOut.write("#Event,Time,Image,X,Y,Z,Omega,Phi,Kappa,Heading,HorVel,UpVel\n")
            for filename in filesList:
                fileInfo = Path(currentDir / filename)
                extension = fileInfo.suffix
                imageName = fileInfo.stem
                eventRecord = eventData[imageNum].split(",")
                horVel = str(np.round(np.sqrt((float(eventRecord[12]))**2 + (float(eventRecord[13]))**2),2))
                allOut.write(str(imageNum+1) + "," + eventRecord[0] + "," + imageName + extension + "," + eventRecord[1] + "," + eventRecord[2] + "," + eventRecord[3] + "," + eventRecord[4] + "," + eventRecord[5] + "," + str(float(eventRecord[6])) + "," + str(float(eventRecord[7])) + "," + horVel + "," + str(float(eventRecord[14])) + "\n")        
                imageNum += 1
            allOut.close()
        
        iHeartLidar2()

##### command line start
if __name__ == "__main__":
    np.set_printoptions(suppress=True)
    print("\nPreprocessing Images Started...")
    currentDir = Path(str(sys.argv[1]))
    main(currentDir)
