# -*- coding: utf-8 -*-

############################################################################
#                            OpenMMS PCAP Check                            #
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

import time
import sys
import numpy as np
import os
import struct
import binascii
import matplotlib.pyplot as plt
from datetime import datetime
from tqdm import tqdm
from pathlib import Path
import multiprocessing as mp

totalpacs = 0
badpacs = 0   
 
def iHeartLidar2(logFile):
    print("\n                        XXX     XXX                                        ")
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
    
    logFile.write("\n                        XXX     XXX                                        \n")
    logFile.write("              IIIII    X   XX XX   X    L          DDDD     AAAA   RRRRR\n")
    logFile.write("                I     X      X      X   L       i  D   D   A    A  R    R\n")
    logFile.write("                I     X             X   L          D    D  A    A  R    R\n")
    logFile.write("                I      XX         XX    L       i  D    D  AAAAAA  RRRRR\n")
    logFile.write("                I        XX     XX      L       i  D    D  A    A  R  R\n")
    logFile.write("                I          XX XX        L       i  D   D   A    A  R   R\n")
    logFile.write("              IIIII         XXX         LLLLLL  i  DDDD    A    A  R    R\n")
    logFile.write("                             X                                               \n\n")


def OpenMMS1():
    print(" _________                                 _______ _____  ______ ______  ________")
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
    
    
def OpenMMS1_log(logFile):        
    logFile.write(" _________                                 _______ _____  ______ ______  ________\n")
    logFile.write("|\\   ___  \\  ________  ________  ________ |\\   __ \\ __  \\|\\   __ \\ __  \\|\\   ____\\\n")
    logFile.write("\\|\\  \\_|\\  \\|\\   __  \\|\\   __  \\|\\   ___  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\___|_\n")
    logFile.write(" \\|\\  \\\\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\\\|\\  \\|\\  \\|\\__\\|\\  \\|\\  \\|\\__\\|\\  \\|\\_____  \\\n")
    logFile.write("  \\|\\  \\\\|\\  \\|\\   ____\\|\\   ____\\|\\  \\\\|\\  \\|\\  \\|__|\\|\\  \\|\\  \\|__|\\|\\  \\|____|\\  \\\n")
    logFile.write("   \\|\\  \\\\_\\  \\|\\  \\___|\\|\\  \\___|\\|\\  \\\\|\\  \\|\\  \\    \\|\\  \\|\\  \\    \\|\\  \\  __\\_\\  \\\n")
    logFile.write("    \\|\\________\\|\\__\\    \\|\\______\\\\|\\__\\\\|\\__\\|\\__\\    \\|\\__\\|\\__\\    \\|\\__\\|\\_______\\\n")
    logFile.write("     \\|________|\\|___|    \\|_______|\\|___|\\|__|\\|___|    \\|__|\\|___|    \\|__|\\|________|\n\n")
    

def LL2NE(lat1,lon1,lat2,lon2):
    lat1 = np.radians(lat1)
    lon1 = np.radians(lon1)
    lat2 = np.radians(lat2)
    lon2 = np.radians(lon2)
    R = 6378000.0
    dLat = lat2 - lat1
    dLon = lon2 - lon1
    
    scR1 = np.cos(lat1) * R
    scR2 = np.cos(lat2) * R
    scR = (scR1 + scR2) / 2.0
    
    dN = dLat * R
    dE = dLon * scR

    return dN, dE


#PCAP FILE HEADER (24 bytes)
def readPcapFileHeader(pcap, bytesRead):

    b = bytearray(pcap.read(24))
    bytesRead += 24

    return bytesRead

#PCAP PACKET HEADER (16 bytes)
def readPcapPacketHeader(pcap, bytesRead, numDataPackets, skippedDataPackets, numPosPackets, badPosPackets, numUnrecPackets, foundGoodPos, unpackL, unpackB):
    #check to make sure that 16 bytes of data exist before the end of the file
    global debug
    checkData = bytearray(pcap.read(16))
    
    continueReading = True
    
    packetType = 0
    packetData = None
    byteSize = 0
    
    if len(checkData) != 16:
        continueReading = False
    else:
        b = checkData[8:12]
        incl_length = struct.unpack('<L', b)[0]    #number of octets of packet saved in file

        try:
            if incl_length == 554:      #Position packet
                packetData = pcap.read(554)
                pps_statusi = unpackB(packetData[244:245])[0]
                try:
                    GPRMCs = str(packetData[248:512], 'utf-8').rstrip(' \t\r\n\0')
                    GPRMCi = GPRMCs.split(",")
                except:
                    GPRMCi = []
                if len(GPRMCi) > 8:
                    if GPRMCi[2].upper() == "A":
                        hh = int(GPRMCi[1][:2]) * 3600
                        mimi = int(GPRMCi[1][2:4]) * 60
                        ss = int(GPRMCi[1][4:6])
                        nmeaTime = np.floor(hh + mimi + ss)
                        timeStamp1 = np.round((hh + float(unpackL(packetData[240:244])[0]) / 1000000.0),1)
                        if not foundGoodPos:
                            if np.floor(timeStamp1) == nmeaTime and pps_statusi == 2:
                                packetType = 1
                                foundGoodPos = True
#                                debug.write("\nFound First Time Match\n\n")
                            else:
#                                debug.write("*\n")
                                pass
                        else:
                            if np.abs(timeStamp1 - nmeaTime) < 2 and pps_statusi == 2:
                                packetType = 1
#                               debug.write(str(nmeaTime) + " " + str(timeStamp1) + "\n")
                            else:
#                               debug.write("-\n")
                                foundGoodPos = False
                    else:
                        badPosPackets += 1
                else:
                    badPosPackets += 1
                
                byteSize = 554
                numPosPackets += 1

            elif incl_length == 1248:   #Data packet
                packetData = pcap.read(1248)
#                timeStamp = float(unpackL(packetData[1242:1246])[0])
                byteSize = 1248
                numDataPackets += 1
                
                if foundGoodPos:
                    packetType = 2
                else:
                    skippedDataPackets += 1
            else:
                numUnrecPackets += 1
                    
        except:
            print("\n\n   **********************************************************")
            print("   ***** AN ERROR OCCURRED WHILE READING THE .PCAP FILE *****")
            print("   **********************************************************\n")
            continueReading = False

    bytesRead += 16    

    return continueReading, packetType, byteSize, bytesRead, packetData, numDataPackets, skippedDataPackets, numPosPackets, badPosPackets, numUnrecPackets, foundGoodPos
    

def parseChunk(statusD, lock, chunkID, packetOrder, miscD, epochD, latD, lonD, pps_statusD, timeStamp1D, NMEAdataIndexD, timeStampD, returnModeD, sensorTypeD, goodPtsD):
    
    hemiEmun = {"N":1, "S":-1, "E":1, "W":-1}
    
    NMEAdataIndex = -1
    pps_status = [0,0,0,0]
    num_points = 0
    
    num_bad_point_blocks = 0
    
    unpackL = struct.Struct('<L').unpack
    unpackB = struct.Struct('<B').unpack
    unpackH = struct.Struct('<H').unpack
    unpackHB = struct.Struct('<HB').unpack
    
    previousAzi = 36100.0
    previousTime = 0.0
    rotRate = -1.0
    dataPacketCount = 0
    
    epochL = []
    latL = []
    lonL = []
    timeStamp1L = []
    NMEAdataIndexL = []
    timeStampL = []
    returnModeL = []
    sensorTypeL = []
    goodPtsL = []
    
    descStr = "Core " + str(chunkID+1)
    if chunkID+1 > 9:
        descStr += " "
    else:
        descStr += "  "
        
    lock.acquire()
    pbar = tqdm(total=len(packetOrder),unit=" packets",desc=descStr,position=chunkID)
    lock.release()
    
    for i in range(0,len(packetOrder)):
        lock.acquire()
        pbar.update()
        lock.release()
        data = packetOrder[i][1]
        
        #position packet
        if packetOrder[i][0] == 1:
            timeStamp1 = float(unpackL(data[240:244])[0])
            pps_statusi = unpackB(data[244:245])[0]
            pps_status[pps_statusi] += 1
            GPRMCs = str(data[248:512], 'utf-8').rstrip(' \t\r\n\0')
            GPRMCi = GPRMCs.split(",")
            if len(GPRMCi) > 8:
                yyyy = int(GPRMCi[9][-2:]) + 2000
                mm = int(GPRMCi[9][2:4])
                dd = int(GPRMCi[9][:2])
                hh = int(GPRMCi[1][:2])
                mimi = int(GPRMCi[1][2:4])
                ss = int(GPRMCi[1][4:6])
                epoch = datetime(yyyy,mm,dd,hh,mimi,ss)
                lat = hemiEmun[GPRMCi[4]] * (np.float32(GPRMCi[3][:2]) + np.float32(GPRMCi[3][2:]) / 60.0)
                lon = hemiEmun[GPRMCi[6]] * (np.float32(GPRMCi[5][:3]) + np.float32(GPRMCi[5][3:]) / 60.0)
                epochL.append(epoch)
                latL.append(lat)
                lonL.append(lon)
                timeStamp1L.append(timeStamp1)
                NMEAdataIndex += 1

        #points (data) packet
        elif packetOrder[i][0] == 2:
            if NMEAdataIndex > -1:
                dataPacketCount += 1
                goodPts = 0
                timeStamp = float(unpackL(data[1242:1246])[0])
                returnMode = unpackB(data[1246:1247])[0]   #55 = Single Return(Strongest), 56 = Single Return (Last), 57 = Dual Returns (Strongest and Last)
                sensorType = unpackB(data[1247:1248])[0]   #33 = Velodyne HDL-32E, 34 = Velodyne VLP-16/Puck Lite

                if rotRate == -1.0:
                    if dataPacketCount > 8000:       #calc rot rate after ~ 10 seconds of data collection
                        azi = float(unpackH(data[44:46])[0])
                        rotDiff = azi - previousAzi
                        if rotDiff > 0.0:
                            dt = timeStamp - previousTime
                            if dt > 0:
                                rotRate = (rotDiff * 10000.0) / (dt * 360.0)  #in Hertz (Hz))
                            else:
                                previousTime = timeStamp
                                previousAzi = azi
                        else:
                            previousTime = timeStamp
                            previousAzi = azi

                for j in range(0,12):
                    o1 = int(j*100)
                    if binascii.hexlify(data[42+o1:44+o1]).decode('utf-8').upper() == "FFEE":
                        for k in range(0,2):
                            o2 = o1 + int(k*48)
                            for m in range(0,16):
                                o3 = o2 + int(m*3)
                                dist,intensity = unpackHB(data[46+o3:49+o3])
                                if not dist:
                                    goodPts += 1
                                    num_points += 1
                    else:
                        num_bad_point_blocks += 1
                        
                NMEAdataIndexL.append(NMEAdataIndex)
                timeStampL.append(timeStamp)
                returnModeL.append(returnMode)
                sensorTypeL.append(sensorType)
                goodPtsL.append(goodPts)
    
    miscD[chunkID] = [num_points, 0, num_bad_point_blocks, rotRate]
    epochD[chunkID] = epochL
    latD[chunkID] = latL
    lonD[chunkID] = lonL
    pps_statusD[chunkID] = pps_status
    timeStamp1D[chunkID] = timeStamp1L
    NMEAdataIndexD[chunkID] = NMEAdataIndexL
    timeStampD[chunkID] = timeStampL
    returnModeD[chunkID] = returnModeL
    sensorTypeD[chunkID] = sensorTypeL
    goodPtsD[chunkID] = goodPtsL
    
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


debug = None

def main(pcap_filename):
    print()
    global debug
    numCores = mp.cpu_count() - 1
    
    #for debugging
    # numCores = 1
    
    pcapPath = Path(pcap_filename)
    pcapName = pcapPath.name
    filePath = pcapPath.parents[0]
    
    OpenMMS1()
    
    timeStart = time.time()
    timeNow = time.localtime()
    timeStr = str(timeNow[0]) + "_" + str(timeNow[1]) + "_" + str(timeNow[2]) + "_" + str(timeNow[3]) + "_" + str(timeNow[4]) + "_" + str(timeNow[5])
    log_filename = "PCAP CHECK RESULTS - " + timeStr + ".log"
    print("*** Note: Results written to file --> '" + log_filename + "' ***")
    log_filename = filePath / log_filename

    bytesRead = 0 
    
    pcapSize = os.path.getsize(pcap_filename)
    pcapfile = open(pcap_filename, 'rb')
    bytesRead = readPcapFileHeader(pcapfile, bytesRead)
    numDataPackets = 0
    numPosPackets = 0
    numUnrecPackets = 0
    skippedDataPackets = 0
    badPosPackets = 0
    
    foundGoodPos = False
    safeToRead = True
    packetOrder = []
    
    unpackL = struct.Struct('<L').unpack
    unpackB = struct.Struct('<B').unpack
    unpackH = struct.Struct('<H').unpack
    unpackHB = struct.Struct('<HB').unpack
    
    print("\nReading binary packet data from PCAP file\n")
    pbar = tqdm(total=pcapSize,unit_scale=True,unit_divisor=0.001e6,unit='B')
    pbar.update(16)
    
    hemiEmun = {"N":1, "S":-1, "E":1, "W":-1}
 
    while safeToRead:
        safeToRead, packetType, byteSize, bytesRead, packetData, numDataPackets, skippedDataPackets, numPosPackets, badPosPackets, numUnrecPackets, foundGoodPos = readPcapPacketHeader(pcapfile, bytesRead, numDataPackets, skippedDataPackets, numPosPackets, badPosPackets, numUnrecPackets, foundGoodPos, unpackL, unpackB)
        if packetType != 0:
            currentPacket = [packetType, packetData]
            packetOrder.append(currentPacket)
            bytesRead += byteSize
            
        pbar.update(byteSize + 16)
        
        #for debugging purposes
#        if numDataPackets > 100000:
#            break
            
    pbar.update(pcapSize - pbar.n)
    pbar.close()
    
    #for debugging purposes
#    numDataPackets = 0

    designNum = 0
    
    foundStuff = False
    if numDataPackets == 0 or numPosPackets == 0:
        print("\n\n****************************************************************")
        print("**************** NO DATA FOUND IN THE PCAP FILE ****************")
        print("****************************************************************\n\n")
    else:
        foundStuff = True
        if numCores > 0:
            run = True
            while run:
                designNum = (numDataPackets - skippedDataPackets) // numCores
                if designNum < 10000: 
                    numCores -= 1
                    if numCores == 0:
                        numCores = 1
                        run = False
                else:
                    run = False
        else:
            print("\n\n****************************************************************")
            print("************* COULD NOT READ CPU INFORMATION, ODD? *************")
            print("****************************************************************\n\n")
            foundStuff = False      
    
    if foundStuff:
        NMEAdataIndex = -1
        NMEAdata = []
        pps_status = [0,0,0,0]
        pointData = []
        distData = []
        
        num_points = 0
        num_bad_point_blocks = 0
        
        previousAzi = 36100.0
        previousTime = 0.0
        rotRate = -1.0
        
        if numCores == 1:
            print("\n\nParsing observations from binary packet data using 1 CPU core\n")
            pbar = tqdm(total=len(packetOrder),unit=' packets')
            
            for i in range(0,len(packetOrder)):
                   
                pbar.update(1)
                data = packetOrder[i][1]
                
                #position packet
                if packetOrder[i][0] == 1:
                    timeStamp1 = float(unpackL(data[240:244])[0])
                    
                    pps_statusi = unpackB(data[244:245])[0]
                    pps_status[pps_statusi] += 1
                    GPRMCs = str(data[248:512], 'utf-8').rstrip(' \t\r\n\0')
                    GPRMCi = GPRMCs.split(",")
                    if len(GPRMCi) > 8:
                        yyyy = int(GPRMCi[9][-2:]) + 2000
                        mm = int(GPRMCi[9][2:4])
                        dd = int(GPRMCi[9][:2])
                        hh = int(GPRMCi[1][:2])
                        mimi = int(GPRMCi[1][2:4])
                        ss = int(GPRMCi[1][4:6])
                        epoch = datetime(yyyy,mm,dd,hh,mimi,ss)
                        lat = hemiEmun[GPRMCi[4]] * (np.float32(GPRMCi[3][:2]) + np.float32(GPRMCi[3][2:]) / 60.0)
                        lon = hemiEmun[GPRMCi[6]] * (np.float32(GPRMCi[5][:3]) + np.float32(GPRMCi[5][3:]) / 60.0)
                        NMEAdata.append([epoch,lat,lon,timeStamp1])
                        NMEAdataIndex += 1
        
                #points (data) packet
                elif packetOrder[i][0] == 2:
                    if NMEAdataIndex > -1:
                        goodPts = 0
                        timeStamp = float(unpackL(data[1242:1246])[0])
                        returnMode = unpackB(data[1246:1247])[0]   #55 = Single Return(Strongest), 56 = Single Return (Last), 57 = Dual Returns (Strongest and Last)
                        sensorType = unpackB(data[1247:1248])[0]   #33 = Velodyne HDL-32E, 34 = Velodyne VLP-16/Puck Lite
        
                        if rotRate == -1.0:
                            if len(pointData) > 8000:       #calc rot rate after ~ 10 seconds of data collection
                                azi = float(unpackH(data[44:46])[0])
                                rotDiff = azi - previousAzi
                                if rotDiff > 0.0:
                                    dt = timeStamp - previousTime
                                    if dt > 0:
                                        rotRate = (rotDiff * 10000.0) / (dt * 360.0)  #in Hertz (Hz))
                                    else:
                                        previousTime = timeStamp
                                        previousAzi = azi
                                else:
                                    previousTime = timeStamp
                                    previousAzi = azi
        
                        for j in range(0,12):
                            o1 = int(j*100)
                            if binascii.hexlify(data[42+o1:44+o1]).decode('utf-8').upper() == "FFEE":
                                for k in range(0,2):
                                    o2 = o1 + int(k*48)
                                    for m in range(0,16):
                                        o3 = o2 + int(m*3)
                                        dist,intensity = unpackHB(data[46+o3:49+o3])
                                        if not dist:
                                            goodPts += 1
                                            num_points += 1
                                            distData.append(dist)
                            else:
                                num_bad_point_blocks += 1
                                
                        pointData.append([NMEAdataIndex,timeStamp,returnMode,sensorType,goodPts])

            pbar.close()
        
        #multiprocessing
        else:     
            chunkIndex = []
            foundFirstPos = False
            numSoFar = 0
            i = 0
            run = True
            while run:
                if packetOrder[i][0] == 1:
                    if not foundFirstPos:
                        chunkIndex.append(i)
                        foundFirstPos = True
                elif packetOrder[i][0] == 2:
                    if foundFirstPos:
                        numSoFar += 1
                        if numSoFar == designNum:
                            for j in range(i+1,len(packetOrder)):
                                if packetOrder[j][0] == 1:
                                    numSoFar = 0
                                    i = j-1
                                    foundFirstPos = False
                                    break
                i += 1
                if i == len(packetOrder):
                    chunkIndex.append(i)
                    run = False
    
            print("\n\nParsing observations from binary packet data using " + str(len(chunkIndex)-1) + " CPU cores, please wait...\n")
            
            procs = []
            
            manager = mp.Manager()
            epochD = manager.dict()
            latD = manager.dict()
            lonD = manager.dict()
            pps_statusD = manager.dict()
            timeStamp1D = manager.dict()
            NMEAdataIndexD = manager.dict()
            timeStampD = manager.dict()
            returnModeD = manager.dict()
            sensorTypeD = manager.dict()
            goodPtsD = manager.dict()
            miscD = manager.dict()
            statusD = manager.dict()
            
            lock = mp.Lock()
            
            for i in range(0,len(chunkIndex)-1):
                statusD[i] = 0
                
            for i in range(0,len(chunkIndex)-1):
                proc1 = mp.Process(target=parseChunk,args=(statusD, lock, i, packetOrder[chunkIndex[i]:chunkIndex[i+1]][:], miscD, epochD, latD, lonD, pps_statusD, timeStamp1D, NMEAdataIndexD, timeStampD, returnModeD, sensorTypeD, goodPtsD))                    
                procs.append(proc1)
                
            for iproc in procs:
                iproc.start()
            
            procs[len(procs)-1].join()
                
            denom = 0
            for miscData in miscD.values():
                num_points += miscData[0]
#                num_bad_position_packets += miscData[1]
                num_bad_point_blocks += miscData[2]
                if miscData[3] != -1.0:
                    rotRate += miscData[3]
                    denom += 1
            
            if denom != 0:
                rotRate /= denom
            
            epochL = []
            latL = []
            lonL = []
            timeStamp1L = []
            NMEAdataIndexL = []
            timeStampL = []
            returnModeL = []
            sensorTypeL = []
            goodPtsL = []
            indexOffset = 0
            
            for i in range(0,len(epochD)):
                epochL.extend(epochD[i])
                latL.extend(latD[i])
                lonL.extend(lonD[i])
                timeStamp1L.extend(timeStamp1D[i])
            
            for i in range(0,len(NMEAdataIndexD)):
                NMEAdataIndexL.extend((np.asarray(NMEAdataIndexD[i]) + indexOffset))
                indexOffset = NMEAdataIndexL[len(NMEAdataIndexL)-1]
                timeStampL.extend(timeStampD[i])
                returnModeL.extend(returnModeD[i])
                sensorTypeL.extend(sensorTypeD[i])
                goodPtsL.extend(goodPtsD[i])
            
            for i in range(0,len(pps_statusD)):
                pps_status[0] += pps_statusD[i][0]
                pps_status[1] += pps_statusD[i][1]
                pps_status[2] += pps_statusD[i][2]
                pps_status[3] += pps_statusD[i][3]
                
            NMEAdata = list(zip(epochL,latL,lonL,timeStamp1L))
            pointData = list(zip(NMEAdataIndexL,timeStampL,returnModeL,sensorTypeL,goodPtsL))        
                
        #done single core or multiprocessing (common code)
        if rotRate == -1.0:
            rotRate = 0.0
        
        rotTimeLimit = 0.0
        if rotRate != 0.0:
            rotTimeLimit = (1.0 / rotRate)      #time (sec) for 1 rotation of the scanner head
        
        NMEAdata = np.asarray(NMEAdata)
        
        if len(pointData) > 1 and len(NMEAdata) > 0:
            index1 = pointData[len(pointData)-2][0]
            index2 = pointData[0][0]            
            
            firstPosEpoch = (NMEAdata[0][0].hour * 3600.0) + (NMEAdata[0][3] / 1000000.0)
            firstDataEpoch = (NMEAdata[index2][0].hour * 3600.0) + (pointData[0][1] / 1000000.0)
            timeDiffd = ((NMEAdata[index1][0].hour * 3600.0) + (pointData[len(pointData)-2][1] / 1000000.0)) - firstDataEpoch
            
            if timeDiffd < 0:
                timeDiffd += 86400.0
            
            pointsPerRot = 0
            
            rotDataTimes = [0]
            rotDataPoints = [0]
            
#            debug = open("debug.txt","w")
            
            for i in range(0,len(pointData)-1):
                epochIndex = pointData[i][0]
                currentDataEpoch = (NMEAdata[epochIndex][0].hour * 3600.0) + (pointData[i][1] / 1000000.0)
#                debug.write(str(i) + " " + str(currentDataEpoch - firstDataEpoch) + "\n")
                if currentDataEpoch - firstDataEpoch < -5400:
                    currentDataEpoch += 86400.0
                if currentDataEpoch - firstDataEpoch < rotTimeLimit:
                    pointsPerRot += pointData[i][4]
                else:
                    rotDataTimes.append((currentDataEpoch - firstPosEpoch) / 60.0)
                    if pointsPerRot == 0:
                        pointsPerRot = 1   #for log scale plotting
                    rotDataPoints.append(pointsPerRot)
                    firstDataEpoch = currentDataEpoch
                    pointsPerRot = 0
            
#            debug.close()
            
            timeEnd = time.time()
            print("\n            *** COMPLETED PCAP CHECK IN " + str(round((timeEnd - timeStart) / 60.,2)) + " mins ***")
            
            badPosPercent = np.round((badPosPackets / numPosPackets) * 100,1)
            skippedDataPercent = np.round((skippedDataPackets / numDataPackets) * 100,1)
            
            logFile = open(log_filename, 'w')
            
            OpenMMS1_log(logFile)
            
            stats =  ("\n            *********************  PCAP CHECK RESULTS  ********************\n\n")
            stats += ("                     INPUT: "+ pcapName +" at "+"{:,}".format(pcapSize)+" bytes\n\n")
            stats += ("                      Approx. Rotation Rate: " + str(np.round(rotRate,1)) + " Hz\n\n")
            stats += ("                        First Position Time: " + str(NMEAdata[0][0]) + " UTC\n")
            stats += ("                         Last Position Time: " + str(NMEAdata[len(NMEAdata)-1][0]) + " UTC\n")
            stats += ("                            First Data Time: " + str(NMEAdata[index2][0]) + " UTC\n")
            stats += ("                             Last Data Time: " + str(NMEAdata[index1][0]) + " UTC\n\n")
            stats += ("                              Data Duration: " + str(np.round(timeDiffd / 60.0,1)) + " minutes\n")
            stats += ("                               Data Packets: " + "{:,}".format(numDataPackets) + "\n")
            stats += ("                       Skipped Data Packets: " + "{:,}".format(skippedDataPackets) + "  (" + str(skippedDataPercent) + "% of total)\n")
            stats += ("                Non-zero Range Observations: " + "{:,}".format(num_points) + "\n")
            stats += ("\n                            Bad Data Blocks: " + "{:,}".format(num_bad_point_blocks) + "\n")
            stats += ("                          Bad NMEA Messages: " + "{:,}".format(badPosPackets) + "  (" + str(badPosPercent) + "% of total)\n")
            stats += ("\n                            No PPS Detected: " + "{:,}".format(pps_status[0]) + "\n")
            stats += ("                       Synchronizing to PPS: " + "{:,}".format(pps_status[1]) + "\n")
            stats += ("                                 PPS Locked: " + "{:,}".format(pps_status[2]) + "\n")
            stats += ("                                  PPS Error: " + "{:,}".format(pps_status[3]) + "\n\n")
            stats += ("            ***************************************************************\n")
            
            logFile.write(stats)
            
            print(stats)
            
            dist_x = [0.0]
            dist_y = [0.0]
            
            for i in range(1,len(NMEAdata)):
                n,e = LL2NE(NMEAdata[0,1],NMEAdata[0,2],NMEAdata[i,1],NMEAdata[i,2])
                dist_y.append(n)
                dist_x.append(e)
                
            min_x = np.min(dist_x)
            min_y = np.min(dist_y)
            
            dist_x -= min_x
            dist_y -= min_y
            
            plt.figure(figsize=(7,7))
            plt.axis('equal')
            plt.xlabel('Easting [m]')
            plt.ylabel('Northing [m]')
            plt.title('2D Flight Path (' + pcapName + ')')
            plt.plot(dist_x,dist_y)
            plt.savefig(filePath / ("Flight Path - " + timeStr + ".jpg"))
            plt.show()
            
            print("\n            *** Note: Flight Path plot saved as image ***\n")
            
            plt.figure(figsize=(10,5))
            plt.yscale("log")
            plt.xlabel('Mission Time (mins)')
            plt.ylabel('Points Per Rotation (logarithmic scale)')
            plt.title('Collected Points during Mission (' + pcapName + ')')
            plt.plot(rotDataTimes, rotDataPoints, 'g.', ms=1) #'g,')
            plt.ylim(bottom=1)
            plt.savefig(filePath / ("Collected Points - " + timeStr + ".jpg"))
            plt.show()
            
            print("            *** Note: Collected Points plot saved as image ***\n")
        
            iHeartLidar2(logFile)
        
            logFile.close()
        else:
            print("\n\n****************************************************************")
            print("*********** NO DATA AND/OR POSITION INFO WAS FOUND *************")
            print("****************************************************************\n\n")


if __name__ == "__main__":
    print("\nPCAP File Check Started...")
    pcap_filename = sys.argv[1]
    main(pcap_filename)

