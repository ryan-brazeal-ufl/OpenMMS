# -*- coding: utf-8 -*-

############################################################################
#                          OpenMMS TRAJ Convert                            #
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

import sys
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from pathlib import Path
import time


def iHeartLidar():
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


def readPositionData(position,lastTime):
    returnValues = []
    recordTime = -1.0
    try:
        hour = float(position[1][0:2])
        minute = float(position[1][2:4])
        second = float(position[1][4:])
        recordTime = hour * 3600.0 + minute * 60.0 + second
        if recordTime - lastTime < 0:
            recordTime += 86400.0
            
        returnValues.append(recordTime)
        
        latD = float(position[2][0:2])
        latDM = float(position[2][2:])
        posSign = 1
        if position[3].upper() == "S":
            posSign = -1
        returnValues.append(posSign * (latD + latDM / 60.))
        
        longD = float(position[4][0:3])
        longDM = float(position[4][3:])
        posSign = 1
        if position[5].upper() == "W":
            posSign = -1
        returnValues.append(posSign * (longD + longDM / 60.))
        returnValues.append(float(position[9]))
    except:
        returnValues = []
        recordTime = -1.0
        returnValues.append(-1)
        returnValues.append(-1)
        returnValues.append(-1)
        returnValues.append(-1)
        
    return returnValues, recordTime


def readOrientationData(recordData,lastTime):
    returnValues = []
    recordTime = -1.0
    try:
        hour = float(recordData[1][0:2])
        minute = float(recordData[1][2:4])
        second = float(recordData[1][4:])
        recordTime = hour * 3600.0 + minute * 60.0 + second
        if recordTime - lastTime < 0:
            recordTime += 86400.0
            
        returnValues.append(recordTime)
        returnValues.append(float(recordData[4])) #roll
        returnValues.append(float(recordData[5])) #pitch
        returnValues.append(float(recordData[2])) #heading
    except:
        returnValues = []
        recordTime = -1.0
        returnValues.append(-1)
        returnValues.append(-1)
        returnValues.append(-1)
        returnValues.append(-1)
        
    return returnValues, recordTime



def main():
    trajFilename = sys.argv[1]
    
    trajPath = Path(trajFilename)
    trajName = trajPath.name
    filePath = trajPath.parents[0]
    
    OpenMMS1()
    
    trajFile = open(trajFilename, 'r')

    print("    TRAJECTORY FILE STATS:")
    
    lastPosTime = 0.0
    lastOriTime = 0.0
    
    posData = []
    oriData = []
    eventData = []
    
    lastPos = []
    for record in trajFile:
        recordData = record.strip("\n").split(",")
        
        if len(recordData) > 0:
            header = recordData[0]
            if header.upper() == "$GNGGA" or header.upper() == "$GPGGA":
                lastPos, newPosTime = readPositionData(recordData,lastPosTime)
                if newPosTime >= 0.0:
                    posData.append(lastPos)
                    lastPosTime = newPosTime

            elif header.upper() == "$PASHR":
                lastOri, newOriTime = readOrientationData(recordData,lastOriTime)
                if newOriTime >= 0.0:
                    oriData.append(lastOri)
                    lastOriTime = newOriTime

            elif header.upper() == "$PTNL":
                if lastPos:
                    newPt = []
                    newPt.append(lastPos[2])
                    newPt.append(lastPos[1])
                    eventData.append(newPt)
                        
    print("\n      Position Records: " + str(len(posData)))
    print("      Orientation Records: " + str(len(oriData)))
    print("      Event Records: " + str(len(eventData)))
    
    trajFile.close()

    print("\n    Writing trajectory file, please wait...\n")

    traj_eo = open(filePath / (trajName[:-5] + "_traj.txt"),"w")
    traj_eo.write("\n")

    pbar = tqdm(total=len(posData),unit=" records",desc="    ")
    
    dgData = np.ones((len(posData),7),dtype=np.float32) * -1.0
    goodData = 0
    lenOriData = len(oriData)
    
    for i in range(0,len(posData)):
        pbar.update()
        startIndex = i
        if i >= lenOriData:
            startIndex = 0
        
        found = False
        for j in range(startIndex,lenOriData):
                    
            if posData[i][0] == oriData[j][0]:
                goodData += 1
                dgData[i,0] = (posData[i][0])
                dgData[i,1] = (posData[i][1])
                dgData[i,2] = (posData[i][2])
                dgData[i,3] = (posData[i][3])
                dgData[i,4] = (oriData[j][1])
                dgData[i,5] = (oriData[j][2])
                dgData[i,6] = (oriData[j][3])
                
                traj_eo.write(str(dgData[i,0])+","+str(dgData[i,1])+","+str(dgData[i,2])+","+str(dgData[i,3])+",0,"+str(dgData[i,4])+","+str(dgData[i,5])+","+str(dgData[i,6])+",0,0,0,0,0,0,0\n")
                found = True
                break
                
        if not found:        
            for j in range(startIndex,-1,-1):
                    
                if posData[i][0] == oriData[j][0]:
                    goodData += 1
                    dgData[i,0] = (posData[i][0])
                    dgData[i,1] = (posData[i][1])
                    dgData[i,2] = (posData[i][2])
                    dgData[i,3] = (posData[i][3])
                    dgData[i,4] = (oriData[j][1])
                    dgData[i,5] = (oriData[j][2])
                    dgData[i,6] = (oriData[j][3])
                    
                    traj_eo.write(str(dgData[i,0])+","+str(dgData[i,1])+","+str(dgData[i,2])+","+str(dgData[i,3])+",0,"+str(dgData[i,4])+","+str(dgData[i,5])+","+str(dgData[i,6])+",0,0,0,0,0,0,0\n")
                    found = True
                    break
            
#                R_d = rollDD[j]
#                P_d = pitchDD[j]
#                H_d = headingDD[j]
#                
#                R = np.dot(formR2(180.), np.dot(formR3(-90.), np.dot(formR3(-H_d), np.dot(formR2(-P_d), np.dot(formR1(-R_d), np.dot(formR1(0.), np.dot(formR2(0.), formR3(0))))))))
#                
#                phi = np.degrees(np.arcsin(R[0,2]))
#                omega = 0
#                kappa = 0
#                
#                if phi == 90 or phi == -90:
#                    kappa = np.degrees(np.arctan2(R[1,0],R[1,1]))
#                else:
#                    omega = np.degrees(np.arctan2(-R[1,2],R[2,2]))
#                    kappa = np.degrees(np.arctan2(-R[0,1],R[0,0]))
#                
#                dgRecord.append(omega)
#                dgRecord.append(phi)
#                dgRecord.append(kappa)
                
                

    pbar.close()
    traj_eo.close()

    print("\n\n    *** TRAJECTORY CONVERSION COMPLETED SUCCESSFULLY ***\n")
    iHeartLidar()
    
    tX = dgData[:,2][:-1]
    tY = dgData[:,1][:-1]
    eventData = np.asarray(eventData,dtype=np.float32)
    eX = eventData[:,0]
    eY = eventData[:,1]
    
    if len(tX) != goodData:
        new_tX = []
        new_tY = []
        for i in range(0,len(dgData)):
            if dgData[i,0] >= 0.0:
                new_tX.append(dgData[i,2])
                new_tY.append(dgData[i,1])
        tX = np.asarray(new_tX,dtype=np.float32)
        tY = np.asarray(new_tY,dtype=np.float32)
    
    dist_x = [0.0]
    dist_y = [0.0]
    
    for i in range(1,len(tX)):
        n,e = LL2NE(tY[0],tX[0],tY[i],tX[i])
        dist_y.append(n)
        dist_x.append(e)
    
    min_x = np.min(dist_x)
    min_y = np.min(dist_y)
    
    dist_x -= min_x
    dist_y -= min_y
    
    event_x = [0.0]
    event_y = [0.0]
    
    for i in range(1,len(eX)):
        n,e = LL2NE(tY[0],tX[0],eY[i],eX[i])
        event_y.append(n)
        event_x.append(e)
    
    event_x -= min_x
    event_y -= min_y
    
    plt.figure(figsize=(7,7))
    plt.axis('equal')
    plt.plot(dist_x,dist_y,'g',event_x,event_y,'rx')
    plt.xlabel('Easting [m]')
    plt.ylabel('Northing [m]')
    plt.title('Flight Path (' + str(len(eventData)) + ' Events)')
    plt.savefig(filePath / "Traj Flight Path.png")
    plt.show()

##### command line start
if __name__ == "__main__":
    print("\nReal-Time Trajectory Conversion Started...\n")
    main()