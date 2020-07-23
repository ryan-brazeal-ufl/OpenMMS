# -*- coding: utf-8 -*-

############################################################################
#                             OpenMMS Colorize                             #
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
import copy
import laspy
import numpy as np
import multiprocessing as mp
import os
import cv2
from tqdm import tqdm
from pathlib import Path
from bisect import bisect_left
from sklearn.neighbors import BallTree
import warnings


def formR1(theta):
    R1 = np.zeros((3,3))
    st = np.sin(np.radians(theta))
    ct = np.cos(np.radians(theta))
    R1[0,0] = 1.
    R1[1,1] = ct
    R1[1,2] = st
    R1[2,1] = -st
    R1[2,2] = ct    
    return R1
    
def formR2(theta):
    R2 = np.zeros((3,3))
    st = np.sin(np.radians(theta))
    ct = np.cos(np.radians(theta))
    R2[0,0] = ct
    R2[0,2] = -st
    R2[1,1] = 1.
    R2[2,0] = st
    R2[2,2] = ct
    return R2
    
def formR3(theta):
    R3 = np.zeros((3,3))
    st = np.sin(np.radians(theta))
    ct = np.cos(np.radians(theta))
    R3[0,0] = ct
    R3[0,1] = st
    R3[1,0] = -st
    R3[1,1] = ct
    R3[2,2] = 1.
    return R3

def getImageRGB(Xp, Yp, Zp, imageData, rotData, cameraData, focal_length_pixel, xo_pixel, yo_pixel):
    
    red = 0
    green = 0
    blue = 0
    
    Xc = cameraData[0]
    Yc = cameraData[1]
    Zc = cameraData[2]
    dX = Xp - Xc
    dY = Yp - Yc
    dZ = Zp - Zc
    R = rotData
    
    #Collinearity Equations
    pixel_x = int(xo_pixel - focal_length_pixel*((R[0,0]*dX+R[0,1]*dY+R[0,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
    pixel_y = int(yo_pixel + focal_length_pixel*((R[1,0]*dX+R[1,1]*dY+R[1,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))

    h, w = imageData.shape[:2]
    if pixel_x >= 0 and pixel_x < w and pixel_y >=0 and pixel_y < h:
        red = imageData[pixel_y,pixel_x,2]
        green = imageData[pixel_y,pixel_x,1]
        blue = imageData[pixel_y,pixel_x,0]
    
    return red, green, blue


def colorBalancing2(r1,g1,b1,r2,g2,b2):
    useRGB1 = False
    useRGB2 = False
                
    if not(r1 == 0 and g1 == 0 and b1 == 0):
        useRGB1 = True
    if not(r2 == 0 and g2 == 0 and b2 == 0):
        useRGB2 = True
                
    R = 0
    G = 0
    B = 0
    
    if useRGB1 and useRGB2:
        R = np.floor((0.667*r1 + 0.333*r2))
        G = np.floor((0.667*g1 + 0.333*g2))
        B = np.floor((0.667*b1 + 0.333*b2))
    elif useRGB1:
        R = r1
        G = g1
        B = b1
    elif useRGB2:
        R = r2
        G = g2
        B = b2
        
    return R, G, B


def colorBalancing3(r1,g1,b1,r2,g2,b2,r3,g3,b3):
    useRGB1 = False
    useRGB2 = False
    useRGB3 = False
                
    if not(r1 == 0 and g1 == 0 and b1 == 0):
        useRGB1 = True
    if not(r2 == 0 and g2 == 0 and b2 == 0):
        useRGB2 = True
    if not(r3 == 0 and g3 == 0 and b3 == 0):
        useRGB3 = True
                
    R = 0
    G = 0
    B = 0
    
    if useRGB1 and useRGB2 and useRGB3:
        R = np.floor((0.57*r1 + 0.285*r2 + 0.145*r3))
        G = np.floor((0.57*g1 + 0.285*g2 + 0.145*g3))
        B = np.floor((0.57*b1 + 0.285*b2 + 0.145*b3))
    elif useRGB1 and useRGB2:
        R = np.floor((0.667*r1 + 0.333*r2))
        G = np.floor((0.667*g1 + 0.333*g2))
        B = np.floor((0.667*b1 + 0.333*b2))
    elif useRGB1 and useRGB3:
        R = np.floor((0.667*r1 + 0.333*r3))
        G = np.floor((0.667*g1 + 0.333*g3))
        B = np.floor((0.667*b1 + 0.333*b3))
    elif useRGB2 and useRGB3:
        R = np.floor((0.667*r2 + 0.333*r3))
        G = np.floor((0.667*g2 + 0.333*g3))
        B = np.floor((0.667*b2 + 0.333*b3))
    elif useRGB1:
        R = r1
        G = g1
        B = b1
    elif useRGB2:
        R = r2
        G = g2
        B = b2
    elif useRGB3:
        R = r3
        G = g3
        B = b3
        
    return R, G, B


def nonblank_lines(f):
    for l in f:
        line = l.rstrip()
        if line:
            yield line


def timeImagesChunk(statusD, minCam, maxCam, camera1S, camera2S, camera3S, lock, chunkID, ptStart, ptEnd, ptTimesS, camTimeData):
    
    descStr = "      Core " + str(chunkID+1)
    if chunkID+1 > 9:
        descStr += " "
    else:
        descStr += "  "
        
    lock.acquire()
    pbari = tqdm(total=ptEnd-ptStart,unit=" pts",desc=descStr,position=chunkID,ncols=87)
    lock.release()

    camera1 = np.frombuffer(camera1S,'H')
    camera2 = np.frombuffer(camera2S,'H')
    camera3 = np.frombuffer(camera3S,'H')
    ptTimes = np.frombuffer(ptTimesS,'d')
    
    minimum = 10000000
    maximum = -10000000
    count = 0
    for i in range(ptStart,ptEnd):
        deltas = [100000.0,100000.0,100000.0]
        closestIndices = [-1,-1,-1]
        
        ptTime = ptTimes[i]
        pos = bisect_left(camTimeData,ptTime)
        closestIndices[0] = pos

        if pos > 1 and pos < len(camTimeData)-1:
            backTime = ptTime - camTimeData[pos-1]
            forwardTime = camTimeData[pos] - ptTime
            if backTime < forwardTime:
                pos = pos-1
            closestIndices[0] = pos
            closestIndices[1] = pos-1
            closestIndices[2] = pos+1
        elif pos == 0:
            closestIndices[1] = 0
            closestIndices[2] = 1
        elif pos == 1:
            closestIndices[1] = 0
            closestIndices[2] = 2
        elif pos == len(camTimeData):
            closestIndices[0] = pos-1
            closestIndices[1] = pos-1
            closestIndices[2] = pos-2
        elif pos == len(camTimeData)-1:
            closestIndices[1] = pos-1
            closestIndices[2] = pos-2
                  
        deltas[0] = np.abs(ptTime - camTimeData[closestIndices[0]])
        deltas[1] = np.abs(ptTime - camTimeData[closestIndices[1]])
        deltas[2] = np.abs(ptTime - camTimeData[closestIndices[2]])
        
        if deltas[1] < deltas[0]:
            temp1 = deltas[1]
            deltas[1] = deltas[0]
            deltas[0] = temp1
            temp2 = closestIndices[1]
            closestIndices[1] = closestIndices[0]
            closestIndices[0] = temp2
        if deltas[2] < deltas[0]:
            temp1 = deltas[2]
            deltas[2] = deltas[0]
            deltas[0] = temp1
            temp2 = closestIndices[2]
            closestIndices[2] = closestIndices[0]
            closestIndices[0] = temp2
        if deltas[2] < deltas[1]:
            temp1 = deltas[2]
            deltas[2] = deltas[1]
            deltas[1] = temp1
            temp2 = closestIndices[2]
            closestIndices[2] = closestIndices[1]
            closestIndices[1] = temp2
            
        camera1[i] = closestIndices[0]
        camera2[i] = closestIndices[1]
        camera3[i] = closestIndices[2]
        
        if closestIndices[0] < minimum:
            minimum = closestIndices[0]
        if closestIndices[1] < minimum:
            minimum = closestIndices[1]
        if closestIndices[2] < minimum:
            minimum = closestIndices[2]
        if closestIndices[0] > maximum:
            maximum = closestIndices[0]
        if closestIndices[1] > maximum:
            maximum = closestIndices[1]
        if closestIndices[2] > maximum:
            maximum = closestIndices[2]
            
        if count % 10000 == 0:
            if pbari.n + 10000 < pbari.total:
                lock.acquire()
                pbari.update(10000)
                lock.release()
                
        count += 1
    
    lock.acquire()
    pbari.update(pbari.total - pbari.n)
    lock.release()
    
    statusD[chunkID] = 3
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
                statusD[chunkID] = 4
                shouldStop = True
            else:
                if statusD[chunkID-1] == 4:
                    statusD[chunkID] = 4
                    shouldStop = True
    
    lock.acquire()
    pbari.close()
    lock.release()
    
    minCam[chunkID] = minimum
    maxCam[chunkID] = maximum


def loadImagesChunk(firstCamIndex, imgWidth, imgHeight, statusD, lock, chunkID, imageStart, imageEnd, colorizeData, imageDataR, imageDataG, imageDataB):
    
    descStr = "      Core " + str(chunkID+1)
    if chunkID+1 > 9:
        descStr += " "
    else:
        descStr += "  "
    
    lock.acquire()
    pbari = tqdm(total=(imageEnd-imageStart),unit=" images",desc=descStr,position=chunkID,ncols=87)
    lock.release()
    
    imageDataRArray = np.frombuffer(imageDataR,'B')
    imageDataGArray = np.frombuffer(imageDataG,'B')
    imageDataBArray = np.frombuffer(imageDataB,'B')
    
    for i in range(imageStart,imageEnd):
        EOPdata_i = colorizeData[i]
        img = cv2.imread(EOPdata_i[1], cv2.IMREAD_IGNORE_ORIENTATION | cv2.IMREAD_COLOR)
        flatR = img[:,:,2].flatten()
        flatG = img[:,:,1].flatten()
        flatB = img[:,:,0].flatten()
        imgNumPixel1 = (i - firstCamIndex) * len(flatR)
        imgNumPixel2 = imgNumPixel1 + len(flatR)
        imageDataRArray[imgNumPixel1:imgNumPixel2] = flatR    #red
        imageDataGArray[imgNumPixel1:imgNumPixel2] = flatG    #green
        imageDataBArray[imgNumPixel1:imgNumPixel2] = flatB    #blue
        
        lock.acquire()
        pbari.update()
        lock.release()
    
    statusD[chunkID] = 3
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
                statusD[chunkID] = 4
                shouldStop = True
            else:
                if statusD[chunkID-1] == 4:
                    statusD[chunkID] = 4
                    shouldStop = True
    
    lock.acquire()
    pbari.close()
    lock.release()


def distImagesChunk(firstCamIndex, notcoloredPtsS, statusD, ptStart, ptEnd, camera4S, camera5S, camera6S, lock, chunkID, xS, yS, images_ballTree):

    descStr = "      Core " + str(chunkID+1)
    if chunkID+1 > 9:
        descStr += " "
    else:
        descStr += "  "
    
    notcoloredPts = np.frombuffer(notcoloredPtsS,'L')
    camera4 = np.frombuffer(camera4S,'H')
    camera5 = np.frombuffer(camera5S,'H')
    camera6 = np.frombuffer(camera6S,'H')
    x = np.frombuffer(xS,'d')
    y = np.frombuffer(yS,'d')
    
    lock.acquire()
    pbari = tqdm(total=ptEnd-ptStart ,unit=" pts",desc=descStr,position=chunkID,ncols=87)
    lock.release()
    
    pt = np.zeros((1,2),dtype=np.float)
    for i in range(ptStart,ptEnd):
        oi = notcoloredPts[i]
        pt[0,0] = x[oi]
        pt[0,1] = y[oi]
        dist, indices = images_ballTree.query(pt,k=3)
        
        camera4[i] = indices[0,0]+firstCamIndex
        camera5[i] = indices[0,1]+firstCamIndex
        camera6[i] = indices[0,2]+firstCamIndex
            
        if i % 10000 == 0:
            if pbari.n + 10000 < pbari.total:
                lock.acquire()
                pbari.update(10000)
                lock.release()
    
    lock.acquire()
    pbari.update(pbari.total - pbari.n)
    lock.release()
    
    statusD[chunkID] = 3
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
                statusD[chunkID] = 4
                shouldStop = True
            else:
                if statusD[chunkID-1] == 4:
                    statusD[chunkID] = 4
                    shouldStop = True
    
    lock.acquire()
    pbari.close()
    lock.release()
    

def timecolorizeChunk(firstCamIndex, colorBalance, resizeFactor, statusD, colorClassS, imageUsedS, redS, greenS, blueS, lock, chunkID, xS, yS, zS, camera1S, camera2S, camera3S, ptStart, ptEnd, cameraData, rotData, imageDataR, imageDataG, imageDataB, io_values):
    warnings.filterwarnings("ignore")
    
    cameraWidth = int(io_values[1] * resizeFactor)
    cameraHeight = int(io_values[2] * resizeFactor)
    cameraFocal = (io_values[3]) * resizeFactor
    cameraPPx = io_values[4] * resizeFactor
    cameraPPy = io_values[5] * resizeFactor
    
    colorClass = np.frombuffer(colorClassS,'B')
    imageUsed = np.frombuffer(imageUsedS,'H')
    red = np.frombuffer(redS,'B')
    green = np.frombuffer(greenS,'B')
    blue = np.frombuffer(blueS,'B')
    
    x = np.frombuffer(xS,'d')
    y = np.frombuffer(yS,'d')
    z = np.frombuffer(zS,'d')
    
    camera1 = np.frombuffer(camera1S,'H')
    camera2 = np.frombuffer(camera2S,'H')
    camera3 = np.frombuffer(camera3S,'H')
    
    #IF mp.Array is used
#    imageDataRArray = np.frombuffer(imageDataR.get_obj(),'B')
    
    #IF my.RawArray is used
    imageDataRArray = np.frombuffer(imageDataR,'B')
    imageDataGArray = np.frombuffer(imageDataG,'B')
    imageDataBArray = np.frombuffer(imageDataB,'B')
    
    descStr = "      Core " + str(chunkID+1)
    if chunkID+1 > 9:
        descStr += " "
    else:
        descStr += "  "
        
    lock.acquire()
    pbari = tqdm(total=ptEnd-ptStart,unit=" pts",desc=descStr,position=chunkID,ncols=87)
    lock.release()
    
    for i in range(ptStart,ptEnd):
        imageUsed[i] = 0
        colored = False
        r2 = 0
        g2 = 0
        b2 = 0
        r3 = 0
        g3 = 0
        b3 = 0
        Red = 0
        Green = 0
        Blue = 0
        
        j = camera1[i]
        if j > -1:
            
            dX = x[i] - cameraData[j][0]
            dY = y[i] - cameraData[j][1]
            dZ = z[i] - cameraData[j][2]
            R = rotData[j]
            
            #Collinearity Equations
            pixel_x = int(cameraPPx - cameraFocal*((R[0,0]*dX+R[0,1]*dY+R[0,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
            pixel_y = int(cameraPPy + cameraFocal*((R[1,0]*dX+R[1,1]*dY+R[1,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
            
            if pixel_x > 0 and pixel_x < cameraWidth-1 and pixel_y > 0 and pixel_y < cameraHeight-1:
#                try:  #used for int overflow, which happens if too much image data is loaded
                    colorIndex = int((j - firstCamIndex) * cameraWidth * cameraHeight + pixel_y * cameraWidth + pixel_x)
                    Red = imageDataRArray[colorIndex]
                    Green = imageDataGArray[colorIndex]
                    Blue = imageDataBArray[colorIndex]
                    colored = True
                    imageUsed[i] = j+1
#                except:
#                    pass
                    
        if colorBalance > 1:
            j = camera2[i]
            if j > -1:
                
                dX = x[i] - cameraData[j][0]
                dY = y[i] - cameraData[j][1]
                dZ = z[i] - cameraData[j][2]
                R = rotData[j]
                
                #Collinearity Equations
                pixel_x = int(cameraPPx - cameraFocal*((R[0,0]*dX+R[0,1]*dY+R[0,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
                pixel_y = int(cameraPPy + cameraFocal*((R[1,0]*dX+R[1,1]*dY+R[1,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
            
                if pixel_x > 0 and pixel_x < cameraWidth-1 and pixel_y > 0 and pixel_y < cameraHeight-1:
#                    try:
                        colorIndex = int((j - firstCamIndex) * cameraWidth * cameraHeight + pixel_y * cameraWidth + pixel_x)
                        r2 = imageDataRArray[colorIndex]
                        g2 = imageDataGArray[colorIndex]
                        b2 = imageDataBArray[colorIndex]
                        colored = True
                        if imageUsed[i] == 0:
                            imageUsed[i] = j+1
#                    except:
#                        pass
            
        if colorBalance > 2:
            j = camera3[i]
            if j > -1:
                
                dX = x[i] - cameraData[j][0]
                dY = y[i] - cameraData[j][1]
                dZ = z[i] - cameraData[j][2]
                R = rotData[j]
                
                #Collinearity Equations
                pixel_x = int(cameraPPx - cameraFocal*((R[0,0]*dX+R[0,1]*dY+R[0,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
                pixel_y = int(cameraPPy + cameraFocal*((R[1,0]*dX+R[1,1]*dY+R[1,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
        
                if pixel_x > 0 and pixel_x < cameraWidth-1 and pixel_y > 0 and pixel_y < cameraHeight-1:
#                    try:
                        colorIndex = int((j - firstCamIndex) * cameraWidth * cameraHeight + pixel_y * cameraWidth + pixel_x)
                        r3 = imageDataRArray[colorIndex]
                        g3 = imageDataGArray[colorIndex]
                        b3 = imageDataBArray[colorIndex]
                        colored = True
                        if imageUsed[i] == 0:
                            imageUsed[i] = j+1
#                    except:
#                        pass
        
        if colorBalance == 2:
            
            useRGB1 = False
            useRGB2 = False
                        
            if not(Red == 0 and Green == 0 and Blue == 0):
                useRGB1 = True
            if not(r2 == 0 and g2 == 0 and b2 == 0):
                useRGB2 = True
            
            if useRGB1 and useRGB2:
                Red = np.floor((0.667*Red + 0.333*r2))
                Green = np.floor((0.667*Green + 0.333*g2))
                Blue = np.floor((0.667*Blue + 0.333*b2))
            elif useRGB2:
                Red = r2
                Green = g2
                Blue = b2
        
        elif colorBalance == 3:
            
            useRGB1 = False
            useRGB2 = False
            useRGB3 = False
                        
            if not(Red == 0 and Green == 0 and Blue == 0):
                useRGB1 = True
            if not(r2 == 0 and g2 == 0 and b2 == 0):
                useRGB2 = True
            if not(r3 == 0 and g3 == 0 and b3 == 0):
                useRGB3 = True
            
            if useRGB1 and useRGB2 and useRGB3:
                Red = np.floor((0.57*Red + 0.285*r2 + 0.145*r3))
                Green = np.floor((0.57*Green + 0.285*g2 + 0.145*g3))
                Blue = np.floor((0.57*Blue + 0.285*b2 + 0.145*b3))
            elif useRGB1 and useRGB2:
                Red = np.floor((0.667*Red + 0.333*r2))
                Green = np.floor((0.667*Green + 0.333*g2))
                Blue = np.floor((0.667*Blue + 0.333*b2))
            elif useRGB1 and useRGB3:
                Red = np.floor((0.667*Red + 0.333*r3))
                Green = np.floor((0.667*Green + 0.333*g3))
                Blue = np.floor((0.667*Blue + 0.333*b3))
            elif useRGB2 and useRGB3:
                Red = np.floor((0.667*r2 + 0.333*r3))
                Green = np.floor((0.667*g2 + 0.333*g3))
                Blue = np.floor((0.667*b2 + 0.333*b3))
            elif useRGB2:
                Red = r2
                Green = g2
                Blue = b2
            elif useRGB3:
                Red = r3
                Green = g3
                Blue = b3
            
        red[i] = np.uint8(Red)
        green[i] = np.uint8(Green)
        blue[i] = np.uint8(Blue)
        
        if colored:
            colorClass[i] = 1
        else:
            colorClass[i] = 0
    
        if i % 10000 == 0:            
            if pbari.n + 10000 < pbari.total:
                lock.acquire()
                pbari.update(10000)
                lock.release()
    
    lock.acquire()
    pbari.update(pbari.total - pbari.n)
    lock.release()
    
    statusD[chunkID] = 3
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
                statusD[chunkID] = 4
                shouldStop = True
            else:
                if statusD[chunkID-1] == 4:
                    statusD[chunkID] = 4
                    shouldStop = True
    
    lock.acquire()
    pbari.close()
    lock.release()
    

def distcolorizeChunk(notcoloredPts_length, firstCamIndex, distcolorPointsS, colorBalance, resizeFactor, statusD, colorClassS, imageUsedS, redS, greenS, blueS, lock, chunkID, xS, yS, zS, ptStart, ptEnd, cameraData, rotData, imageDataR, imageDataG, imageDataB, io_values):
    warnings.filterwarnings("ignore")
    
    cameraWidth = int(io_values[1] * resizeFactor)
    cameraHeight = int(io_values[2] * resizeFactor)
    cameraFocal = (io_values[3]) * resizeFactor
    cameraPPx = io_values[4] * resizeFactor
    cameraPPy = io_values[5] * resizeFactor
    
    distcolorPoints = np.frombuffer(distcolorPointsS,'L').reshape((notcoloredPts_length,4))
    
    colorClass = np.frombuffer(colorClassS,'B')
    imageUsed = np.frombuffer(imageUsedS,'H')
    
    red = np.frombuffer(redS,'B')
    green = np.frombuffer(greenS,'B')
    blue = np.frombuffer(blueS,'B')
    
    x = np.frombuffer(xS,'d')
    y = np.frombuffer(yS,'d')
    z = np.frombuffer(zS,'d')
    
    imageDataRArray = np.frombuffer(imageDataR,'B')
    imageDataGArray = np.frombuffer(imageDataG,'B')
    imageDataBArray = np.frombuffer(imageDataB,'B')
    
    descStr = "      Core " + str(chunkID+1)
    if chunkID+1 > 9:
        descStr += " "
    else:
        descStr += "  "
        
    lock.acquire()
    pbari = tqdm(total=ptEnd-ptStart ,unit=" pts",desc=descStr,position=chunkID,ncols=87)
    lock.release()

    for i in range(ptStart,ptEnd):
        colored = False
        r2 = 0
        g2 = 0
        b2 = 0
        r3 = 0
        g3 = 0
        b3 = 0
        Red = 0
        Green = 0
        Blue = 0
        
        oi = distcolorPoints[i,0]
        
        j = distcolorPoints[i,1]
        if j > -1:
            
            dX = x[oi] - cameraData[j][0]
            dY = y[oi] - cameraData[j][1]
            dZ = z[oi] - cameraData[j][2]
            R = rotData[j]
            
            #Collinearity Equations
            pixel_x = int(cameraPPx - cameraFocal*((R[0,0]*dX+R[0,1]*dY+R[0,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
            pixel_y = int(cameraPPy + cameraFocal*((R[1,0]*dX+R[1,1]*dY+R[1,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
        
            if pixel_x > 0 and pixel_x < cameraWidth-1 and pixel_y > 0 and pixel_y < cameraHeight-1:
#                try:
                    colorIndex = int((j - firstCamIndex) * cameraWidth * cameraHeight + pixel_y * cameraWidth + pixel_x)
                    Red = imageDataRArray[colorIndex]
                    Green = imageDataGArray[colorIndex]
                    Blue = imageDataBArray[colorIndex]
                    colored = True
#                    imageUsed[oi] = j+1
#                except:
#                    pass
                    
        if colorBalance > 1:
            j = distcolorPoints[i,2]
            if j > -1:
                
                dX = x[oi] - cameraData[j][0]
                dY = y[oi] - cameraData[j][1]
                dZ = z[oi] - cameraData[j][2]
                R = rotData[j]
                
                #Collinearity Equations
                pixel_x = int(cameraPPx - cameraFocal*((R[0,0]*dX+R[0,1]*dY+R[0,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
                pixel_y = int(cameraPPy + cameraFocal*((R[1,0]*dX+R[1,1]*dY+R[1,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
            
                if pixel_x > 0 and pixel_x < cameraWidth-1 and pixel_y > 0 and pixel_y < cameraHeight-1:
#                    try:
                        colorIndex = int((j - firstCamIndex) * cameraWidth * cameraHeight + pixel_y * cameraWidth + pixel_x)
                        r2 = imageDataRArray[colorIndex]
                        g2 = imageDataGArray[colorIndex]
                        b2 = imageDataBArray[colorIndex]
                        colored = True
                        if imageUsed[oi] == 0:
                            imageUsed[oi] = j+1
#                    except:
#                        pass
                        
        if colorBalance > 2:
            j = distcolorPoints[i,3]
            if j > -1:
                
                dX = x[oi] - cameraData[j][0]
                dY = y[oi] - cameraData[j][1]
                dZ = z[oi] - cameraData[j][2]
                R = rotData[j]
                
                #Collinearity Equations
                pixel_x = int(cameraPPx - cameraFocal*((R[0,0]*dX+R[0,1]*dY+R[0,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
                pixel_y = int(cameraPPy + cameraFocal*((R[1,0]*dX+R[1,1]*dY+R[1,2]*dZ)/(R[2,0]*dX+R[2,1]*dY+R[2,2]*dZ)))
            
                if pixel_x > 0 and pixel_x < cameraWidth-1 and pixel_y > 0 and pixel_y < cameraHeight-1:
#                    try:
                        colorIndex = int((j - firstCamIndex) * cameraWidth * cameraHeight + pixel_y * cameraWidth + pixel_x)
                        r3 = imageDataRArray[colorIndex]
                        g3 = imageDataGArray[colorIndex]
                        b3 = imageDataBArray[colorIndex]
                        colored = True
                        if imageUsed[oi] == 0:
                            imageUsed[oi] = j+1
#                    except:
#                        pass
        
        if colorBalance == 2:
            
            useRGB1 = False
            useRGB2 = False
                        
            if not(Red == 0 and Green == 0 and Blue == 0):
                useRGB1 = True
            if not(r2 == 0 and g2 == 0 and b2 == 0):
                useRGB2 = True
            
            if useRGB1 and useRGB2:
                Red = np.floor((0.667*Red + 0.333*r2))
                Green = np.floor((0.667*Green + 0.333*g2))
                Blue = np.floor((0.667*Blue + 0.333*b2))
            elif useRGB2:
                Red = r2
                Green = g2
                Blue = b2
        
        elif colorBalance == 3:
            
            useRGB1 = False
            useRGB2 = False
            useRGB3 = False
                        
            if not(Red == 0 and Green == 0 and Blue == 0):
                useRGB1 = True
            if not(r2 == 0 and g2 == 0 and b2 == 0):
                useRGB2 = True
            if not(r3 == 0 and g3 == 0 and b3 == 0):
                useRGB3 = True
            
            if useRGB1 and useRGB2 and useRGB3:
                Red = np.floor((0.57*Red + 0.285*r2 + 0.145*r3))
                Green = np.floor((0.57*Green + 0.285*g2 + 0.145*g3))
                Blue = np.floor((0.57*Blue + 0.285*b2 + 0.145*b3))
            elif useRGB1 and useRGB2:
                Red = np.floor((0.667*Red + 0.333*r2))
                Green = np.floor((0.667*Green + 0.333*g2))
                Blue = np.floor((0.667*Blue + 0.333*b2))
            elif useRGB1 and useRGB3:
                Red = np.floor((0.667*Red + 0.333*r3))
                Green = np.floor((0.667*Green + 0.333*g3))
                Blue = np.floor((0.667*Blue + 0.333*b3))
            elif useRGB2 and useRGB3:
                Red = np.floor((0.667*r2 + 0.333*r3))
                Green = np.floor((0.667*g2 + 0.333*g3))
                Blue = np.floor((0.667*b2 + 0.333*b3))
            elif useRGB2:
                Red = r2
                Green = g2
                Blue = b2
            elif useRGB3:
                Red = r3
                Green = g3
                Blue = b3
            
        red[oi] = np.uint8(Red)
        green[oi] = np.uint8(Green)
        blue[oi] = np.uint8(Blue)
        
        if colored:
            colorClass[oi] = 2
    
        if i % 10000 == 0:            
            if pbari.n + 10000 < pbari.total:
                lock.acquire()
                pbari.update(10000)
                lock.release()  
        
    lock.acquire()
    pbari.update(pbari.total - pbari.n)
    lock.release()
    
    statusD[chunkID] = 3
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
                statusD[chunkID] = 4
                shouldStop = True
            else:
                if statusD[chunkID-1] == 4:
                    statusD[chunkID] = 4
                    shouldStop = True
    
    lock.acquire()
    pbari.close()
    lock.release()


def main(currentDir, las_file, temp_las_file, new_las_file, colorizeData, resizeFactor, colorBalance, doDistanceSearch, io_values):
    
    startPCP = time.time()
    imgWidth = int(io_values[1] * resizeFactor)
    imgHeight = int(io_values[2] * resizeFactor)
    
    print("************************** POINT CLOUD COLORIZATION (V1.3.0) ***************************\n")

    lasFileOrg = laspy.file.File(currentDir / las_file, mode = "r")
    
    record_length = lasFileOrg.point_format.rec_len
    
    num_records = len(lasFileOrg.points)
    total_size = record_length * num_records
    split_files = 0
    split_at = [0]
    
    if total_size >= 2**31:
        ratio = total_size / (2**31-1)
        split_files = int(np.ceil(ratio))
    
    points_per_file = int(np.floor(num_records / split_files))
    for i in range(0,split_files-1):
        split_at.append(points_per_file * (i+1))
        
    split_at.append(num_records)
    
    num_dims = 0
    for dim in lasFileOrg.point_format:
        num_dims += 1
    
    new_path = Path(new_las_file).parents[0]
    new_name = Path(new_las_file).stem
    new_ext = Path(new_las_file).suffix
    
    lasFileOrg.close()

    if split_files > 1:
        print("           IMPORTANT NOTICE: The colorized point cloud data will be exported\n                             into " + str(split_files) + " new LAS files, one for each Data Chunk.")

    for k in range(0,split_files):
        if split_files > 1:
            print("\n   ---------------------------------  DATA CHUNK " + chr(k+65) + "  ---------------------------------\n")
        print("   Please wait, copying the necessary data from the original LAS file: " + las_file.name + "\n")
        
        lasFileOrg = laspy.file.File(currentDir / las_file, mode = "r")
        
        if split_files > 1:
            new_las_file = new_path / (new_name + "_" + chr(k+65) + new_ext)
    
        hdr = copy.copy(lasFileOrg.header)
        my_vlrs = copy.copy(lasFileOrg.header.vlrs)
        
        hdr.version = "1.2"
        hdr.format = 1.2
        hdr.data_format_id = 3
        hdr.pt_dat_format_id = 3
        
        #the following ID fields must be less than or equal to 32 characters in length
        System_ID = "OpenMMS"
        Software_ID = "OpenMMS v1.3.0"
        
        if len(System_ID) < 32:
            missingLength = 32 - len(System_ID)
            for i in range(0,missingLength):
                System_ID += " "
        
        if len(Software_ID) < 32:
            missingLength = 32 - len(Software_ID)
            for i in range(0,missingLength):
                Software_ID += " "
        
        hdr.system_id = System_ID
        hdr.software_id = Software_ID
        
        lasFileTemp = laspy.file.File(currentDir / temp_las_file, mode = "w", vlrs = my_vlrs, header = hdr)
        lasFileTemp.define_new_dimension(name="color_class",data_type=1,description="")
        lasFileTemp.define_new_dimension(name="image_used",data_type=3,description="")
        
        pbar = tqdm(unit=" dimensions", unit_scale=1, total=num_dims, desc = "      ", ncols=87)
        
        for dimension in lasFileOrg.point_format:
            
            dat = lasFileOrg.reader.get_dimension(dimension.name)
            dat = dat[split_at[k]:split_at[k+1]]
            lasFileTemp.writer.set_dimension(dimension.name, dat)
            pbar.update()
    
        pbar.close()
        
        lasFileOrg.close()
        lasFileTemp.close()
        time.sleep(1)
        
        lasFile = laspy.file.File(str(temp_las_file), mode = "rw")
    
        numPoints = len(lasFile.X)    
        scales = lasFile.header.scale
        offsets = lasFile.header.offset
        
        #https://docs.python.org/3/library/array.html#module-array for mp Array datatypes
        
        x = mp.RawArray('d',numPoints)
        y = mp.RawArray('d',numPoints)
        z = mp.RawArray('d',numPoints)
        ptTimes = mp.RawArray('d',numPoints)
        
        print("\n      Reading X coordinates ...")
        xt = np.frombuffer(x,'d')
        xt[:] = lasFile.X * scales[0] + offsets[0]
        
        print("      Reading Y coordinates ...")
        yt = np.frombuffer(y,'d')
        yt[:] = lasFile.Y * scales[1] + offsets[1]
        
        print("      Reading Z coordinates ...\n")
        zt = np.frombuffer(z,'d')
        zt[:] = lasFile.Z * scales[2] + offsets[2]
        
        ptTimes_t = np.frombuffer(ptTimes,'d')
        ptTimes_t[:] = lasFile.gps_time
        
        if split_files > 1:
            print("      Total Points: " + '{:,}'.format(numPoints) + "  (Chunk " + str(k+1) + " of " + str(split_files) + ")\n") 
        else:
            print("      Total Points: " + '{:,}'.format(numPoints) + "\n") 
    
        print("   Reading images metadata ...\n")
        pbar = tqdm(unit=" images", unit_scale=1, total=len(imagesData), desc = "      ", ncols=87)
        timeDiffs = []
        lastTime = 0
        
        rotData = []
        camTimeData = []
        
        cameraData = np.zeros((len(colorizeData),3),dtype=np.float)
        
        for i in range(0, len(colorizeData)):
            pbar.update()
            
            EOPdata_i = colorizeData[i]
            R = np.dot(formR3(float(EOPdata_i[7])),np.dot(formR2(float(EOPdata_i[6])),formR1(float(EOPdata_i[5]))))
            rotData.append(R)
            camTimeData.append(float(EOPdata_i[0]))
            cameraData[i,0] = EOPdata_i[2]
            cameraData[i,1] = EOPdata_i[3]
            cameraData[i,2] = EOPdata_i[4]
            
            if i > 0:
                timeDiff = float(EOPdata_i[0]) - lastTime
                if timeDiff < -86390.0:
                    timeDiff += 86400.0
                elif timeDiff < -3590:
                    timeDiff += 3600.0
                timeDiffs.append(timeDiff)
                
            lastTime = float(EOPdata_i[0])
    
        averageCamInterval = np.average(timeDiffs)
        
        pbar.close()
        print("\n      Average Camera Interval = " + str(round(averageCamInterval,2)) + " secs\n\n")    
        
        numCores = mp.cpu_count() - 1
        
        #debugging purposes
    #    numCores = 4
        
        designNum = 0
        
        run = True
        while run:
            designNum = numPoints // numCores
            if designNum < 1000000: 
                numCores -= 1
                if numCores == 0:
                    numCores = 1
                    run = False
            else:
                run = False
        
        chunkIndex = [0]
        indexCount = designNum
        for i in range(1,numCores):
            chunkIndex.append(indexCount)
            indexCount += designNum
        chunkIndex.append(numPoints)
        
        
        ######################## FIND CLOSEST IMAGES BY TIME ############################
        camera1 = mp.RawArray('H',numPoints)
        camera2 = mp.RawArray('H',numPoints)
        camera3 = mp.RawArray('H',numPoints)
        firstCamIndex = -1
        lastCamIndex = -1
        
        with mp.Manager() as manager1: 
            statusD = manager1.dict()
            minCam = manager1.dict()
            maxCam = manager1.dict()
            lock1 = mp.Lock()
                
            for i in range(0,len(chunkIndex)-1):
                statusD[i] = 0
            
            procs1 = []
            for i in range(0,len(chunkIndex)-1):
                sI = chunkIndex[i]
                eI = chunkIndex[i+1]
                proc1 = mp.Process(target=timeImagesChunk,args=(statusD, minCam, maxCam, camera1, camera2, camera3, lock1, i, sI, eI, ptTimes, camTimeData))
                procs1.append(proc1)
            
            message = "   Determining the 3 closest images for each LAS point (time based) ...\n"
            if colorBalance == 1:
                message = "   Determining the closest image for each LAS point (time based) ...\n"
            elif colorBalance == 2:
                message = "   Determining the 2 closest images for each LAS point (time based) ...\n"
                
            print(message)  
            
            for iproc in procs1:
                iproc.start()
            
            for iproc in procs1:
                iproc.join()
            
            firstCamIndex = int(np.min(list(minCam.values())))
            lastCamIndex = int(np.max(list(maxCam.values())))
            
            numImages = (lastCamIndex-firstCamIndex)+1
            firstFile = Path(colorizeData[firstCamIndex][1]).name
            lastFile = Path(colorizeData[lastCamIndex][1]).name
            print("\n      First image used: " + firstFile)
            print("       Last image used: " + lastFile)
            print("      Num. images used: " + str(numImages))
        
        
        ######################## LOADING IMAGES ###############################
        
        imageDataR = mp.RawArray('B',int(imgWidth * imgHeight * numImages))
        imageDataG = mp.RawArray('B',int(imgWidth * imgHeight * numImages))
        imageDataB = mp.RawArray('B',int(imgWidth * imgHeight * numImages))
        
        imageChunks = [firstCamIndex]
        imgChunkSize = numImages // numCores
        equalImagesCount = imgChunkSize * numCores
        diffImage = numImages - equalImagesCount
        imgCount = firstCamIndex
        for i in range(1,numCores):
            adj = 0
            if diffImage != 0:
                adj = 1
                diffImage -= 1
            imgCount += imgChunkSize + adj
            imageChunks.append(imgCount)
        imageChunks.append(firstCamIndex + numImages)
        
        with mp.Manager() as manager5: 
            statusD = manager5.dict()
            lock5 = mp.Lock()
                
            for i in range(0,len(imageChunks)-1):
                statusD[i] = 0
            
            procs2 = []
            for i in range(0,len(imageChunks)-1):
                sI = imageChunks[i]
                eI = imageChunks[i+1]
                proc2 = mp.Process(target=loadImagesChunk,args=(firstCamIndex, imgWidth, imgHeight, statusD, lock5, i, sI, eI, colorizeData, imageDataR, imageDataG, imageDataB))
                procs2.append(proc2)
            
            print("\n\n   Loading images data into shared memory (images found to be resized by " + str(resizeFactor) + ") ...\n")
            
            for iproc in procs2:
                iproc.start()
            
            for iproc in procs2:
                iproc.join()   
    
    
        ######################## COLORIZE POINTS BY TIME ############################    
        colorClass = mp.RawArray('B',numPoints)
        imageUsed = mp.RawArray('H',numPoints)
        red = mp.RawArray('B',numPoints)
        green = mp.RawArray('B',numPoints)
        blue = mp.RawArray('B',numPoints)
        
        with mp.Manager() as manager2:
            statusD = manager2.dict()
            lock2 = mp.Lock()
        
            for i in range(0,len(chunkIndex)-1):
                statusD[i] = 0
            
            procs2 = []
            for i in range(0,len(chunkIndex)-1):
                sI = chunkIndex[i]
                eI = chunkIndex[i+1]
    
                proc2 = mp.Process(target=timecolorizeChunk,args=(firstCamIndex, colorBalance, resizeFactor, statusD, colorClass, imageUsed, red, green, blue, lock2, i, x, y, z, camera1, camera2, camera3, sI, eI, cameraData, rotData, imageDataR, imageDataG, imageDataB, io_values))
                procs2.append(proc2)
            
            message = "\n\n   Colorizing LAS points using color balance from 3 closest images (time based) ...\n"
            if colorBalance == 1:
                message = "\n\n   Colorizing LAS points using color from closest image (time based) ...\n"
            elif colorBalance == 2:
                message = "\n\n   Colorizing LAS points using color balance from 2 closest images (time based) ...\n"
                
            print(message)
            
            for iproc in procs2:
                iproc.start()
            
            for iproc in procs2:
                iproc.join()
        
        currentcolorClass = np.frombuffer(colorClass,'B')
        
        notcoloredPts_length = len(currentcolorClass) - len(np.nonzero(currentcolorClass)[0])
        notcoloredPts = mp.RawArray('L',notcoloredPts_length)
        notcoloredPts_t = np.frombuffer(notcoloredPts,'L')
        
        ncCount = 0
        for i in range(0,numPoints):
            if not currentcolorClass[i]:
                notcoloredPts_t[ncCount] = i
                ncCount += 1
          
        ######## IMAGES DISTANCE SEARCH AND COLORIZATION ########
        if doDistanceSearch:
            
            notcoloredChunks = [0]
            idealChunkSize = notcoloredPts_length // numCores
            chunkCount = 0
            for i in range(1,numCores):
                chunkCount += idealChunkSize
                notcoloredChunks.append(chunkCount)
            notcoloredChunks.append(notcoloredPts_length)
            
            
            ######################## FIND CLOSEST IMAGES BY DISTANCE ############################
            camera4 = mp.RawArray('H',notcoloredPts_length)
            camera5 = mp.RawArray('H',notcoloredPts_length)
            camera6 = mp.RawArray('H',notcoloredPts_length)
            
            with mp.Manager() as manager3:
                statusD = manager3.dict()
                lock3 = mp.Lock()
        
                #Setup Search Tree
                cameraData_twoDim = cameraData[firstCamIndex:(lastCamIndex+1),0:2]
                images_ballTree = BallTree(cameraData_twoDim,leaf_size=2)
                
                for i in range(0,len(notcoloredChunks)-1):
                    statusD[i] = 0
                
                procs3 = []
                for i in range(0,len(notcoloredChunks)-1):
                    sI = notcoloredChunks[i]
                    eI = notcoloredChunks[i+1]
                    
                    proc3 = mp.Process(target=distImagesChunk,args=(firstCamIndex, notcoloredPts, statusD, sI, eI, camera4, camera5, camera6, lock3, i, x, y, images_ballTree))
                    procs3.append(proc3)
                
                message = "\n\n   Determining the 3 closest images for each uncolored LAS point (dist. based) ...\n"
                if colorBalance == 1:
                    message = "\n\n   Determining the closest image for each uncolored LAS point (dist. based) ...\n"
                elif colorBalance == 2:
                    message = "\n\n   Determining the 2 closest images for each uncolored LAS point (dist. based) ...\n"
                    
                print(message)  
                
                for iproc in procs3:
                    iproc.start()
                
                for iproc in procs3:
                    iproc.join()
            
            distcolorPoints = mp.RawArray('L',int(notcoloredPts_length * 4))
            distcolorPoints_t = np.frombuffer(distcolorPoints,'L').reshape((notcoloredPts_length,4))
    
            distcolorPoints_t[:,0] = notcoloredPts
            distcolorPoints_t[:,1] = camera4
            distcolorPoints_t[:,2] = camera5
            distcolorPoints_t[:,3] = camera6
            
            #sort 2D array based on closest image (2nd column)
            distcolorPoints_t = distcolorPoints_t[distcolorPoints_t[:,1].argsort()]          
            
            ######################## COLORIZE POINTS BY DISTANCE ############################
            
            with mp.Manager() as manager4:
                statusD = manager4.dict()
                lock4 = mp.Lock()
                
                for i in range(0,len(notcoloredChunks)-1):
                    statusD[i] = 0
                
                procs4 = []
                for i in range(0,len(notcoloredChunks)-1):
                    sI = notcoloredChunks[i]
                    eI = notcoloredChunks[i+1]
    
                    proc4 = mp.Process(target=distcolorizeChunk,args=(notcoloredPts_length, firstCamIndex, distcolorPoints, colorBalance, resizeFactor, statusD, colorClass, imageUsed, red, green, blue, lock4, i, x, y, z, sI, eI, cameraData, rotData, imageDataR, imageDataG, imageDataB, io_values))
                    procs4.append(proc4)
                
        
                message = "\n\n   Colorizing LAS points using color balance from 3 closest images (dist. based) ...\n"
                if colorBalance == 1:
                    message = "\n\n   Colorizing LAS points using color from closest image (dist. based) ...\n"
                elif colorBalance == 2:
                    message = "\n\n   Colorizing LAS points using color balance from 2 closest images (dist. based) ...\n"
                    
                print(message)
                
                for iproc in procs4:
                    iproc.start()
                
                for iproc in procs4:
                    iproc.join()
                
                notcoloredPts_length = len(currentcolorClass) - len(np.nonzero(currentcolorClass)[0])
                 
        notcoloredPerc = np.round((notcoloredPts_length/len(x))*100.0,2)
        
        if len(notcoloredPts):
            print("\n\n   WARNING: (" + str(notcoloredPerc) + "%) " + '{:,}'.format(notcoloredPts_length) + " points were not colorized ...\n")
    
        if split_files > 1:
            print("\n   Saving colorized points to a new LAS file: " + new_name + "_" + chr(k+65) + new_ext + "\n")
        else:
            print("\n   Saving colorized points to a new LAS file: " + new_name + new_ext + "\n")

        #ADD color CLASS and IMAGE USED ATTRIBUTES TO ALL POINTS
        lasFile.color_class = colorClass
        lasFile.image_used = imageUsed
        lasFile.red = red 
        lasFile.green = green 
        lasFile.blue = blue 
        
        lasFile.close()
        time.sleep(0.5)
        
        if os.path.isfile(new_las_file):
            os.remove(new_las_file)
            time.sleep(1)
        
        os.rename(temp_las_file,new_las_file)

    endPCP = time.time()
    print("\n***************************** DONE COLORIZING in " + str(round((endPCP - startPCP) / 60.,2)) + " mins. " + "*****************************")
    iHeartLidar2()


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

### command line start
if __name__ == "__main__":
    
    print("\nColorizing Point Cloud Started...")
    OpenMMS1()
    
    currentDir = Path(sys.argv[1])
    las_file = Path(sys.argv[2])
    colorBalance = int(sys.argv[3])
    doDistanceSearchAfter = sys.argv[4]
    io_values = []
    
    if os.path.isfile(currentDir / "camera_io.csv"):
        camera_io = open(currentDir / "camera_io.csv", "r")
        for lines in camera_io:
            io_values = lines.strip("\n").split(",")
            break

        if len(io_values) == 11:

            io_values = np.array(io_values,dtype=np.float64)
            # for i in range(0,len(io_values)):
            #     io_values[i] = float(io_values[i])
            
            os.chdir(currentDir)
            las_path = las_file.parents[0]
            
            doDistance = False
            if doDistanceSearchAfter.upper() == "TRUE":
                doDistance = True
        
            if colorBalance <= 0:
                colorBalance = 1
            elif colorBalance > 3:
                colorBalance = 3
        
            currentDir_parts = list(currentDir.parts)[1:]
        
            try:
                resizeFactorStr = currentDir_parts[-1]
                resizeFactorParts = resizeFactorStr.split("_")
                resizeFactor = float(resizeFactorParts[-1])
            except:
                resizeFactor = -1.0
            
            if resizeFactor != -1.0:
                resizeFactor = np.round(resizeFactor,3)
                if os.path.isfile(las_file):
                    las_extension = las_file.suffix
                    new_las_file = las_path / (las_file.stem + "_RGB" + str(colorBalance) + "_" + str(resizeFactor) + las_extension)
        
                    if os.path.isfile(new_las_file):
                        os.remove(new_las_file)
                        time.sleep(0.5)
                    temp_las_file = las_path / (las_file.stem + "_temp" + las_extension)
                    if os.path.isfile(temp_las_file):
                        os.remove(temp_las_file)
                        time.sleep(0.5)
        
                    if os.path.isfile(currentDir / "images_colorize.csv"):
                        imagesData = []
                        count = 0
                        with open(currentDir / "images_colorize.csv", "r") as f:
                            previousTime = None
                            for line in nonblank_lines(f):
                                if line.startswith('#') == False:
                                    terms = line.strip("\n").split(",")
                                    record = []
                                    for i in range(0,len(terms)):
                                        if i == 0:
                                            currentTime = float(terms[0])
                                            if previousTime:
                                                timeDiff = currentTime - previousTime
                                                if timeDiff < -86390:
                                                    currentTime += 86400.0
                                                elif timeDiff < -3590.0:
                                                    currentTime += 3600.0
                                                terms[0] = str(currentTime)
                                            terms[1] = str(currentDir / terms[1])
                                            previousTime = currentTime
                                        record.append(terms[i])
                                    imagesData.append(record)
                                    
                        main(currentDir, las_file, temp_las_file, new_las_file, imagesData, resizeFactor, colorBalance, doDistance, io_values)
                                        
                    else:
                        print("\n\n*******************************************************************************************")
                        print("*** It appears that Image Pre-processing has not been performed, you must do this first! ***")
                        print("*******************************************************************************************\n\n")
                else:
                    print("\n\n*******************************************************************************************")
                    print("**** Specified LAS Path and Filename appears incorrect, check the trigger file inputs *****")
                    print("*******************************************************************************************\n\n")
            else:
                print("\n\n***********************************************************************************************************")
                print("****** The resize factor could not be determined from the current directory name, did you rename it? ******")
                print("***********************************************************************************************************\n\n")
        else:
            print("\n\n*******************************************************************************************")
            print("* The camera_io.csv file does not contain the expected data? Pre-process the images again *")
            print("*******************************************************************************************\n\n")
    else:
        print("\n\n*******************************************************************************************")
        print("*** It appears that Image Pre-processing has not been performed, you must do this first! ***")
        print("*******************************************************************************************\n\n")
            

