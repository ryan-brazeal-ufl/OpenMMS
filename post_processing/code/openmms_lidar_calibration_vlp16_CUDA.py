# -*- coding: utf-8 -*-

############################################################################
#                       OpenMMS Boresight Calibration                      #
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

import pycuda.autoinit
import pycuda.gpuarray as gpuarray
import skcuda.linalg as cudalinalg
from numba import cuda, vectorize
import numdifftools as nd
import numpy as np
import time
import os
import sys
import math
from pathlib import Path


def sin(a):
    return np.sin(a)

def cos(a):
    return np.cos(a)

def tan(a):
    return np.tan(a)


@vectorize(['float32(float32, float32, float32, float32, float32, float32)'], target='parallel', nopython=True)
def geoRefCalcSocsX(azi, vert, dist, distSF, aziCorr, distCorr):
    return (dist * distSF + distCorr) * math.cos(vert) * (math.sin(azi)*math.cos(aziCorr)-math.cos(azi)*math.sin(aziCorr))

@vectorize(['float32(float32, float32, float32, float32, float32, float32)'], target='parallel', nopython=True)
def geoRefCalcSocsY(azi, vert, dist, distSF, aziCorr, distCorr):
    return (dist * distSF + distCorr) * math.cos(vert) * (math.cos(azi)*math.cos(aziCorr)+math.sin(azi)*math.sin(aziCorr))

@vectorize(['float32(float32, float32, float32, float32)'], target='parallel', nopython=True)
def geoRefCalcSocsZ(vert, dist, distSF, distCorr):
    #corrIndex = int(vert) + 15
    return (dist * distSF + distCorr) * math.sin(vert) - 0.0419 * math.tan(vert)

@vectorize(['float64(float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float64)'], target='parallel', nopython=True)
def geoRefCalcGlcsX2(x0,y0,z0,R,P,H,X,Y,Z,Lat,Lon,Xg):
    sinR = math.sin(R)
    cosR = math.cos(R)
    sinP = math.sin(P)
    cosP = math.cos(P)
    sinH = math.sin(H)
    cosH = math.cos(H)
    sinX = math.sin(X)
    cosX = math.cos(X)
    sinY = math.sin(Y)
    cosY = math.cos(Y)
    sinZ = math.sin(Z)
    cosZ = math.cos(Z)
    sinLat = math.sin(Lat)
    cosLat = math.cos(Lat)
    sinLon = math.sin(Lon)
    cosLon = math.cos(Lon)
    
    return (Xg + x0*(cosY*(cosZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon)) + sinZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)))) - sinY*(cosR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)) - sinR*(-cosH*sinLon + cosLon*sinH*sinLat))) + y0*(cosX*(cosZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon))) - sinZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon))) + sinX*(cosY*(cosR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)) - sinR*(-cosH*sinLon + cosLon*sinH*sinLat)) + sinY*(cosZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon)) + sinZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)))))) + z0*(cosX*(cosY*(cosR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)) - sinR*(-cosH*sinLon + cosLon*sinH*sinLat)) + sinY*(cosZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon)) + sinZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon))))) - sinX*(cosZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon))) - sinZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon)))))

@vectorize(['float64(float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float64)'], target='parallel', nopython=True)
def geoRefCalcGlcsY2(x0,y0,z0,R,P,H,X,Y,Z,Lat,Lon,Yg):
    sinR = math.sin(R)
    cosR = math.cos(R)
    sinP = math.sin(P)
    cosP = math.cos(P)
    sinH = math.sin(H)
    cosH = math.cos(H)
    sinX = math.sin(X)
    cosX = math.cos(X)
    sinY = math.sin(Y)
    cosY = math.cos(Y)
    sinZ = math.sin(Z)
    cosZ = math.cos(Z)
    sinLat = math.sin(Lat)
    cosLat = math.cos(Lat)
    sinLon = math.sin(Lon)
    cosLon = math.cos(Lon)
    
    return (Yg + x0*(cosY*(cosZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH)) + sinZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)))) - sinY*(cosR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)) - sinR*(cosH*cosLon + sinH*sinLat*sinLon))) + y0*(cosX*(cosZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH))) - sinZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH))) + sinX*(cosY*(cosR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)) - sinR*(cosH*cosLon + sinH*sinLat*sinLon)) + sinY*(cosZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH)) + sinZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)))))) + z0*(cosX*(cosY*(cosR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)) - sinR*(cosH*cosLon + sinH*sinLat*sinLon)) + sinY*(cosZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH)) + sinZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH))))) - sinX*(cosZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH))) - sinZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH)))))
    
@vectorize(['float64(float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float64)'], target='parallel', nopython=True)
def geoRefCalcGlcsZ2(x0,y0,z0,R,P,H,X,Y,Z,Lat,Zg):
    sinR = math.sin(R)
    cosR = math.cos(R)
    sinP = math.sin(P)
    cosP = math.cos(P)
    sinH = math.sin(H)
    cosH = math.cos(H)
    sinX = math.sin(X)
    cosX = math.cos(X)
    sinY = math.sin(Y)
    cosY = math.cos(Y)
    sinZ = math.sin(Z)
    cosZ = math.cos(Z)
    sinLat = math.sin(Lat)
    cosLat = math.cos(Lat)

    return (Zg + x0*(cosY*(cosZ*(cosH*cosLat*cosP + sinLat*sinP) + sinZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat))) - sinY*(cosLat*sinH*sinR + cosR*(cosH*cosLat*sinP - cosP*sinLat))) + y0*(cosX*(cosZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat)) - sinZ*(cosH*cosLat*cosP + sinLat*sinP)) + sinX*(cosY*(cosLat*sinH*sinR + cosR*(cosH*cosLat*sinP - cosP*sinLat)) + sinY*(cosZ*(cosH*cosLat*cosP + sinLat*sinP) + sinZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat))))) + z0*(cosX*(cosY*(cosLat*sinH*sinR + cosR*(cosH*cosLat*sinP - cosP*sinLat)) + sinY*(cosZ*(cosH*cosLat*cosP + sinLat*sinP) + sinZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat)))) - sinX*(cosZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat)) - sinZ*(cosH*cosLat*cosP + sinLat*sinP))))

#read in set of points from a text file (comma delimited), data order needs to be x,y,z [in socs], Omega, Phi, Kappa, X,Y,Z [of scanner in object space]
#offset value allows for an integer offset from the start of the line to where the x_socs value is located
def readPlaneData(filename,offsets):

    count = 0
    foundZeroLonCount = 0
    with open(filename,"r") as csvData:
        for record in csvData:
            if len(record) != 0:
                if record[0:2] != "//":
                    elements = record.strip("\n").split(",")
                    if float(elements[3]) != 0:
                        count += 1
                    else:
                        foundZeroLonCount += 1
                    
    data = np.zeros((count,15))
    index = 0
    
    if foundZeroLonCount > 0:
        print(" *** WARNING: Skipped using " + str(foundZeroLonCount) + " points in " + Path(filename).name + " with Longitude = 0 values ***")
    
    laserCount = np.zeros(16)
    
    with open(filename,"r") as csvData:
        for record in csvData:
            if len(record) != 0:
                if record[0:2] != "//":
                    elements = record.strip("\n").split(",")
                    if float(elements[3]) != 0:
                        laserIndex = (int(round(float(elements[11]),0)) + 15) // 2
                        laserCount[laserIndex] += 1
                        data[index][0] = float(elements[0])     #original Xp in M
                        data[index][1] = float(elements[1])     #original Yp in M
                        data[index][2] = float(elements[2])     #original Zp in M
                        data[index][3] = float(elements[13])     #dist_socs
                        data[index][4] = np.radians(float(elements[12]))     #azi_socs  [radians]
                        data[index][5] = np.radians(float(elements[11]))     #vert_socs [radians]
                        data[index][6] = float(elements[10])     #Roll
                        data[index][7] = float(elements[9])     #Pitch
                        data[index][8] = float(elements[8])     #Heading
                        data[index][9] = float(elements[7]) + offsets[0]    #Xs in M
                        data[index][10] = float(elements[6]) + offsets[1]    #Ys in M
                        data[index][11] = float(elements[5]) + offsets[2]    #Zs in M
                        data[index][12] = float(elements[4])    #lat
                        data[index][13] = float(elements[3])    #lon
                        data[index][14] = int(round(float(elements[11]),0))     #vert_socs [degrees]
                        index+=1
                    
    return laserCount, count, data

def readPlaneFiles(curDir, planeFiles):
    
    if not os.path.isfile(curDir / "verbose_offsets.csv"):
        print("\n *** ERROR: The 'verbose_offsets.csv' file is missing from the current directory, program is stopping\n ***")
        sys.exit()
    
    offsets_file = open(curDir / "verbose_offsets.csv","r")
    offsets = offsets_file.readline().strip("\n").split(",")
    offsets[0] = float(offsets[0])
    offsets[1] = float(offsets[1])
    offsets[2] = float(offsets[2])
    
    planes = []
    totalPoints = 0
    laserCounts = np.zeros(16)
    for i in range(0,len(planeFiles)):
        laserCount, count, planeData = readPlaneData(planeFiles[i][0], offsets)
        totalPoints += count
        laserCounts += laserCount
        planes.append(planeData)
        
    return laserCounts, totalPoints, planes


def planeFit(Xs, Ys, Zs):
    count = len(Xs)

    Q = np.zeros((count,3))
    Q[:,0] = Xs
    Q[:,1] = Ys
    Q[:,2] = Zs

    #GPU processing
    Qa = np.ascontiguousarray(Q.T,dtype=np.float32)
    Qcuda = gpuarray.to_gpu(Qa)
    vt2, _, _ = cudalinalg.svd(Qcuda)
    vt2 = vt2.T
    planeNormal = -vt2.get()[2]
    
    rmsi = 0.0
    pVec = np.empty(3)
    for ii in range(0,count):
        pVec[0] = Q[ii,0]
        pVec[1] = Q[ii,1]
        pVec[2] = Q[ii,2]
        d = planeNormal.dot(pVec)

        rmsi += d**2
        
    rmsErr = np.sqrt(rmsi / float(count))

    return rmsErr


def planeFiti(X_norm, Y_norm, Z_norm):
    count = len(X_norm)

    Q = np.empty((count,3))
    Q[:,0] = X_norm
    Q[:,1] = Y_norm
    Q[:,2] = Z_norm

    #GPU processing
    Qa = np.ascontiguousarray(Q.T,dtype=np.float32)
    Qcuda = gpuarray.to_gpu(Qa)
    vt2, _, _ = cudalinalg.svd(Qcuda)
    vt2 = vt2.T
    planeNormal = -vt2.get()[2]
    
    pVec = np.empty(3)
    d_values = np.empty(count)
    for ii in range(0,count):
        pVec[0] = Q[ii,0]
        pVec[1] = Q[ii,1]
        pVec[2] = Q[ii,2]
        d = planeNormal.dot(pVec)
        d_values[ii] = d

    return d_values


def reCalcPlanePoints(cd,bx,by,bz,fileName="",export=False,timeStr=""):
    
    dist = cd[0]
    azi = cd[1]
    vert = cd[2]
    distSF = 1.0
    distCorr = 0.0
    aziCorr = 0.0
    
    x0 = geoRefCalcSocsX(azi, vert, dist, distSF, aziCorr, distCorr)
    y0 = geoRefCalcSocsY(azi, vert, dist, distSF, aziCorr, distCorr)
    z0 = geoRefCalcSocsZ(vert, dist, distSF, distCorr)
    
    #x0,y0,z0,R,P,H,X,Y,Z,Lat,Lon,Xg
    newPts_X = geoRefCalcGlcsX2(x0,y0,z0,cd[3],cd[4],cd[5],bx,by,bz,cd[6],cd[7],cd[8])
    newPts_Y = geoRefCalcGlcsY2(x0,y0,z0,cd[3],cd[4],cd[5],bx,by,bz,cd[6],cd[7],cd[9])
    newPts_Z = geoRefCalcGlcsZ2(x0,y0,z0,cd[3],cd[4],cd[5],bx,by,bz,cd[6],cd[10])
    
    avgx = np.average(newPts_X)
    avgy = np.average(newPts_Y)
    avgz = np.average(newPts_Z)
    
    newPts_X_norm = newPts_X - avgx
    newPts_Y_norm = newPts_Y - avgy
    newPts_Z_norm = newPts_Z - avgz

    if export:
        exportFile = open(fileName[:-4] + "_bore_" + timeStr + ".txt","w")
        exportFile.write("//X,Y,Z\n")
        for i in range(0,len(newPts_X)):
            exportFile.write(str(round(newPts_X[i],4))+","+str(round(newPts_Y[i],4))+","+str(round(newPts_Z[i],4))+"\n")
        exportFile.close()
        
    return np.asarray(newPts_X_norm, np.float32), np.asarray(newPts_Y_norm, np.float32), np.asarray(newPts_Z_norm, np.float32)
    

def reCalcPlanePoints2(cd,bx,by,bz,vlp_params,fileName="",export=False,timeStr=""):
    
    dist = cd[0]
    azi = cd[1]
    vert = cd[11]
    adjVert = np.zeros(len(vert),dtype=np.float32)
    distSF = np.ones(len(vert),dtype=np.float32)
    aziCorr = np.zeros(len(vert),dtype=np.float32)
    
    for i in range(0,len(vert)):
        laserIndex = (vert[i] + 15) // 2
        adjVert[i] = vlp_params[laserIndex]
        distSF[i] = vlp_params[laserIndex+16]
        aziCorr[i] = vlp_params[laserIndex+32]
    
    distCorr = 0.0
    
    x0 = geoRefCalcSocsX(azi, adjVert, dist, distSF, aziCorr, distCorr)
    y0 = geoRefCalcSocsY(azi, adjVert, dist, distSF, aziCorr, distCorr)
    z0 = geoRefCalcSocsZ(adjVert, dist, distSF, distCorr)
    
    #x0,y0,z0,R,P,H,X,Y,Z,Lat,Lon,Xg
    newPts_X = geoRefCalcGlcsX2(x0,y0,z0,cd[3],cd[4],cd[5],bx,by,bz,cd[6],cd[7],cd[8])
    newPts_Y = geoRefCalcGlcsY2(x0,y0,z0,cd[3],cd[4],cd[5],bx,by,bz,cd[6],cd[7],cd[9])
    newPts_Z = geoRefCalcGlcsZ2(x0,y0,z0,cd[3],cd[4],cd[5],bx,by,bz,cd[6],cd[10])
    
    avgx = np.average(newPts_X)
    avgy = np.average(newPts_Y)
    avgz = np.average(newPts_Z)
    
    newPts_X_norm = newPts_X - avgx
    newPts_Y_norm = newPts_Y - avgy
    newPts_Z_norm = newPts_Z - avgz

    if export:
        exportFile = open(fileName[:-4] + "_ECEF_" + timeStr + ".txt","w")
        exportFile.write("//X,Y,Z\n")
        for i in range(0,len(newPts_X)):
            exportFile.write(str(round(newPts_X[i],4))+","+str(round(newPts_Y[i],4))+","+str(round(newPts_Z[i],4))+"\n")
        exportFile.close()
        
    return np.asarray(newPts_X_norm, np.float32), np.asarray(newPts_Y_norm, np.float32), np.asarray(newPts_Z_norm, np.float32)
    

def planesRMS(cuda_data,boresights,doPrint):
    bx = boresights[0]
    by = boresights[1]
    bz = boresights[2]
    if doPrint:
        print()
        
    initialRMSvalues = []
    for i in range(0,len(cuda_data)):
        Xs, Ys, Zs = reCalcPlanePoints(cuda_data[i],bx,by,bz)
        rmsErr = planeFit(Xs, Ys, Zs)
        initialRMSvalues.append(rmsErr**2)
        if doPrint:
            print("              " + str(i) + "_rms:" + str(round(rmsErr,4)))
    
    rms = np.sqrt(np.average(np.asarray(initialRMSvalues)))
    
    if doPrint:
        print()
        
    return rms


def planesRMS2(cuda_data,boresights,planeFiles,export=False,timeStr=""):
    bx = boresights[0]
    by = boresights[1]
    bz = boresights[2]
    
    rmsValues = []
    pts_per_plane = []
    for i in range(0,len(cuda_data)):
        Xs, Ys, Zs = reCalcPlanePoints(cuda_data[i],bx,by,bz,planeFiles[i][0],export,timeStr)
        rmsErr = planeFit(Xs, Ys, Zs)
        rmsValues.append(rmsErr)
        pts_per_plane.append(len(Xs))
        
    rms_avg = np.sqrt(np.average(np.asarray(rmsValues)**2))
    weights = np.asarray(pts_per_plane) /  np.sum(np.asarray(pts_per_plane))
    rms_w_avg = np.sqrt(np.average(np.asarray(rmsValues)**2,weights=weights))
    
    return rmsValues, rms_avg, rms_w_avg


def planesRMS3(planes):
    
    rmsValues = []
    for i in range(0,len(planes)):
        plane = planes[i]
        Xs = plane[:,0]
        Ys = plane[:,1]
        Zs = plane[:,2]
        
        avgx = np.average(Xs)
        avgy = np.average(Ys)
        avgz = np.average(Zs)
        
        X_norm = Xs - avgx
        Y_norm = Ys - avgy
        Z_norm = Zs - avgz
        
        rmsErr = planeFit(X_norm, Y_norm, Z_norm)
        rmsValues.append(rmsErr)
    
    return rmsValues


def planesRMS4(cuda_data,boresights,vlp_params,planeFiles,export=False,timeStr=""):
    bx = boresights[0]
    by = boresights[1]
    bz = boresights[2]
    
    rmsValues = []
    pts_per_plane = []
    for i in range(0,len(cuda_data)):
        Xs, Ys, Zs = reCalcPlanePoints2(cuda_data[i],bx,by,bz,vlp_params,planeFiles[i][0],export,timeStr)
        rmsErr = planeFit(Xs, Ys, Zs)
        rmsValues.append(rmsErr)
        pts_per_plane.append(len(Xs))
        
    rms_avg = np.sqrt(np.average(np.asarray(rmsValues)**2))
    weights = np.asarray(pts_per_plane) /  np.sum(np.asarray(pts_per_plane))
    rms_w_avg = np.sqrt(np.average(np.asarray(rmsValues)**2,weights=weights))
    
    return rmsValues, rms_avg, rms_w_avg


def planesRMSi(cuda_data,boresights,doPrint):
    bx = boresights[0]
    by = boresights[1]
    bz = boresights[2]
    
    if doPrint:
        print()
        
    count = 0
    pts_per_plane = []
    pts_per_plane.append(0)
    for i in range(0,len(cuda_data)):
        count += len(cuda_data[i][0])
        pts_per_plane.append(count)
    
    resValues = np.empty(count)    
    for i in range(0,len(cuda_data)):
        X_norm, Y_norm, Z_norm = reCalcPlanePoints(cuda_data[i],bx,by,bz)
        d_values = planeFiti(X_norm, Y_norm, Z_norm)
        pts = 0
        for j in range(int(pts_per_plane[i]),int(pts_per_plane[i+1])):
            resValues[j] = d_values[pts]
            pts += 1
        
    return resValues


def planesRMSi2(cuda_data,boresights,vlp_params,doPrint):
    bx = boresights[0]
    by = boresights[1]
    bz = boresights[2]
    
    if doPrint:
        print()
        
    count = 0
    pts_per_plane = []
    pts_per_plane.append(0)
    for i in range(0,len(cuda_data)):
        count += len(cuda_data[i][0])
        pts_per_plane.append(count)
    
    resValues = np.empty(count)    
    for i in range(0,len(cuda_data)):
        X_norm, Y_norm, Z_norm = reCalcPlanePoints2(cuda_data[i],bx,by,bz,vlp_params)
        d_values = planeFiti(X_norm, Y_norm, Z_norm)
        pts = 0
        for j in range(int(pts_per_plane[i]),int(pts_per_plane[i+1])):
            resValues[j] = d_values[pts]
            pts += 1
        
    return resValues


def tightenPlanes(plane1,plane2,angleIncrement,bx,by,bz,iteration,doPrint=False):
    optimal_bx = 0.0
    optimal_by = 0.0
    plusMinus = 1.0
    
    if True: #iteration % 2 == 1:
        plane_a = reCalcPlanePoints(plane1,bx,by,bz)
        _, rmsErr, _ = planeFit(plane_a)
        latest_rms = rmsErr
        count = 1
        dontStop = True
        print("\n          Optimizing the boresight X and Y parameters (" + str(iteration) + ")")
        while dontStop:
            
            bx += (plusMinus * angleIncrement)
            plane_a = reCalcPlanePoints(plane1,bx,by,bz)
            _, rmsErr_a, _ = planeFit(plane_a)
        
            if count == 1:
                if rmsErr_a > latest_rms:
                    plusMinus *= -1.0
                    bx += (plusMinus * angleIncrement)
            
            if rmsErr_a > latest_rms and count > 1:
                dontStop = False
            else:
                latest_rms = rmsErr_a
                optimal_bx = bx
                if doPrint:
                    print("          RMS_X = " + str(round(latest_rms,4)))
            count += 1

    
    plane_a = reCalcPlanePoints(plane2,bx,by,bz)
    _, rmsErr, _ = planeFit(plane_a)
    latest_rms = rmsErr
    count = 1
    dontStop = True
#    print("\n          Optimizing the boresight Y parameter...")
    while dontStop:
        
        by += (plusMinus * angleIncrement)
        plane_a = reCalcPlanePoints(plane2,bx,by,bz)
        _, rmsErr, _ = planeFit(plane_a)
    
        if count == 1:
            if rmsErr > latest_rms:
                plusMinus *= -1.0
                by += (plusMinus * angleIncrement)
        
        if rmsErr > latest_rms and count > 1:
            dontStop = False
        else:
            latest_rms = rmsErr
            optimal_by = by
            if doPrint:
                print("          RMS_Y = " + str(round(latest_rms,4)))
        count += 1
    
    if True: #iteration % 2 == 1:
        plane_a = reCalcPlanePoints(plane1,bx,by,bz)
        _, rmsErr_a, _ = planeFit(plane_a)
    
    if False: #iteration % 2 == 0:
        plane_a = reCalcPlanePoints(plane1,bx,by,bz)
        _, rmsErr, _ = planeFit(plane_a)
        latest_rms = rmsErr
        count = 1
        dontStop = True
        print("\n          Optimizing the boresight X and Y parameters (" + str(iteration) + ")")
        while dontStop:
            
            bx += (plusMinus * angleIncrement)
            plane_a = reCalcPlanePoints(plane1,bx,by,bz)
            _, rmsErr_a, _ = planeFit(plane_a)
        
            if count == 1:
                if rmsErr_a > latest_rms:
                    plusMinus *= -1.0
                    bx += (plusMinus * angleIncrement)
            
            if rmsErr_a > latest_rms and count > 1:
                dontStop = False
            else:
                latest_rms = rmsErr_a
                optimal_bx = bx
                if doPrint:
                    print("          RMS_X = " + str(round(latest_rms,4)))
            count += 1
            
        plane_a = reCalcPlanePoints(plane2,bx,by,bz)
        _, rmsErr_a, _ = planeFit(plane_a)
    
    return round(optimal_bx,3), round(optimal_by,3), rmsErr_a, latest_rms


def tightenPlanes2(planes,angleIncrement,bx,by,bz,iteration,doPrint=False):
    optimal_bx = 0.0
    optimal_by = 0.0
    plusMinus = 1.0
    
    latest_rms = planesRMS(planes,bx,by,bz,False)

    if doPrint:
        print("\n          Optimizing the boresight X parameter (" + str(iteration) + ")")
    else:
        print("\n          Optimizing the boresight X and Y parameters (" + str(iteration) + ")")
        
    count = 1
    dontStop = True
    
    while dontStop:
        
        bx += (plusMinus * angleIncrement)
        rmsErr = planesRMS(planes,bx,by,bz,doPrint)
    
        if count == 1:
            if rmsErr > latest_rms:
                plusMinus *= -1.0
                bx += (plusMinus * angleIncrement)
        
        if rmsErr > latest_rms and count > 1:
            dontStop = False
            bx = optimal_bx
        else:
            latest_rms = rmsErr
            optimal_bx = bx
            if doPrint:
                print("          RMS_X = " + str(round(latest_rms,4)))
        count += 1
    
    if doPrint:
        print("\n          Optimizing the boresight Y parameter...")
    
    count = 1
    dontStop = True
        
    while dontStop:
        
        by += (plusMinus * angleIncrement)
        rmsErr = planesRMS(planes,bx,by,bz,doPrint)
    
        if count == 1:
            if rmsErr > latest_rms:
                plusMinus *= -1.0
                by += (plusMinus * angleIncrement)
        
        if rmsErr > latest_rms and count > 1:
            dontStop = False
        else:
            latest_rms = rmsErr
            optimal_by = by
            if doPrint:
                print("          RMS_Y = " + str(round(latest_rms,4)))
        count += 1
        
    return round(optimal_bx,3), round(optimal_by,3), latest_rms


def tightenPlanes3(cuda_data,angleIncrement,bx,by,bz,iteration,doPrint=False):
    optimal_bx = 0.0
    optimal_by = 0.0
    optimal_bz = 0.0
    plusMinus = 1.0

    latest_rms = planesRMS(cuda_data,bx,by,bz,False)

    if doPrint:
        print("\n          Optimizing the boresight Z parameter (" + str(iteration) + ")")
    
    count = 1
    dontStop = True
        
    while dontStop:
        
        bz += (plusMinus * angleIncrement)
        rmsErr = planesRMS(cuda_data,bx,by,bz,doPrint)
    
        if count == 1:
            if rmsErr > latest_rms:
                plusMinus *= -1.0
                bz += (plusMinus * angleIncrement)
        
        if rmsErr > latest_rms and count > 1:
            dontStop = False
        else:
            latest_rms = rmsErr
            optimal_bz = bz
            if doPrint:
                print("          RMS_Z = " + str(round(latest_rms,4)))
        count += 1
        
    if doPrint:
        print("\n          Optimizing the boresight Y parameter...")
    
    count = 1
    dontStop = True
        
    while dontStop:
        
        by += (plusMinus * angleIncrement)
        rmsErr = planesRMS(cuda_data,bx,by,bz,doPrint)
    
        if count == 1:
            if rmsErr > latest_rms:
                plusMinus *= -1.0
                by += (plusMinus * angleIncrement)
        
        if rmsErr > latest_rms and count > 1:
            dontStop = False
        else:
            latest_rms = rmsErr
            optimal_by = by
            if doPrint:
                print("          RMS_Y = " + str(round(latest_rms,4)))
        count += 1
    
    if doPrint:
        print("\n          Optimizing the boresight X parameter...")
    
    count = 1
    dontStop = True
    
    while dontStop:
        
        bx += (plusMinus * angleIncrement)
        rmsErr = planesRMS(cuda_data,bx,by,bz,doPrint)
    
        if count == 1:
            if rmsErr > latest_rms:
                plusMinus *= -1.0
                bx += (plusMinus * angleIncrement)
        
        if rmsErr > latest_rms and count > 1:
            dontStop = False
            bx = optimal_bx
        else:
            latest_rms = rmsErr
            optimal_bx = bx
            if doPrint:
                print("          RMS_X = " + str(round(latest_rms,4)))
        count += 1
    
    return round(optimal_bx,3), round(optimal_by,3), round(optimal_bz,3), latest_rms


def coincidentPlanesRMS(planeNormalA, planeCentroidA, planeNormalB, planeCentroidB):
    S = planeCentroidB - planeCentroidA
    dA = np.dot(S,planeNormalA)
    dB = np.dot(S,planeNormalB)
    rms = np.sqrt(((np.abs(np.asscalar(dA))+np.abs(np.asscalar(dB)))/2.)**2)
    
    return rms


def convergePlanesRMS(planes,matches,bx,by,bz,doPrint):
    if doPrint:
        print()
        
    newPlanes = []
    for i in range(0,len(planes)):
        newPlanes.append(reCalcPlanePoints(planes[i],bx,by,bz))
    
    initialRMSvalues = []
    completed = 0
    for i in range(0,len(matches)):
        if i > 0:
            completed += len(matches[i-1])
        for j in range(0,len(matches[i])-1):
            planeA = newPlanes[int(matches[i][j])-1]
            planeNormalA, _, planeCentroidA = planeFit(planeA)
            for k in range(j+1,len(matches[i])):
                planeB = newPlanes[int(matches[i][k])-1]
                planeNormalB, _, planeCentroidB = planeFit(planeB)

                rmsErr = coincidentPlanesRMS(planeNormalA, planeCentroidA, planeNormalB, planeCentroidB)
                initialRMSvalues.append(rmsErr**2)
                if doPrint:
                    print("              " + str(completed+j+1) + "-" + str(completed+k+1) + " dist : " + str(round(rmsErr,4)))
        
    rms = np.sqrt(np.average(np.asarray(initialRMSvalues)))
    
    if doPrint:
        print()
    
    return rms


def convergePlanesX(planes,matches,angleIncrement,bx,by,bz,iteration,doPrint=False):
    optimal_bx = 0.0
    plusMinus = 1.0
    
    latest_rms = convergePlanesRMS(planes,matches,bx,by,bz,False)
    
    count = 1
    dontStop = True
    print("\n          Optimizing the boresight X parameter (" + str(iteration) + ")")
    while dontStop:
        
        bx += (plusMinus * angleIncrement)
        rmsErr = convergePlanesRMS(planes,matches,bx,by,bz,False)
    
        if count == 1:
            if rmsErr > latest_rms:
                plusMinus *= -1.0
                bx += (plusMinus * angleIncrement)
        
        if rmsErr > latest_rms and count > 1:
            dontStop = False
        else:
            latest_rms = rmsErr
            optimal_bx = bx
            
        if doPrint:
            print("          RMS_X = " + str(round(latest_rms,4)))
        count += 1

    return round(optimal_bx,4), round(latest_rms,3)


def convergePlanesY(planes,matches,angleIncrement,bx,by,bz,iteration,doPrint=False):
    optimal_by = 0.0
    plusMinus = 1.0
    
    latest_rms = convergePlanesRMS(planes,matches,bx,by,bz,False)
    
    count = 1
    dontStop = True
    print("\n          Optimizing the boresight Y parameter (" + str(iteration) + ")")
    while dontStop:
        
        by += (plusMinus * angleIncrement)
        rmsErr = convergePlanesRMS(planes,matches,bx,by,bz,False)
    
        if count == 1:
            if rmsErr > latest_rms:
                plusMinus *= -1.0
                by += (plusMinus * angleIncrement)
        
        if rmsErr > latest_rms and count > 1:
            dontStop = False
        else:
            latest_rms = rmsErr
            optimal_by = by
            
        if doPrint:
            print("          RMS_Y = " + str(round(latest_rms,4)))
        count += 1

    return round(optimal_by,4), round(latest_rms,3)


def convergePlanesZ(planes,matches,angleIncrement,bx,by,bz,iteration,doPrint=False):
    optimal_bz = 0.0
    plusMinus = 1.0
    
    latest_rms = convergePlanesRMS(planes,matches,bx,by,bz,False)
    
    count = 1
    dontStop = True
    print("\n          Optimizing the boresight Z parameter (" + str(iteration) + ")")
    while dontStop:
        
        bz += (plusMinus * angleIncrement)
        rmsErr = convergePlanesRMS(planes,matches,bx,by,bz,False)
    
        if count == 1:
            if rmsErr > latest_rms:
                plusMinus *= -1.0
                bz += (plusMinus * angleIncrement)
        
        if rmsErr > latest_rms and count > 1:
            dontStop = False
        else:
            latest_rms = rmsErr
            optimal_bz = bz
            
        if doPrint:
            print("          RMS_Z = " + str(round(latest_rms,4)))
        count += 1

    return round(optimal_bz,4), round(latest_rms,3)


def planesTightAndDirectionRMS(planes,matches,bx,by,bz):
    
    initialTightRMSvalues = []        
    
    newPlanes = []
    for i in range(0,len(planes)):
        newPlane = reCalcPlanePoints(planes[i],bx,by,bz)
        newPlanes.append(newPlane)
        _,rmsErr,_ = planeFit(newPlane)
        initialTightRMSvalues.append(rmsErr**2)
    
    tightRMS = np.sqrt(np.average(np.asarray(initialTightRMSvalues)))
    
    initialRMSvalues = []
    for i in range(0,len(matches)):
        for j in range(0,len(matches[i])-1):
            planeA = newPlanes[int(matches[i][j])-1]
            planeNormalA, _, planeCentroidA = planeFit(planeA)
            for k in range(j+1,len(matches[i])):
                planeB = newPlanes[int(matches[i][k])-1]
                planeNormalB, _, planeCentroidB = planeFit(planeB)

                rmsErr = np.degrees(np.arccos(np.dot(planeNormalA,planeNormalB)/(np.linalg.norm(planeNormalA) * np.linalg.norm(planeNormalB))))
                rmsErr2 = np.degrees(np.arccos(np.dot(-planeNormalA,planeNormalB)/(np.linalg.norm(planeNormalA) * np.linalg.norm(planeNormalB))))
                
                if rmsErr2 < rmsErr:
                    rmsErr = rmsErr2
                
                initialRMSvalues.append(rmsErr**2)
  
    directionRMS = np.sqrt(np.average(np.asarray(initialRMSvalues)))
    
    return tightRMS, directionRMS


def corrFromCov(Cov):
    
    Corr = np.array(Cov)
    
    for ii in range(0, len(Cov)):
        for jj in range(0, len(Cov)):
            Corr[ii][jj] = Cov[ii][jj]/(np.sqrt(Cov[ii][ii])*np.sqrt(Cov[jj][jj]))
            Corr[jj][ii] = Cov[jj][ii]/(np.sqrt(Cov[ii][ii])*np.sqrt(Cov[jj][jj]))
            
    return Corr


def formatLine(message,lineLen,ending):
    newStr = message
    num = lineLen - len(message)
    if num > 0:
        for i in range(0,num-1):
            newStr += " "
        newStr += ending
    
    return newStr


def iHeartLidar1():
    print("                        XXX     XXX")
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
    print("                             X\n\n")

def iHeartLidar2(logFile):
    logFile.write("                        XXX     XXX\n")
    logFile.write("              IIIII    X   XX XX   X    L          DDDD     AAAA   RRRRR\n")
    logFile.write("                I     X      X      X   L       i  D   D   A    A  R    R\n")
    logFile.write("                I     X             X   L          D    D  A    A  R    R\n")
    logFile.write("                I      XX         XX    L       i  D    D  AAAAAA  RRRRR\n")
    logFile.write("                I        XX     XX      L       i  D    D  A    A  R  R\n")
    logFile.write("                I          XX XX        L       i  D   D   A    A  R   R\n")
    logFile.write("              IIIII         XXX         LLLLLL  i  DDDD    A    A  R    R\n")
    logFile.write("                             X\n\n\n")

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

def OpenMMS2(logFile):
    logFile.write("\n _________                                 _______ _____  ______ ______  ________\n")
    logFile.write("|\\   ___  \\  ________  ________  ________ |\\   __ \\ __  \\|\\   __ \\ __  \\|\\   ____\\\n")
    logFile.write("\\|\\  \\_|\\  \\|\\   __  \\|\\   __  \\|\\   ___  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\___|_\n")
    logFile.write(" \\|\\  \\\\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\\\|\\  \\|\\  \\|\\__\\|\\  \\|\\  \\|\\__\\|\\  \\|\\_____  \\\n")
    logFile.write("  \\|\\  \\\\|\\  \\|\\   ____\\|\\   ____\\|\\  \\\\|\\  \\|\\  \\|__|\\|\\  \\|\\  \\|__|\\|\\  \\|____|\\  \\\n")
    logFile.write("   \\|\\  \\\\_\\  \\|\\  \\___|\\|\\  \\___|\\|\\  \\\\|\\  \\|\\  \\    \\|\\  \\|\\  \\    \\|\\  \\  __\\_\\  \\\n")
    logFile.write("    \\|\\________\\|\\__\\    \\|\\______\\\\|\\__\\\\|\\__\\|\\__\\    \\|\\__\\|\\__\\    \\|\\__\\|\\_______\\\n")
    logFile.write("     \\|________|\\|___|    \\|_______|\\|___|\\|__|\\|___|    \\|__|\\|___|    \\|__|\\|________|\n\n\n")

######  PROGRAM START  #####
def main(planeFiles,internal_cal,numIts,exportFinalPlanes,logFile):
    cuda.select_device(0)
    cudalinalg.init()
    
    curDir = Path(planeFiles[0][0]).parents[0]
    
    startProcessing = time.time()         
    
    #initial boresight values [degrees]
    Bx = 0.0
    By = 0.0
    Bz = 0.0
    
    boresights = np.array((np.radians(Bx),np.radians(By),np.radians(Bz)))
    
    #initial VLP-16 internal parameters
    vlp_params = np.array((-15.0,-13.0,-11.0,-9.0,-7.0,-5.0,-3.0,-1.0,1.0,3.0,5.0,7.0,9.0,11.0,13.0,15.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))

    for i in range(0,16):
        vlp_params[i] = np.radians(vlp_params[i])
        
    print("          Reading in point observations from " + str(len(planeFiles)) + " different planes")
    logFile.write("          Reading in point observations from " + str(len(planeFiles)) + " different planes\n")
    
    laserCounts, totalPts, planes = readPlaneFiles(curDir, planeFiles)
    
    print("          Analyzing a total of " + str(totalPts) + " point observations\n")
    print("          Individual VLP-16 Laser Observations Distribution")
    print("          -------------------------------------------------")
    print("                    Laser 0: " + str(int(laserCounts[0])) + " pts (" + str(round(laserCounts[0]/totalPts*100,2)) + "%)")
    print("                    Laser 2: " + str(int(laserCounts[1])) + " pts (" + str(round(laserCounts[1]/totalPts*100,2)) + "%)")
    print("                    Laser 4: " + str(int(laserCounts[2])) + " pts (" + str(round(laserCounts[2]/totalPts*100,2)) + "%)")
    print("                    Laser 6: " + str(int(laserCounts[3])) + " pts (" + str(round(laserCounts[3]/totalPts*100,2)) + "%)")
    print("                    Laser 8: " + str(int(laserCounts[4])) + " pts (" + str(round(laserCounts[4]/totalPts*100,2)) + "%)")
    print("                   Laser 10: " + str(int(laserCounts[5])) + " pts (" + str(round(laserCounts[5]/totalPts*100,2)) + "%)")
    print("                   Laser 12: " + str(int(laserCounts[6])) + " pts (" + str(round(laserCounts[6]/totalPts*100,2)) + "%)")
    print("                   Laser 14: " + str(int(laserCounts[7])) + " pts (" + str(round(laserCounts[7]/totalPts*100,2)) + "%)")
    print("                    Laser 1: " + str(int(laserCounts[8])) + " pts (" + str(round(laserCounts[8]/totalPts*100,2)) + "%)")
    print("                    Laser 3: " + str(int(laserCounts[9])) + " pts (" + str(round(laserCounts[9]/totalPts*100,2)) + "%)")
    print("                    Laser 5: " + str(int(laserCounts[10])) + " pts (" + str(round(laserCounts[10]/totalPts*100,2)) + "%)")
    print("                    Laser 7: " + str(int(laserCounts[11])) + " pts (" + str(round(laserCounts[11]/totalPts*100,2)) + "%)")
    print("                    Laser 9: " + str(int(laserCounts[12])) + " pts (" + str(round(laserCounts[12]/totalPts*100,2)) + "%)")
    print("                   Laser 11: " + str(int(laserCounts[13])) + " pts (" + str(round(laserCounts[13]/totalPts*100,2)) + "%)")
    print("                   Laser 13: " + str(int(laserCounts[14])) + " pts (" + str(round(laserCounts[14]/totalPts*100,2)) + "%)")
    print("                   Laser 15: " + str(int(laserCounts[15])) + " pts (" + str(round(laserCounts[15]/totalPts*100,2)) + "%)\n\n")
    
    logFile.write("          Analyzing a total of " + str(totalPts) + " point observations\n\n")
    logFile.write("          Individual VLP-16 Laser Observations Distribution\n")
    logFile.write("          -------------------------------------------------\n")
    logFile.write("                    Laser 0: " + str(int(laserCounts[0])) + " pts (" + str(round(laserCounts[0]/totalPts*100,2)) + "%)\n")
    logFile.write("                    Laser 2: " + str(int(laserCounts[1])) + " pts (" + str(round(laserCounts[1]/totalPts*100,2)) + "%)\n")
    logFile.write("                    Laser 4: " + str(int(laserCounts[2])) + " pts (" + str(round(laserCounts[2]/totalPts*100,2)) + "%)\n")
    logFile.write("                    Laser 6: " + str(int(laserCounts[3])) + " pts (" + str(round(laserCounts[3]/totalPts*100,2)) + "%)\n")
    logFile.write("                    Laser 8: " + str(int(laserCounts[4])) + " pts (" + str(round(laserCounts[4]/totalPts*100,2)) + "%)\n")
    logFile.write("                   Laser 10: " + str(int(laserCounts[5])) + " pts (" + str(round(laserCounts[5]/totalPts*100,2)) + "%)\n")
    logFile.write("                   Laser 12: " + str(int(laserCounts[6])) + " pts (" + str(round(laserCounts[6]/totalPts*100,2)) + "%)\n")
    logFile.write("                   Laser 14: " + str(int(laserCounts[7])) + " pts (" + str(round(laserCounts[7]/totalPts*100,2)) + "%)\n")
    logFile.write("                    Laser 1: " + str(int(laserCounts[8])) + " pts (" + str(round(laserCounts[8]/totalPts*100,2)) + "%)\n")
    logFile.write("                    Laser 3: " + str(int(laserCounts[9])) + " pts (" + str(round(laserCounts[9]/totalPts*100,2)) + "%)\n")
    logFile.write("                    Laser 5: " + str(int(laserCounts[10])) + " pts (" + str(round(laserCounts[10]/totalPts*100,2)) + "%)\n")
    logFile.write("                    Laser 7: " + str(int(laserCounts[11])) + " pts (" + str(round(laserCounts[11]/totalPts*100,2)) + "%)\n")
    logFile.write("                    Laser 9: " + str(int(laserCounts[12])) + " pts (" + str(round(laserCounts[12]/totalPts*100,2)) + "%)\n")
    logFile.write("                   Laser 11: " + str(int(laserCounts[13])) + " pts (" + str(round(laserCounts[13]/totalPts*100,2)) + "%)\n")
    logFile.write("                   Laser 13: " + str(int(laserCounts[14])) + " pts (" + str(round(laserCounts[14]/totalPts*100,2)) + "%)\n")
    logFile.write("                   Laser 15: " + str(int(laserCounts[15])) + " pts (" + str(round(laserCounts[15]/totalPts*100,2)) + "%)\n\n\n")


    rmsValuesInit = planesRMS3(planes)
    
    cuda_data = []
    for i in range(0,len(planes)):
        plane = planes[i]
        
        a1 = np.ascontiguousarray(plane[:,3],dtype=np.float32)
        a2 = np.ascontiguousarray(plane[:,4],dtype=np.float32)
        a3 = np.ascontiguousarray(plane[:,5],dtype=np.float32)
        a4 = np.ascontiguousarray(np.radians(plane[:,6]),dtype=np.float32)
        a5 = np.ascontiguousarray(np.radians(plane[:,7]),dtype=np.float32)
        a6 = np.ascontiguousarray(np.radians(plane[:,8]),dtype=np.float32)
        a7 = np.ascontiguousarray(np.radians(plane[:,12]),dtype=np.float32)
        a8 = np.ascontiguousarray(np.radians(plane[:,13]),dtype=np.float32)
        a9 = np.ascontiguousarray(plane[:,9],dtype=np.float64)
        a10 = np.ascontiguousarray(plane[:,10],dtype=np.float64)
        a11 = np.ascontiguousarray(plane[:,11],dtype=np.float64)
        a12 = np.ascontiguousarray(plane[:,14],dtype=np.int8)
        
        cuda_data.append([a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12])
    
    converged = False
    runCalBore = True
    for itr in range(0,numIts):
        if itr == 1 and internal_cal and converged:
            runCalBore = True
            
        if runCalBore:
            runCalBore = False
        
            print("************************* VLP-16 Lidar Boresight Calibration *************************\n")
            logFile.write("************************* VLP-16 Lidar Boresight Calibration *************************\n\n")
            
            n = 16
            converged = False
            l = None
            s0_prev = 1e10
            
            apriori = 1.0 / (0.05 ** 2)
            
            for j in range(0,10):
                if j == 1:
                    n = 10
                    
                print("          ___________________________________________________________________")
                print("          Iteration " + str(j+1) + ":")
            
                print("             Computing misclosure vector")
                l = -planesRMSi2(cuda_data,boresights,vlp_params,False)
                
                print("             Computing Jacobian matrix (please wait...)")
                fun = lambda boresights: (planesRMSi2(cuda_data,boresights,vlp_params,False))
                Jfun = nd.Jacobian(fun,step=0.0001,step_nom=1,order=n)
                fullJac  = Jfun(boresights)
                
                print("             Computing least-squares estimate")
                cor = np.linalg.lstsq(fullJac, l,rcond=None)[0]
                boresights[0] += cor[0]
                boresights[1] += cor[1]
                boresights[2] += cor[2]
                
                print("\n                New lidar boresight estimates")
                print("                Bx = " + str(np.round(np.degrees(boresights[0]),3)) + " degs")
                print("                By = " + str(np.round(np.degrees(boresights[1]),3)) + " degs")
                print("                Bz = " + str(np.round(np.degrees(boresights[2]),3)) + " degs")
                
                s02 = (np.dot((apriori * l.T),l)).item() / (len(l) - 3)
                s0 = np.sqrt(s02)
                if np.abs(s0 - s0_prev) <= 0.01:
                    converged = True
                else:
                    print("\n                s0 = " + str(np.round(s0,4)))
                    s0_prev = s0
            
                endProcessing = time.time()
                timeE = endProcessing-startProcessing
                print("\n             Elapsed Time: " + str(round((timeE)/60.,2)) + " mins\n")
                
                logFile.write("          ___________________________________________________________________\n")
                logFile.write("          Iteration " + str(j+1) + ":\n")           
                logFile.write("             Computing misclosure vector\n")
                logFile.write("             Computing Jacobian matrix (please wait...)\n")
                logFile.write("             Computing least-squares estimate\n")
                logFile.write("\n                New lidar boresight estimates\n")
                logFile.write("                Bx = " + str(np.round(np.degrees(boresights[0]),3)) + " degs\n")
                logFile.write("                By = " + str(np.round(np.degrees(boresights[1]),3)) + " degs\n")
                logFile.write("                Bz = " + str(np.round(np.degrees(boresights[2]),3)) + " degs\n")
                
                if np.abs(s0 - s0_prev) <= 0.01:
                    pass
                else:
                    logFile.write("\n                s0 = " + str(np.round(s0,4)) + "\n")

                logFile.write("\n             Elapsed Time: " + str(round((timeE)/60.,2)) + " mins\n\n")
                
                if converged:
                    break
            
            if converged:
                
                N = np.dot(np.transpose(apriori*fullJac), fullJac)
                Cov = s02 * np.linalg.inv(N)
                
                rmsValues, rms_avg, rms_w_avg = planesRMS4(cuda_data,boresights,vlp_params,planeFiles,exportFinalPlanes,timeStr)
            
                std = [np.degrees(np.sqrt(Cov[ii][ii])) for ii in range(0, len(fullJac[0,:]))]
                Corr = corrFromCov(Cov)
                
                boreFile = open(curDir / ("boresight_params_" + timeStr + ".bor"),"w")
                boreFile.write(str(np.round(np.degrees(boresights[0]),3)) + "\n")
                boreFile.write(str(np.round(np.degrees(boresights[1]),3)) + "\n")
                boreFile.write(str(np.round(np.degrees(boresights[2]),3)) + "\n")
                boreFile.close()
                
                lineLength = 74
                print("             **********  LIDAR BORESIGHT CALIBRATION CONVERGED  **********")
                print(formatLine("             *",lineLength,"*"))
                print(formatLine("             *         Bx: " + str(np.round(np.degrees(boresights[0]),3)) + " deg",lineLength,"*"))
                print(formatLine("             *         By: " + str(np.round(np.degrees(boresights[1]),3)) + " deg",lineLength,"*"))
                print(formatLine("             *         Bz: " + str(np.round(np.degrees(boresights[2]),3)) + " deg",lineLength,"*"))
                print(formatLine("             *",lineLength,"*"))
                print(formatLine("             *         Std.Dev of Unit Weight (s0): " + str(np.round(np.sqrt(s02),4)),lineLength,"*"))
                print(formatLine("             *         (NOTE: Obs. accuracy assumed = 0.05 m)",lineLength,"*"))
                print(formatLine("             *",lineLength,"*"))
                print(formatLine("             *         Std.Devs:",lineLength,"*"))
                print(formatLine("             *             s_Bx: +/- " + str(np.round(np.degrees(std[0]),3)) + " deg",lineLength,"*"))
                print(formatLine("             *             s_By: +/- " + str(np.round(np.degrees(std[1]),3)) + " deg",lineLength,"*"))
                print(formatLine("             *             s_Bz: +/- " + str(np.round(np.degrees(std[2]),3)) + " deg",lineLength,"*"))
                print(formatLine("             *",lineLength,"*"))
                print(formatLine("             *         Correlations:",lineLength,"*"))
                print(formatLine("             *             Bx <--> By: " + str(np.round(Corr[0,1],2)),lineLength,"*"))
                print(formatLine("             *             Bx <--> Bz: " + str(np.round(Corr[0,2],2)),lineLength,"*"))
                print(formatLine("             *             By <--> Bz: " + str(np.round(Corr[1,2],2)),lineLength,"*"))
                print(formatLine("             *",lineLength,"*"))
                print(formatLine("             *         Average RMS: " + str(np.round(rms_avg,3)) + " m",lineLength,"*"))
                print(formatLine("             *         Weighted Average RMS: " + str(np.round(rms_w_avg,3)) + " m",lineLength,"*"))
                print(formatLine("             *",lineLength,"*"))
                print(formatLine("             *         RMS for each plane:",lineLength,"*"))
                print(formatLine("             *             Plane:     Initial     Adjusted",lineLength,"*"))
                
                logFile.write("             **********  LIDAR BORESIGHT CALIBRATION CONVERGED  **********" + "\n")
                logFile.write(formatLine("             *",lineLength,"*") + "\n")
                logFile.write(formatLine("             *         Bx: " + str(np.round(np.degrees(boresights[0]),3)) + " deg",lineLength,"*") + "\n")
                logFile.write(formatLine("             *         By: " + str(np.round(np.degrees(boresights[1]),3)) + " deg",lineLength,"*") + "\n")
                logFile.write(formatLine("             *         Bz: " + str(np.round(np.degrees(boresights[2]),3)) + " deg",lineLength,"*") + "\n")
                logFile.write(formatLine("             *",lineLength,"*") + "\n")
                logFile.write(formatLine("             *         Std.Dev of Unit Weight (s0): " + str(np.round(np.sqrt(s02),4)),lineLength,"*") + "\n")
                logFile.write(formatLine("             *         (NOTE: Obs. accuracy assumed = 0.05 m)",lineLength,"*") + "\n")
                logFile.write(formatLine("             *",lineLength,"*") + "\n")
                logFile.write(formatLine("             *         Std.Devs:",lineLength,"*") + "\n")
                logFile.write(formatLine("             *             s_Bx: +/- " + str(np.round(np.degrees(std[0]),3)) + " deg",lineLength,"*") + "\n")
                logFile.write(formatLine("             *             s_By: +/- " + str(np.round(np.degrees(std[1]),3)) + " deg",lineLength,"*") + "\n")
                logFile.write(formatLine("             *             s_Bz: +/- " + str(np.round(np.degrees(std[2]),3)) + " deg",lineLength,"*") + "\n")
                logFile.write(formatLine("             *",lineLength,"*") + "\n")
                logFile.write(formatLine("             *         Correlations:",lineLength,"*") + "\n")
                logFile.write(formatLine("             *             Bx <--> By: " + str(np.round(Corr[0,1],2)),lineLength,"*") + "\n")
                logFile.write(formatLine("             *             Bx <--> Bz: " + str(np.round(Corr[0,2],2)),lineLength,"*") + "\n")
                logFile.write(formatLine("             *             By <--> Bz: " + str(np.round(Corr[1,2],2)),lineLength,"*") + "\n")
                logFile.write(formatLine("             *",lineLength,"*") + "\n")
                logFile.write(formatLine("             *         Average RMS: " + str(np.round(rms_avg,3)) + " m",lineLength,"*") + "\n")
                logFile.write(formatLine("             *         Weighted Average RMS: " + str(np.round(rms_w_avg,3)) + " m",lineLength,"*") + "\n")
                logFile.write(formatLine("             *",lineLength,"*") + "\n")
                logFile.write(formatLine("             *         RMS for each plane:",lineLength,"*") + "\n")
                logFile.write(formatLine("             *             Plane:     Initial     Adjusted",lineLength,"*") + "\n")
                
                for k in range(0,len(rmsValues)):
                    print(formatLine(formatLine("             *             " + str(Path(planeFiles[k][0]).name),39,"") + formatLine(str(np.round(rmsValuesInit[k],3)) + " m",13,"") + str(np.round(rmsValues[k],3)) + " m" ,lineLength,"*"))
                    logFile.write(formatLine(formatLine("             *             " + str(Path(planeFiles[k][0]).name),39,"") + formatLine(str(np.round(rmsValuesInit[k],3)) + " m",13,"") + str(np.round(rmsValues[k],3)) + " m" ,lineLength,"*") + "\n")
                
                print("             *************************************************************\n\n")
                logFile.write("             *************************************************************\n\n\n")
            else:
                print("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
                print("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
                print("   %% LIDAR BORESIGHT CALIBRATION DID NOT CONVERGE AFTER 10 ITERATIONS %%")
                print("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
                print("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n")
            
                logFile.write("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" + "\n")
                logFile.write("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" + "\n")
                logFile.write("   %% LIDAR BORESIGHT CALIBRATION DID NOT CONVERGE AFTER 10 ITERATIONS %%" + "\n")
                logFile.write("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" + "\n")
                logFile.write("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n" + "\n")
            
            #VLP-16 internal calibration adjustment
            if converged and internal_cal and itr < 1:
                print("************************** VLP-16 Lidar Internal Calibration ***************************\n")
                logFile.write("************************** VLP-16 Lidar Internal Calibration ***************************\n\n")
                
                n = 5
                converged = False
                l = None
                s0_prev = 1e10
                
                apriori = 1.0 / (0.05 ** 2)
                
                for j in range(0,10):
                    if j == 1:
                        n = 3
                        
                    print("          ________________________________________________________________")
                    print("          Iteration " + str(j+1) + ":")
                
                    print("             Computing misclosure vector")
                    l = -planesRMSi2(cuda_data,boresights,vlp_params,False)
                    
                    stepArray = np.zeros(48)
                    for i in range(0,16):
                        stepArray[i] = 0.001
                        stepArray[i+16] = 0.0001
                        stepArray[i+32] = 0.001
                    
                    print("             Computing Jacobian matrix (please wait, will take a while...)")
                    fun = lambda vlp_params: (planesRMSi2(cuda_data,boresights,vlp_params,False))
                    Jfun = nd.Jacobian(fun,step=stepArray)#,step_nom=1,order=n)
                    fullJac  = Jfun(vlp_params)
                    
                    print("             Computing least-squares estimate")
                    cor = np.linalg.lstsq(fullJac, l,rcond=None)[0]
                    
                    s02 = (np.dot((apriori * l.T),l)).item() / (len(l) - 48)
                    s0 = np.sqrt(s02)
                    if np.abs(s0 - s0_prev) <= 0.01:
                        converged = True
                    else:
                        print("\n                s0 = " + str(np.round(s0,4)))
                        s0_prev = s0
                
                    endProcessing = time.time()
                    timeE = endProcessing-startProcessing
                    print("\n             Elapsed Time: " + str(round((timeE)/60.,2)) + " mins\n")
                    
                    logFile.write("          ________________________________________________________________" + "\n")
                    logFile.write("          Iteration " + str(j+1) + ":" + "\n")
                    logFile.write("             Computing misclosure vector" + "\n")
                    logFile.write("             Computing Jacobian matrix (please wait, will take a while...)" + "\n")
                    logFile.write("             Computing least-squares estimate" + "\n")

                    if np.abs(s0 - s0_prev) <= 0.01:
                        pass
                    else:
                        logFile.write("\n                s0 = " + str(np.round(s0,4)) + "\n")
                
                    logFile.write("\n             Elapsed Time: " + str(round((timeE)/60.,2)) + " mins\n" + "\n")
                    
                    if converged:
                        break
                    
                    for i in range(0,len(cor)):
                        vlp_params[i] += cor[i]
                    
                    print("\n                New VLP-16 Lidar calibration estimates")
                    print("                Laser 0  VA = " + formatLine(str(np.round(np.degrees(vlp_params[0]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[16],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[32]),3)),7,"") + " deg")
                    print("                Laser 2  VA = " + formatLine(str(np.round(np.degrees(vlp_params[1]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[17],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[33]),3)),7,"") + " deg")
                    print("                Laser 4  VA = " + formatLine(str(np.round(np.degrees(vlp_params[2]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[18],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[34]),3)),7,"") + " deg")
                    print("                Laser 6  VA = " + formatLine(str(np.round(np.degrees(vlp_params[3]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[19],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[35]),3)),7,"") + " deg")
                    print("                Laser 8  VA = " + formatLine(str(np.round(np.degrees(vlp_params[4]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[20],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[36]),3)),7,"") + " deg")
                    print("                Laser 10 VA = " + formatLine(str(np.round(np.degrees(vlp_params[5]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[21],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[37]),3)),7,"") + " deg")
                    print("                Laser 12 VA = " + formatLine(str(np.round(np.degrees(vlp_params[6]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[22],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[38]),3)),7,"") + " deg")
                    print("                Laser 14 VA = " + formatLine(str(np.round(np.degrees(vlp_params[7]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[23],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[39]),3)),7,"") + " deg")
                    print("                Laser 1  VA = " + formatLine(str(np.round(np.degrees(vlp_params[8]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[24],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[40]),3)),7,"") + " deg")
                    print("                Laser 3  VA = " + formatLine(str(np.round(np.degrees(vlp_params[9]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[25],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[41]),3)),7,"") + " deg")
                    print("                Laser 5  VA = " + formatLine(str(np.round(np.degrees(vlp_params[10]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[26],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[42]),3)),7,"") + " deg")
                    print("                Laser 7  VA = " + formatLine(str(np.round(np.degrees(vlp_params[11]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[27],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[43]),3)),7,"") + " deg")
                    print("                Laser 9  VA = " + formatLine(str(np.round(np.degrees(vlp_params[12]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[28],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[44]),3)),7,"") + " deg")
                    print("                Laser 11 VA = " + formatLine(str(np.round(np.degrees(vlp_params[13]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[29],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[45]),3)),7,"") + " deg")
                    print("                Laser 13 VA = " + formatLine(str(np.round(np.degrees(vlp_params[14]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[30],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[46]),3)),7,"") + " deg")
                    print("                Laser 15 VA = " + formatLine(str(np.round(np.degrees(vlp_params[15]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[31],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[47]),3)),7,"") + " deg\n")
                    
                    logFile.write("\n                New VLP-16 Lidar calibration estimates" + "\n")
                    logFile.write("                Laser 0  VA = " + formatLine(str(np.round(np.degrees(vlp_params[0]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[16],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[32]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 2  VA = " + formatLine(str(np.round(np.degrees(vlp_params[1]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[17],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[33]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 4  VA = " + formatLine(str(np.round(np.degrees(vlp_params[2]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[18],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[34]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 6  VA = " + formatLine(str(np.round(np.degrees(vlp_params[3]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[19],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[35]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 8  VA = " + formatLine(str(np.round(np.degrees(vlp_params[4]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[20],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[36]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 10 VA = " + formatLine(str(np.round(np.degrees(vlp_params[5]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[21],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[37]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 12 VA = " + formatLine(str(np.round(np.degrees(vlp_params[6]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[22],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[38]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 14 VA = " + formatLine(str(np.round(np.degrees(vlp_params[7]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[23],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[39]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 1  VA = " + formatLine(str(np.round(np.degrees(vlp_params[8]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[24],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[40]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 3  VA = " + formatLine(str(np.round(np.degrees(vlp_params[9]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[25],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[41]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 5  VA = " + formatLine(str(np.round(np.degrees(vlp_params[10]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[26],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[42]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 7  VA = " + formatLine(str(np.round(np.degrees(vlp_params[11]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[27],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[43]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 9  VA = " + formatLine(str(np.round(np.degrees(vlp_params[12]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[28],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[44]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 11 VA = " + formatLine(str(np.round(np.degrees(vlp_params[13]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[29],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[45]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 13 VA = " + formatLine(str(np.round(np.degrees(vlp_params[14]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[30],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[46]),3)),7,"") + " deg" + "\n")
                    logFile.write("                Laser 15 VA = " + formatLine(str(np.round(np.degrees(vlp_params[15]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[31],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[47]),3)),7,"") + " deg\n" + "\n")
                
                if converged:
                    
                    N = np.dot(np.transpose(apriori*fullJac), fullJac)
                    Cov = s02 * np.linalg.inv(N)
                    
                    rmsValues, rms_avg, rms_w_avg = planesRMS4(cuda_data,boresights,vlp_params,planeFiles,exportFinalPlanes,timeStr)
                
                    std = [np.degrees(np.sqrt(Cov[ii][ii])) for ii in range(0, len(fullJac[0,:]))]
                    Corr = corrFromCov(Cov)
                    
                    lineLength = 90
                    print("             ****************  VLP-16 LIDAR INTERNAL CALIBRATION CONVERGED ***************")
                    print(formatLine("             *",lineLength,"*"))
                    print(formatLine("             *    Laser 0  VA = " + formatLine(str(np.round(np.degrees(vlp_params[0]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[16],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[32]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 2  VA = " + formatLine(str(np.round(np.degrees(vlp_params[1]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[17],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[33]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 4  VA = " + formatLine(str(np.round(np.degrees(vlp_params[2]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[18],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[34]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 6  VA = " + formatLine(str(np.round(np.degrees(vlp_params[3]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[19],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[35]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 8  VA = " + formatLine(str(np.round(np.degrees(vlp_params[4]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[20],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[36]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 10 VA = " + formatLine(str(np.round(np.degrees(vlp_params[5]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[21],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[37]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 12 VA = " + formatLine(str(np.round(np.degrees(vlp_params[6]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[22],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[38]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 14 VA = " + formatLine(str(np.round(np.degrees(vlp_params[7]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[23],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[39]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 1  VA = " + formatLine(str(np.round(np.degrees(vlp_params[8]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[24],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[40]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 3  VA = " + formatLine(str(np.round(np.degrees(vlp_params[9]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[25],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[41]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 5  VA = " + formatLine(str(np.round(np.degrees(vlp_params[10]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[26],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[42]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 7  VA = " + formatLine(str(np.round(np.degrees(vlp_params[11]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[27],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[43]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 9  VA = " + formatLine(str(np.round(np.degrees(vlp_params[12]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[28],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[44]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 11 VA = " + formatLine(str(np.round(np.degrees(vlp_params[13]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[29],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[45]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 13 VA = " + formatLine(str(np.round(np.degrees(vlp_params[14]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[30],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[46]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *    Laser 15 VA = " + formatLine(str(np.round(np.degrees(vlp_params[15]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[31],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[47]),3)),7,"") + " deg",lineLength,"*"))
                    print(formatLine("             *",lineLength,"*"))
                    print(formatLine("             *    Std.Dev of Unit Weight (s0): " + str(np.round(np.sqrt(s02),4)),lineLength,"*"))
                    print(formatLine("             *    (NOTE: Obs. accuracy assumed = 0.05 m)",lineLength,"*"))
                    print(formatLine("             *",lineLength,"*"))
                    print(formatLine("             *    Std.Devs:",lineLength,"*"))
                    print(formatLine("             *        s_L 0  VA: +/- " + formatLine(str(np.round(np.degrees(std[0]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[16],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[32]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 2  VA: +/- " + formatLine(str(np.round(np.degrees(std[1]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[17],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[33]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 4  VA: +/- " + formatLine(str(np.round(np.degrees(std[2]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[18],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[34]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 6  VA: +/- " + formatLine(str(np.round(np.degrees(std[3]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[19],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[35]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 8  VA: +/- " + formatLine(str(np.round(np.degrees(std[4]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[20],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[36]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 10 VA: +/- " + formatLine(str(np.round(np.degrees(std[5]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[21],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[37]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 12 VA: +/- " + formatLine(str(np.round(np.degrees(std[6]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[22],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[38]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 14 VA: +/- " + formatLine(str(np.round(np.degrees(std[7]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[23],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[39]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 1  VA: +/- " + formatLine(str(np.round(np.degrees(std[8]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[24],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[40]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 3  VA: +/- " + formatLine(str(np.round(np.degrees(std[9]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[25],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[41]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 5  VA: +/- " + formatLine(str(np.round(np.degrees(std[10]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[26],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[42]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 7  VA: +/- " + formatLine(str(np.round(np.degrees(std[11]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[27],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[43]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 9  VA: +/- " + formatLine(str(np.round(np.degrees(std[12]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[28],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[44]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 11 VA: +/- " + formatLine(str(np.round(np.degrees(std[13]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[29],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[45]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 13 VA: +/- " + formatLine(str(np.round(np.degrees(std[14]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[30],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[46]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *        s_L 15 VA: +/- " + formatLine(str(np.round(np.degrees(std[15]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[31],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[47]),3)),6,"") + " deg",lineLength,"*"))
                    print(formatLine("             *",lineLength,"*"))
                    
                    logFile.write("             ****************  VLP-16 LIDAR INTERNAL CALIBRATION CONVERGED ***************" + "\n")
                    logFile.write(formatLine("             *",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 0  VA = " + formatLine(str(np.round(np.degrees(vlp_params[0]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[16],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[32]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 2  VA = " + formatLine(str(np.round(np.degrees(vlp_params[1]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[17],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[33]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 4  VA = " + formatLine(str(np.round(np.degrees(vlp_params[2]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[18],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[34]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 6  VA = " + formatLine(str(np.round(np.degrees(vlp_params[3]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[19],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[35]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 8  VA = " + formatLine(str(np.round(np.degrees(vlp_params[4]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[20],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[36]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 10 VA = " + formatLine(str(np.round(np.degrees(vlp_params[5]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[21],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[37]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 12 VA = " + formatLine(str(np.round(np.degrees(vlp_params[6]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[22],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[38]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 14 VA = " + formatLine(str(np.round(np.degrees(vlp_params[7]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[23],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[39]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 1  VA = " + formatLine(str(np.round(np.degrees(vlp_params[8]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[24],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[40]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 3  VA = " + formatLine(str(np.round(np.degrees(vlp_params[9]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[25],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[41]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 5  VA = " + formatLine(str(np.round(np.degrees(vlp_params[10]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[26],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[42]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 7  VA = " + formatLine(str(np.round(np.degrees(vlp_params[11]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[27],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[43]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 9  VA = " + formatLine(str(np.round(np.degrees(vlp_params[12]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[28],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[44]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 11 VA = " + formatLine(str(np.round(np.degrees(vlp_params[13]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[29],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[45]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 13 VA = " + formatLine(str(np.round(np.degrees(vlp_params[14]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[30],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[46]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Laser 15 VA = " + formatLine(str(np.round(np.degrees(vlp_params[15]),3)),8,"") + " deg   SF = " + formatLine(str(np.round(vlp_params[31],5)),8,"") + "   AC = " + formatLine(str(np.round(np.degrees(vlp_params[47]),3)),7,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Std.Dev of Unit Weight (s0): " + str(np.round(np.sqrt(s02),4)),lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    (NOTE: Obs. accuracy assumed = 0.05 m)",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Std.Devs:",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 0  VA: +/- " + formatLine(str(np.round(np.degrees(std[0]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[16],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[32]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 2  VA: +/- " + formatLine(str(np.round(np.degrees(std[1]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[17],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[33]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 4  VA: +/- " + formatLine(str(np.round(np.degrees(std[2]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[18],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[34]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 6  VA: +/- " + formatLine(str(np.round(np.degrees(std[3]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[19],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[35]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 8  VA: +/- " + formatLine(str(np.round(np.degrees(std[4]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[20],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[36]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 10 VA: +/- " + formatLine(str(np.round(np.degrees(std[5]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[21],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[37]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 12 VA: +/- " + formatLine(str(np.round(np.degrees(std[6]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[22],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[38]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 14 VA: +/- " + formatLine(str(np.round(np.degrees(std[7]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[23],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[39]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 1  VA: +/- " + formatLine(str(np.round(np.degrees(std[8]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[24],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[40]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 3  VA: +/- " + formatLine(str(np.round(np.degrees(std[9]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[25],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[41]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 5  VA: +/- " + formatLine(str(np.round(np.degrees(std[10]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[26],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[42]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 7  VA: +/- " + formatLine(str(np.round(np.degrees(std[11]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[27],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[43]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 9  VA: +/- " + formatLine(str(np.round(np.degrees(std[12]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[28],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[44]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 11 VA: +/- " + formatLine(str(np.round(np.degrees(std[13]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[29],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[45]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 13 VA: +/- " + formatLine(str(np.round(np.degrees(std[14]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[30],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[46]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        s_L 15 VA: +/- " + formatLine(str(np.round(np.degrees(std[15]),3)),6,"") + " deg   SF: +/- " + formatLine(str(np.round(std[31],5)),8,"") + "   AC: +/- " + formatLine(str(np.round(np.degrees(std[47]),3)),6,"") + " deg",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *",lineLength,"*") + "\n")
                    
                    
                    corrFile = open(curDir / ("correlations_" + timeStr + ".txt"),"w")
                    for j in range(0,len(Corr)):
                        for k in range(0,j+1):
                            corrFile.write(formatLine(str(np.round(Corr[j,k],3)),7,"  "))
                        corrFile.write("\n")
                    corrFile.close()
                    
                    vlpParamsFile = open(curDir / ("vlp16_params_" + timeStr + ".cal"),"w")
                    vlpParamsFile.write("//Vertical Angles\n")
                    dataString = ""
                    for j in range(0,8):
                        dataString += (str(np.round(np.degrees(vlp_params[j]),3)) + "," + str(np.round(np.degrees(vlp_params[j+8]),3)) + ",")
                    dataString = dataString[:-1]
                    vlpParamsFile.write(dataString + "\n//Distance Scale Factors\n")
                    dataString = ""
                    for j in range(16,24):
                        dataString += (str(np.round(vlp_params[j],6)) + "," + str(np.round(vlp_params[j+8],6)) + ",")
                    dataString = dataString[:-1]
                    vlpParamsFile.write(dataString + "\n//Azimuth Corrections\n")
                    dataString = ""
                    for j in range(32,40):
                        dataString += (str(np.round(np.degrees(vlp_params[j]),3)) + "," + str(np.round(np.degrees(vlp_params[j+8]),3)) + ",")
                    dataString = dataString[:-1]
                    vlpParamsFile.write(dataString + "\n")
                    vlpParamsFile.close()
                    
                    print(formatLine("             *    Correlations:",lineLength,"*"))
                    print(formatLine("             *        See 'correlations.txt' file that has been created",lineLength,"*"))
                    print(formatLine("             *",lineLength,"*"))
                    print(formatLine("             *    Average RMS: " + str(np.round(rms_avg,3)) + " m",lineLength,"*"))
                    print(formatLine("             *    Weighted Average RMS: " + str(np.round(rms_w_avg,3)) + " m",lineLength,"*"))
                    print(formatLine("             *",lineLength,"*"))
                    print(formatLine("             *    RMS for each plane:",lineLength,"*"))
                    print(formatLine("             *        Plane:     Initial     Adjusted",lineLength,"*"))
                    
                    logFile.write(formatLine("             *    Correlations:",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        See 'correlations.txt' file that has been created",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Average RMS: " + str(np.round(rms_avg,3)) + " m",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    Weighted Average RMS: " + str(np.round(rms_w_avg,3)) + " m",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *    RMS for each plane:",lineLength,"*") + "\n")
                    logFile.write(formatLine("             *        Plane:     Initial     Adjusted",lineLength,"*") + "\n")
                    
                    for k in range(0,len(rmsValues)):
                        print(formatLine(formatLine("             *        " + str(Path(planeFiles[k][0]).name),34,"") + formatLine(str(np.round(rmsValuesInit[k],3)) + " m",13,"") + str(np.round(rmsValues[k],3)) + " m" ,lineLength,"*"))
                        logFile.write(formatLine(formatLine("             *        " + str(Path(planeFiles[k][0]).name),34,"") + formatLine(str(np.round(rmsValuesInit[k],3)) + " m",13,"") + str(np.round(rmsValues[k],3)) + " m" ,lineLength,"*") + "\n")
                        
                    print("             *****************************************************************************\n\n")
                    logFile.write("             *****************************************************************************\n\n" + "\n")

                else:
                    print("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
                    print("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
                    print("   %%% VLP-16 LIDAR CALIBRATION DID NOT CONVERGE AFTER 10 ITERATIONS %%%%")
                    print("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
                    print("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n")
                    
                    logFile.write("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" + "\n")
                    logFile.write("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" + "\n")
                    logFile.write("   %%% VLP-16 LIDAR CALIBRATION DID NOT CONVERGE AFTER 10 ITERATIONS %%%%" + "\n")
                    logFile.write("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" + "\n")
                    logFile.write("   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n" + "\n")
            

### command line start
if __name__ == "__main__":
    
    print("\nVLP-16 Lidar Sensor Calibration Started...")
    OpenMMS1()
    
    plane_files = ""
    
    curPath = Path(sys.argv[1])

    timeNow = time.localtime()
    timeStr = str(timeNow[0]) + "_" + str(timeNow[1]) + "_" + str(timeNow[2]) + "_" + str(timeNow[3]) + "_" + str(timeNow[4]) + "_" + str(timeNow[5])
    logFilename = curPath / ("LIDAR_CALIBRATION_" + timeStr + ".log")
    print("          LOG FILE: " + logFilename.name + "\n")
    logFile = open(logFilename,'w')

    OpenMMS2(logFile)

    curFiles = []

    for (dirpath, dirnames, filenames) in os.walk(str(curPath)):
        if str(curPath) == dirpath:
            curFiles = filenames

    for i in range(0,len(curFiles)):
        filename = curPath / str(curFiles[i])
        extension = filename.suffix
        if extension.upper() == ".TXT":
            checkFile = open(filename,"r")
            firstLine = checkFile.readline()
            checkFile.close()
            if firstLine.startswith("//X,Y,Z,scanner_lon,scanner_lat,scanner_z,scanner_y,scanner_x,heading,pitch,roll,vert_socs,azi_socs,dist_socs"):
                plane_files += str(curPath / filename) + ","
            
    plane_files = plane_files[:-1]
    plane_files = plane_files.split(",")
    planeFiles = []
    
    for i in range(0,len(plane_files)):
        fileAsList = [plane_files[i]]
        planeFiles.append(fileAsList)   

    internal_calibration = sys.argv[2]
    internal_cal = False
    
    if internal_calibration.upper() == "TRUE":
        internal_cal = True
    
    num_iterations = 1
    extra_bore = sys.argv[3]
    if extra_bore.upper() == "TRUE":
        num_iterations = 2
    
    export = sys.argv[4]
    exportFinalPlanes = False 
    
    if export.upper() == "TRUE":
        exportFinalPlanes = True
    
    if len(plane_files) > 0:
        main(planeFiles,internal_cal,num_iterations,exportFinalPlanes,logFile)
        iHeartLidar1()
        iHeartLidar2(logFile)
    else:
        print("\n\n************ NO VALID PLANE TXT FILES WERE FOUND IN THE CURRENT DIRECTORY ***********\n\n")
    
    logFile.close()
