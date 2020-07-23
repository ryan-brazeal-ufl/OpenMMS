# -*- coding: utf-8 -*-

############################################################################
#                  OpenMMS Camera Boresight Calibration                    #
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
import sys
import numpy as np
from numpy import sin, cos, arcsin, arctan, arctan2
from numba import vectorize
import numdifftools as nd
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


def OpenMMS2(logFile):
    logFile.write("\n _________                                 _______ _____  ______ ______  ________\n")
    logFile.write("|\\   ___  \\  ________  ________  ________ |\\   __ \\ __  \\|\\   __ \\ __  \\|\\   ____\\\n")
    logFile.write("\\|\\  \\_|\\  \\|\\   __  \\|\\   __  \\|\\   ___  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\___|_\n")
    logFile.write(" \\|\\  \\\\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\\\|\\  \\|\\  \\|\\__\\|\\  \\|\\  \\|\\__\\|\\  \\|\\_____  \\\n")
    logFile.write("  \\|\\  \\\\|\\  \\|\\   ____\\|\\   ____\\|\\  \\\\|\\  \\|\\  \\|__|\\|\\  \\|\\  \\|__|\\|\\  \\|____|\\  \\\n")
    logFile.write("   \\|\\  \\\\_\\  \\|\\  \\___|\\|\\  \\___|\\|\\  \\\\|\\  \\|\\  \\    \\|\\  \\|\\  \\    \\|\\  \\  __\\_\\  \\\n")
    logFile.write("    \\|\\________\\|\\__\\    \\|\\______\\\\|\\__\\\\|\\__\\|\\__\\    \\|\\__\\|\\__\\    \\|\\__\\|\\_______\\\n")
    logFile.write("     \\|________|\\|___|    \\|_______|\\|___|\\|__|\\|___|    \\|__|\\|___|    \\|__|\\|________|\n\n\n")



def formR1(theta):
    R1 = np.zeros((3,3),dtype=np.float64)
    st = np.sin(theta)
    ct = np.cos(theta)
    R1[0,0] = 1.
    R1[1,1] = ct
    R1[1,2] = st
    R1[2,1] = -st
    R1[2,2] = ct	
    return R1
	
def formR2(theta):
    R2 = np.zeros((3,3),dtype=np.float64)
    st = np.sin(theta)
    ct = np.cos(theta)
    R2[0,0] = ct
    R2[0,2] = -st
    R2[1,1] = 1.
    R2[2,0] = st
    R2[2,2] = ct
    return R2
	
def formR3(theta):
    R3 = np.zeros((3,3),dtype=np.float64)
    st = np.sin(theta)
    ct = np.cos(theta)
    R3[0,0] = ct
    R3[0,1] = st
    R3[1,0] = -st
    R3[1,1] = ct
    R3[2,2] = 1.
    return R3


#diffs = np.dot(R1w,np.dot(R2o,np.dot(R3k,np.dot(R2_180,np.dot(R3bz,np.dot(R2by,np.dot(R1bx,LA))))))) + ENUr - ENUc

@vectorize(['float64(float64,float64,float64,float64,float64,float64,float64,float64,float64,float64,float64,float64)'], target='parallel')
def computeF_dX(LAx,LAy,LAz,Er,Nr,Ur,Or,Pr,Kr,Ec,Nc,Uc):
    diff = Er + LAz*sin(Pr) + (-LAx*cos(Kr) + LAy*sin(Kr))*cos(Pr) - Ec

    return diff

@vectorize(['float64(float64,float64,float64,float64,float64,float64,float64,float64,float64,float64,float64,float64)'], target='parallel')
def computeF_dY(LAx,LAy,LAz,Er,Nr,Ur,Or,Pr,Kr,Ec,Nc,Uc):
    diff = Nr + (LAx*sin(Kr) + LAy*cos(Kr))*cos(Or) + (-LAz*cos(Pr) + (-LAx*cos(Kr) + LAy*sin(Kr))*sin(Pr))*sin(Or) - Nc

    return diff

@vectorize(['float64(float64,float64,float64,float64,float64,float64,float64,float64,float64,float64,float64,float64)'], target='parallel')
def computeF_dZ(LAx,LAy,LAz,Er,Nr,Ur,Or,Pr,Kr,Ec,Nc,Uc):
    diff = Ur - (LAx*sin(Kr) + LAy*cos(Kr))*sin(Or) + (-LAz*cos(Pr) + (-LAx*cos(Kr) + LAy*sin(Kr))*sin(Pr))*cos(Or) - Uc
    
    return diff

@vectorize(['float64(float64,float64,float64,float64,float64,float64,float64,float64,float64)'], target='parallel')
def computeF_dO(Bx,By,Bz,Or,Pr,Kr,Oc,Pc,Kc):
    diff = -arctan2(-(((sin(Bx)*sin(Kr)*cos(By) + sin(By)*cos(Kr))*sin(Pr) - cos(Bx)*cos(By)*cos(Pr))*sin(Or) + (sin(Bx)*cos(By)*cos(Kr) - sin(By)*sin(Kr))*cos(Or)),-(((sin(Bx)*sin(Kr)*cos(By) + sin(By)*cos(Kr))*sin(Pr) - cos(Bx)*cos(By)*cos(Pr))*cos(Or) - (sin(Bx)*cos(By)*cos(Kr) - sin(By)*sin(Kr))*sin(Or))) - Oc

    return diff

@vectorize(['float64(float64,float64,float64,float64,float64,float64,float64,float64,float64)'], target='parallel')
def computeF_dP(Bx,By,Bz,Or,Pr,Kr,Oc,Pc,Kc):
    diff = -arcsin((sin(Bx)*sin(Kr)*cos(By) + sin(By)*cos(Kr))*cos(Pr) + sin(Pr)*cos(Bx)*cos(By)) - Pc
    
    return diff

@vectorize(['float64(float64,float64,float64,float64,float64,float64,float64,float64,float64)'], target='parallel')
def computeF_dK(Bx,By,Bz,Or,Pr,Kr,Oc,Pc,Kc):
    diff = -arctan2((((sin(Bx)*sin(By)*sin(Bz) + cos(Bx)*cos(Bz))*sin(Kr) - sin(Bz)*cos(By)*cos(Kr))*cos(Pr) - (sin(Bx)*cos(Bz) - sin(By)*sin(Bz)*cos(Bx))*sin(Pr)),-(((sin(Bx)*sin(By)*cos(Bz) - sin(Bz)*cos(Bx))*sin(Kr) - cos(By)*cos(Bz)*cos(Kr))*cos(Pr) - (-sin(Bx)*sin(Bz) - sin(By)*cos(Bx)*cos(Bz))*sin(Pr))) - Kc
    
    return diff


def lambda_compute_F(x,Er,Nr,Ur,Or,Pr,Kr,Ec,Nc,Uc,Oc,Pc,Kc,numImages):
    
    LAx = x[0]
    LAy = x[1]
    LAz = x[2]
    Bx = x[3]
    By = x[4]
    Bz = x[5]
    
    results = np.zeros(numImages*6,dtype=np.float64)
    
    dX = computeF_dX(LAx,LAy,LAz,Er,Nr,Ur,-Or,-Pr,-Kr,Ec,Nc,Uc)
    dY = computeF_dY(LAx,LAy,LAz,Er,Nr,Ur,-Or,-Pr,-Kr,Ec,Nc,Uc)
    dZ = computeF_dZ(LAx,LAy,LAz,Er,Nr,Ur,-Or,-Pr,-Kr,Ec,Nc,Uc)
    dO = computeF_dO(Bx,By,Bz,-Or,-Pr,-Kr,Oc,Pc,Kc)
    dP = computeF_dP(Bx,By,Bz,-Or,-Pr,-Kr,Oc,Pc,Kc)
    dK = computeF_dK(Bx,By,Bz,-Or,-Pr,-Kr,Oc,Pc,Kc)
    
    for i in range(0,numImages):
        results[i*6] = dX[i]
        results[i*6+1] = dY[i]
        results[i*6+2] = dZ[i]
        results[i*6+3] = dO[i]
        results[i*6+4] = dP[i]
        results[i*6+5] = dK[i]
        
    return results


def corrFromCov(Cov):
    
    Corr = np.array(Cov)
    
    for ii in range(0, len(Cov)):
        for jj in range(0, len(Cov)):
            Corr[ii][jj] = Cov[ii][jj]/(np.sqrt(Cov[ii][ii])*np.sqrt(Cov[jj][jj]))
            Corr[jj][ii] = Cov[jj][ii]/(np.sqrt(Cov[ii][ii])*np.sqrt(Cov[jj][jj]))
            
    return Corr


def main(currentDir,max_images):
    
    startProcessing = time.time()
    
    OpenMMS1()
    
    for (dirpath, dirnames, filenames) in os.walk(str(currentDir)):
        if str(currentDir) == dirpath:
            curFiles = filenames

    num_ext_para_files = 0
    
    images_file = ""
    ext_para_file = ""

    for i in range(0,len(curFiles)):
        filename = currentDir / str(curFiles[i])
        name_no_ext = (filename.stem).upper()
        extension = (filename.suffix).upper()

        if extension == ".TXT":
            if "_CALIBRATED_EXTERNAL_CAMERA_PARAMETERS" in name_no_ext:
                ext_para_file = filename
                num_ext_para_files += 1

        elif extension == ".CSV":
            if name_no_ext == "IMAGES_MATCHED_TO_EVENTS":
                images_file = filename

    if num_ext_para_files == 1:
        
        timeNow = time.localtime()
        timeStr = str(timeNow[0]) + "_" + str(timeNow[1]) + "_" + str(timeNow[2]) + "_" + str(timeNow[3]) + "_" + str(timeNow[4]) + "_" + str(timeNow[5])
        logFilename = currentDir / ("CAMERA_CALIBRATION_" + timeStr + ".log")
        print("      LOG FILE: " + logFilename.name + "\n")
        
        logFile = open(logFilename,'w')
        OpenMMS2(logFile)
    
        if images_file != "":
            input_eo_data = []
            output_eo_data = []
            input_eo = open(images_file,'r')
            output_eo = open(ext_para_file,'r')

            current_la_bore = []
            line_count = 0            
            for image_data in input_eo:
                line_count += 1
                if line_count > 3:
                    elements = image_data.strip('\n').split(",")
                    input_eo_data.append(list(elements))
                elif line_count == 2:
                    elements = image_data.strip('\n').split(",")
                    for i in range(0,len(elements)):
                        current_la_bore.append(float(elements[i]))
            input_eo.close()
            
            for image_data in output_eo:
                elements = image_data.strip('\n').split(" ")
                output_eo_data.append(list(elements))
            output_eo.close()
                
            column_matches = []
            for i in range(0,len(output_eo_data[0])):
                for j in range(0,len(output_eo_data[0])):
                    if output_eo_data[0][j].upper() == "IMAGENAME" and len(column_matches) == 0:
                        column_matches.append(j)
                        break
                    elif output_eo_data[0][j].upper() == "X" and len(column_matches) == 1:
                        column_matches.append(j)
                        break
                    elif output_eo_data[0][j].upper() == "Y" and len(column_matches) == 2:
                        column_matches.append(j)
                        break
                    elif output_eo_data[0][j].upper() == "Z" and len(column_matches) == 3:
                        column_matches.append(j)
                        break
                    elif output_eo_data[0][j].upper() == "OMEGA" and len(column_matches) == 4:
                        column_matches.append(j)
                        break
                    elif output_eo_data[0][j].upper() == "PHI" and len(column_matches) == 5:
                        column_matches.append(j)
                        break
                    elif output_eo_data[0][j].upper() == "KAPPA" and len(column_matches) == 6:
                        column_matches.append(j)
                        break         

            imagenames_list = []
            
            for i in range(0,len(output_eo_data)):
                for j in range(0,len(input_eo_data)):
                    if input_eo_data[j][2].upper() == output_eo_data[i][int(column_matches[0])].upper():
                        imagenames_list.append(input_eo_data[j][2].upper())                 #image name

            numImages = len(imagenames_list)
            print("      TOTAL NUMBER OF IMAGES FOUND: " + str(numImages))
            logFile.write("      TOTAL NUMBER OF IMAGES FOUND: " + str(numImages) + "\n")

            #initial approximates to calibration unknowns
            x = np.zeros(6,dtype=np.float64)
            x[0] = current_la_bore[0]
            x[1] = current_la_bore[1]
            x[2] = current_la_bore[2]
            #convert arcmins to degrees to radians
            x[3] = np.radians(current_la_bore[3] / 60.0)
            x[4] = np.radians(current_la_bore[4] / 60.0)
            x[5] = np.radians(current_la_bore[5] / 60.0)

            LAr = np.zeros(3,dtype=np.float64)
            LAr[0] = x[0]
            LAr[1] = x[1]
            LAr[2] = x[2]
            Rr2c = np.dot(formR3(-x[5]),np.dot(formR2(-x[4]),formR1(-x[3])))
            
            data_list = []
            
            #parse observations and convert Applanix Camera EO values to Ref.Pt EO values (used as constants)
            for i in range(0,len(output_eo_data)):
                for j in range(0,len(input_eo_data)):
                    if input_eo_data[j][2].upper() == output_eo_data[i][int(column_matches[0])].upper():                      
                        ind_data = []
                        ENUc = np.zeros(3,dtype=np.float64)
                        ENUc[0] = float(input_eo_data[j][3])
                        ENUc[1] = float(input_eo_data[j][4])
                        ENUc[2] = float(input_eo_data[j][5])
                        
                        A1 = np.radians(float(input_eo_data[j][6]))
                        A2 = np.radians(float(input_eo_data[j][7]))
                        A3 = np.radians(float(input_eo_data[j][8]))
                        A4 = np.radians(180)
                        
                        Rr2o = np.dot(formR1(-A1),np.dot(formR2(-A2),np.dot(formR3(-A3),np.dot(formR2(A4),Rr2c))))
                        
                        #Ref.Pt Omega,Phi,Kappa orientation angles
                        P = -np.arcsin(Rr2o[0,2])
                        O = 0
                        K = 0
                        if P == np.radians(90.0) or P == np.radians(-90.0):
                            K = -np.arctan2(Rr2o[1,0],Rr2o[1,1])
                        else:
                            O = -np.arctan2(-Rr2o[1,2],-Rr2o[2,2])
                            K = -np.arctan2(Rr2o[0,1],-Rr2o[0,0])
                            
                        #Ref.Pt X,Y,Z object space coordinates
                        ENUr = ENUc - np.dot(Rr2o,LAr)
                        
                        ind_data.append(ENUr[0])                                                        #0 - ref. pt. X (const)
                        ind_data.append(ENUr[1])                                                        #1 - ref. pt. Y (const)
                        ind_data.append(ENUr[2])                                                        #2 - ref. pt. Z (const)
                        ind_data.append(O)                                                              #3 - ref. frame omega
                        ind_data.append(P)                                                              #4 - ref. frame phi
                        ind_data.append(K)                                                              #5 - ref. frame kappa
                        ind_data.append(float(output_eo_data[i][int(column_matches[1])]))               #6 - BBA cam. X
                        ind_data.append(float(output_eo_data[i][int(column_matches[2])]))               #7 - BBA cam. Y
                        ind_data.append(float(output_eo_data[i][int(column_matches[3])]))               #8 - BBA cam. Z
                        ind_data.append(np.radians(float(output_eo_data[i][int(column_matches[4])])))   #9 - BBA cam. frame omega
                        ind_data.append(np.radians(float(output_eo_data[i][int(column_matches[5])])))   #10 - BBA cam. frame phi
                        ind_data.append(np.radians(float(output_eo_data[i][int(column_matches[6])])))   #11 - BBA cam. frame kappa

                        data_list.append(ind_data)
                        
                        # ENUc_check = np.dot(formR1(-O),np.dot(formR2(-P),np.dot(formR3(-K),np.dot(formR2(np.radians(180)),LAr)))) + ENUr
                        # print(input_eo_data[j][2].upper(),ENUc_check[0],ENUc_check[1],ENUc_check[2])
                        
                        # print(input_eo_data[j][2].upper(),ENUr[0],ENUr[1],ENUr[2],np.degrees(O),np.degrees(P),np.degrees(K))
                        
                        break                 
            
            # sys.exit(0)
            #check and force max_images input (sampled evenly from the total images)         
            new_data_list = []
            new_imagenames_list = []
            used_indices = []
            message_str = "      NUMBER OF IMAGES USED IN CALIBRATION"
            if numImages > max_images:
                sel_inc = float(numImages) / float(max_images)
                index = 0
                index_f = 0.0
                for i in range(0,max_images):
                    if index < len(data_list):
                        if not (index in used_indices):
                            new_data_list.append(data_list[index])
                            new_imagenames_list.append(imagenames_list[index])
                            used_indices.append(index)
                    index_f += sel_inc
                    index = int(np.round(index_f,0))
                message_str += " (EVEN SAMPLE FROM TOTAL)"
            else:
                new_data_list = list(data_list)
                new_imagenames_list = list(imagenames_list)
            
            numImages = len(new_data_list)
            print(message_str + ": " + str(numImages) + "\n")
            logFile.write(message_str + ": " + str(numImages) + "\n\n")
            time.sleep(2)
            
            data = np.array(new_data_list)
            imagenames = list(new_imagenames_list)
            
            dof = numImages*6 - 6
            s0_prev = 1e10
            converged = False
            itCount = 0
            
            w = np.zeros(numImages*6,dtype=np.float64)
            
            Er = np.zeros(numImages,dtype=np.float64)
            Nr = np.zeros(numImages,dtype=np.float64)
            Ur = np.zeros(numImages,dtype=np.float64)
            Or = np.zeros(numImages,dtype=np.float64)
            Pr = np.zeros(numImages,dtype=np.float64)
            Kr = np.zeros(numImages,dtype=np.float64)
            Ec = np.zeros(numImages,dtype=np.float64)
            Nc = np.zeros(numImages,dtype=np.float64)
            Uc = np.zeros(numImages,dtype=np.float64)
            Oc = np.zeros(numImages,dtype=np.float64)
            Pc = np.zeros(numImages,dtype=np.float64)
            Kc = np.zeros(numImages,dtype=np.float64)
            
            converged = False
    
            for i in range(0,numImages):
                Er[i] = data[i,0]
                Nr[i] = data[i,1]
                Ur[i] = data[i,2]
                Or[i] = data[i,3]
                Pr[i] = data[i,4]
                Kr[i] = data[i,5]
                Ec[i] = data[i,6]
                Nc[i] = data[i,7]
                Uc[i] = data[i,8]
                Oc[i] = data[i,9]
                Pc[i] = data[i,10]
                Kc[i] = data[i,11]               
    
            print("\n         ********************************************************")
            print("         ********* LEAST SQUARES ESTIMATION HAS STARTED *********")
            print("         ********************************************************\n")
            
            logFile.write("\n         ********************************************************\n")
            logFile.write("         ********* LEAST SQUARES ESTIMATION HAS STARTED *********\n")
            logFile.write("         ********************************************************\n\n")
            
            #GLS iterations
            for j in range(0,20):
                itCount = j+1
                print("         Iteration: " + str(itCount))
                print("         --------------------------------------------------------")
                print("            Creating matrices (please wait...)\n")
                print("               Misclosure Vector ............. ", end="\r")
                
                logFile.write("         Iteration: " + str(itCount) + "\n")
                logFile.write("         --------------------------------------------------------" + "\n")
                logFile.write("            Creating matrices (please wait...)\n" + "\n")
                
                #compute misclosure vector (w)                            
                w = -lambda_compute_F(x,Er,Nr,Ur,Or,Pr,Kr,Ec,Nc,Uc,Oc,Pc,Kc,numImages)

                # print()
                # w = np.round(w,4)
                # for i in range(0,numImages):
                #     print(imagenames[i],w[i*6],w[i*6+1],w[i*6+2],w[i*6+3],w[i*6+4],w[i*6+5])
                # sys.exit(0)

                print("               Misclosure Vector ............. DONE")
                logFile.write("               Misclosure Vector ............. DONE" + "\n")
                time.sleep(1)
                print("               Jacobian - A Matrix (dF/dX) ... ", end="\r")
                
                #create Jacobian matrices
                obsFunc = lambda x: (lambda_compute_F(x,Er,Nr,Ur,Or,Pr,Kr,Ec,Nc,Uc,Oc,Pc,Kc,numImages))
                jacobian = nd.Jacobian(obsFunc,step=1e-10)
                A = jacobian(x)
                
                print("               Jacobian - A Matrix (dF/dX) ... DONE")
                logFile.write("               Jacobian - A Matrix (dF/dX) ... DONE" + "\n")
                time.sleep(1)

                Ninv = np.linalg.inv(np.dot(A.T,A))
                u = np.dot(A.T,w)
                delta = np.dot(Ninv,u)

                v = np.dot(A,delta) - w
                s0 = np.sqrt(np.dot(v.T,v).item() / dof)
    
                if np.abs(s0_prev - s0) < 0.0005:
                    converged = True
                    Cx = s0**2 * Ninv
                    sds = []
                    sds.append(np.round(np.sqrt(Cx[0,0]),4))
                    sds.append(np.round(np.sqrt(Cx[1,1]),4))
                    sds.append(np.round(np.sqrt(Cx[2,2]),4))
                    sds.append(np.round(np.degrees(np.sqrt(Cx[3,3]))*60.0,4))
                    sds.append(np.round(np.degrees(np.sqrt(Cx[4,4]))*60.0,4))
                    sds.append(np.round(np.degrees(np.sqrt(Cx[5,5]))*60.0,4))
                    
                    print("\n            **************************************")
                    print("            ********* Solution Converged *********")
                    print("            **************************************")
                    print("              s0 = " + str(np.round(s0,5)))
                    print("               X = " + str(np.round(x[0],3)) + " +/- " + str(sds[0]) + " [m]")
                    print("               Y = " + str(np.round(x[1],3)) + " +/- " + str(sds[1]) + " [m]")
                    print("               Z = " + str(np.round(x[2],3)) + " +/- " + str(sds[2]) + " [m]")
                    print("              TX = " + str(np.round(np.degrees(x[3])*60.0,3)) + " +/- " + str(sds[3]) + " [arcmin]")
                    print("              TY = " + str(np.round(np.degrees(x[4])*60.0,3)) + " +/- " + str(sds[4]) + " [arcmin]")
                    print("              TZ = " + str(np.round(np.degrees(x[5])*60.0,3)) + " +/- " + str(sds[5]) + " [arcmin]")
                    
                    logFile.write("\n            **************************************" + "\n")
                    logFile.write("            ********* Solution Converged *********" + "\n")
                    logFile.write("            **************************************" + "\n")
                    logFile.write("              s0 = " + str(np.round(s0,5)) + "\n")
                    logFile.write("               X = " + str(np.round(x[0],3)) + " +/- " + str(sds[0]) + " [m]" + "\n")
                    logFile.write("               Y = " + str(np.round(x[1],3)) + " +/- " + str(sds[1]) + " [m]" + "\n")
                    logFile.write("               Z = " + str(np.round(x[2],3)) + " +/- " + str(sds[2]) + " [m]" + "\n")
                    logFile.write("              TX = " + str(np.round(np.degrees(x[3])*60.0,3)) + " +/- " + str(sds[3]) + " [arcmin]" + "\n")
                    logFile.write("              TY = " + str(np.round(np.degrees(x[4])*60.0,3)) + " +/- " + str(sds[4]) + " [arcmin]" + "\n")
                    logFile.write("              TZ = " + str(np.round(np.degrees(x[5])*60.0,3)) + " +/- " + str(sds[5]) + " [arcmin]" + "\n")
                    
                    print("\n              Correlations")
                    print("              ------------",end="")
                    
                    logFile.write("\n              Correlations\n")
                    logFile.write("              ------------")
                    
                    corr = corrFromCov(Cx)
                    
                    for i in range(0,6):
                        print("\n              ", end="")
                        logFile.write("\n              ")
                        for j in range(0,i+1):
                            value = str(np.round(corr[i,j],2))
                            diff = 5 - len(value)
                            for k in range(0,diff):
                                value += "0"
                            print(value + " ", end="")
                            logFile.write(value + " ")
                    print() 
                    logFile.write("\n")
                    
                    endProcessing = time.time()
                    timeE = endProcessing-startProcessing
                    print("\n            Processing Time: " + str(round((timeE)/60.,2)) + " mins")
                    print("            **************************************\n")
                    logFile.write("\n            Processing Time: " + str(round((timeE)/60.,2)) + " mins" + "\n")
                    logFile.write("            **************************************\n" + "\n")
                    
                    checks = lambda_compute_F(x,Er,Nr,Ur,Or,Pr,Kr,Ec,Nc,Uc,Oc,Pc,Kc,numImages)
                    Ec_check = np.zeros(numImages,dtype=np.float64)
                    Nc_check = np.zeros(numImages,dtype=np.float64)
                    Uc_check = np.zeros(numImages,dtype=np.float64)
                    Oc_check = np.zeros(numImages,dtype=np.float64)
                    Pc_check = np.zeros(numImages,dtype=np.float64)
                    Kc_check = np.zeros(numImages,dtype=np.float64)
                    
                    for i in range(0,numImages):
                        Ec_check[i] = np.round(checks[i*6]+Ec[i],4)
                        Nc_check[i] = np.round(checks[i*6+1]+Nc[i],4)
                        Uc_check[i] = np.round(checks[i*6+2]+Uc[i],4)
                        Oc_check[i] = np.round(np.degrees(checks[i*6+3]+Oc[i]),4)
                        Pc_check[i] = np.round(np.degrees(checks[i*6+4]+Pc[i]),4)
                        Kc_check[i] = np.round(np.degrees(checks[i*6+5]+Kc[i]),4)
                        # print(imagenames[i],Ec_check[i],Nc_check[i],Uc_check[i],Oc_check[i],Pc_check[i],Kc_check[i])
                
                    stats = []
                
                    stats.append(str(np.round(np.mean(Ec_check-Ec),3)))
                    stats.append(str(np.round(np.std(Ec_check-Ec),3)))
                    stats.append(str(np.round(np.min(Ec_check-Ec),3)))
                    stats.append(str(np.round(np.max(Ec_check-Ec),3)))
                    
                    stats.append(str(np.round(np.mean(Nc_check-Nc),3)))
                    stats.append(str(np.round(np.std(Nc_check-Nc),3)))
                    stats.append(str(np.round(np.min(Nc_check-Nc),3)))
                    stats.append(str(np.round(np.max(Nc_check-Nc),3)))
                    
                    stats.append(str(np.round(np.mean(Uc_check-Uc),3)))
                    stats.append(str(np.round(np.std(Uc_check-Uc),3)))
                    stats.append(str(np.round(np.min(Uc_check-Uc),3)))
                    stats.append(str(np.round(np.max(Uc_check-Uc),3)))
                    
                    stats.append(str(np.round(np.mean(Oc_check-np.degrees(Oc)),3)))
                    stats.append(str(np.round(np.std(Oc_check-np.degrees(Oc)),3)))
                    stats.append(str(np.round(np.min(Oc_check-np.degrees(Oc)),3)))
                    stats.append(str(np.round(np.max(Oc_check-np.degrees(Oc)),3)))
                    
                    stats.append(str(np.round(np.mean(Pc_check-np.degrees(Pc)),3)))
                    stats.append(str(np.round(np.std(Pc_check-np.degrees(Pc)),3)))
                    stats.append(str(np.round(np.min(Pc_check-np.degrees(Pc)),3)))
                    stats.append(str(np.round(np.max(Pc_check-np.degrees(Pc)),3)))
                    
                    stats.append(str(np.round(np.mean(Kc_check-np.degrees(Kc)),3)))
                    stats.append(str(np.round(np.std(Kc_check-np.degrees(Kc)),3)))       
                    stats.append(str(np.round(np.min(Kc_check-np.degrees(Kc)),3)))       
                    stats.append(str(np.round(np.max(Kc_check-np.degrees(Kc)),3)))
                    
                    frmt_stats = []
                    
                    for i in range(0,24):
                        diff = 6 - len(stats[i])
                        new_stat = stats[i]
                        for j in range(0,diff):
                            new_stat += "0"
                        if i < 12:
                            new_stat += " m "
                        else:
                            new_stat += " deg "
                        frmt_stats.append(new_stat)
                            
                
                    print("\n                            Statistics on Resulting EO Differences")
                    print("            ----------------------------------------------------------------------")
                    print("                     X         Y         Z       Omega        Phi        Kappa")
                    print("            Avg. ",frmt_stats[0],frmt_stats[4],frmt_stats[8],frmt_stats[12],frmt_stats[16],frmt_stats[20])
                    print("            Std. ",frmt_stats[1],frmt_stats[5],frmt_stats[9],frmt_stats[13],frmt_stats[17],frmt_stats[21])
                    print("            Min. ",frmt_stats[2],frmt_stats[6],frmt_stats[10],frmt_stats[14],frmt_stats[18],frmt_stats[22])
                    print("            Max. ",frmt_stats[3],frmt_stats[7],frmt_stats[11],frmt_stats[15],frmt_stats[19],frmt_stats[23])
                    print("\n")
                    
                    logFile.write("\n                            Statistics on Resulting EO Differences" + "\n")
                    logFile.write("            ----------------------------------------------------------------------" + "\n")
                    logFile.write("                     X         Y         Z       Omega        Phi        Kappa" + "\n")
                    logFile.write("            Avg. " + " " + frmt_stats[0] + " " + frmt_stats[4] + " " + frmt_stats[8] + " " + frmt_stats[12] + " " + frmt_stats[16] + " " + frmt_stats[20] + "\n")
                    logFile.write("            Std. " + " " + frmt_stats[1] + " " + frmt_stats[5] + " " + frmt_stats[9] + " " + frmt_stats[13] + " " + frmt_stats[17] + " " + frmt_stats[21] + "\n")
                    logFile.write("            Min. " + " " + frmt_stats[2] + " " + frmt_stats[6] + " " + frmt_stats[10] + " " + frmt_stats[14] + " " + frmt_stats[18] + " " + frmt_stats[22] + "\n")
                    logFile.write("            Max. " + " " + frmt_stats[3] + " " + frmt_stats[7] + " " + frmt_stats[11] + " " + frmt_stats[15] + " " + frmt_stats[19] + " " + frmt_stats[23] + "\n")
                    logFile.write("\n\n")
                    
                    break
                
                
                else:
                    s0_prev = s0
                    x += delta
                    print("\n            Iteration Updates (s0 --> " + str(np.round(s0,5)) + ")\n")
                    print("               X --> " + str(np.round(x[0],4)) + " [m]")
                    print("               Y --> " + str(np.round(x[1],4)) + " [m]")
                    print("               Z --> " + str(np.round(x[2],4)) + " [m]")
                    print("              TX --> " + str(np.round(np.degrees(x[3])*60.0,4)) + " [arcmin]")
                    print("              TY --> " + str(np.round(np.degrees(x[4])*60.0,4)) + " [arcmin]")
                    print("              TZ --> " + str(np.round(np.degrees(x[5])*60.0,4)) + " [arcmin]")
                    print("\n")
                    
                    logFile.write("\n            Iteration Updates (s0 --> " + str(np.round(s0,5)) + ")\n" + "\n")
                    logFile.write("               X --> " + str(np.round(x[0],4)) + " [m]" + "\n")
                    logFile.write("               Y --> " + str(np.round(x[1],4)) + " [m]" + "\n")
                    logFile.write("               Z --> " + str(np.round(x[2],4)) + " [m]" + "\n")
                    logFile.write("              TX --> " + str(np.round(np.degrees(x[3])*60.0,4)) + " [arcmin]" + "\n")
                    logFile.write("              TY --> " + str(np.round(np.degrees(x[4])*60.0,4)) + " [arcmin]" + "\n")
                    logFile.write("              TZ --> " + str(np.round(np.degrees(x[5])*60.0,4)) + " [arcmin]" + "\n")
                    logFile.write("\n\n")
                    
                    time.sleep(1)
                
            if not converged:
                print("\n#########################################################")
                print("##### SOLUTION DID NOT CONVERGE AFTER 20 ITERATIONS #####")
                print("#########################################################\n\n\n")
                
                logFile.write("\n#########################################################\n")
                logFile.write("##### SOLUTION DID NOT CONVERGE AFTER 20 ITERATIONS #####\n")
                logFile.write("#########################################################\n\n\n\n")
        
        else:
            print("\n      *** ERROR: The \'images_matched_to_events.csv\' file is missing from the current directory ***\n")
    
        logFile.close()
        
    else:
        if num_ext_para_files == 0:
            print("\n      *** ERROR: The \'_calibrated_external_camera_parameters.txt\' file, produced by Pix4D, is missing from the current directory ***\n")
        else:
            print("\n      *** ERROR: Multiple \'_calibrated_external_camera_parameters.txt\' files, produced by Pix4D, are present within the current directory ***\n")
        
    

##### command line start
if __name__ == "__main__":
    np.set_printoptions(suppress=True)
    print("\nCamera Boresight Calibration Started...\n")
    currentDir = Path(sys.argv[1])
    maxImages = int(sys.argv[2])
    
    main(currentDir,maxImages)