# -*- coding: utf-8 -*-

############################################################################
#                            OpenMMS Georeference                          #
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
import csv
import laspy
import numpy as np
import scipy.interpolate as spinterp 
import multiprocessing as mp
import os
import struct
import binascii
import matplotlib.pyplot as plt
from numba import vectorize
import subprocess
import math
from pyproj import Transformer, CRS
from datetime import datetime
from pathlib import Path
import warnings
    
    
def iHeartLidar2(logFile):
    sys.stdout = sys.__stdout__
    print("\n                         XXX     XXX                                        ")
    time.sleep(0.1)
    print("               IIIII    X   XX XX   X    L          DDDD     AAAA   RRRRR")
    time.sleep(0.1)
    print("                 I     X      X      X   L       i  D   D   A    A  R    R")
    time.sleep(0.1)
    print("                 I     X             X   L          D    D  A    A  R    R")
    time.sleep(0.1)
    print("                 I      XX         XX    L       i  D    D  AAAAAA  RRRRR")
    time.sleep(0.1)
    print("                 I        XX     XX      L       i  D    D  A    A  R  R")
    time.sleep(0.1)
    print("                 I          XX XX        L       i  D   D   A    A  R   R")
    time.sleep(0.1)
    print("               IIIII         XXX         LLLLLL  i  DDDD    A    A  R    R")
    time.sleep(0.1)
    print("                              X\n\n")
    
    sys.stdout = logFile
    print("\n                         XXX     XXX                                        ")
    print("               IIIII    X   XX XX   X    L          DDDD     AAAA   RRRRR")
    print("                 I     X      X      X   L       i  D   D   A    A  R    R")
    print("                 I     X             X   L          D    D  A    A  R    R")
    print("                 I      XX         XX    L       i  D    D  AAAAAA  RRRRR")
    print("                 I        XX     XX      L       i  D    D  A    A  R  R")
    print("                 I          XX XX        L       i  D   D   A    A  R   R")
    print("               IIIII         XXX         LLLLLL  i  DDDD    A    A  R    R")
    print("                              X")
    

def OpenMMS2(logFile):
    sys.stdout = sys.__stdout__
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
    print("     \\|________|\\|___|    \\|_______|\\|___|\\|__|\\|___|    \\|__|\\|___|    \\|__|\\|________|")
        
    sys.stdout = logFile
    print("\n _________                                 _______ _____  ______ ______  ________")
    print("|\\   ___  \\  ________  ________  ________ |\\   __ \\ __  \\|\\   __ \\ __  \\|\\   ____\\")
    print("\\|\\  \\_|\\  \\|\\   __  \\|\\   __  \\|\\   ___  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\___|_")
    print(" \\|\\  \\\\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\|\\  \\\\|\\  \\|\\  \\|\\__\\|\\  \\|\\  \\|\\__\\|\\  \\|\\_____  \\")
    print("  \\|\\  \\\\|\\  \\|\\   ____\\|\\   ____\\|\\  \\\\|\\  \\|\\  \\|__|\\|\\  \\|\\  \\|__|\\|\\  \\|____|\\  \\")
    print("   \\|\\  \\\\_\\  \\|\\  \\___|\\|\\  \\___|\\|\\  \\\\|\\  \\|\\  \\    \\|\\  \\|\\  \\    \\|\\  \\  __\\_\\  \\")
    print("    \\|\\________\\|\\__\\    \\|\\______\\\\|\\__\\\\|\\__\\|\\__\\    \\|\\__\\|\\__\\    \\|\\__\\|\\_______\\")
    print("     \\|________|\\|___|    \\|_______|\\|___|\\|__|\\|___|    \\|__|\\|___|    \\|__|\\|________|")
    

#PCAP FILE HEADER (24 bytes)
def readPcapFileHeader(pcap, bytesRead):

#    b = bytearray(pcap.read(4))
#    mn = struct.unpack('<L', b)[0]          #magic number

#    b = bytearray(pcap.read(2))
#    vmaj = struct.unpack('H', b)[0]        #major version number

#    b = bytearray(pcap.read(2))
#    vmin = struct.unpack('H', b)[0]        #minor version number

#    b = bytearray(pcap.read(4))
#    gmt = struct.unpack('i', b)[0]         #GMT to local correction

#    b = bytearray(pcap.read(4))
#    sigfigs = struct.unpack('<L', b)[0]     #accuracy of timestamps

#    b = bytearray(pcap.read(4))
#    snaplen = struct.unpack('<L', b)[0]     #max length of captured packets, in octets

#    b = bytearray(pcap.read(4))
#    network = struct.unpack('<L', b)[0]     #data link type

    pcap.read(24)
    bytesRead += 24

    return bytesRead


#PCAP PACKET HEADER (16 bytes)
def readPcapPacketHeader(pcap, num_position_packets, num_data_packets, num_used_data_packets, skipped_bytes, bytesRead, position_packets, num_bad_position_packets, utcHour, points_timing, points_azi, points_vert, points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, num_outside_points, firstNavTime, lastNavTime, runQuick, hourRollOver, num_points_per_packet, startTime, endTime, packetInterval, continueRead, points_scanNum, scans_count, badPPSmessages, packet_timings, points_dist_sf, points_azi_corr, points_dist_corr, vert_angles, laser_scale_factors, azimuth_corrections, distance_corrections):
    #check to make sure that 16 bytes of data exist before the end of the file

    checkData = bytearray(pcap.read(16))
    
    continueReading = True
    
    if len(checkData) != 16:
        continueReading = False
    else:
        b = checkData[0:4]
#        ts_sec = struct.unpack('<L', b)[0]         #timestamp seconds

        b = checkData[4:8]
#        ts_usec = struct.unpack('<L', b)[0]        #timestamp microseconds

        b = checkData[8:12]
        incl_length = struct.unpack('<L', b)[0]    #number of octets of packet saved in file

        b = checkData[12:16]
#        orig_length = struct.unpack('<L', b)[0]    #actual length of packet

        try:

            if incl_length == 554:      #Position packet
                num_position_packets += 1
                position_packets, num_bad_position_packets, bytesRead, utcHour, badPPSmessages = readPositionPacket(pcap, position_packets, num_bad_position_packets, bytesRead, utcHour, badPPSmessages)
                    
            elif incl_length == 1248:   #Data packet
                old_total_points = len(points_timing)
                num_data_packets += 1
                if np.mod(num_data_packets, packetInterval) == 0:
                    points_timing, points_azi, points_vert, points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, bytesRead, num_outside_points, packet_points_included, continueRead, points_scanNum, scans_count, points_dist_sf, points_azi_corr, points_dist_corr = readDataPacket(pcap, points_timing, points_azi, points_vert, points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, bytesRead, utcHour, num_outside_points, firstNavTime, lastNavTime, runQuick, startTime, endTime, continueRead, points_scanNum, scans_count, points_dist_sf, points_azi_corr, points_dist_corr, vert_angles, laser_scale_factors, azimuth_corrections, distance_corrections)
                    if packet_points_included:
                        num_used_data_packets += 1
                    
                    if len(points_timing) > 0:
                        num_points_per_packet.append(len(points_timing) - old_total_points)
                        packet_timings.append(points_timing[len(points_timing)-1])
                    continueReading = continueRead
                else:
                    pcap.read(1248)
                    bytesRead += 1248
            else:
                skipped_bytes += 16
        except:
            print("\n\n   **********************************************************")
            print("   ***** AN ERROR OCCURRED WHILE READING THE .PCAP FILE *****")
            print("   **********************************************************\n")
            continueReading = False

    bytesRead += 16    

    return continueReading, num_position_packets, num_data_packets, num_used_data_packets, skipped_bytes, bytesRead, position_packets, num_bad_position_packets, utcHour, points_timing, points_azi, points_vert, points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, num_outside_points, num_points_per_packet, continueRead, points_scanNum, scans_count, badPPSmessages, packet_timings, points_dist_sf, points_azi_corr, points_dist_corr


#POSITION PACKET (554 bytes)
def readPositionPacket(pcap, position_packets, num_bad_position_packets, bytesRead, utcHour, badPPSmessages):
    
    singlePosPac = []
    
    pcap.read(240)
    b = bytearray(pcap.read(4))
    timeStamp = struct.unpack('<L', b)[0]         #timestamp seconds
    
    b = bytearray(pcap.read(1))
    pps_status = struct.unpack('<B', b)[0]        #PPS Status Message
    
    pcap.read(3)
    
    try:
        GPRMCb = bytearray(pcap.read(72))
        GPRMC = str(GPRMCb, 'utf-8')
    
        singlePosPac.append(pps_status)
        singlePosPac.append(timeStamp)      #microseconds past the current UTC hour
    except:
        GPRMC = " "
    
    if GPRMC[0:1] != '$':
        num_bad_position_packets += 1
    else:
        PosTime = str(GPRMC).split(",")
        if PosTime[2].upper() == "A":   #OK NMEA message status
            PosTime[1] = PosTime[1].replace(".","")
            utcHour = int(PosTime[1][0:2])
            if pps_status != 2:
                utcHour = -1
                badPPSmessages += 1
            singlePosPac.append(int(PosTime[1]))    #UTC time HHMMSS
            singlePosPac.append(int(PosTime[9]))    #UTC date DDMMYY
            
            latDeg = float(PosTime[3][0:2])
            latMin = float(PosTime[3][2:len(PosTime[3])]) / 60.
            PosTime[3] = str(round((latDeg + latMin),6))
            lonDeg = float(PosTime[5][0:3])
            lonMin = float(PosTime[5][3:len(PosTime[5])]) / 60.
            PosTime[5] = str(round((lonDeg + lonMin),6))
            
            if PosTime[4].upper() == "S":
                PosTime[3] = str(-1 * float(PosTime[3]))
            
            PosTime[3] = PosTime[3].replace(".","")
            singlePosPac.append(int(PosTime[3]))    #latitude DDMMSSSSSSSS (minus sign for S)
            
            if PosTime[6].upper() == "W":
                PosTime[5] = str(-1 * float(PosTime[5]))
            
            PosTime[5] = PosTime[5].replace(".","")
            singlePosPac.append(int(PosTime[5]))    #longitude DDDMMSSSSSSSS (minus sign for W)
            position_packets.append(singlePosPac)
        
        else:                   #Invalid NMEA message status
            num_bad_position_packets += 1
    
    pcap.read(234)
    bytesRead += 554

    return position_packets, num_bad_position_packets, bytesRead, utcHour, badPPSmessages 


#DATA PACKET (1248 bytes)
def readDataPacket(pcap, points_timing, points_azi, points_vert, points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, bytesRead, utcHour, num_outside_points, firstNavTime, lastNavTime, runQuick, startTime, endTime, continueRead, points_scanNum, scans_count, points_dist_sf, points_azi_corr, points_dist_corr, vert_angles, laser_scale_factors, azimuth_corrections, distance_corrections):
    
    pcap.read(42)   # packet header
    data_block = []
    azi_block = []
    goodPacket = True
    packet_points_included = False
    for i in range(0,12):   #Read through data blocks
        b = bytearray(pcap.read(2))
        #print(binascii.hexlify(b).decode('ascii'))
        if binascii.hexlify(b).decode('ascii').upper() == "FFEE":   #Ensure data block flag is found
            azi_block.append(bytearray(pcap.read(2)))
            data_block.append(bytearray(pcap.read(96)))
        else:
            num_bad_data_packets += 1
            goodPacket = False
            break
    if goodPacket and utcHour != -1:
        
        b = bytearray(pcap.read(4))     #timestamp
        timeStamp = float(struct.unpack('<L', b)[0])
#        print(str(utcHour) + " : " + str(timeStamp))        
        lastPacketTime = 0.
        if len(points_timing) > 0:
            lastPacketTime = points_timing[len(points_timing)-1]
        
        packetStartTime = timeStamp / 1000000. + float(utcHour) * 3600.

        if packetStartTime < lastPacketTime:
            #print("\n" + str(packetStartTime) + " " + str(lastPacketTime) + "\n")
            if (lastPacketTime - packetStartTime) < 3602.:      #accounting for UTC hour crossovers
                packetStartTime += 3600.
            else:
                packetStartTime += 86400.                       #accounting for UTC day crossovers
        
        if (packetStartTime > startTime and packetStartTime <= endTime) or (startTime == -1. and endTime == -1.):
            #print(str(packetStartTime) + " " + str(startTime) + " " + str(endTime))
            b = bytearray(pcap.read(2))     #factory data
            sensor_info = str(binascii.hexlify(b),'utf-8')
            returnMode = sensor_info[0:2]   #37 = Single Return(Strongest), 38 = Single Return (Last), 39 = Dual Returns (Strongest and Last)
            sensorType = sensor_info[2:4]   #21 = Velodyne HDL-32E, 22 = Velodyne VLP-16/Puck Lite
    
            if returnMode == "39" and sensorType == "22":
                #Calc rotation rate of the sensor
                azi1 = float(struct.unpack('H', azi_block[0])[0]) / 100.
                if oldAzi == -1:    #Startup condition
                    oldAzi = azi1
                    oldTime = timeStamp
                else:    
                    diffAzi = azi1 - oldAzi
                    diffTime = (timeStamp - oldTime) / 1000000.
                    if diffAzi < 0:
                        diffAzi += 360.
                        scans_count += 1
                    if diffTime < 0:
                        diffTime += 3600.
                    rotRate = round((diffAzi / diffTime / 360.),2) #Rotate Rate in Hz
                    oldAzi = azi1
                    oldTime = timeStamp
                
                recent_good_azi_diff = 0.
                
                time_offset = 0.
                startLaser = 0
                endLaser = 16
                pulseUpdate = 2.304
                    
                #if quick processing is enabled then only use the -3 deg laser (which is index 12 in the vert_angles list)
                if runQuick:
                    startLaser = 12 # 5     #12 #index number of desired laser from vert_angles list
                    endLaser = startLaser + 1
                    pulseUpdate = (16 - startLaser) * 2.304
                    
                #in dual return mode the 12 blocks are really 6 blocks of two points per pulse
                for i in range(0,6):
                    azi_time_offset = 0.0
                    b1 = azi_block[i*2]
                    azimuth1 = struct.unpack('H', b1)[0] /100.
                    diffAzi = 0
                    if i < 5:
                        b2 = azi_block[i*2+2]
                        azimuth2 = struct.unpack('H', b2)[0] /100.
                        diffAzi = azimuth2 - azimuth1
                        if abs(diffAzi) < 1.0:  #check to make sure that the azimuth difference is not greater that 1 deg / 111 us (faster than 20Hz), cropped FOV
                            if diffAzi < 0:
                                diffAzi += 360.
                                scans_count += 1
                            recent_good_azi_diff = diffAzi
                        else:
                            diffAzi = recent_good_azi_diff
                    else:   #extrapolate the azimuth for the last 2 firing sequences based on the most recent appropriate azimuth difference
                        diffAzi = recent_good_azi_diff
                    
                    for j in range(0,2):    #Read through channel data twice (2 groupings of 16 pulses)
                        if runQuick:
                            time_offset += (startLaser * 2.304)
                            azi_time_offset += (startLaser * 2.304)
                        for k in range(startLaser,endLaser):
                            if True: #k != 0 and k != 2 and k != 4:    #not using -15,-13,-11 lasers due to researched poor performance (Glennie)
                                #if utcHour != -1:
                                pulseTiming = round((timeStamp + time_offset) / 1000000. + float(utcHour) * 3600., 7)
                                if pulseTiming < lastPacketTime:
                                    pulseTiming += 86400.
                                if pulseTiming > firstNavTime and pulseTiming < lastNavTime:
                                    packet_points_included = True
                                    pulseAzimuth = round(float(azimuth1 + diffAzi * azi_time_offset / 110.592),6)
                                    b1 = data_block[i*2][3*k+j*48:3*(k+1)+j*48]
                                    d1 = b1[0:2]
                                    r1 = b1[2:3]
                                    lastDist = struct.unpack('H', d1)[0]
                                    lastInt = struct.unpack('B', r1)[0]
                                    b2 = data_block[i*2+1][3*k+j*48:3*(k+1)+j*48]
                                    d2 = b2[0:2]
                                    r2 = b2[2:3]
                                    strongDist = struct.unpack('H', d2)[0]
                                    strongInt = struct.unpack('B', r2)[0]
                                    
                                    if lastDist == strongDist:
                                        if strongDist != 0:
                                            points_timing.append(pulseTiming)
                                            points_azi.append(pulseAzimuth)
                                            points_vert.append(vert_angles[k])
                                            points_dist.append(strongDist)
                                            points_intens.append(strongInt)
                                            points_return.append(1)
                                            points_scanNum.append(scans_count)
                                            points_dist_sf.append(laser_scale_factors[k])
                                            points_azi_corr.append(azimuth_corrections[k])
                                            points_dist_corr.append(distance_corrections[k])
                                    else:
                                        if lastDist != 0 and strongDist != 0:
                                            points_timing.append(pulseTiming)
                                            points_azi.append(pulseAzimuth)
                                            points_vert.append(vert_angles[k])
                                            points_dist.append(strongDist)
                                            points_intens.append(strongInt)
                                            points_return.append(1)
                                            points_scanNum.append(scans_count)
                                            points_dist_sf.append(laser_scale_factors[k])
                                            points_azi_corr.append(azimuth_corrections[k])
                                            points_dist_corr.append(distance_corrections[k])
                                            points_timing.append(pulseTiming)
                                            points_azi.append(pulseAzimuth)
                                            points_vert.append(vert_angles[k])
                                            points_dist.append(lastDist)
                                            points_intens.append(lastInt)
                                            points_return.append(2)
                                            points_scanNum.append(scans_count)
                                            points_dist_sf.append(laser_scale_factors[k])
                                            points_azi_corr.append(azimuth_corrections[k])
                                            points_dist_corr.append(distance_corrections[k])
                                        elif lastDist != 0: # and strongDist == 0
                                            points_timing.append(pulseTiming)
                                            points_azi.append(pulseAzimuth)
                                            points_vert.append(vert_angles[k])
                                            points_dist.append(lastDist)
                                            points_intens.append(lastInt)
                                            points_return.append(2)
                                            points_scanNum.append(scans_count)
                                            points_dist_sf.append(laser_scale_factors[k])
                                            points_azi_corr.append(azimuth_corrections[k])
                                            points_dist_corr.append(distance_corrections[k])
                                        else: #lastDist == 0 and strongDist != 0
                                            points_timing.append(pulseTiming)
                                            points_azi.append(pulseAzimuth)
                                            points_vert.append(vert_angles[k])
                                            points_dist.append(strongDist)
                                            points_intens.append(strongInt)
                                            points_return.append(1)
                                            points_scanNum.append(scans_count)
                                            points_dist_sf.append(laser_scale_factors[k])
                                            points_azi_corr.append(azimuth_corrections[k])
                                            points_dist_corr.append(distance_corrections[k])
                                else:
                                    num_outside_points += 1
                                    
                            time_offset += pulseUpdate
                            azi_time_offset += pulseUpdate
                        #end k loop
                        time_offset += 18.432
                        azi_time_offset += 18.432
                    #end j loop
                #end i loop
            else:
                print("\n\n***** PROCESSING ONLY WORKS FOR DATASETS FROM A VLP-16 SENSOR IN DUAL RETURN MODE *****\n\n")
                sys.exit()
            bytesRead += 1248
        else:
#            print("**** packet start time outside of bounds *****")
            pcap.read(2)
            bytesRead += 1248
            if packetStartTime > endTime:
                continueRead = False

    return points_timing, points_azi, points_vert, points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, bytesRead, num_outside_points, packet_points_included, continueRead, points_scanNum, scans_count, points_dist_sf, points_azi_corr, points_dist_corr


def readUFDataPacket(pcap, points_timing, points_azi, points_vert, points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, bytesRead, utcHour, num_outside_points, firstNavTime, lastNavTime, runQuick, startTime, endTime):

    data_block = []
    azi_block = []
    packet_points_included = False
    while True:
        goodPacket = True
        for i in range(0,12):   #Read through data blocks
            b = bytearray(pcap.read(2))
            if str(binascii.hexlify(b)).upper() == "FFEE":   #Ensure data block flag is found
                azi_block.append(bytearray(pcap.read(2)))
                data_block.append(bytearray(pcap.read(96)))
            else:
                num_bad_data_packets += 1
                goodPacket = False
                break
        if goodPacket:
            break
        elif b == '':
            break
    if goodPacket and utcHour != -1:
        b = bytearray(pcap.read(4))
        timeStamp = struct.unpack('<L', b)[0]

        lastPacketTime = 0
        if len(points_timing) > 0:
            lastPacketTime = points_timing[len(points_timing)-1]
        
        packetStartTime = float(timeStamp) / 1000000. + utcHour * 3600.
        if packetStartTime < lastPacketTime:
            packetStartTime += 86400.
        
        if (packetStartTime > startTime and packetStartTime <= endTime) or (startTime == -1. and endTime == -1.):

            b = bytearray(pcap.read(2))
            sensor_info = binascii.hexlify(b)
            returnMode = sensor_info[0:2]   #37 = Single Return(Strongest), 38 = Single Return (Last), 39 = Dual Returns (Strongest and Last)
            sensorType = sensor_info[2:4]   #21 = Velodyne HDL-32E, 22 = Velodyne VLP-16
    
            if returnMode == "39" and sensorType == "22":
                #Calc rotation rate of the sensor
                azi1 = struct.unpack('H', azi_block[0])[0] / 100.
                if oldAzi == -1:    #Startup condition
                    oldAzi = azi1
                    oldTime = timeStamp
                else:    
                    diffAzi = azi1 - oldAzi
                    diffTime = float(timeStamp - oldTime) / 1000000.
                    if diffAzi < 0:
                        diffAzi += 360.
                    if diffTime < 0:
                        diffTime += 3600.
                    rotRate = round((diffAzi / diffTime / 360.),2) #Rotate Rate in Hz
                    oldAzi = azi1
                    oldTime = timeStamp
                #index values    0, 1,  2,  3,   4, 5,  6, 7,  8, 9, 10, 11, 12, 13, 14, 15
                vert_angles = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
                recent_good_azi_diff = 0
                
                time_offset = 0
                startLaser = 0
                endLaser = 16
                pulseUpdate = 2.304
                azi_time_init = 0
                    
                #if quick processing is enabled then only use the -3 deg laser (which is index 12 in the vert_angles list)
                if runQuick:
                    startLaser = 12 #index number of desired laser from vert_angles list
                    endLaser = startLaser + 1
                    time_offset = startLaser * 2.304
                    pulseUpdate = (16 - startLaser) * 2.304
                    
                #in dual return mode the 12 blocks are really 6 blocks of two points per pulse
                for i in range(0,6):
                    azi_time_offset = azi_time_init
                    b1 = azi_block[i*2]
                    azimuth1 = struct.unpack('H', b1)[0] /100.
                    diffAzi = 0
                    if i < 5:
                        b2 = azi_block[i*2+2]
                        azimuth2 = struct.unpack('H', b2)[0] /100.
                        diffAzi = azimuth2 - azimuth1
                        if abs(diffAzi) < 1.0:  #check to make sure that the azimuth difference is not greater that 1 deg / 111 us (faster than 20Hz), cropped FOV
                            if diffAzi < 0:
                                diffAzi += 360.
                            recent_good_azi_diff = diffAzi
                        else:
                            diffAzi = recent_good_azi_diff
                    else:   #extrapolate the azimuth for the last 2 firing sequences based on the most recent appropriate azimuth difference
                        diffAzi = recent_good_azi_diff
                    
                    for j in range(0,2):    #Read through channel data twice (2 groupings of 16 pulses)
                        for k in range(startLaser,endLaser):
                            #if utcHour != -1:
                            pulseTiming = round(float(timeStamp + time_offset) / 1000000. + utcHour * 3600., 7)
                            if pulseTiming < lastPacketTime:
                                pulseTiming += 86400.
                            if pulseTiming > firstNavTime and pulseTiming < lastNavTime:
                                packet_points_included = True
                                pulseAzimuth = round(float(azimuth1 + diffAzi * azi_time_offset / (2 * 55.296)),6)
                                b1 = data_block[i*2][3*k+j*48:3*(k+1)+j*48]
                                d1 = b1[0:2]
                                r1 = b1[2:3]
                                lastDist = struct.unpack('H', d1)[0]
                                lastInt = struct.unpack('B', r1)[0]
                                b2 = data_block[i*2+1][3*k+j*48:3*(k+1)+j*48]
                                d2 = b2[0:2]
                                r2 = b2[2:3]
                                strongDist = struct.unpack('H', d2)[0]
                                strongInt = struct.unpack('B', r2)[0]
                                
                                if lastDist == strongDist:
                                    if strongDist != 0:
                                        points_timing.append(pulseTiming)
                                        points_azi.append(pulseAzimuth)
                                        points_vert.append(vert_angles[k])
                                        points_dist.append(strongDist)
                                        points_intens.append(strongInt)
                                        points_return.append(1)
                                else:
                                    if lastDist != 0 and strongDist != 0:
                                        points_timing.append(pulseTiming)
                                        points_azi.append(pulseAzimuth)
                                        points_vert.append(vert_angles[k])
                                        points_dist.append(strongDist)
                                        points_intens.append(strongInt)
                                        points_return.append(1)
                                        points_timing.append(pulseTiming)
                                        points_azi.append(pulseAzimuth)
                                        points_vert.append(vert_angles[k])
                                        points_dist.append(lastDist)
                                        points_intens.append(lastInt)
                                        points_return.append(2)
                                    elif lastDist != 0: # and strongDist == 0
                                        points_timing.append(pulseTiming)
                                        points_azi.append(pulseAzimuth)
                                        points_vert.append(vert_angles[k])
                                        points_dist.append(lastDist)
                                        points_intens.append(lastInt)
                                        points_return.append(2)
                                    else: #lastDist == 0 and strongDist != 0
                                        points_timing.append(pulseTiming)
                                        points_azi.append(pulseAzimuth)
                                        points_vert.append(vert_angles[k])
                                        points_dist.append(strongDist)
                                        points_intens.append(strongInt)
                                        points_return.append(1)
                            else:
                                num_outside_points += 1
                                
                            time_offset += pulseUpdate
                            azi_time_offset += pulseUpdate
                        #end k loop
                        time_offset += 18.432
                        azi_time_offset += 18.432
                    #end j loop
                #end i loop
            else:
                print("\n\n***** PROCESSING ONLY WORKS FOR DATASETS FROM A VLP-16 SENSOR IN DUAL RETURN MODE *****\n\n")
                sys.exit()
            bytesRead += 1206
        else:
            pcap.read(2)
            bytesRead += 1206
    
    return points_timing, points_azi, points_vert, points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, bytesRead, num_outside_points, packet_points_included 


def curv2cart(a, e2, lat, lon, ellH):
    latr = np.radians(lat)
    lonr = np.radians(lon)
    N = (a/(math.sqrt(1.0 - e2 * math.pow(math.sin(latr),2))))
    X = ((N + ellH) * math.cos(latr) * math.cos(lonr))
    Y = ((N + ellH) * math.cos(latr) * math.sin(lonr))
    Z = ((N*(1-e2) + ellH) * math.sin(latr))
    return X,Y,Z


def readCSVNav(filename):
    dateRollOver = False
    hourRollOver = 0
    nav = []
    lastHour = 0
    counter = 0
    readHeader = False
    dayAdjustment = 0.
    header_items = []

    with open(filename, "r") as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            if readHeader:
                if len(row) > 0:                                    
                    nav.append([float(i) for i in row])
                    
                    timeStr = "{:0.12f}".format(nav[counter][0] / 3600.)
                    decPt = timeStr.find(".")
                    hour = int(timeStr[0:decPt])
                    
                    if (hour - lastHour) < 0:
                        dayAdjustment = 86400.
                        dateRollOver = True
                        hourRollOver += 1
                    elif lastHour - hour < 0 and counter > 0:
                        hourRollOver += 1
                    lastHour = hour
                    nav[counter][0] += dayAdjustment
                    counter += 1
            else:
                header_items = list(row)
                readHeader = True
    
    return nav, hourRollOver, dateRollOver, header_items


def navRPH2SinCos(nav_filename, nav, semi_major, eccentricity2, verboseOut):
    filePath = Path(nav_filename).parents[0]
    sinLat = []
    cosLat = []
    sinLon = []
    cosLon = []
    sinR = []
    cosR = []
    sinP = []
    cosP = []
    sinH = []
    cosH = []
    totalAccel = []
    ZangRate = []
    oneG = 9.81
    deltaRoll = []
    vel_hor = []
    Xg = []
    Yg = []
    Zg = []
    
    for i in range(0,len(nav)):
        lat = float(nav[i][1])
        lon = float(nav[i][2])
        ellH = float(nav[i][3])
        sinLat.append(np.sin(np.radians(lat)))
        cosLat.append(np.cos(np.radians(lat)))
        sinLon.append(np.sin(np.radians(lon)))
        cosLon.append(np.cos(np.radians(lon)))
        
        r = np.radians(float(nav[i][5]))
        p = np.radians(float(nav[i][6]))
        h = np.radians(float(nav[i][7]))
        sinR.append(np.sin(r))
        cosR.append(np.cos(r))
        sinP.append(np.sin(p))
        cosP.append(np.cos(p))
        sinH.append(np.sin(h))
        cosH.append(np.cos(h))

        X,Y,Z = curv2cart(semi_major,eccentricity2,lat,lon,ellH)
        
        Xg.append(X)
        Yg.append(Y)
        Zg.append(Z)

        totalAccel.append(np.sqrt((float(nav[i][8]) / oneG)*(float(nav[i][8]) / oneG) + (float(nav[i][9]) / oneG)*(float(nav[i][9]) / oneG) + (float(nav[i][10]) / oneG)*(float(nav[i][10]) / oneG)))
        ZangRate.append(float(nav[i][11]))

        if i == 0:
            deltaRoll.append(0.0)
        else:
            deltaRoll.append(float(nav[i][5]) - float(nav[i-1][5]))
            
        vel_hor.append(np.sqrt(float(nav[i][12])**2 + float(nav[i][13])**2))
       
    Xg_offset = np.floor(np.min(Xg))
    Yg_offset = np.floor(np.min(Yg))
    Zg_offset = np.floor(np.min(Zg))
    
    if verboseOut:
        count = 1
        while True:
            dirName = filePath / ("calibration" + str(count))
            if os.path.isdir(str(dirName)):
                count += 1
            else:
                os.mkdir(dirName)
                time.sleep(0.5)
                verbose_offsets = open(str(dirName / "verbose_offsets.csv"),"w")
                verbose_offsets.write(str(Xg_offset) + "," + str(Yg_offset) + "," + str(Zg_offset) + "\n")
                verbose_offsets.close()
                break
    
    navA = np.array(nav)
    
    #update: added X accel, Y accel, Z accel and Z body angular rate to NAV data - June 2019, but calculate total acceleration and using that
    return navA[:,0], sinLat, cosLat, sinLon, cosLon, navA[:,3], navA[:,4], sinR, cosR, sinP, cosP, sinH, cosH, Xg, Yg, Zg, totalAccel, ZangRate, deltaRoll, vel_hor, navA[:,14], Xg_offset, Yg_offset, Zg_offset


@vectorize(['float32(float32, float32, float32, float32, float32, float32)'], target='cpu', nopython=True)
def geoRefCalcSocsX(azi, vert, dist, distSF, aziCorr, distCorr):
    return (dist * 0.002 * distSF + distCorr) * math.cos(math.radians(vert)) * (math.sin(math.radians(azi))*math.cos(math.radians(aziCorr))-math.cos(math.radians(azi))*math.sin(math.radians(aziCorr)))


@vectorize(['float32(float32, float32, float32, float32, float32, float32)'], target='cpu', nopython=True)
def geoRefCalcSocsY(azi, vert, dist, distSF, aziCorr, distCorr):
    return (dist * 0.002 * distSF + distCorr) * math.cos(math.radians(vert)) * (math.cos(math.radians(azi))*math.cos(math.radians(aziCorr))+math.sin(math.radians(azi))*math.sin(math.radians(aziCorr)))


@vectorize(['float32(float32, float32, float32, float32)'], target='cpu', nopython=True)
def geoRefCalcSocsZ(vert, dist, distSF, distCorr):
    return (dist * 0.002 * distSF + distCorr) * math.sin(math.radians(vert)) - 0.0419 * math.tan(math.radians(vert))


@vectorize(['float64(float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float64)'], target='cpu', nopython=True)
def geoRefCalcGlcsX1(x0,y0,z0,sinR,cosR,sinP,cosP,sinH,cosH,sinX,cosX,sinY,cosY,sinZ,cosZ,sinLat,cosLat,sinLon,cosLon,Xg):
    return (Xg + x0*(cosY*(cosZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon)) + sinZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)))) - sinY*(cosR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)) - sinR*(-cosH*sinLon + cosLon*sinH*sinLat))) + y0*(cosX*(cosZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon))) - sinZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon))) + sinX*(cosY*(cosR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)) - sinR*(-cosH*sinLon + cosLon*sinH*sinLat)) + sinY*(cosZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon)) + sinZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)))))) + z0*(cosX*(cosY*(cosR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon)) - sinR*(-cosH*sinLon + cosLon*sinH*sinLat)) + sinY*(cosZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon)) + sinZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon))))) - sinX*(cosZ*(cosR*(-cosH*sinLon + cosLon*sinH*sinLat) + sinR*(-cosLat*cosLon*cosP + sinP*(-cosH*cosLon*sinLat - sinH*sinLon))) - sinZ*(cosLat*cosLon*sinP + cosP*(-cosH*cosLon*sinLat - sinH*sinLon)))))


@vectorize(['float64(float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float64)'], target='cpu', nopython=True)
def geoRefCalcGlcsY1(x0,y0,z0,sinR,cosR,sinP,cosP,sinH,cosH,sinX,cosX,sinY,cosY,sinZ,cosZ,sinLat,cosLat,sinLon,cosLon,Yg):
    return (Yg + x0*(cosY*(cosZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH)) + sinZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)))) - sinY*(cosR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)) - sinR*(cosH*cosLon + sinH*sinLat*sinLon))) + y0*(cosX*(cosZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH))) - sinZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH))) + sinX*(cosY*(cosR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)) - sinR*(cosH*cosLon + sinH*sinLat*sinLon)) + sinY*(cosZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH)) + sinZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)))))) + z0*(cosX*(cosY*(cosR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH)) - sinR*(cosH*cosLon + sinH*sinLat*sinLon)) + sinY*(cosZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH)) + sinZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH))))) - sinX*(cosZ*(cosR*(cosH*cosLon + sinH*sinLat*sinLon) + sinR*(-cosLat*cosP*sinLon + sinP*(-cosH*sinLat*sinLon + cosLon*sinH))) - sinZ*(cosLat*sinLon*sinP + cosP*(-cosH*sinLat*sinLon + cosLon*sinH)))))
    

@vectorize(['float64(float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float32,float64)'], target='cpu', nopython=True)
def geoRefCalcGlcsZ1(x0,y0,z0,sinR,cosR,sinP,cosP,sinH,cosH,sinX,cosX,sinY,cosY,sinZ,cosZ,sinLat,cosLat,sinLon,cosLon,Zg):
    return (Zg + x0*(cosY*(cosZ*(cosH*cosLat*cosP + sinLat*sinP) + sinZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat))) - sinY*(cosLat*sinH*sinR + cosR*(cosH*cosLat*sinP - cosP*sinLat))) + y0*(cosX*(cosZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat)) - sinZ*(cosH*cosLat*cosP + sinLat*sinP)) + sinX*(cosY*(cosLat*sinH*sinR + cosR*(cosH*cosLat*sinP - cosP*sinLat)) + sinY*(cosZ*(cosH*cosLat*cosP + sinLat*sinP) + sinZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat))))) + z0*(cosX*(cosY*(cosLat*sinH*sinR + cosR*(cosH*cosLat*sinP - cosP*sinLat)) + sinY*(cosZ*(cosH*cosLat*cosP + sinLat*sinP) + sinZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat)))) - sinX*(cosZ*(-cosLat*cosR*sinH + sinR*(cosH*cosLat*sinP - cosP*sinLat)) - sinZ*(cosH*cosLat*cosP + sinLat*sinP))))


@vectorize(['float32(float32, float32)'], target='cpu', nopython=True)
def getAngleBack(sinA, cosA):
    return math.degrees(math.atan2(sinA,cosA))


def geoReferenceCalcs(points_timing, points_azi, points_vert, points_dist, points_intens, points_return, navinterpSinLat, navinterpCosLat, navinterpSinLon, navinterpCosLon, navinterpEllH, navinterpGeoidSep, navinterpSinR, navinterpCosR, navinterpSinP, navinterpCosP, navinterpSinH, navinterpCosH, navinterpXg, navinterpYg, navinterpZg, navinterpTa, navinterpZar, navinterpDr, navinterpVh, navinterpVu, params, points_dist_sf, points_azi_corr, points_dist_corr):
    boreX = np.radians(float(params[0]))
    boreY = np.radians(float(params[1]))
    boreZ = np.radians(float(params[2]))
    
    sinX = np.sin(boreX)
    cosX = np.cos(boreX)
    sinY = np.sin(boreY)
    cosY = np.cos(boreY)
    sinZ = np.sin(boreZ)
    cosZ = np.cos(boreZ)
    
    dist0 = np.asarray(points_dist, dtype=np.float32)
    azi0 = np.asarray(points_azi, dtype=np.float32)
    vert0 = np.asarray(points_vert, dtype=np.float32)
    times = np.asarray(points_timing, dtype=np.float64)
    distSF = np.asarray(points_dist_sf, dtype=np.float32)
    aziCorr = np.asarray(points_azi_corr, dtype=np.float32)
    distCorr = np.asarray(points_dist_corr, dtype=np.float32)
        
    sinLat = np.asarray(navinterpSinLat(times), dtype=np.float32)
    cosLat = np.asarray(navinterpCosLat(times), dtype=np.float32)
    sinLon = np.asarray(navinterpSinLon(times), dtype=np.float32)
    cosLon = np.asarray(navinterpCosLon(times), dtype=np.float32)
    geoidSep = np.asarray(navinterpGeoidSep(times), dtype=np.float32)
    sinR = np.asarray(navinterpSinR(times), dtype=np.float32)
    cosR = np.asarray(navinterpCosR(times), dtype=np.float32)
    sinP = np.asarray(navinterpSinP(times), dtype=np.float32)
    cosP = np.asarray(navinterpCosP(times), dtype=np.float32)
    sinH = np.asarray(navinterpSinH(times), dtype=np.float32)
    cosH = np.asarray(navinterpCosH(times), dtype=np.float32)
    Xg = np.asarray(navinterpXg(times), dtype=np.float64)
    Yg = np.asarray(navinterpYg(times), dtype=np.float64)
    Zg = np.asarray(navinterpZg(times), dtype=np.float64)
    totalAccel = np.asarray(navinterpTa(times), dtype=np.float32)
    ZangRate = np.asarray(navinterpZar(times), dtype=np.float32)
    deltaRoll = np.asarray(navinterpDr(times), dtype=np.float32) 
    horVelo = np.asarray(navinterpVh(times), dtype=np.float32) 
    upVelo = np.asarray(navinterpVu(times), dtype=np.float32) 
    
    Lat = getAngleBack(sinLat, cosLat)  #degrees
    Lon = getAngleBack(sinLon, cosLon)  #degrees
    R = getAngleBack(sinR, cosR)    #degrees
    P = getAngleBack(sinP, cosP)    #degrees
    H = getAngleBack(sinH, cosH)    #degrees
    
    x0 = np.empty_like(dist0,dtype=np.float32)
    y0 = np.empty_like(dist0,dtype=np.float32)
    z0 = np.empty_like(dist0,dtype=np.float32)
    
    x0 = geoRefCalcSocsX(azi0, vert0, dist0, distSF, aziCorr, distCorr)
    y0 = geoRefCalcSocsY(azi0, vert0, dist0, distSF, aziCorr, distCorr)
    z0 = geoRefCalcSocsZ(vert0, dist0, distSF, distCorr)

    X = np.empty_like(dist0,dtype=np.float64)
    Y = np.empty_like(dist0,dtype=np.float64)
    Z = np.empty_like(dist0,dtype=np.float64)
    
    X = np.round(geoRefCalcGlcsX1(x0,y0,z0,sinR,cosR,sinP,cosP,sinH,cosH,sinX,cosX,sinY,cosY,sinZ,cosZ,sinLat,cosLat,sinLon,cosLon,Xg),3)
    Y = np.round(geoRefCalcGlcsY1(x0,y0,z0,sinR,cosR,sinP,cosP,sinH,cosH,sinX,cosX,sinY,cosY,sinZ,cosZ,sinLat,cosLat,sinLon,cosLon,Yg),3)
    Z = np.round(geoRefCalcGlcsZ1(x0,y0,z0,sinR,cosR,sinP,cosP,sinH,cosH,sinX,cosX,sinY,cosY,sinZ,cosZ,sinLat,cosLat,sinLon,cosLon,Zg),3)

    return x0, y0, z0, X, Y, Z, R, P, H, Xg, Yg, Zg, Lat, Lon, geoidSep, totalAccel, ZangRate, deltaRoll, horVelo, upVelo


def myProj(Xg,Yg,Zg,geoidSep,geo_epsg,proj_epsg,singlePoint=False):
    warnings.filterwarnings("ignore")
    
    geocs = CRS("epsg:" + str(geo_epsg))
    
    geocs_info = list(geocs.to_dict().keys())
    ellipseFound = False
    datumFound = False
    for i in range(0,len(geocs_info)):
        testStr = str(geocs_info[i]).upper()
        if testStr == "ELLPS":
            ellipseFound = True
        elif testStr == "DATUM":
            datumFound = True
    
    E = None
    N = None
    H = None
    ecef = None
    projcs = None
    ok2Cont = False
    
    if ellipseFound:
        try:
            ecef = CRS({"proj":'geocent',"ellps":geocs.to_dict()['ellps']})
            ok2Cont = True
        except:
            pass

    elif datumFound:
        try:
            ecef = CRS({"proj":'geocent',"datum":geocs.to_dict()['datum']})
            ok2Cont = True
        except:
            pass

            
    if ok2Cont:
        try:
            projcs = CRS("epsg:" + str(proj_epsg))
            ecef2pcsTransformer = Transformer.from_proj(ecef,projcs)
        except:
            ok2Cont = False
            
        if ok2Cont:
            projPts = np.asarray(ecef2pcsTransformer.transform(Xg,Yg,Zg),dtype=np.float64)
            if singlePoint:
                E = projPts[0]
                N = projPts[1]
                H = projPts[2] - geoidSep
            else:    
                E = projPts[0,:]
                N = projPts[1,:]
                H = projPts[2,:] - geoidSep
    
    return E, N, H, ok2Cont
        

def outputPrcsPoints_a(filePath,csv_filename, x, y, z, time, intens, retur, dist, heading, vert, scanNum, totalAccel, ZangRate, deltaOmega):

    csv = open(filePath / (csv_filename + "_temp.csv"),"w")
    csv.write("//X,Y,Z,Time,Intensity,Return Number,range,heading,laser_num,scan_num,total_accel,Z_ang_rate,delta_omega\n")

    print("                TEMP FILE " + csv_filename + "_temp.csv is being CREATED ...")
    logMessStr = "                TEMP FILE " + csv_filename + "_temp.csv is being CREATED ...\n"
    for i in range(0,len(x)):
        csv.write("{:0.3f}".format(x[i]) + "," + "{:0.3f}".format(y[i]) + "," + "{:0.3f}".format(z[i]) + "," + "{:0.2f}".format(time[i]) + "," + str(intens[i]) + "," + str(retur[i]) + "," + "{:0.3f}".format(dist[i]*0.002) + "," + "{:0.1f}".format(heading[i]) + "," + str(vert[i]) + "," + str(scanNum[i]) + "," + "{:0.3f}".format(totalAccel[i]) + "," + "{:0.3f}".format(ZangRate[i]) + "," + "{:0.3f}".format(deltaOmega[i]) + "\n")

    print("                TEMP FILE " + csv_filename + "_temp.csv is DONE!")
    logMessStr += "                TEMP FILE " + csv_filename + "_temp.csv is DONE!\n"
    csv.close()
    
    return logMessStr


def outputPrcsPoints_a_verbose(filePath,csv_filename, X, Y, Z, time, intens, retur, dist, vert, azi, scanNum, x0, y0, z0, R, P, H, Es, Ns, hs):

    csv = open(filePath / (csv_filename + "_temp.csv"),"w")
    csv.write("//X,Y,Z,Time,Intensity,range,laser_num,azimuth,scan_num,x_socs,y_socs,z_socs,omega,phi,kappa,scanner_x,scanner_y,scanner_z\n")

    print("                TEMP FILE " + csv_filename + "_temp.csv is being CREATED ...")
    logMessStr = "                TEMP FILE " + csv_filename + "_temp.csv is being CREATED ...\n"
    for i in range(0,len(X)):
        if retur[i] == 1:
            csv.write("{:0.3f}".format(X[i]) + "," + "{:0.3f}".format(Y[i]) + "," + "{:0.3f}".format(Z[i]) + "," + "{:0.7f}".format(time[i]) + "," + str(intens[i]) + "," + "{:0.3f}".format(dist[i]*0.002) + "," + str(vert[i]) + "," + str(azi[i]) + "," + str(scanNum[i]) + "," + "{:0.4f}".format(x0[i]) + "," + "{:0.4f}".format(y0[i]) + "," + "{:0.4f}".format(z0[i]) + "," + "{:0.4f}".format(R[i]) + "," + "{:0.4f}".format(P[i]) + "," + "{:0.4f}".format(H[i]) + "," + "{:0.3f}".format(Es[i]) + "," + "{:0.3f}".format(Ns[i]) + "," + "{:0.3f}".format(hs[i]) + "\n")

    print("                TEMP FILE " + csv_filename + "_temp.csv is DONE!")
    logMessStr += "                TEMP FILE " + csv_filename + "_temp.csv is DONE!\n"
    csv.close() 
    
    return logMessStr


def outputPrcsPoints_a_verbose2(filePath,csv_filename, X, Y, Z, time, retur, x0, y0, z0, R, P, H, Xg, Yg, Zg, Lat, Lon, Xg_offset, Yg_offset, Zg_offset):

    csv = open(filePath / (csv_filename + "_temp.csv"),"w")
    csv.write("//X,Y,Z,Time,x_socs,y_socs,z_socs,roll,pitch,heading,scanner_x,scanner_y,scanner_z,scanner_lat,scanner_lon\n")

    print("                TEMP FILE " + csv_filename + "_temp.csv is being CREATED ...")
    logMessStr = "                TEMP FILE " + csv_filename + "_temp.csv is being CREATED ...\n"
    for i in range(0,len(X)):
        if retur[i] == 1:
            csv.write("{:0.3f}".format(X[i]) + "," + "{:0.3f}".format(Y[i]) + "," + "{:0.3f}".format(Z[i]) + "," + "{:0.7f}".format(time[i]) + "," + "{:0.4f}".format(x0[i]) + "," + "{:0.4f}".format(y0[i]) + "," + "{:0.4f}".format(z0[i]) + "," + "{:0.4f}".format(R[i]) + "," + "{:0.4f}".format(P[i]) + "," + "{:0.4f}".format(H[i]) + "," + "{:0.3f}".format(Xg[i]-Xg_offset) + "," + "{:0.3f}".format(Yg[i]-Yg_offset) + "," + "{:0.3f}".format(Zg[i]-Zg_offset) + "," + "{:0.8f}".format(Lat[i]) + "," + "{:0.8f}".format(Lon[i]) + "\n")

    print("                TEMP FILE " + csv_filename + "_temp.csv is DONE!")
    logMessStr += "                TEMP FILE " + csv_filename + "_temp.csv is DONE!\n"
    csv.close() 
    
    return logMessStr


def outputPrcsPoints_l(filePath,las_filename, x, y, z, gpstime, intens, retur, dist, heading, vert, azi, scanNum, totalAccel, ZangRate, deltaRoll, horVelo, upVelo):

    print("                TEMP FILE " + las_filename + "_temp.las is being CREATED ...")
    logMessStr = "                TEMP FILE " + las_filename + "_temp.las is being CREATED ...\n"
    
    #See: https://pythonhosted.org//laspy/tut_part_3.html
    
    hdr = laspy.header.Header()
    hdr.version = "1.2"
    hdr.data_format_id = 3
    
    #the following ID fields must be less than or equal to 32 characters in length
    System_ID = "OpenMMS"
    Software_ID = "OpenMMS OSS v1.3.0"
    
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
    
    lasfile = laspy.file.File(filePath / (las_filename + "_temp.las"), mode="w", header=hdr)
    lasfile.define_new_dimension(name="range",data_type=4,description="") #Range from Scanner Dimension")
    lasfile.define_new_dimension(name="laser_num",data_type=4,description="") #Velodyne Laser Unit No. Dimension")
    lasfile.define_new_dimension(name="heading",data_type=4,description="") #Heading of Scanner Dimension")
    lasfile.define_new_dimension(name="scan_num",data_type=5,description="")
    lasfile.define_new_dimension(name="total_accel",data_type=9,description="")
    lasfile.define_new_dimension(name="h_vel",data_type=9,description="")
    lasfile.define_new_dimension(name="v_vel",data_type=9,description="")
    lasfile.define_new_dimension(name="z_ang_rate",data_type=9,description="")
    lasfile.define_new_dimension(name="delta_roll",data_type=9,description="")
    lasfile.define_new_dimension(name="scan_azi",data_type=4,description="")
    
    xmin = np.floor(np.min(x))
    ymin = np.floor(np.min(y))
    zmin = np.floor(np.min(z))    
    lasfile.header.offset = [xmin,ymin,zmin]
    timeMin = np.min(gpstime)
    timeMax = np.max(gpstime)
    distm = np.asarray(dist) * 0.002
    
    lasfile.header.scale = [0.001,0.001,0.001]
    
    lasfile.x = x
    lasfile.y = y
    lasfile.z = z
    lasfile.gps_time = np.asarray(gpstime,dtype=np.float64)
    lasfile.intensity = intens
    lasfile.return_num = retur
    lasfile.scan_num = np.asarray(scanNum, dtype=np.uint32)
    lasfile.laser_num = np.asarray(vert, dtype=np.int16)
    lasfile.total_accel = np.round(np.asarray(totalAccel, dtype=np.float16),3)
    lasfile.h_vel = np.round(np.asarray(horVelo, dtype=np.float16),3)
    lasfile.v_vel = np.round(np.asarray(upVelo, dtype=np.float16),3)
    lasfile.z_ang_rate = np.round(np.asarray(ZangRate, dtype=np.float16),3)
    lasfile.delta_roll = np.asarray(deltaRoll, dtype=np.float16)
    lasfile.scan_azi = np.asarray(azi).astype(int)
    lasfile.range = distm.astype(int)
    lasfile.heading = np.asarray(heading).astype(int)
    
    lasfile.close()
    
    print("                TEMP FILE " + las_filename + "_temp.las is DONE!")
    logMessStr += "                TEMP FILE " + las_filename + "_temp.las is DONE!\n"
    
    
    return timeMin, timeMax, logMessStr


def outputPrcsPoints_l_verbose(filePath,las_filename, x, y, z, gpstime, intens, x0, y0, z0, R, P, H, Xg, Yg, Zg, Lat, Lon, Xg_offset, Yg_offset, Zg_offset):
    
    #x0 = distance in socs [metres]
    #y0 = azimuth in socs [degrees]
    #z0 = vert. angle in socs [degrees]
    
    hdr = laspy.header.Header()
    hdr.version = "1.2"
    hdr.data_format_id = 3
    
    #the following ID fields must be less than or equal to 32 characters in length
    System_ID = "OpenMMS"
    Software_ID = "OpenMMS OSS v1.3.0"
    
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
    
    lasfile = laspy.file.File(filePath / (las_filename + "_temp.las"), mode="w", header=hdr)
    lasfile.define_new_dimension(name="dist_socs",data_type=9,description="")
    lasfile.define_new_dimension(name="azi_socs",data_type=9,description="")
    lasfile.define_new_dimension(name="vert_socs",data_type=9,description="")
    lasfile.define_new_dimension(name="roll",data_type=9,description="")
    lasfile.define_new_dimension(name="pitch",data_type=9,description="")
    lasfile.define_new_dimension(name="heading",data_type=9,description="")
    lasfile.define_new_dimension(name="scanner_x",data_type=9,description="")
    lasfile.define_new_dimension(name="scanner_y",data_type=9,description="")
    lasfile.define_new_dimension(name="scanner_z",data_type=9,description="")
    lasfile.define_new_dimension(name="scanner_lat",data_type=9,description="")
    lasfile.define_new_dimension(name="scanner_lon",data_type=9,description="")
    
    print("                TEMP FILE " + las_filename + "_temp.las is being CREATED ...")
    logMessStr = "                TEMP FILE " + las_filename + "_temp.las is being CREATED ...\n"
    
    xmin = np.floor(np.min(x))
    ymin = np.floor(np.min(y))
    zmin = np.floor(np.min(z))
    timeMin = np.min(gpstime)
    timeMax = np.max(gpstime)
    
    lasfile.header.offset = [xmin,ymin,zmin]
    lasfile.header.scale = [0.001,0.001,0.001]
    
    lasfile.x = x
    lasfile.y = y
    lasfile.z = z
    lasfile.gps_time = np.asarray(gpstime,dtype=np.float64)
    lasfile.intensity = intens
    xm = np.asarray(x0,dtype=np.float32) * 0.002
    lasfile.dist_socs = xm
    lasfile.azi_socs = y0
    lasfile.vert_socs = z0
    lasfile.roll = np.round(R,4)
    lasfile.pitch = np.round(P,4)
    lasfile.heading = np.round(H,4)
    lasfile.scanner_x = np.round((Xg - Xg_offset),4)
    lasfile.scanner_y = np.round((Yg - Yg_offset),4)
    lasfile.scanner_z = np.round((Zg - Zg_offset),4)
    lasfile.scanner_lat = np.round(Lat,7)
    lasfile.scanner_lon = np.round(Lon,7)
        
    print("                TEMP FILE " + las_filename + "_temp.las is DONE!")
    logMessStr += "                TEMP FILE " + las_filename + "_temp.las is DONE!\n"
    lasfile.close()
    
    return timeMin, timeMax, logMessStr


def nonblank_lines(f):
    for l in f:
        line = l.rstrip()
        if line:
            yield line


def georefChunk(processID, fileExtension, processChunkName, params, hourRollOver, navinterpSinLat, navinterpCosLat, navinterpSinLon, navinterpCosLon, navinterpEllH, navinterpGeoidSep, navinterpSinR, navinterpCosR, navinterpSinP, navinterpCosP, navinterpSinH, navinterpCosH, navinterpXg, navinterpYg, navinterpZg, navinterpTa, navinterpZar, navinterpDr, navinterpVh, navinterpVu, firstNavTime, lastNavTime, pcap_filename, quick, verbose, start_time, end_time, packet_interval, X_dict, Y_dict, time_dict, posPack_dict, badPPSmessages_dict, logMessages_dict, geo_epsg, proj_epsg, Xg_offset, Yg_offset, Zg_offset, vert_angles, laser_scale_factors, azimuth_corrections, distance_corrections, filePath):

    processID -= 1
    #program variables
    runQuick = False
    verboseOut = False
    
    startTime = float(start_time)
    endTime = float(end_time)
    packetInterval = int(packet_interval)
    
    if quick.upper() == "TRUE":
        runQuick = True
        
    if verbose.upper() == "TRUE":
        verboseOut = True
    
    bytesRead = 0
    position_packets = []
    points_timing = []
    points_azi = []
    points_vert = []
    points_dist = []
    points_intens = []
    points_return = []
    points_scanNum = []
    points_dist_sf = []
    points_azi_corr = []
    points_dist_corr = []
    scans_count = (processID+1) * 100000
    badPPSmessages = 0
    num_points_per_packet = []
    packet_timings = []
    num_position_packets = 0
    num_data_packets = 0
    num_used_data_packets = 0
    num_bad_position_packets = 0
    num_bad_data_packets = 0
    num_outside_points = 0
    skipped_bytes = 0
    oldAzi = -1
    oldTime = -1
    rotRate = 0
    utcHour = -1
    previousHour = 0
    continueRead = True

    pcapfile = open(pcap_filename, 'rb')
    
#    sys.stdout = sys.__stdout__
    print("                CHUNK " + processChunkName + " is being PROCESSED ...")
    logMessStr = ""
    time.sleep(0.01)

    bytesRead = readPcapFileHeader(pcapfile, bytesRead)

    safeToRead = True
    count = 0
    while safeToRead:
        bytesRead = 0
        
        safeToRead, num_position_packets, num_data_packets, num_used_data_packets, skipped_bytes, bytesRead, position_packets, num_bad_position_packets, utcHour, points_timing, points_azi, \
        points_vert, points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, num_outside_points, num_points_per_packet, continueRead, points_scanNum, \
        scans_count, badPPSmessages, packet_timings, points_dist_sf, points_azi_corr, points_dist_corr = readPcapPacketHeader(pcapfile, \
        num_position_packets, num_data_packets, num_used_data_packets, skipped_bytes, bytesRead, position_packets, num_bad_position_packets, utcHour, points_timing, points_azi, points_vert, \
        points_dist, points_intens, points_return, num_bad_data_packets, oldAzi, oldTime, rotRate, num_outside_points, firstNavTime, lastNavTime, runQuick, hourRollOver, num_points_per_packet, \
        startTime, endTime, packetInterval, continueRead, points_scanNum, scans_count, badPPSmessages, packet_timings, points_dist_sf, points_azi_corr, points_dist_corr, vert_angles, \
        laser_scale_factors, azimuth_corrections, distance_corrections)
        
        count += 1
        
        if utcHour != -1:
            if utcHour - previousHour < 0:
                utcHour += 24
            previousHour = utcHour
        
    pcapfile.close()
    
    timeMin = 0
    timeMax = 0

    timeInfo = "-1,-1,-1"
    
    if len(points_timing) > 0:                                                                                                                                
        x0, y0, z0, X, Y, Z, R, P, H, Xg, Yg, Zg, Lat, Lon, geoidSep, totalAccel, ZangRate, deltaRoll, horVelo, upVelo = geoReferenceCalcs(points_timing, points_azi, points_vert, points_dist, points_intens, points_return, navinterpSinLat, navinterpCosLat, navinterpSinLon, navinterpCosLon, navinterpEllH, navinterpGeoidSep, navinterpSinR, navinterpCosR, navinterpSinP, navinterpCosP, navinterpSinH, navinterpCosH, navinterpXg, navinterpYg, navinterpZg, navinterpTa, navinterpZar, navinterpDr, navinterpVh, navinterpVu, params, points_dist_sf, points_azi_corr, points_dist_corr)
        
        # if int(utmZone) > 0:
        X, Y, Z, Success = myProj(X,Y,Z,geoidSep,geo_epsg,proj_epsg)
            

        if verboseOut == False:
            if fileExtension == ".LAS" or fileExtension == ".LAZ":
                timeMin, timeMax, logMessStr = outputPrcsPoints_l(filePath,processChunkName,X,Y,Z,points_timing,points_intens,points_return,points_dist,H,points_vert,points_azi,points_scanNum,totalAccel,ZangRate,deltaRoll,horVelo,upVelo)
            else:
                logMessStr = outputPrcsPoints_a(filePath,processChunkName,X,Y,Z,points_timing,points_intens,points_return,points_dist,H,points_vert,points_scanNum,totalAccel,ZangRate,deltaRoll)
        else:
            if fileExtension == ".LAS" or fileExtension == ".LAZ":
                timeMin, timeMax, logMessStr = outputPrcsPoints_l_verbose(filePath,processChunkName,X,Y,Z,points_timing,points_intens,points_dist,points_azi,points_vert,R,P,H,Xg,Yg,Zg,Lat,Lon,Xg_offset,Yg_offset,Zg_offset)
            else:
                logMessStr = outputPrcsPoints_a_verbose2(filePath,processChunkName,X,Y,Z,points_timing,points_return,x0,y0,z0,R,P,H,Xg,Yg,Zg,Lat,Lon,Xg_offset,Yg_offset,Zg_offset)
            
        timeInfo = str(timeMin) + "," + str(timeMax) + "," + str(rotRate)
        
        logMessStr = "                CHUNK " + processChunkName + " is being PROCESSED ...\n" + logMessStr
    
                        
    else:
        print("                CHUNK " + processChunkName + " is OUTSIDE THE VALID NAV TIMES")
        logMessStr = "                CHUNK " + processChunkName + " is OUTSIDE THE VALID NAV TIMES\n" + logMessStr
        
    time_dict[processID] = timeInfo
    X_dict[processID] = packet_timings
    Y_dict[processID] = num_points_per_packet
    posPack_dict[processID] = position_packets
    badPPSmessages_dict[processID] = badPPSmessages
    logMessages_dict[processID] = logMessStr
    
    
def georef_project(params_filename, nav_filename, pcap_filename, las_filename, log_filename, quick, verbose, start_time, end_time, packet_interval, logFile, geo_epsg, proj_epsg, geocs_name, projcs_name, cal_filename, semi_major, semi_minor, filePath, pcapName, navName):
    
    minTimeProc = 0
    X_dict = []
    Y_dict = []
    posPack_dict = []
    rotRate = 0.0
    Success = False
    
    versionNum = "1.3.0"
    
    eccentricity2 = 1.0 - ((semi_minor * semi_minor) / (semi_major * semi_major))
    
    runQuick = False
    if quick.upper() == "TRUE":
        runQuick = True
    
    verboseOut = False
    if verbose.upper() == "TRUE":
        verboseOut = True
    
    lasFileSeries = las_filename[:-4]
    fileExtension = las_filename[-4:].upper()
    
    numCores = mp.cpu_count()
    
    if runQuick:
        numCores = int(numCores / 2)
    else:
        numCores -= 1
        
    if numCores == 0:
        numCores = 1
    
    #Debugging purposes
#    numCores = 1
    
    startPCP = time.time()
    endPCP = time.time()
    
    osType = os.name.upper()    #  POSIX or NT
    
    if fileExtension == ".LAS" or fileExtension == ".LAZ" or fileExtension == ".CSV":
        if quick.upper() == "TRUE":
            print("\n\n******************** QUICK LIDAR POINT CLOUD PROCESSING HAS STARTED ********************")
            sys.stdout = sys.__stdout__
            print("\n\n******************** QUICK LIDAR POINT CLOUD PROCESSING HAS STARTED ********************")
            sys.stdout = logFile
        else:
            print("\n\n********************* FULL LIDAR POINT CLOUD PROCESSING HAS STARTED ********************")
            sys.stdout = sys.__stdout__
            print("\n\n********************* FULL LIDAR POINT CLOUD PROCESSING HAS STARTED ********************")
            sys.stdout = logFile
    
        #read in boresight parameters for specific system being used
        params = []
        with open(params_filename) as paramsfile:
            for line in nonblank_lines(paramsfile):
                if line.startswith('#') == False:
                    params.append(line.strip('\n'))
        
        paramsPath = Path(params_filename)
        paramsName = paramsPath.name
        print("\n            Software Version: " + versionNum)
        print("\n            System Parameters File -> " + str(paramsName))
        print("                Boresight X [deg]: " + str(params[0]))
        print("                Boresight Y [deg]: " + str(params[1]))
        print("                Boresight Z [deg]: " + str(params[2]))        
        sys.stdout = sys.__stdout__

        print("\n            Processing Log File -> " + log_filename)
        print("\n            Software Version: " + versionNum)
        print("\n            System Parameters File -> " + str(paramsName))
        print("                Boresight X [deg]: " + str(params[0]))
        print("                Boresight Y [deg]: " + str(params[1]))
        print("                Boresight Z [deg]: " + str(params[2]))
        sys.stdout = logFile
        
        vert_angles = [-15., 1., -13., 3., -11., 5., -9., 7., -7., 9., -5., 11., -3., 13., -1., 15.]
        laser_scale_factors = [1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]
        azimuth_corrections = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        distance_corrections = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        
        #read in VLP-16 calibration parameters for specific system being used
        if cal_filename != "":
            print("\n            VLP-16 Internal Calibration File -> " + str(cal_filename.name))
            sys.stdout = sys.__stdout__
            print("\n            VLP-16 Internal Calibration File -> " + str(cal_filename.name))
            sys.stdout = logFile
            
            with open(cal_filename) as calfile:
                count = 0
                for line in nonblank_lines(calfile):
                    if line.startswith('#') == False and line.startswith('//') == False:
                        count += 1
                        calData = line.strip("\n").split(",")
                        if len(calData) == 16:
                            if count == 1:
                                vert_angles = []
                                for i in range(0,16):
                                    vert_angles.append(float(calData[i]))
                            elif count == 2:
                                laser_scale_factors = []
                                for i in range(0,16):
                                    laser_scale_factors.append(float(calData[i]))
                            elif count == 3:
                                azimuth_corrections = []
                                for i in range(0,16):
                                    azimuth_corrections.append(float(calData[i]))
                            elif count == 4:
                                distance_corrections = []
                                for i in range(0,16):
                                    distance_corrections.append(float(calData[i]))
        else:
            print("\n            VLP-16 Internal Calibration File -> SENSOR DEFAULT")
            sys.stdout = sys.__stdout__
            print("\n            VLP-16 Internal Calibration File -> SENSOR DEFAULT")
            sys.stdout = logFile
            
        print("\n            Reading and Structuring NAV Data...\n")
        sys.stdout = sys.__stdout__
        print("\n            Reading and Structuring NAV Data...\n")
        sys.stdout = logFile
        
        start = time.time()
    
        nav0, hourRollOver, dateRollOver, header_items = readCSVNav(nav_filename)
        navTime, sinLat, cosLat, sinLon, cosLon, EllH, GeoidSep, sinR, cosR, sinP, cosP, sinH, cosH, Xg, Yg, Zg, totalAccel, ZangRate, deltaRoll, Vh, Vu, Xg_offset, Yg_offset, Zg_offset = navRPH2SinCos(nav_filename, nav0, semi_major, eccentricity2, verboseOut)
        
        navinterpSinLat = spinterp.interp1d(navTime, sinLat, axis=0)
        navinterpCosLat = spinterp.interp1d(navTime, cosLat, axis=0)
        navinterpSinLon = spinterp.interp1d(navTime, sinLon, axis=0)
        navinterpCosLon = spinterp.interp1d(navTime, cosLon, axis=0)
        navinterpEllH = spinterp.interp1d(navTime, EllH, axis=0)
        navinterpGeoidSep = spinterp.interp1d(navTime, GeoidSep, axis=0)
        navinterpSinR = spinterp.interp1d(navTime, sinR, axis=0)
        navinterpCosR = spinterp.interp1d(navTime, cosR, axis=0)
        navinterpSinP = spinterp.interp1d(navTime, sinP, axis=0)
        navinterpCosP = spinterp.interp1d(navTime, cosP, axis=0)
        navinterpSinH = spinterp.interp1d(navTime, sinH, axis=0)
        navinterpCosH = spinterp.interp1d(navTime, cosH, axis=0)
        navinterpXg = spinterp.interp1d(navTime, Xg, axis=0)
        navinterpYg = spinterp.interp1d(navTime, Yg, axis=0)
        navinterpZg = spinterp.interp1d(navTime, Zg, axis=0)
        navinterpTa = spinterp.interp1d(navTime, totalAccel, axis=0)
        navinterpZar = spinterp.interp1d(navTime, ZangRate, axis=0)
        navinterpDr = spinterp.interp1d(navTime, deltaRoll, axis=0)
        navinterpVh = spinterp.interp1d(navTime, Vh, axis=0)
        navinterpVu = spinterp.interp1d(navTime, Vu, axis=0)

        startTime = 0
        endTime = 0

        firstNavTime = navTime[0]
        lastNavTime = navTime[len(navTime)-1]
        
        if start_time == "-1":
            startTime = firstNavTime
        else:
            terms = []
            if "+" in start_time:
                terms = start_time.split("+")
                for i in range(0,len(terms)):
                    startTime += float(terms[i])
            elif "-" in start_time:
                terms = start_time.split("-")
                print(terms[0])
                startTime = float(terms[0])
                for i in range(1,len(terms)):
                    startTime -= float(terms[i])
            else:
                startTime = float(start_time)
            
        if end_time == "-1":
            endTime = lastNavTime
        else:
            terms = []
            if "+" in end_time:
                terms = end_time.split("+")
                for i in range(0,len(terms)):
                    endTime += float(terms[i])
            elif "-" in end_time:
                terms = end_time.split("-")
                endTime = float(terms[0])
                for i in range(1,len(terms)):
                    endTime -= float(terms[i])
            else:
                endTime = float(end_time)
        
        timeChunk = endTime - startTime
        if timeChunk < 0:
            timeChunk += 86400.
        timeChunk1 = int(timeChunk / numCores)
        timeChunk2 = timeChunk - timeChunk1 * (numCores - 1)
        
        end = time.time()
        
        if geocs_name[0] == "\"":
            geocs_name = geocs_name[1:]
        if geocs_name[-1] == "\"":
            geocs_name = geocs_name[:-1]
        
        if projcs_name[0] == "\"":
            projcs_name = projcs_name[1:]
        if projcs_name[-1] == "\"":
            projcs_name = projcs_name[:-1]
            
        print("            --------------NAV SUMMARY--------------")
        print("            File: " + navName)
        print("            UTC Hour Crossings    -> " + str(hourRollOver))
        print("            UTC Date Crossing     -> " + str(dateRollOver))
        print("            ---------------------------------------")
        print("            Done in " + str(round(end-start,2)) + " seconds\n")
        
        print("            Reading Lidar Data File: " + pcapName + " (using " + str(numCores) + " CPUs) ...")
        print("                         Start Time: " + str(round(startTime,3)) + " seconds")
        print("                           End Time: " + str(round(endTime,3)) + " seconds")
        print("                         Total Time: " + str(round(timeChunk,3)) + " seconds")
        print("                    CPUs Chunk Time: " + str(round(timeChunk / numCores,3))  + " seconds")
        print("\n            Input Geographic Coordinate System: " + geocs_name)
        print("            Output Projected Coordinate System: " + projcs_name + "\n\n")
        print("            ----------------- PROCESSING STATUS MESSAGES -----------------\n")
        sys.stdout = sys.__stdout__
        
        print("            --------------NAV SUMMARY--------------")
        print("            File: " + navName)
        print("            UTC Hour Crossings    -> " + str(hourRollOver))
        print("            UTC Date Crossing     -> " + str(dateRollOver))
        print("            ---------------------------------------")
        print("            Done in " + str(round(end-start,2)) + " seconds\n")
        
        print("            Reading Lidar Data File: " + pcapName + " (using " + str(numCores) + " CPUs) ...")
        print("                         Start Time: " + str(round(startTime,3)) + " seconds")
        print("                           End Time: " + str(round(endTime,3)) + " seconds")
        print("                         Total Time: " + str(round(timeChunk,3)) + " seconds")
        print("                    CPUs Chunk Time: " + str(round(timeChunk / numCores,3))  + " seconds")
        print("\n            Input Geographic Coordinate System: " + geocs_name)
        print("            Output Projected Coordinate System: " + projcs_name + "\n\n")
        print("            ----------------- PROCESSING STATUS MESSAGES -----------------\n")
#        sys.stdout = logFile
        
        #Manual check to make sure the coordinate system info is useable
        _, _, _, Success = myProj(-1094900,-3936200,4882000,0,geo_epsg,proj_epsg,True)
        
        if Success:
        
            procs = []
            
            listExt = ""
            if fileExtension == ".LAZ" or fileExtension == ".LAS":
                listExt = ".las"
            else:
                listExt = ".csv"
            
            manager = mp.Manager()
            time_dict = manager.dict()
            X_dict = manager.dict()
            Y_dict = manager.dict()
            posPack_dict = manager.dict()
            badPPSmessages_dict = manager.dict()
            logMessages_dict = manager.dict()
            
            fileListTxt = open(filePath / (lasFileSeries + "_fileList.temp"),"w")
            
            fileListValues = []
            
            for i in range(1,numCores):
                chunkStartTime = startTime + (i-1) * timeChunk1
                chunkEndTime = chunkStartTime + timeChunk1
                processChunkName = lasFileSeries + "_" + str(i)
                chunkFileName = processChunkName + "of" + str(numCores) 
                fileListValues.append(str(filePath / (chunkFileName + "_temp" + listExt)) + "\n")
                proc1 = mp.Process(target=georefChunk,args=(i, fileExtension, chunkFileName, params, hourRollOver, navinterpSinLat, navinterpCosLat, navinterpSinLon, navinterpCosLon, navinterpEllH, navinterpGeoidSep, navinterpSinR, navinterpCosR, navinterpSinP, navinterpCosP, navinterpSinH, navinterpCosH, navinterpXg, navinterpYg, navinterpZg, navinterpTa, navinterpZar, navinterpDr, navinterpVh, navinterpVu, firstNavTime, lastNavTime, pcap_filename, quick, verbose, chunkStartTime, chunkEndTime, packet_interval, X_dict, Y_dict, time_dict, posPack_dict, badPPSmessages_dict, logMessages_dict, geo_epsg, proj_epsg, Xg_offset, Yg_offset, Zg_offset, vert_angles, laser_scale_factors, azimuth_corrections, distance_corrections, filePath))
                procs.append(proc1)
                
            if numCores == 1:
                chunkEndTime = startTime
                i = 0
                
            chunkStartTime = chunkEndTime
            chunkEndTime = chunkStartTime + timeChunk2
            processChunkName = lasFileSeries + "_" + str(i+1)
            chunkFileName = processChunkName + "of" + str(numCores)
            fileListValues.append(str(filePath / (chunkFileName + "_temp" + listExt)) + "\n")
            proc2 = mp.Process(target=georefChunk,args=(i+1, fileExtension, chunkFileName, params, hourRollOver, navinterpSinLat, navinterpCosLat, navinterpSinLon, navinterpCosLon, navinterpEllH, navinterpGeoidSep, navinterpSinR, navinterpCosR, navinterpSinP, navinterpCosP, navinterpSinH, navinterpCosH, navinterpXg, navinterpYg, navinterpZg, navinterpTa, navinterpZar, navinterpDr, navinterpVh, navinterpVu, firstNavTime, lastNavTime, pcap_filename, quick, verbose, chunkStartTime, chunkEndTime, packet_interval, X_dict, Y_dict, time_dict, posPack_dict, badPPSmessages_dict, logMessages_dict, geo_epsg, proj_epsg, Xg_offset, Yg_offset, Zg_offset, vert_angles, laser_scale_factors, azimuth_corrections, distance_corrections, filePath))
            procs.append(proc2)
            
            for iproc in procs:
                iproc.start()
                time.sleep(0.01)
            
            print("")
            
            for iproc in procs:
                iproc.join()
            
            minTimeProc = 1000000.
            rotRate = 0
            rotRateList = []
            
            sys.stdout = logFile
            for logMessStr in logMessages_dict.values():
                print(logMessStr)
            
            chunk_index = 0
            for times in time_dict.values():
                if times != '-1,-1,-1':
                    fileListTxt.write(fileListValues[chunk_index])
                chunk_index += 1
                    
            fileListTxt.close()
            

            
            for minMaxTime in time_dict.values():
                times = minMaxTime.split(",")
                if float(times[0]) < minTimeProc and float(times[0]) >= 0:
                    minTimeProc = float(times[0])
                #odd spot to collect the rotation rate of the sensor but it was convenient as times for each process where already being reported back to the main process (ie., HERE)
                if float(times[2]) >= 0:
                    rotRateList.append(float(times[2]))
            
            for i in range(0,len(rotRateList)):
                rotRate += rotRateList[i] / len(rotRateList)
            
            time.sleep(0.1)
            
            if badPPSmessages_dict[numCores-1] != 0:
                print("\n*** WARNING: " + str(badPPSmessages_dict[numCores-1]) + " BAD PPS MESSAGES FOUND")
                sys.stdout = sys.__stdout__
                print("\n*** WARNING: " + str(badPPSmessages_dict[numCores-1]) + " BAD PPS MESSAGES FOUND")
                sys.stdout = logFile
            
            print("\n            Merging Point Cloud  ...")
            sys.stdout = sys.__stdout__
            print("\n            Merging Point Cloud ...")
            sys.stdout = logFile
            
            start = time.time()
            
            if fileExtension == ".LAS" or fileExtension == ".LAZ":
                if osType == "NT": #Windows
                    print()
    
                    args=[r"C:\OpenMMS\code\las2las", '-lof', str(filePath / (lasFileSeries + "_fileList.temp")), '-meter', '-merged', '-epsg', str(proj_epsg), '-o', str(filePath / (lasFileSeries + fileExtension.lower()))]
                    
                    proc=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,shell=True)
                    output,error=proc.communicate()
                else:
                    os.chdir(r"/OpenMMS/code/")
                    subprocess.call(r"/OpenMMS/code/las2las" + ' -lof '+ str(filePath / (lasFileSeries + "_fileList.temp")) + ' -meter ' + '-merged' + ' -epsg ' + str(proj_epsg) + ' -o ' + str(filePath / (lasFileSeries + fileExtension.lower())), shell=True)
            else:
                mergedCSV = open(filePath / (lasFileSeries + fileExtension.lower()),"w")
                skipHeader = False
                with open(filePath / (lasFileSeries + "_fileList.temp")) as csvsList:
                    for fileline in nonblank_lines(csvsList):
                        if os.path.isfile(fileline):
                            with open(fileline) as csvFile:
                                count = 0
                                for dataline in nonblank_lines(csvFile):
                                    if count > 0 or skipHeader == False: 
                                        mergedCSV.write(dataline.strip("\n")+"\n")
                                    count += 1
                                if skipHeader == False:
                                    skipHeader == True
                mergedCSV.close()
            
            print("            Deleting temp files...")
            sys.stdout = sys.__stdout__
            print("            Deleting temp files...")
            sys.stdout = logFile
            
            for i in range(1,numCores+1):
                if os.path.isfile(filePath / (lasFileSeries + "_" + str(i) + "of" + str(numCores) + "_temp" + listExt)):
                    os.remove(filePath / (lasFileSeries + "_" + str(i) + "of" + str(numCores) + "_temp" + listExt))
            
            end = time.time()
            print("            Done in " + str(round(end-start,2)) + " seconds")
            sys.stdout = sys.__stdout__
            print("            Done in " + str(round(end-start,2)) + " seconds")
            sys.stdout = logFile
            
            if os.path.isfile(filePath / (lasFileSeries + "_fileList.temp")):    
                os.remove(filePath / (lasFileSeries + "_fileList.temp"))
            
            endPCP = time.time()
        else:
            sys.stdout = logFile
            print("*** ERROR: PROBLEM WITH THE GEODETIC AND/OR PROJECTED COORDINATE SYSTEM (CHECK EPSG IDS) ***\n")
            sys.stdout = sys.__stdout__
            print("*** ERROR: PROBLEM WITH THE GEODETIC AND/OR PROJECTED COORDINATE SYSTEM (CHECK EPSG IDS) ***\n")
            sys.stdout = logFile
            
    else:
        print("\n********** UNRECOGNIZED OUTPUT FILE FORMAT (" + fileExtension + ") **********")
        sys.stdout = sys.__stdout__
        print("\n********** UNRECOGNIZED OUTPUT FILE FORMAT (" + fileExtension + ") **********")
        sys.stdout = logFile

    return startPCP, endPCP, lasFileSeries, fileExtension.lower(), minTimeProc, X_dict, Y_dict, posPack_dict, rotRate, numCores, Success

##### command line start
if __name__ == "__main__":
    print("\nVLP-16 Lidar Georeferencing Started...\n")
    
    proj_data_path = sys.argv[1]
    params_filename = sys.argv[2]
    nav_filename = sys.argv[3]
    pcap_filename = sys.argv[4]
    las_filename = sys.argv[5]
    quick = sys.argv[6]
    verbose = sys.argv[7]
    start_time = sys.argv[8]
    end_time = sys.argv[9]
    packet_interval = sys.argv[10]
    display_plots = sys.argv[11]
    geo_epsgStr = sys.argv[12]
    proj_epsgStr = sys.argv[13]

    geo_epsg = -1
    proj_epsg = -1
    
    semi_major = 0.0
    semi_minor = 0.0
    
    try:
        geo_epsg = int(geo_epsgStr)
        proj_epsg = int(proj_epsgStr)
    except:
        pass
    
    curPath = ""
    try:
        curPath = sys.argv[14]
    except:
        pass
    
    vlp16Cal = False
    try:
        if sys.argv[15].upper() == "TRUE":
            vlp16Cal = True
    except:
        pass
    
    foundGeoCS = False
    foundProjCS = False
    geocs_name = ""
    projcs_name = ""
    
    if geo_epsg != -1:
        if proj_epsg != -1:
            
            with open(proj_data_path + "gcs.csv","r") as geocs:
                for line in geocs:
                    geocs_info = line.strip("\n").split(",")
                    if geocs_info[0] == str(geo_epsg):
                        foundGeoCS = True
                        geocs_name = geocs_info[1]
                        ellipsoid = CRS("epsg:" + str(geo_epsg)).ellipsoid
                        semi_major = ellipsoid.semi_major_metre
                        semi_minor = ellipsoid.semi_minor_metre
                        break
                    
            with open(proj_data_path + "pcs.csv","r") as projcs:
                for line in projcs:
                    projcs_info = line.strip("\n").split(",")
                    if projcs_info[0] == str(proj_epsg):
                        foundProjCS = True
                        projcs_name = projcs_info[1]
                        break
                    
            if foundGeoCS:
                if foundProjCS:
            
                    pcapPath = Path(pcap_filename)
                    navPath = Path(nav_filename)
                    pcapName = pcapPath.name
                    navName = navPath.name
                    filePath = pcapPath.parents[0]
                    
                    cal_filename = ""
                    
                    if curPath != "":
                        curFiles = []
            
                        for (dirpath, dirnames, filenames) in os.walk(curPath):
                            if curPath == dirpath:
                                curFiles = filenames
                        
                        latestBorFile = ""
                        latestDateTime = datetime(1,1,1,0,0,0)
                        for i in range(0,len(curFiles)):
                            filename = str(curFiles[i])
                            if filename[-4:].upper() == ".BOR" and filename[0:17].upper() == "BORESIGHT_PARAMS_":
                                timeInfo = filename[17:-4].split('_')
                                if len(timeInfo) == 6:
                                    currentDateTime = datetime(int(timeInfo[0]),int(timeInfo[1]),int(timeInfo[2]),int(timeInfo[3]),int(timeInfo[4]),int(timeInfo[5]))
                                    timeDiff = currentDateTime - latestDateTime
                                    if timeDiff.days >= 0:
                                        latestDateTime = currentDateTime
                                        latestBorFile = filePath / filename
                                        
                        latestCalFile = ""
                        if vlp16Cal:
                            latestDateTime = datetime(1,1,1,0,0,0)
                            for i in range(0,len(curFiles)):
                                filename = str(curFiles[i])
                                if filename[-4:].upper() == ".CAL" and filename[0:13].upper() == "VLP16_PARAMS_":
                                    timeInfo = filename[13:-4].split('_')
                                    if len(timeInfo) == 6:
                                        currentDateTime = datetime(int(timeInfo[0]),int(timeInfo[1]),int(timeInfo[2]),int(timeInfo[3]),int(timeInfo[4]),int(timeInfo[5]))
                                        timeDiff = currentDateTime - latestDateTime
                                        if timeDiff.days >= 0:
                                            latestDateTime = currentDateTime
                                            latestCalFile = filePath / filename
                                        
                        if latestBorFile != "":
                            params_filename = latestBorFile
                            
                        if latestCalFile != "":
                            cal_filename = latestCalFile
                                    
                    osType = str(os.name.upper())    #  POSIX or NT
                    
                    timeNow = time.localtime()
                    
                    log_filename = "processing_" + str(timeNow[0]) + "_" + str(timeNow[1]) + "_" + str(timeNow[2]) + "_" + str(timeNow[3]) + "_" + str(timeNow[4]) + "_" + str(timeNow[5]) + ".log"
                    
                    logFile = open(filePath / log_filename, 'w')
                    OpenMMS2(logFile)
                    
                    startPCP, endPCP, outfileName, fileExtension, minTimeProc, X_dict, Y_dict, posPack_dict, rotRate, numCores, Success = georef_project(params_filename, nav_filename, pcap_filename, las_filename, log_filename, quick, verbose, start_time, end_time, packet_interval, logFile, geo_epsg, proj_epsg, geocs_name, projcs_name, cal_filename, semi_major, semi_minor, filePath, pcapName, navName)
                
                    if Success:
                
                        if display_plots.upper() == "TRUE":
                    
                            if os.path.isfile(filePath / (outfileName + fileExtension)):
                                if osType == "NT": #Windows
                                    if os.path.isfile(r"C:\Program Files\CloudCompare\CloudCompare.exe"):
                                        print("\n            Creating a CloudCompare .BIN point cloud, please wait ...")
                                        print("               TIME SCALAR FIELD OFFSET = " + str(minTimeProc) + " sec.\n")
                                        sys.stdout = sys.__stdout__
                                        print("\n            Creating a CloudCompare .BIN point cloud, please wait ...")
                                        print("               TIME SCALAR FIELD OFFSET = " + str(minTimeProc) + " sec.\n")
                                        sys.stdout = logFile
                                        
                                        args1=[r"C:\Program Files\CloudCompare\CloudCompare",'-SILENT','-AUTO_SAVE','OFF','-O','-GLOBAL_SHIFT','AUTO',str(filePath / (outfileName + fileExtension)),'-COORD_TO_SF','Z','-C_EXPORT_FMT','BIN','-NO_TIMESTAMP','-SAVE_CLOUDS'] #,'-LOG_FILE',outfileName + "_CC.log"]
                                        proc1=subprocess.Popen(args1,stdout=subprocess.PIPE,stderr=subprocess.PIPE,shell=True)
                                        output,error=proc1.communicate()
                                        
                                #Mac OS X
                                else:
                                    if os.path.isfile(r"/Applications/CloudCompare.app/Contents/MacOS/CloudCompare"):
                                        print("\n            Creating a CloudCompare .BIN point cloud, please wait ...")
                                        print("               TIME SCALAR FIELD OFFSET = " + str(minTimeProc) + " sec.\n")
                                        sys.stdout = sys.__stdout__
                                        print("\n            Creating a CloudCompare .BIN point cloud, please wait ...")
                                        print("               TIME SCALAR FIELD OFFSET = " + str(minTimeProc) + " sec.\n")
                                        sys.stdout = logFile
                                        subprocess.call(r"/Applications/CloudCompare.app/Contents/MacOS/CloudCompare -SILENT -AUTO_SAVE OFF -O -GLOBAL_SHIFT AUTO " + str(filePath / (outfileName + fileExtension)) + " -COORD_TO_SF Z -C_EXPORT_FMT BIN -NO_TIMESTAMP -SAVE_CLOUDS", shell=True)
                                
                                time.sleep(1)
                                endPCP = time.time()
                                print("\n***************************** DONE PROCESSING in " + str(round((endPCP - startPCP) / 60.,2)) + " mins. " + "*****************************")
                                sys.stdout = sys.__stdout__
                                print("\n***************************** DONE PROCESSING in " + str(round((endPCP - startPCP) / 60.,2)) + " mins. " + "*****************************")
                                sys.stdout = logFile
                                iHeartLidar2(logFile)
                                
                                if os.path.isfile(str(filePath / (outfileName + "_Z_TO_SF.bin"))):
                                    if os.path.isfile(str(filePath / (outfileName + ".bin"))):
                                        os.remove(str(filePath / (outfileName + ".bin")))
                                    os.rename(str(filePath / (outfileName + "_Z_TO_SF.bin")), str(filePath / (outfileName + ".bin")))
                                    time.sleep(1)
    
                                    if osType == "NT": #Windows
                                        print("\n            The point cloud is now opening in CloudCompare!")
                                        sys.stdout = sys.__stdout__
                                        print("\n            The point cloud is now opening in CloudCompare!")
                                        sys.stdout = logFile
                                        args2=[r"C:\Program Files\CloudCompare\CloudCompare.exe", str(filePath / (outfileName + ".bin"))]
                                        proc2=subprocess.Popen(args2,stdout=subprocess.PIPE,stderr=subprocess.PIPE,shell=True)
                                        
                                    #Mac OS X
                                    else:
                                        print("\n  The point cloud is now opening in CloudCompare!")
                                        sys.stdout = sys.__stdout__
                                        print("\n  The point cloud is now opening in CloudCompare!")
                                        sys.stdout = logFile
                                        subprocess.Popen(r"/Applications/CloudCompare.app/Contents/MacOS/CloudCompare " + str(filePath / (outfileName + ".bin")), shell=True)
                                    
                            else:
                                print("\n***************************** DONE PROCESSING in " + str(round((endPCP - startPCP) / 60.,2)) + " mins. " + "*****************************")
                                sys.stdout = sys.__stdout__
                                print("\n***************************** DONE PROCESSING in " + str(round((endPCP - startPCP) / 60.,2)) + " mins. " + "*****************************")
                                sys.stdout = logFile
                                iHeartLidar2(logFile)
                            
                            print("\n            Generating Data Collection Analysis Plot...")
                            sys.stdout = sys.__stdout__
                            print("\n            Generating Data Collection Analysis Plot...")
                    
                            logFile.close()
                            
                            newPointsDataX = []
                            newPointsDataY = []
                            Xoffset = 1
                            for i in range(0,numCores):
                                if i > 0:
                                    Xoffset += len(X_dict[i-1])
                                newPointsDataX += list(X_dict[i])
                                newPointsDataY += list(Y_dict[i])
                        
                            print("\n            Close the Plot figure to end the program\n")
                            plt.figure(figsize=(13,6))
                            plt.plot(newPointsDataX,newPointsDataY,linewidth=1.0)
                            plt.xlabel('UTC time [secs in the day]')
                            plt.ylabel('Collected Points per Packet')
                            plt.title("Data Collection Analysis (Rot. Rate = " + str(round(rotRate,2)) + " Hz)")
                            plt.savefig(filePath / "Data Collection Analysis.png")
                            plt.show()
                            plt.close()
                        
                        else:
                            print("\n***************************** DONE PROCESSING in " + str(round((endPCP - startPCP) / 60.,2)) + " mins. " + "*****************************")
                            sys.stdout = sys.__stdout__
                            print("\n***************************** DONE PROCESSING in " + str(round((endPCP - startPCP) / 60.,2)) + " mins. " + "*****************************")
                            sys.stdout = logFile
                            iHeartLidar2(logFile)
                            print("\n")
                            
                    logFile.close()
                else:
                    print("\n********** Unsupported Projected CS EPSG Number **********\n")
            else:
                print("\n********** Unsupported Geographic CS EPSG Number **********\n")
        else:
            print("\n********** Missing or Non-numeric Projected CS EPSG Number **********\n")
    else:
        print("\n********** Missing or Non-numeric Geographic CS EPSG Number **********\n")
