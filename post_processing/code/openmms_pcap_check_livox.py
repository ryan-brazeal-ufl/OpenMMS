# -*- coding: utf-8 -*-

############################################################################
#                        OpenMMS PCAP Check (Livox)                        #
############################################################################
# Version: 1.3.0                                                           #
# Date:    September 2020                                                  #
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
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
from tqdm import tqdm
from pathlib import Path
import multiprocessing as mp


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


def parseChunk(statusD, statsD, pointsD, start_time_ref, lock, chunkID, pointPackets, firmware):
    
    temp_warnings = 0 
    volt_warnings = 0
    motor_warnings = 0
    temp_errors = 0
    volt_errors = 0
    motor_errors = 0
    dirty_warnings = 0
    firmware_errors = 0
    pps_warnings = 0
    device_warnings = 0
    self_heating_warnings = 0
    time_sync_warnings = 0
    numReturn1Pts = 0
    numReturn2Pts = 0
    numReturn3Pts = 0
    numNullPts = 0
    points_per_sec = 0
    point_times_quant = []
    time_ref_prev = start_time_ref
    
    descStr = "Core " + str(chunkID+1)
    if chunkID+1 > 9:
        descStr += " "
    else:
        descStr += "  "
        
    lock.acquire()
    pbar = tqdm(total=len(pointPackets),unit=" packets",desc=descStr,position=chunkID)
    lock.release()
    
    for i in range(0,len(pointPackets)):
        
        lock.acquire()
        pbar.update()
        lock.release()
        
        time_ref = pointPackets[i][0]
        data_pc = pointPackets[i][1]
         
        time_since_start = time_ref - time_ref_prev
        if time_since_start > timedelta(seconds=1):
            point_times_quant.append([time_ref, points_per_sec])
            time_ref_prev = time_ref
            points_per_sec = 0
        
        # version = int.from_bytes(data_pc[42:43], byteorder='little')
        # slot_id = int.from_bytes(data_pc[43:44], byteorder='little')
        # lidar_id = int.from_bytes(data_pc[44:45], byteorder='little')
        timestamp_type = int.from_bytes(data_pc[50:51], byteorder='little')
        datatype = int.from_bytes(data_pc[51:52], byteorder='little')
        
        status_bits = str(bin(int.from_bytes(data_pc[46:47], byteorder='little')))[2:].zfill(8)
        status_bits += str(bin(int.from_bytes(data_pc[47:48], byteorder='little')))[2:].zfill(8)
        status_bits += str(bin(int.from_bytes(data_pc[48:49], byteorder='little')))[2:].zfill(8)
        status_bits += str(bin(int.from_bytes(data_pc[49:50], byteorder='little')))[2:].zfill(8)

        temp_status = int(status_bits[0:2], 2)
        volt_status = int(status_bits[2:4], 2)
        motor_status = int(status_bits[4:6], 2)
        dirty_status = int(status_bits[6:8], 2)
        firmware_status = int(status_bits[8:9], 2)
        pps_status = int(status_bits[9:10], 2)
        device_status = int(status_bits[10:11], 2)
        # fan_status = int(status_bits[11:12], 2)
        self_heating_status = int(status_bits[12:13], 2)
        # ptp_status = int(status_bits[13:14], 2)
        time_sync_status = int(status_bits[14:16], 2)
        # system_status = int(status_bits[30:], 2)
                         
        if temp_status == 1:
            temp_warnings += 1
        if volt_status == 1:
            volt_warnings += 1
        if motor_status == 1:
            motor_warnings += 1
        if temp_status == 2:
            temp_errors += 1
        if volt_status == 2:
            volt_errors += 1
        if motor_status == 2:
            motor_errors += 1
        if dirty_status == 1:
            dirty_warnings += 1
        if firmware_status == 1:
            firmware_errors += 1
        if pps_status == 1:
            pps_warnings += 1
        if device_status == 1:
            device_warnings += 1
        if self_heating_status == 0:
            self_heating_warnings += 1
        if time_sync_status == 0 or time_sync_status == 4:
            time_sync_warnings += 1
        
        if timestamp_type == 4:
            #mid-40 spherical data
            if datatype == 1:
                time_delta = timedelta(microseconds=10)
                time_divisor = 1
                if firmware == "03.03.0006":
                    time_divisor = 2
                elif firmware == "03.03.0007":
                    time_divisor = 3
                    time_delta = timedelta(microseconds=16.667)
                
                pt_time = time_ref - time_delta
                byte_pos = 60
                
                for j in range(0,100):
                    zeroORtwoORthree = j % time_divisor
                    pt_time += float(not (zeroORtwoORthree)) * time_delta
                    returnNum = 1 + zeroORtwoORthree
                    distance = float(struct.unpack('<I', data_pc[byte_pos:byte_pos+4])[0]) # / 1000.0
                    # zenith = float(struct.unpack('<H', data_pc[byte_pos+4:byte_pos+6])[0]) / 100.0
                    # azimuth = float(struct.unpack('<H', data_pc[byte_pos+6:byte_pos+8])[0]) / 100.0
                    # reflectivity = int.from_bytes(data_pc[byte_pos+8:byte_pos+9], byteorder='little')
                    byte_pos += 9
                    
                    if distance:
                        points_per_sec += 1
                        if returnNum == 1:
                            numReturn1Pts += 1
                        elif returnNum == 2:
                            numReturn2Pts += 1
                        elif returnNum == 3:
                            numReturn3Pts += 1
                    else:
                        numNullPts += 1
                        
            #horizon spherical data
            elif datatype == 5:
                pass
            
    statsD[chunkID] = [temp_warnings,volt_warnings,motor_warnings,temp_errors,volt_errors,\
                       motor_errors,dirty_warnings,firmware_errors,pps_warnings,device_warnings,\
                       self_heating_warnings,time_sync_warnings,numReturn1Pts,numReturn2Pts,\
                       numReturn3Pts,numNullPts]
        
    pointsD[chunkID] = point_times_quant
    
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


def main(pcap_filename, livox_filename):
    print()
    numCores = mp.cpu_count() - 1
    
    firmware = ""
    time_ref = None
    time_offsets = []
    pointPackets = []
    previous_timestamp = -2
    
    with open(livox_filename, 'r') as livox_file:
        firmware = livox_file.readline().strip("\n")
        yyyy = 2000 + int(livox_file.readline().strip("\n"))
        mm = int(livox_file.readline().strip("\n"))
        dd = int(livox_file.readline().strip("\n"))
        hh = int(livox_file.readline().strip("\n"))
        mins = int(livox_file.readline().strip("\n"))
        secs = float(livox_file.readline().strip("\n"))
        time_offsets.append(float(livox_file.readline().strip("\n")))
        time_offsets.append(float(livox_file.readline().strip("\n")))
        time_offsets.append(float(livox_file.readline().strip("\n")))
        secs_i = int(np.floor(secs))
        us = int(round((secs - float(secs_i)) * 1000000.0,0))
        time_ref = datetime(year=yyyy,month=mm,day=dd,hour=hh,minute=mins,second=secs_i,microsecond=us)

    epoch = time_ref    
    
    #time offset from when the next PPS pulse was detected to the start of data file opening
    data_time_offset = time_offsets[2] - time_offsets[0]
    
    # #time offset from receiving the PPS pulse to reading the UTC time from the serial port
    # pulse_time_offset = time_offsets[1] - time_offsets[0]        
    
    # #decimal number of seconds (using RPi's time, which should be closely in-sync with UTC time already)
    # clock_time = time_offsets[2] - np.floor(time_offsets[2])
    
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
    pcap = open(pcap_filename, 'rb')
    
    # PCAP file header
    b = bytearray(pcap.read(24))
    # magic_num = struct.unpack('<I', b[0:4])[0]
    # version_major = struct.unpack('<H', b[4:6])[0]
    # version_minor = struct.unpack('<H', b[6:8])[0]
    # thiszone = struct.unpack('<i', b[8:12])[0]
    # sigfigs = struct.unpack('<I', b[12:16])[0]
    # snaplen = struct.unpack('<I', b[16:20])[0]
    # network = struct.unpack('<I', b[20:24])[0]
    bytesRead += 24
    
    print("\nReading binary packet data from PCAP file\n")
    pbar = tqdm(total=pcapSize,unit_scale=True,unit_divisor=0.001e6,unit='B')

    # read through data packets in PCAP file
    while True:
        checkData = bytearray(pcap.read(16))
        
        if len(checkData) != 16:
            break
        else:
            bytesRead += 16
            # ts_sec = struct.unpack('<I', checkData[0:4])[0]
            # ts_usec = struct.unpack('<I', checkData[4:8])[0]
            incl_len = struct.unpack('<I', checkData[8:12])[0]
            # orig_len = struct.unpack('<I', checkData[12:16])[0]
            
            packetData = bytearray(pcap.read(incl_len))
            if len(packetData) == incl_len:
                bytesRead += incl_len            
    
                #mid-40 spherical
                if incl_len == 960:
                    timestamp_type = int.from_bytes(packetData[50:51], byteorder='little')
                    
                    if timestamp_type > 0:
                        timestamp_sec = 0
                        
                        # nanosecond timestamp
                        if timestamp_type == 1 or timestamp_type == 4:
                            timestamp_sec = round(float(struct.unpack('<Q', packetData[52:60])[0]) / 1000000000.0, 6) # convert to seconds
                            
                        # UTC timestamp, microseconds past the hour
                        elif timestamp_type == 3:
                            # timestamp_year = int.from_bytes(packetData[52:53], byteorder='little')
                            # timestamp_month = int.from_bytes(packetData[53:54], byteorder='little')
                            # timestamp_day = int.from_bytes(packetData[54:55], byteorder='little')
                            timestamp_hour = int.from_bytes(packetData[55:56], byteorder='little')
                            timestamp_sec = round(float(struct.unpack('<L', packetData[56:60])[0]) / 1000000.0, 6)  # convert to seconds
                            timestamp_sec += timestamp_hour * 3600.  # seconds into the day
                        
                        # startup condition
                        if previous_timestamp == -2:
                            previous_timestamp = timestamp_sec
                            
                        # looking for 1 second increments in the time data
                        if timestamp_sec - previous_timestamp < -0.9:
                            epoch += timedelta(seconds=1)
                        
                        previous_timestamp = timestamp_sec
                        packet_timestamp = epoch + timedelta(seconds=timestamp_sec)
                        
                        pointPackets.append([packet_timestamp, packetData])
                        pbar.update(976)
            
                #horizon spherical (dual return)
                elif incl_len == 1660:
                    pass
            
            else:
                break
    
    pbar.close()
    
    designNum = 0
    
    foundStuff = False
    if len(pointPackets) == 0:
        print("\n\n****************************************************************")
        print("**************** NO DATA FOUND IN THE PCAP FILE ****************")
        print("****************************************************************\n\n")
    else:
        foundStuff = True
        if numCores > 0:
            run = True
            while run:
                designNum = len(pointPackets) // numCores
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
    
    #for debugging purposes
    # numCores = 1
    
    if foundStuff:
        chunkIndex = [0]
        chunkSize = int(np.floor(float(len(pointPackets)) / float(numCores)))
        for i in range(1,numCores):
            chunkIndex.append(i * chunkSize)
        chunkIndex.append(len(pointPackets))

        print("\nParsing observations from binary packet data using " + str(len(chunkIndex)-1) + " CPU core(s), please wait...\n")
        
        procs = []
        
        manager = mp.Manager()
        statusD = manager.dict()
        statsD = manager.dict()
        pointsD = manager.dict()
        
        lock = mp.Lock()
        
        for i in range(0,len(chunkIndex)-1):
            statusD[i] = 0
            
        for i in range(0,len(chunkIndex)-1):
            proc1 = mp.Process(target=parseChunk,args=(statusD, statsD, pointsD, pointPackets[0][0], lock, i, pointPackets[chunkIndex[i]:chunkIndex[i+1]], firmware))                    
            procs.append(proc1)
            
        for iproc in procs:
            iproc.start()
        
        procs[len(procs)-1].join()
        
        temp_warnings = 0 
        volt_warnings = 0
        motor_warnings = 0
        temp_errors = 0
        volt_errors = 0
        motor_errors = 0
        dirty_warnings = 0
        firmware_errors = 0
        pps_warnings = 0
        device_warnings = 0
        self_heating_warnings = 0
        time_sync_warnings = 0
        numReturn1Pts = 0
        numReturn2Pts = 0
        numReturn3Pts = 0
        numNullPts = 0
        mission_times = []
        mission_points = []
        
        for i in range(0,len(chunkIndex)-1):
            temp_warnings += statsD[i][0] 
            volt_warnings += statsD[i][1]
            motor_warnings += statsD[i][2]
            temp_errors += statsD[i][3]
            volt_errors += statsD[i][4]
            motor_errors += statsD[i][5]
            dirty_warnings += statsD[i][6]
            firmware_errors += statsD[i][7]
            pps_warnings += statsD[i][8]
            device_warnings += statsD[i][9]
            self_heating_warnings += statsD[i][10]
            time_sync_warnings += statsD[i][11]
            numReturn1Pts += statsD[i][12]
            numReturn2Pts += statsD[i][13]
            numReturn3Pts += statsD[i][14]
            numNullPts += statsD[i][15]
            
            points_chunk = pointsD[i]
            for j in range(0,len(points_chunk)):
                mission_times.append(points_chunk[j][0])
                mission_points.append(points_chunk[j][1])

        total_points = (numReturn1Pts + numReturn2Pts + numReturn3Pts + numNullPts)
        if total_points > 1:
            
            timeEnd = time.time()
            print("\n\n            COMPLETED PCAP CHECK IN " + str(round((timeEnd - timeStart) / 60.,2)) + " mins \n")
            
            logFile = open(log_filename, 'w')
            
            OpenMMS1_log(logFile)
            
            stats =  ("\n            *********************  PCAP CHECK RESULTS  ********************\n\n")            
            stats += ("                     INPUT: "+ pcapName +" at "+"{:,}".format(pcapSize)+" bytes\n\n")
            stats += ("                  Time of Initial PPS Pulse: " + str(time_ref) + "\n")
            stats += ("              Initial PPS before Start Data: " + str(np.round(data_time_offset,3)) + "s\n\n")

            stats += ("                            First Data Time: " + str(pointPackets[0][0]) + " UTC\n")
            stats += ("                             Last Data Time: " + str(pointPackets[len(pointPackets)-1][0]) + " UTC\n")
            stats += ("                              Data Duration: " + str(pointPackets[len(pointPackets)-1][0] - pointPackets[0][0]) + "\n\n")
            stats += ("                              Total Packets: " + "{:,}".format(len(pointPackets)) + "\n")
            stats += ("                               Total Points: " + "{:,}".format(total_points - numNullPts) + "\n")
            stats += ("                                    Returns: " + "1st: " + "{:,}".format(numReturn1Pts) + "\n")
            stats += ("                                             " + "2nd: " + "{:,}".format(numReturn2Pts) + "\n")
            
            if firmware == "03.03.0007":
                stats += ("                                             " + "3rd: " + "{:,}".format(numReturn3Pts) + "\n")
            
            stats += ("\n                              LIVOX LIDAR SENSOR MESSAGES\n\n")
            stats += ("              Temperature Errors: " + str(temp_errors) + "           Temperature Warnings: " + str(temp_warnings) + "\n")
            stats += ("                  Voltage Errors: " + str(volt_errors) + "               Voltage Warnings: " + str(volt_warnings) + "\n")
            stats += ("                    Motor Errors: " + str(motor_errors) + "                 Motor Warnings: " + str(motor_warnings) + "\n")
            stats += ("                                                    Dirty Warnings: " + str(dirty_warnings) + "\n")
            stats += ("                                                      PPS Warnings: " + str(pps_warnings) + "\n")
            stats += ("                                                Time Sync Warnings: " + str(time_sync_warnings) + "\n")
            stats += ("                                      End Of Service Life Warnings: " + str(device_warnings) + "\n")

            stats += ("\n            ***************************************************************\n")
            
            logFile.write(stats)
            
            print(stats)
        
            plt.figure(figsize=(10,5))
            plt.yscale("log")
            plt.xlabel('Mission Time (UTC)')
            plt.ylabel('Points Per Second (logarithmic scale)')
            plt.title('Collected Points during Mission (' + pcapName + ')')
            plt.plot(mission_times, mission_points, 'g.', ms=1) #'g,')
            max_pts = np.max(mission_points)
            plt.ylim(bottom=1, top=(max_pts*10))
            plt.savefig(filePath / ("Collected Points - " + timeStr + ".jpg"))
            plt.show()
            
            print("            *** Note: Collected Points plot saved as image ***\n")
        
            iHeartLidar2(logFile)
        
            logFile.close()
        else:
            print("\n\n****************************************************************")
            print("*********** NO DATA AND/OR POSITION INFO WAS FOUND *************")
            print("****************************************************************\n\n")
    

### command line start
if __name__ == "__main__":
    print("\nLivox PCAP File Check Started...")
    pcap_filename = sys.argv[1]
    livox_filename = sys.argv[2]
    main(pcap_filename, livox_filename)