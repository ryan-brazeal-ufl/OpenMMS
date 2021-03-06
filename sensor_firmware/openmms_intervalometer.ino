/*##########################################################################
#                  OpenMMS Camera Intervalometer Firmware                  #
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
# Updates:                                                                 #
#    April 2019 - Removed 100msec from each of the final delay commands in #
#                 order to account for the ~100msec of internal processing #
#                 time needed                                              #
#                                                                          #
#    May 2020 - Updated intervals                                          #
##########################################################################*/

int inputPin1 = 9;
int inputPin2 = 10;
int inputPin3 = 11;
int cameraPin = 13;

int input1Value = 0;
int input2Value = 0;
int input3Value = 0;

int signal_duration = 700;

void setup() {
  
  pinMode(inputPin1, INPUT_PULLUP);
  pinMode(inputPin2, INPUT_PULLUP);
  pinMode(inputPin3, INPUT_PULLUP);
  pinMode(cameraPin, OUTPUT);
}

//allow 100 msec for internal processing (recursively for each additionally second of delay) before next trigger pulse is sent, keeps the intervals closer to proper values
void loop() {
  
  input1Value = digitalRead(inputPin1);
  input2Value = digitalRead(inputPin2);
  input3Value = digitalRead(inputPin3);
  
  if (input1Value == LOW && input2Value == LOW && input3Value == LOW)
  {
    //Binary = 0, 1.5 second interval
    digitalWrite(cameraPin,HIGH);
    delay(signal_duration);
    digitalWrite(cameraPin,LOW);
    delay(750);
  }
  else if (input1Value == HIGH && input2Value == LOW && input3Value == LOW)
  {
    //Binary = 1, 2 second interval
    digitalWrite(cameraPin,HIGH);
    delay(signal_duration);
    digitalWrite(cameraPin,LOW);
    delay(1200);
  }
  else if (input1Value == LOW && input2Value == HIGH && input3Value == LOW)
  {
    //Binary = 2, 2.5 second interval
    digitalWrite(cameraPin,HIGH);
    delay(signal_duration);
    digitalWrite(cameraPin,LOW);
    delay(1650);
  }
  else if (input1Value == HIGH && input2Value == HIGH && input3Value == LOW)
  {
    //Binary = 3, 3 second interval
    digitalWrite(cameraPin,HIGH);
    delay(signal_duration);
    digitalWrite(cameraPin,LOW);
    delay(2100);
  }
  else if (input1Value == LOW && input2Value == LOW && input3Value == HIGH)
  {
    //Binary = 4, 3.5 second interval
    digitalWrite(cameraPin,HIGH);
    delay(signal_duration);
    digitalWrite(cameraPin,LOW);
    delay(2550);    
  }
  else if (input1Value == HIGH && input2Value == LOW && input3Value == HIGH)
  {
    //Binary = 5, 4 second interval
    digitalWrite(cameraPin,HIGH);
    delay(signal_duration);
    digitalWrite(cameraPin,LOW);
    delay(3000);
  }
  else if (input1Value == LOW && input2Value == HIGH && input3Value == HIGH)
  {
    //Binary = 6, 5 second interval
    digitalWrite(cameraPin,HIGH);
    delay(signal_duration);
    digitalWrite(cameraPin,LOW);
    delay(3900);    
  }
  else if (input1Value == HIGH && input2Value == HIGH && input3Value == HIGH)
  {
    //Binary = 7, DO NOTHING/REST STATE
  }
}
