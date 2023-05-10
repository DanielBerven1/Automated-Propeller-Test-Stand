#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import ADS1256
import DAC8532
import RPi.GPIO as GPIO
import pandas as pd
import threading
import time
import pigpio

df = pd.DataFrame(columns = ['Setpoint', 'Left Load Cell', 'Right Load Cell'])
setpoint = 0

ESC=18  #Connect the ESC in this GPIO pin 
# pi = pigpio.pi()
# pi.set_servo_pulsewidth(ESC, 0) 

max_value = 2000 #change this if your ESC's max value is different or leave it be
min_value = 1000  #change this if your ESC's min value is different or leave it be
speed = 1050

def wait_for_input(RPM):
    global df
    global setpoint
    inp = input()

   #  if inp == "":
   #    #   series1 = pd.DataFrame([[RPM, ADC_Value[2]*5.0/0x7fffff, ADC_Value[4]*5.0/0x7fffff]], columns=['Setpoint', 'Left Load Cell', 'Right Load Cell'])
   #    #   df = pd.concat([df,series1], ignore_index = True)
   #  elif inp == "n":
   #      setpoint += 1
   
    print('noted')

class reader:
   """
   A class to read speedometer pulses and calculate the RPM.
   """
   def __init__(self, pi, gpio, pulses_per_rev=1.0, weighting=0.0, min_RPM=5.0):
      """
      Instantiate with the Pi and gpio of the RPM signal
      to monitor.

      Optionally the number of pulses for a complete revolution
      may be specified.  It defaults to 1.

      Optionally a weighting may be specified.  This is a number
      between 0 and 1 and indicates how much the old reading
      affects the new reading.  It defaults to 0 which means
      the old reading has no effect.  This may be used to
      smooth the data.

      Optionally the minimum RPM may be specified.  This is a
      number between 1 and 1000.  It defaults to 5.  An RPM
      less than the minimum RPM returns 0.0.
      """
      self.pi = pi
      self.gpio = gpio
      self.pulses_per_rev = pulses_per_rev

      if min_RPM > 1000.0:
         min_RPM = 1000.0
      elif min_RPM < 1.0:
         min_RPM = 1.0

      self.min_RPM = min_RPM

      self._watchdog = 200 # Milliseconds.

      if weighting < 0.0:
         weighting = 0.0
      elif weighting > 0.99:
         weighting = 0.99

      self._new = 1.0 - weighting # Weighting for new reading.
      self._old = weighting       # Weighting for old reading.

      self._high_tick = None
      self._period = None

      pi.set_mode(gpio, pigpio.INPUT)

      self._cb = pi.callback(gpio, pigpio.RISING_EDGE, self._cbf)
      pi.set_watchdog(gpio, self._watchdog)

   def _cbf(self, gpio, level, tick):

      if level == 1: # Rising edge.

         if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)

            if self._period is not None:
               self._period = (self._old * self._period) + (self._new * t)
            else:
               self._period = t

         self._high_tick = tick

      elif level == 2: # Watchdog timeout.

         if self._period is not None:
            if self._period < 2000000000:
               self._period += (self._watchdog * 1000)

   def RPM(self):
      """
      Returns the RPM.
      """
      RPM = 0.0
      if self._period is not None:
         RPM = 60000000.0 / (self._period * self.pulses_per_rev)
         if RPM < self.min_RPM:
            RPM = 0.0

      return RPM

   def cancel(self):
      """
      Cancels the reader and releases resources.
      """
      self.pi.set_watchdog(self.gpio, 0) # cancel watchdog
      self._cb.cancel()

def arm(): #This is the arming procedure of an ESC 
    print ("Connect the battery and press Enter")
    inp = input("type Enter")    
    if inp == '':
        pi.set_servo_pulsewidth(ESC, 0)
        time.sleep(1)
        pi.set_servo_pulsewidth(ESC, max_value)
        time.sleep(1)
        pi.set_servo_pulsewidth(ESC, min_value)
        time.sleep(1)

def control():
   inp = input()

   if inp == "q":
      speed -= 100    # decrementing the speed like hell
      print ("speed = %d" % speed)
   elif inp == "e":    
      speed += 100    # incrementing the speed like hell
      print ("speed = %d" % speed)
   elif inp == "d":
      speed += 5     # incrementing the speed 
      print ("speed = %d" % speed)
   elif inp == "a":
      speed -= 10     # decrementing the speed
      print ("speed = %d" % speed)
   elif inp == "stop":
      stop()          #going for the stop function

def stop(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC, 0)
    pi.stop()

import time
import pigpio
#import main

arm()

try:
   RPM_GPIO = 24
   RUN_TIME = 100000.0
   SAMPLE_TIME = 0.5

   ADC = ADS1256.ADS1256()
   DAC = DAC8532.DAC8532()
   ADC.ADS1256_init()

   DAC.DAC8532_Out_Voltage(0x30, 3)
   DAC.DAC8532_Out_Voltage(0x34, 3)

   p = reader(pi, RPM_GPIO)


   print('here')
   # starting the motor now
   time.sleep(1)

   start = time.time()

   while (time.time() - start) < RUN_TIME:
      pi.set_servo_pulsewidth(ESC, speed)
      time.sleep(SAMPLE_TIME)

      RPM = p.RPM()
     
      #print("RPM={}".format(int(RPM+0.5)))
      ADC_Value = ADC.ADS1256_GetAll()
      print ("0 ADC = %lf"%(ADC_Value[0]*5.0/0x7fffff))
      print ("1 ADC = %lf"%(ADC_Value[1]*5.0/0x7fffff))
      print ("2 ADC = %lf"%(ADC_Value[2]*5.0/0x7fffff))
      print ("3 ADC = %lf"%(
arm()

try:
   RPM_GPIO = 24
   RUN_TIME = 100000.0
   SAMPLE_TIME = 0.5

   ADC = ADS1256.ADS1256()
   DAC = DAC8532.DAC8532()
   ADC.ADS1256_init()

   DAC.DAC8532_Out_Voltage(0x30, 3)
   DAC.DAC8532_Out_Voltage(0x34, 3)

   p = reader(pi, RPM_GPIO)

C_Value[3]*5.0/0x7fffff))
      print ("4 ADC = %lf"%(ADC_Value[4]*5.0/0x7fffff))
      print ("5 ADC = %lf"%(ADC_Value[5]*5.0/0x7fffff))
      print ("6 ADC = %lf"%(ADC_Value[6]*5.0/0x7fffff))
      print ("7 ADC = %lf"%(ADC_Value[7]*5.0/0x7fffff))

      temp = (ADC_Value[0]>
arm()

try:
   RPM_GPIO = 24
   RUN_TIME = 100000.0
   SAMPLE_TIME = 0.5

   ADC = ADS1256.ADS1256()
   DAC = DAC8532.DAC8532()
   ADC.ADS1256_init()

   DAC.DAC8532_Out_Voltage(0x30, 3)
   DAC.DAC8532_Out_Voltage(0x34, 3)

   p = reader(pi, RPM_GPIO)

ge(DAC8532.channel_A, temp)
      DAC.DAC8532_Out_Voltage(DAC8532.channel_B, 3.3 - temp)
      
      #threading.Thread(target = wait_for_input, args=((int(RPM+0.5)),), daemon=True).start()
      #threading.Thread(target = control, daemon=True).start()

except :
    p.cancel()
    pi.stop()
    GPIO.cleanup()
    print ("\r\nProgram end     ")
    df.to_csv('out.csv', sep = '\t', encoding = 'utf-8')
    exit()

# try:
#     ADC = ADS1256.ADS1256()
#     DAC = DAC8532.DAC8532()
#     ADC.ADS1256_init()

#     DAC.DAC8532_Out_Voltage(0x30, 3)
#     DAC.DAC8532_Out_Voltage(0x34, 3)

#     while(1):
#         ADC_Value = ADC.ADS1256_GetAll()
#         print ("0 ADC = %lf"%(ADC_Value[0]*5.0/0x7fffff))
#         print ("1 ADC = %lf"%(ADC_Value[1]*5.0/0x7fffff))
#         print ("2 ADC = %lf"%(ADC_Value[2]*5.0/0x7fffff))
#         print ("3 ADC = %lf"%(ADC_Value[3]*5.0/0x7fffff))
#         print ("4 ADC = %lf"%(ADC_Value[4]*5.0/0x7fffff))
#         print ("5 ADC = %lf"%(ADC_Value[5]*5.0/0x7fffff))
#         print ("6 ADC = %lf"%(ADC_Value[6]*5.0/0x7fffff))
#         print ("7 ADC = %lf"%(ADC_Value[7]*5.0/0x7fffff))

#         temp = (ADC_Value[0]>>7)*5.0/0xffff
#         print ("DAC :",temp)
#         print ("\33[10A")
#         DAC.DAC8532_Out_Voltage(DAC8532.channel_A, temp)
#         DAC.DAC8532_Out_Voltage(DAC8532.channel_B, 3.3 - temp)

#         threading.Thread(target = wait_for_input, daemon=True).start()

# except :
#     GPIO.cleanup()
#     print ("\r\nProgram end     ")
#     df.to_csv('out.csv', sep = '\t', encoding = 'utf-8')
#     exit()
