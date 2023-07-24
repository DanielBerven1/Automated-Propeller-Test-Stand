#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import sys
import os
from ADS1256_definitions import *
from ADS1256_PiGPIO import ADS1256
import pandas as pd
import threading
import pigpio
import numpy as np
import RPi.GPIO as GPIO

# %% Setup utilities

# GPIO.setmode(GPIO.BOARD)

print ("Enter Propeller Type:")
prop_type = input() 

print ("Enter Prop Diameter:")
prop_diameter = input()


print("max speed (RPM): ")
max_RPM = float(input())

df = [['increment', 'ESC_1', 'ESC_2', 'RPM_1','Left Load Cell 1', 'Right Load Cell 1', 'Thrust 1', 'RPM_2','Left Load Cell 2', 'Right Load Cell 2', 'Thrust 2']]
setpoint = 0

ESC_1=25
ESC_2=20  #Connect the ESC in this GPIO pin 
pi = pigpio.pi()


max_value = 2000 #change this if your ESC's max value is different or leave it be
min_value = 1000  #change this if your ESC's min value is different or leave it be
speed = 1055
pi.set_servo_pulsewidth(ESC_1, min_value) 
pi.set_servo_pulsewidth(ESC_2, min_value) 

first_time = True
RPM_last = float(0)
skip_speed = False
skip_counter = 0

increment_speed = 0
increment = 0

# %% RPM reader class section

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

      self.prev_pin_val = 0

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

# %% Motor Control section



def calibrate_1():   #This is the auto calibration procedure of a normal ESC
    pi.set_servo_pulsewidth(ESC_1, 0)
    print("Disconnect the battery and press Enter")
    inp = input()
    if inp == "":
        pi.set_servo_pulsewidth(ESC_1, max_value)
        print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = input("type enter")
        if inp == '':            
            pi.set_servo_pulsewidth(ESC_1, min_value)
            print ("Special tone")
            time.sleep(12)
            print ("Arming ESC now...")
            pi.set_servo_pulsewidth(ESC_1, min_value)

def stop_1(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC_1, min_value)


def calibrate_2():   #This is the auto calibration procedure of a normal ESC
    pi.set_servo_pulsewidth(ESC_2, 0)
    print("Disconnect the battery and press Enter")
    inp = input()
    if inp == "":
        pi.set_servo_pulsewidth(ESC_2, max_value)
        print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = input("type enter")
        if inp == '':            
            pi.set_servo_pulsewidth(ESC_2, min_value)
            print ("Special tone")
            time.sleep(12)
            print ("Arming ESC now...")
            pi.set_servo_pulsewidth(ESC_2, min_value)

def stop_2(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC_2, min_value)

# %% record measurements

def wait_for_input(increment, Speed_1, Speed_2, RPM_1,RPM_2,voltages):
    global df
    
    df.append([increment, Speed_1, Speed_2, int(RPM_1+0.5), voltages[0], voltages[1], voltages[2], 
      int(RPM_2+0.5), voltages[3], voltages[4], voltages[5]])

    print('noted')

# %% waveshare section

# Change this to the local DNS name of your Pi (often raspberrypi.local, if you have changed it) or
# make it blank to connect to localhost.
PI_HOST = ''

# if not os.path.exists("/dev/spidev0.1"):
#     raise IOError("Error: No SPI device. Check settings in /boot/config.txt")

### START EXAMPLE ###
################################################################################
###  STEP 0: CONFIGURE CHANNELS AND USE DEFAULT OPTIONS FROM CONFIG FILE: ###
#
# For channel code values (bitmask) definitions, see ADS1256_definitions.py.
# The values representing the negative and positive input pins connected to
# the ADS1256 hardware multiplexer must be bitwise OR-ed to form eight-bit
# values, which will later be sent to the ADS1256 MUX register. The register
# can be explicitly read and set via ADS1256.mux property, but here we define
# a list of differential channels to be input to the ADS1256.read_sequence()
# method which reads all of them one after another.
#
# ==> Each channel in this context represents a differential pair of physical
# input pins of the ADS1256 input multiplexer.
#
# ==> For single-ended measurements, simply select AINCOM as the negative input.
#
# AINCOM does not have to be connected to AGND (0V), but it is if the jumper
# on the Waveshare board is set.
#
# Input pin for the potentiometer on the Waveshare Precision ADC board:
POTI = POS_AIN0|NEG_AINCOM
# Light dependant resistor of the same board:
LDR  = POS_AIN1|NEG_AINCOM
# The other external input screw terminals of the Waveshare board:
EXT2, EXT3, EXT4 = POS_AIN2|NEG_AINCOM, POS_AIN3|NEG_AINCOM, POS_AIN4|NEG_AINCOM
EXT5, EXT6, EXT7 = POS_AIN5|NEG_AINCOM, POS_AIN6|NEG_AINCOM, POS_AIN7|NEG_AINCOM

# You can connect any pin as well to the positive as to the negative ADC input.
# The following reads the voltage of the potentiometer with negative polarity.
# The ADC reading should be identical to that of the POTI channel, but negative.
POTI_INVERTED = POS_AINCOM|NEG_AIN0

# For fun, connect both ADC inputs to the same physical input pin.
# The ADC should always read a value close to zero for this.
SHORT_CIRCUIT = POS_AIN0|NEG_AIN0

# Specify here an arbitrary length list (tuple) of arbitrary input channel pair
# eight-bit code values to scan sequentially from index 0 to last.
# Eight channels fit on the screen nicely for this example..
CH_SEQUENCE = (EXT2, EXT3, EXT4, EXT5, EXT6, EXT7)
################################################################################

def do_calibrate_measurement(ads, increment):
   # Preform taring measurements
   global left_min_voltage
   global right_min_voltage
   global first_time

   RPM_GPIO_1 = 24
   RPM_GPIO_2 = 16 

   p_1 = reader(pi, RPM_GPIO_1)
   p_2 = reader(pi, RPM_GPIO_2)

   raw_channels = ads.read_sequence(CH_SEQUENCE)
   voltages     = [i * ads.v_per_digit for i in raw_channels]
   # axial load 50g below initial  (0.0005V/g)
   left_min_voltage = voltages[0] - 0.025 # Volts
   right_min_voltage = voltages[1] - 0.025 # Volts

   print("Taring...")
   for i in range(20):
      raw_channels = ads.read_sequence(CH_SEQUENCE)
      voltages     = [i * ads.v_per_digit for i in raw_channels]
      RPM_1 = p_1.RPM()
      RPM_2 = p_2.RPM()
      wait_for_input(increment, min_value, min_value, RPM_1, RPM_2, voltages)
   first_time = False

def do_measurement():
    ### STEP 1: Initialise ADC object using default configuration:
    # (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
    # (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
    global first_time
    global RPM_last
    global skip_counter 
    global skip_speed
    global increment_speed
    global increment
    ads = ADS1256(pi=pigpio.pi(PI_HOST))

    ### STEP 2: Gain and offset self-calibration:
    ads.cal_self()
    RPM_1_GPIO = 24
    RPM_2_GPIO = 16
   
    p_1 = reader(pi, RPM_1_GPIO)
    p_2 = reader(pi, RPM_2_GPIO)
    time.sleep(1)

    speed = 1050
    speed_1 = 1160
    speed_2 = 1160
    

    pi.set_servo_pulsewidth(ESC_1, speed)
    pi.set_servo_pulsewidth(ESC_2, speed)

    start = time.time()
    
    if (first_time == False):
      pi.set_servo_pulsewidth(ESC_1,speed_1)
      pi.set_servo_pulsewidth(ESC_2, speed_2)
      
      while True:
         ### STEP 3: Get data:

         


         raw_channels = ads.read_sequence(CH_SEQUENCE)
         voltages     = [i * ads.v_per_digit for i in raw_channels]


         RPM_1 = p_1.RPM()
         RPM_2 = p_2.RPM()

         # if RPM_last != 0 and abs(RPM-RPM_last) > 1000:
         #    print('here1')
         #    skip_speed = True
         #    skip_counter += 1
         # 
         
         if RPM_1 > max_RPM and abs(RPM_1 - RPM_1_last) < 500:
            stop_1()
            stop_2()
            increment_speed = 1
            increment += 1

         
         if RPM_2 > max_RPM and abs(RPM_2 - RPM_2_last) < 500:
            stop_1()
            stop_2()
            format_save_csv()
         
         # if skip_counter == 2:
         #    stop()
         #    format_save_csv

            
      
         #print("RPM={}".format(int(RPM+0.5)))

         ### STEP 4: DONE. Have fun!
         #nice_output(raw_channels, voltages)

         

         wait_for_input(increment, speed_1, speed_2, RPM_1, RPM_2, voltages)


         # FOR INITIAL DEBUGGING  WITH NO MOTOR POWER
         ################################################
         
         # if speed_1 > 1200: 
         #    # Stop the motor via ESC
         #    # Set flag for new pitch value setpoint
         #    increment_speed = True

         # if speed_2 > 1200:
         #    stop_1()
         #    stop_2()
         #    format_save_csv()

         ################################################

         # if skip_speed == True:
         #    print('here2')
         #    speed += 20
         #    print(speed)
         #    pi.set_servo_pulsewidth(ESC, speed)
         #    print(int(RPM+0.5))
         #    start = time.time()
         #    skip_speed == False

         if (time.time() - start) >= 8.0 and increment_speed == 0:
            speed_1 += 30
            pi.set_servo_pulsewidth(ESC_1, speed_1)
            start = time.time()
            print(int(RPM_1+0.5))
            print(int(RPM_2+0.5))
            print(f'ESC 1:    {speed_1}')
            print(f'ESC 2:    {speed_2}')
         
         if increment_speed == 1:
            time.sleep(7)
            do_calibrate_measurement(ads, increment)
            time.sleep(3)
            speed_2 += 30
            speed_1 = 1160
            pi.set_servo_pulsewidth(ESC_1, speed_1)
            pi.set_servo_pulsewidth(ESC_2, speed_2)
            start = time.time()
            print(int(RPM_1+0.5))
            print(int(RPM_2+0.5))
            increment_speed=0

         
         
         RPM_1_last = RPM_1
         RPM_2_last = RPM_2
       
    elif (first_time == True):

      do_calibrate_measurement(ads, increment)



### END EXAMPLE ###

def RPM_test():
   RPM_1_GPIO = 24
   RPM_2_GPIO = 16
   p_1 = reader(pi, RPM_1_GPIO)
   p_2 = reader(pi, RPM_2_GPIO)
   
   RPM_1 = p_1.RPM()
   RPM_2 = p_2.RPM()

   while RPM_1 == 0:
      print('No RPM 1 reading')
      time.sleep(1)
      RPM_1 = p_1.RPM()

   while RPM_2 == 0:
      print('No RPM 2 reading')
      time.sleep(1)
      RPM_2 = p_2.RPM()

   print('RPM sensor test successful')
   print('Close Enclosure')
   


#############################################################################
# Format nice looking text output:
def nice_output(digits, volts):
    sys.stdout.write(
          "\0337" # Store cursor position
        +
"""
These are the raw sample values for the channels:
Poti_CH0,  LDR_CH1,     AIN2,     AIN3,     AIN4,     AIN7, Poti NEG, Short 0V
"""
        + ", ".join(["{: 8d}".format(i) for i in digits])
        +
"""

These are the sample values converted to voltage in V for the channels:
Poti_CH0,  LDR_CH1,     AIN2,     AIN3,     AIN4,     AIN7, Poti NEG, Short 0V
"""
        + ", ".join(["{: 8.3f}".format(i) for i in volts])
        + "\n\033[J\0338" # Restore cursor position etc.
    )

def format_save_csv():
    pd.options.mode.chained_assignment = None
    global df
    print("\n"*8 + "Max Speed Reached.\n")
    path_Calib = r'/home/ubuntu/src/Co_Axial_Stand/Calibration'
    path_Data = r'/home/ubuntu/src/Co_Axial_Stand/experiment'
    df = pd.DataFrame(df)
    df.columns = df.iloc[0]
    df = df.drop(0)
    
    increments = np.arange(0, df['increment'].max()+1, 1)
    #df.to_csv(path + f'/{prop_type}_{prop_diameter}_{prop_direction}_raw.csv', sep = '\t', encoding = 'utf-8')

    MLP_calibrated_fits_1 = pd.read_csv(path_Calib + f'/Stand1_MLP_calibrated_fits.csv')
    Torque_calibrated_fits_1 = pd.read_csv(path_Calib + f'/Stand1_Torque_Force_Calibrated_Fits.csv')
    Thrust_calibrated_fits_1 = pd.read_csv(path_Calib + f'/Stand1_Thrust_Calibrated_Fits.csv')

    MLP_calibrated_fits_2 = pd.read_csv(path_Calib + f'/Stand2_MLP_calibrated_fits.csv')
    Torque_calibrated_fits_2 = pd.read_csv(path_Calib + f'/Stand2_Torque_Force_Calibrated_Fits.csv')
    Thrust_calibrated_fits_2 = pd.read_csv(path_Calib + f'/Stand2_Thrust_Calibrated_Fits.csv')

    ###################################

    Left_msrd_to_theo_slope_1 = float(Torque_calibrated_fits_1.iloc[1,1])

    Left_msrd_to_theo_intcpt_1 = float(Torque_calibrated_fits_1.iloc[1,2])

    Right_msrd_to_theo_slope_1 = float(Torque_calibrated_fits_1.iloc[1,3])

    Right_msrd_to_theo_intcpt_1 = float(Torque_calibrated_fits_1.iloc[1,4])

    Left_mV_to_g_slope_1 = float(MLP_calibrated_fits_1.iloc[1,1])

    Right_mv_to_g_slope_1 = float(MLP_calibrated_fits_1.iloc[1,2])

    Thrust_mV_to_g_slope_1 = float(Thrust_calibrated_fits_1.iloc[1,1])


    ###################################################33

    Left_msrd_to_theo_slope_2 = float(Torque_calibrated_fits_2.iloc[1,1])

    Left_msrd_to_theo_intcpt_2 = float(Torque_calibrated_fits_2.iloc[1,2])

    Right_msrd_to_theo_slope_2 = float(Torque_calibrated_fits_2.iloc[1,3])

    Right_msrd_to_theo_intcpt_2 = float(Torque_calibrated_fits_2.iloc[1,4])

    Left_mV_to_g_slope_2 = float(MLP_calibrated_fits_2.iloc[1,1])

    Right_mv_to_g_slope_2 = float(MLP_calibrated_fits_2.iloc[1,2])

    Thrust_mV_to_g_slope_2 = float(Thrust_calibrated_fits_2.iloc[1,1])
    
    df2 = pd.DataFrame()

    for i in range(len(increments)):

      ESC_2_speed_subset = df.loc[df['increment'] == increments[i]]
      
      Left_mv_to_g_intcpt_1 = ESC_2_speed_subset.iloc[1:31,4]
      Left_mv_to_g_intcpt_1 = Left_mv_to_g_intcpt_1.to_numpy()
      Left_mv_to_g_intcpt_1 = np.sum(Left_mv_to_g_intcpt_1)/len(Left_mv_to_g_intcpt_1)

      Right_mv_to_g_intcpt_1 = ESC_2_speed_subset.iloc[1:31,5]
      Right_mv_to_g_intcpt_1 = Right_mv_to_g_intcpt_1.to_numpy()
      Right_mv_to_g_intcpt_1 = np.sum(Right_mv_to_g_intcpt_1)/len(Right_mv_to_g_intcpt_1)

      Thrust_mv_to_g_intcpt_1 = ESC_2_speed_subset.iloc[1:31,6] 
      Thrust_mv_to_g_intcpt_1 = Thrust_mv_to_g_intcpt_1.to_numpy()
      Thrust_mv_to_g_intcpt_1 = np.sum(Thrust_mv_to_g_intcpt_1)/len(Thrust_mv_to_g_intcpt_1)
      
      
      Left_mv_to_g_intcpt_2 = ESC_2_speed_subset.iloc[1:31,8]
      Left_mv_to_g_intcpt_2 = Left_mv_to_g_intcpt_2.to_numpy()
      Left_mv_to_g_intcpt_2 = np.sum(Left_mv_to_g_intcpt_2)/len(Left_mv_to_g_intcpt_2)

      Right_mv_to_g_intcpt_2 = ESC_2_speed_subset.iloc[1:31,9]
      Right_mv_to_g_intcpt_2 = Right_mv_to_g_intcpt_2.to_numpy()
      Right_mv_to_g_intcpt_2 = np.sum(Right_mv_to_g_intcpt_2)/len(Right_mv_to_g_intcpt_2)

      Thrust_mv_to_g_intcpt_2 = ESC_2_speed_subset.iloc[1:31,10] 
      Thrust_mv_to_g_intcpt_2 = Thrust_mv_to_g_intcpt_2.to_numpy()
      Thrust_mv_to_g_intcpt_2 = np.sum(Thrust_mv_to_g_intcpt_2)/len(Thrust_mv_to_g_intcpt_2)
      
      ESC_2_speed_subset['Left Load Cell 1'] = ESC_2_speed_subset['Left Load Cell 1'].apply(lambda x: (x - Left_mv_to_g_intcpt_1)*Left_mV_to_g_slope_1)
      ESC_2_speed_subset['Right Load Cell 1'] = ESC_2_speed_subset['Right Load Cell 1'].apply(lambda x: (x-Right_mv_to_g_intcpt_1)*Right_mv_to_g_slope_1)

      ESC_2_speed_subset['Left Load Cell 1'] = ESC_2_speed_subset['Left Load Cell 1'].apply(lambda x: (x*Left_msrd_to_theo_slope_1 + Left_msrd_to_theo_intcpt_1))
      ESC_2_speed_subset['Right Load Cell 1'] = ESC_2_speed_subset['Right Load Cell 1'].apply(lambda x: (x*Right_msrd_to_theo_slope_1+Right_msrd_to_theo_intcpt_1)) 

      ESC_2_speed_subset['Thrust 1'] = ESC_2_speed_subset['Thrust 1'].apply(lambda x: (x-Thrust_mv_to_g_intcpt_1)*0.001*Thrust_mV_to_g_slope_1)  

      Torque_1 = 16.25*(ESC_2_speed_subset['Left Load Cell 1'] + ESC_2_speed_subset['Right Load Cell 1'])*0.0098*10**(-3)

      ESC_2_speed_subset['Torque 1'] = Torque_1

      ESC_2_speed_subset['Left Load Cell 2'] = ESC_2_speed_subset['Left Load Cell 2'].apply(lambda x: (x - Left_mv_to_g_intcpt_2)*Left_mV_to_g_slope_2)
      ESC_2_speed_subset['Right Load Cell 2'] = ESC_2_speed_subset['Right Load Cell 2'].apply(lambda x: (x-Right_mv_to_g_intcpt_2)*Right_mv_to_g_slope_2)

      ESC_2_speed_subset['Left Load Cell 2'] = ESC_2_speed_subset['Left Load Cell 2'].apply(lambda x: (x*Left_msrd_to_theo_slope_2 + Left_msrd_to_theo_intcpt_2))
      ESC_2_speed_subset['Right Load Cell 2'] = ESC_2_speed_subset['Right Load Cell 2'].apply(lambda x: (x*Right_msrd_to_theo_slope_2+Right_msrd_to_theo_intcpt_2)) 

      ESC_2_speed_subset['Thrust 2'] = ESC_2_speed_subset['Thrust 2'].apply(lambda x: (x-Thrust_mv_to_g_intcpt_2)*0.001*Thrust_mV_to_g_slope_2)  

      Torque_2 = 16.25*(ESC_2_speed_subset['Left Load Cell 2'] + ESC_2_speed_subset['Right Load Cell 2'])*0.0098*10**(-3)

      ESC_2_speed_subset['Torque 2'] = Torque_2

      df2 = pd.concat([df2, ESC_2_speed_subset])

    df2.to_csv(path_Data + f'/{prop_type}_{prop_diameter}_Coaxial.csv', sep = ',', encoding = 'utf-8')
    pd.options.mode.chained_assignment = 'warn'


# %% main program
try:

   print('Ensure RPM can be read')
   RPM_test()

   print('First time running ESC? (y) or (n): ')
   inp = input()
   if inp == 'y': 
      print("Run ESC Calibration file")
      sys.exit()


   if inp == 'n': 

      do_measurement()
      print('Power motor and press enter to begin test')
      inp = input()
      if inp == '':
         time.sleep(1)
         do_measurement()

except (KeyboardInterrupt):
    print("\n"*8 + "User exit.\n")
    stop_1()
    stop_2()
    pi.set_servo_pulsewidth(ESC_1, 0)
    pi.set_servo_pulsewidth(ESC_2, 0)
    pi.stop()
    format_save_csv()



