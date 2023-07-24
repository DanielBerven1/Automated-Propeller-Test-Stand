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

GPIO.setmode(GPIO.BOARD)

print ("Enter Propeller Type:")
prop_type = input() 

print ("Enter Prop Diameter:")
prop_diameter = input()

print ("cw or ccw:")
prop_direction = input()

df = [['RPM','Left Load Cell', 'Right Load Cell', 'Thrust']]
setpoint = 0

ESC=25  #Connect the ESC in this GPIO pin 
pi = pigpio.pi()


max_value = 2000 #change this if your ESC's max value is different or leave it be
min_value = 1000  #change this if your ESC's min value is different or leave it be
speed = 1055
pi.set_servo_pulsewidth(ESC, min_value) 

first_time = True

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
def arm(): #This is the arming procedure of an ESC 
    print ("Connect the battery and press Enter")
    inp = input("type Enter")    
   #  if inp == '':
   #      pi.set_servo_pulsewidth(ESC, 0)
   #      time.sleep(1)
   #      pi.set_servo_pulsewidth(ESC, max_value)
   #      time.sleep(1)
   #      pi.set_servo_pulsewidth(ESC, min_value)
   #      time.sleep(1)

def stop(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC, min_value)
    pi.stop()

# %% record measurements

def wait_for_input(RPM,voltages):
    global df
    
    df.append([int(RPM+0.5), voltages[2], voltages[4], voltages[5]])

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
CH_SEQUENCE = (POTI, LDR, EXT2, EXT3, EXT4, EXT7, POTI_INVERTED, SHORT_CIRCUIT)
################################################################################


def do_measurement():
    ### STEP 1: Initialise ADC object using default configuration:
    # (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
    # (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
    global first_time
    ads = ADS1256(pi=pigpio.pi(PI_HOST))

    ### STEP 2: Gain and offset self-calibration:
    ads.cal_self()
    RPM_GPIO = 24
   
    p = reader(pi, RPM_GPIO)
    time.sleep(1)

    speed = 1050

    pi.set_servo_pulsewidth(ESC, speed)

    start = time.time()
    
    if (first_time == False):
      while True:
         ### STEP 3: Get data:
         raw_channels = ads.read_sequence(CH_SEQUENCE)
         voltages     = [i * ads.v_per_digit for i in raw_channels]


         RPM = p.RPM()
      
         #print("RPM={}".format(int(RPM+0.5)))

         ### STEP 4: DONE. Have fun!
         #nice_output(raw_channels, voltages)
         wait_for_input(RPM,voltages)


         if (time.time() - start) >= 8.0:
            speed += 20
            pi.set_servo_pulsewidth(ESC, speed)
            start = time.time()
            print(int(RPM+0.5))
       
    elif (first_time == True):
      for i in range(20):
         raw_channels = ads.read_sequence(CH_SEQUENCE)
         voltages     = [i * ads.v_per_digit for i in raw_channels]


         RPM = p.RPM()
      
         #print("RPM={}".format(int(RPM+0.5)))

         ### STEP 4: DONE. Have fun!
         #nice_output(raw_channels, voltages)
         wait_for_input(RPM,voltages)

      first_time = False
      print('Done taring')


### END EXAMPLE ###


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

# %% main program
try:
   do_measurement()
   arm()
   time.sleep(1)

   do_measurement()

except (KeyboardInterrupt):
    print("\n"*8 + "User exit.\n")
    stop()
    #path = r'/home/ubuntu/src/waveshare_a2d_d2a_python/experiment'
    path = r'/home/ubuntu/src/waveshare_a2d_d2a_python/optimization_data'
    df = pd.DataFrame(df)
    df.columns = df.iloc[0]
    df = df.drop(0)
    
    #df.to_csv(path + f'/{prop_type}_{prop_diameter}_{prop_direction}_raw.csv', sep = '\t', encoding = 'utf-8')

    MLP_calibrated_fits = pd.read_csv('/home/ubuntu/src/waveshare_a2d_d2a_python/experiment/MLP_calibrated_fits.csv')

    Left_msrd_to_theo_slope = float(MLP_calibrated_fits.iloc[1,1])

    Left_msrd_to_theo_intcpt = float(MLP_calibrated_fits.iloc[1,2])

    Right_msrd_to_theo_slope = float(MLP_calibrated_fits.iloc[1,3])

    Right_msrd_to_theo_intcpt = float(MLP_calibrated_fits.iloc[1,4])

    Left_mV_to_g_slope = float(MLP_calibrated_fits.iloc[1,5])

    Right_mv_to_g_slope = float(MLP_calibrated_fits.iloc[1,6])

    Thrust_mV_to_g_slope = 0.0004365

    Left_mv_to_g_intcpt = df.iloc[1:31,1]
    Left_mv_to_g_intcpt = Left_mv_to_g_intcpt.to_numpy()
    Left_mv_to_g_intcpt = np.sum(Left_mv_to_g_intcpt)/len(Left_mv_to_g_intcpt)

    Right_mv_to_g_intcpt = df.iloc[1:31,2]
    Right_mv_to_g_intcpt = Right_mv_to_g_intcpt.to_numpy()
    Right_mv_to_g_intcpt = np.sum(Right_mv_to_g_intcpt)/len(Right_mv_to_g_intcpt)

    Thrust_mv_to_g_intcpt = df.iloc[1:31,3] 
    Thrust_mv_to_g_intcpt = Thrust_mv_to_g_intcpt.to_numpy()
    Thrust_mv_to_g_intcpt = np.sum(Thrust_mv_to_g_intcpt)/len(Thrust_mv_to_g_intcpt)
   
    df['Left Load Cell'] = df['Left Load Cell'].apply(lambda x: (x - Left_mv_to_g_intcpt)/Left_mV_to_g_slope )
    df['Right Load Cell'] = df['Right Load Cell'].apply(lambda x: (x-Right_mv_to_g_intcpt)/Right_mv_to_g_slope)

    df['Left Load Cell'] = df['Left Load Cell'].apply(lambda x: (x*Left_msrd_to_theo_slope + Left_msrd_to_theo_intcpt))
    df['Right Load Cell'] = df['Right Load Cell'].apply(lambda x: (x*Right_msrd_to_theo_slope+Right_msrd_to_theo_intcpt)) 

    df['Thrust'] = df['Thrust'].apply(lambda x: (x-Thrust_mv_to_g_intcpt)*0.001/Thrust_mV_to_g_slope)  

    Torque = 16.75*(df['Left Load Cell'] + df['Right Load Cell'])*0.0098*10**(-3)

    df['Torque'] = Torque

    df.to_csv(path + f'/{prop_type}_{prop_diameter}_{prop_direction}.csv', sep = ',', encoding = 'utf-8')

    sys.exit(0)

