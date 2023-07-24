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
from math import sqrt

import paho.mqtt.client as mqtt
import MQTT_RunTest_Definitions_Coaxial_VarPitch as mqtt_com

import Uncertainty_and_Plotting as up

path_Calib = r'/home/ubuntu/src/Co_Axial_Stand/Calibration'
path_Data = r'/home/ubuntu/src/Co_Axial_Stand/experiment/csv'

# %% Set up MQTT Communications

client = mqtt_com.setup_test_client()


left_min_voltage = 0
right_min_voltage = 0

msg_flag_DC_Front_position = 0
msg_flag_DC_Front_zero = 0

DC_Front_position = 0 
DC_Front_Zero_State = 0


def on_message(client, userdata, msg):
    # print(msg.topic + '' + str(msg.payload))


    if msg.topic == "DC_Front_Position":
        message = str(msg.payload.decode("utf-8"))

        global DC_Front_position
        DC_Front_position = float(message)

        global msg_flag_DC_Front_position
        msg_flag_DC_Front_position = 1  
        
    if msg.topic == "DC_Front_Zero":
        message = str(msg.payload.decode("utf-8"))

        global DC_Front_Zero_State
        DC_Front_Zero_State = float(message)

        global msg_flag_DC_Front_zero
        msg_flag_DC_Front_zero = 1


client.on_message = on_message 

def Set_DC_Front_Zero():

    try:    
        # publishing setpoint as string
            msg = str("")
            pubMsg = client.publish(
                topic='rpi/FrontSetDCZero',
                payload=msg.encode('utf-8'),
                qos=0,
            )
            pubMsg.wait_for_publish()
            print(pubMsg.is_published())
    except Exception as e:
        print(e)

def Set_DC_Front_Setpoint(setpoint):
    try:
        # publishing setpoint as string
        msg = str(setpoint)
        pubMsg = client.publish(
            topic='rpi/FrontPitchSetpoint',
            payload=msg.encode('utf-8'),
            qos=0,
        )
        pubMsg.wait_for_publish()
        print(pubMsg.is_published())
    except Exception as e:
        print(e)


# %% Setup utilities

# GPIO.setmode(GPIO.BOARD)
time.sleep(1)
print ("Enter Propeller Type:")
prop_type = input() 

print ("Enter Prop Diameter:")
prop_diameter = input()

print("max speed (RPM): ")
max_RPM = float(input())

df = [['Pitch', 'RPM','Left Load Cell', 'Right Load Cell', 'Thrust']]
setpoint = 0

ESC=25  #Connect the ESC in this GPIO pin 
pi = pigpio.pi()


max_value = 2000 #change this if your ESC's max value is different or leave it be
min_value = 1000  #change this if your ESC's min value is different or leave it be
speed = 1055
pi.set_servo_pulsewidth(ESC, min_value) 

first_time = True
pitch_counter = 0
new_pitch = False


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



def calibrate():   #This is the auto calibration procedure of a normal ESC
    pi.set_servo_pulsewidth(ESC, 0)
    print("Disconnect the battery and press Enter")
    inp = input()
    if inp == "":
        pi.set_servo_pulsewidth(ESC, max_value)
        print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = input("type enter")
        if inp == '':            
            pi.set_servo_pulsewidth(ESC, min_value)
            print ("Special tone")
            time.sleep(12)
            print ("Arming ESC now...")
            pi.set_servo_pulsewidth(ESC, min_value)

def stop(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC, min_value)
    # pi.stop()

# %% record measurements

def wait_for_input(pitch, RPM, voltages):
    global df
    
    df.append([pitch, int(RPM+0.5), voltages[0], voltages[1], voltages[2]])

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
CH_SEQUENCE = (EXT2, EXT3, EXT4)
################################################################################


def do_calibrate_measurement(ads, pitch_setpoint):
   # Preform taring measurements
   global left_min_voltage
   global right_min_voltage
   global first_time
   

   RPM_GPIO = 24
   
   p = reader(pi, RPM_GPIO)
   raw_channels = ads.read_sequence(CH_SEQUENCE)
   voltages     = [i * ads.v_per_digit for i in raw_channels]
   # axial load 50g below initial  (0.0005V/g)
   left_min_voltage = voltages[0] - 0.05 # Volts
   right_min_voltage = voltages[1] - 0.05 # Volts

   print("Taring...")
   for i in range(20):
      raw_channels = ads.read_sequence(CH_SEQUENCE)
      voltages     = [i * ads.v_per_digit for i in raw_channels]
      RPM = p.RPM()
      wait_for_input(pitch_setpoints[pitch_counter], RPM,voltages)
   first_time = False





def do_measurement(calculated_DC_setpoints, pitch_setpoints):
    ### STEP 1: Initialise ADC object using default configuration:
    # (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
    # (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
    global left_min_voltage
    global right_min_voltage

    global first_time
    global pitch_counter
    global new_pitch

    global msg_flag_DC_Front_position

    ads = ADS1256(pi=pigpio.pi(PI_HOST))

    ads.cal_self()
    RPM_GPIO = 24
   
    p = reader(pi, RPM_GPIO)
    time.sleep(1)

    speed = 1050

    pi.set_servo_pulsewidth(ESC, speed)

    start = time.time()

    # If taring process has been preformed
    if (first_time == False):
      # Testing loop
      while True:
         
         # Grab channel readings
         raw_channels = ads.read_sequence(CH_SEQUENCE)
         voltages     = [i * ads.v_per_digit for i in raw_channels]

         if voltages[0] < left_min_voltage or voltages[1] < right_min_voltage:
            stop()
            pi.stop()
            print("MLP sensors loosened, Torque data unreliable.")
            time.sleep(3)
            setpoint = 0
            Set_DC_Front_Setpoint(setpoint)
            # Format CSV and exit script
            while msg_flag_DC_Front_position == 0:
               print('waiting for pitch adjustment', end='\r')
            msg_flag_DC_Front_position = 0

            pi.set_servo_pulsewidth(ESC, 0)
            format_save_csv(pitch_setpoints)



         # Grab RPM reading
         RPM = p.RPM()

         # Append Data to CSV
         wait_for_input(pitch_setpoints[pitch_counter], RPM,voltages)


         # FOR INITIAL DEBUGGING  WITH NO MOTOR POWER
         ################################################
         
         # if speed > 1110: 
         #    # Stop the motor via ESC
         #    stop()

         #    # Increment the pitch counter
         #    pitch_counter +=1

         #    # Set flag for new pitch value setpoint
         #    new_pitch = True
         ################################################

         # If RPM has exceeded maximum RPM set by user
         if RPM > max_RPM and new_pitch == False:

            # Stop the motor via ESC
            stop()

            time.sleep(5)
            # Increment the pitch counter
            pitch_counter +=1

            # Set flag for new pitch value setpoint
            new_pitch = True

         # If pitch counter exceeds the number of setpoints set by user
         if pitch_counter == len(pitch_setpoints):
            stop()
            pi.set_servo_pulsewidth(ESC, 0)
            pi.stop()
            time.sleep(3)
            setpoint = 0
            Set_DC_Front_Setpoint(0)
            # Format CSV and exit script
            while msg_flag_DC_Front_position == 0:
               print('waiting for pitch adjustment', end='\r')
            msg_flag_DC_Front_position = 0

            format_save_csv(pitch_setpoints)

         # If new pitch value has been set
         if new_pitch == True:

            # Grab next DC motor setpoint
            DC_setpoint = calculated_DC_setpoints[pitch_counter]
            print(f' Pitch setpoint: {pitch_setpoints[pitch_counter]}')
            Set_DC_Front_Setpoint(DC_setpoint)

            while msg_flag_DC_Front_position == 0:
               print('waiting for pitch adjustment', end='\r')
            msg_flag_DC_Front_position = 0

            print('\n')
            time.sleep(7.5)
            print('message recieved')

            do_calibrate_measurement(ads, pitch_setpoints[pitch_counter])



            # Set ESC pulse signal to minimum ESC value
            speed = 1050
            pi.set_servo_pulsewidth(ESC, speed)
            new_pitch = False
            # Reset time 
            start = time.time()

         
         
         # if 8 seconds have passed for enough measurements
         if (time.time() - start) >= 8.0:

            # Increment pulse signal to ESC
            speed += 30
            pi.set_servo_pulsewidth(ESC, speed)

            # Reset time
            start = time.time()
            # print(int(RPM+0.5))
            print(RPM)

         

    # Initial calibration process upon script startup
    elif (first_time == True):
      # Set the DC setpoint to the first pitch value
      DC_setpoint = calculated_DC_setpoints[pitch_counter]
      # Publish the current DC setpoint to ESP32
      Set_DC_Front_Setpoint(DC_setpoint)

      while msg_flag_DC_Front_position == 0:
               print('waiting for pitch adjustment', end='\r')
      msg_flag_DC_Front_position = 0

      print('\n')
      time.sleep(7.5)
      print('message recieved')

      do_calibrate_measurement(ads, pitch_setpoints[pitch_counter])

      


### END EXAMPLE ###

def RPM_test():
   RPM_GPIO = 24
   p = reader(pi, RPM_GPIO)
   
   RPM = p.RPM()

   while RPM == 0:
      print('No RPM reading')
      time.sleep(1)
      RPM = p.RPM()

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

def format_save_csv(pitch_setpoints):
   pd.options.mode.chained_assignment = None
   global df

   print("\n"*8 + "Max Speed Reached.\n")

   df = pd.DataFrame(df)
   df.columns = df.iloc[0]
   
   
   df.to_csv(path_Data + f'/{prop_type}_{prop_diameter}_raw_Stand1.csv', encoding = 'utf-8')

   MLP_calibrated_fits = pd.read_csv(path_Calib + f'/Stand1_MLP_calibrated_fits.csv')
   Torque_calibrated_fits = pd.read_csv(path_Calib + f'/Stand1_Torque_Force_Calibrated_Fits.csv')
   Thrust_calibrated_fits = pd.read_csv(path_Calib + f'/Stand1_Thrust_Calibrated_Fits.csv')

   Left_msrd_to_theo_slope = float(Torque_calibrated_fits.iloc[1,1])

   Left_msrd_to_theo_intcpt = float(Torque_calibrated_fits.iloc[1,2])

   Right_msrd_to_theo_slope = float(Torque_calibrated_fits.iloc[1,3])

   Right_msrd_to_theo_intcpt = float(Torque_calibrated_fits.iloc[1,4])

   Left_mV_to_g_slope = float(MLP_calibrated_fits.iloc[1,1])

   Right_mv_to_g_slope = float(MLP_calibrated_fits.iloc[1,2])

   Thrust_mV_to_g_slope = float(Thrust_calibrated_fits.iloc[1,1])

   df2 = pd.DataFrame()

   for i in range(len(pitch_setpoints)):

      pitch_subset = df.loc[df['Pitch'] == pitch_setpoints[i]]

      Left_mv_to_g_intcpt = pitch_subset.iloc[1:21,2]
      Left_mv_to_g_intcpt = Left_mv_to_g_intcpt.to_numpy()
      Left_mv_to_g_intcpt = np.sum(Left_mv_to_g_intcpt)/len(Left_mv_to_g_intcpt)

      Right_mv_to_g_intcpt = pitch_subset.iloc[1:21,3]
      Right_mv_to_g_intcpt = Right_mv_to_g_intcpt.to_numpy()
      Right_mv_to_g_intcpt = np.sum(Right_mv_to_g_intcpt)/len(Right_mv_to_g_intcpt)

      Thrust_mv_to_g_intcpt = pitch_subset.iloc[1:21,4] 
      Thrust_mv_to_g_intcpt = Thrust_mv_to_g_intcpt.to_numpy()
      Thrust_mv_to_g_intcpt = np.sum(Thrust_mv_to_g_intcpt)/len(Thrust_mv_to_g_intcpt)

      print(Left_mv_to_g_intcpt)
      print(Right_mv_to_g_intcpt)
      print(Thrust_mv_to_g_intcpt)

      pitch_subset['Left Load Cell (g)'] = pitch_subset['Left Load Cell'].apply(lambda x: (x - Left_mv_to_g_intcpt)*Left_mV_to_g_slope)
      pitch_subset['Right Load Cell (g)'] = pitch_subset['Right Load Cell'].apply(lambda x: (x-Right_mv_to_g_intcpt)*Right_mv_to_g_slope)

      pitch_subset['Left Load Cell (g)'] = pitch_subset['Left Load Cell (g)'].apply(lambda x: (x*Left_msrd_to_theo_slope + Left_msrd_to_theo_intcpt))
      pitch_subset['Right Load Cell (g)'] = pitch_subset['Right Load Cell (g)'].apply(lambda x: (x*Right_msrd_to_theo_slope + Right_msrd_to_theo_intcpt)) 

      pitch_subset['Thrust (kgf)'] = pitch_subset['Thrust'].apply(lambda x: (x-Thrust_mv_to_g_intcpt)*0.001*Thrust_mV_to_g_slope)  

      Torque = 16.25*(pitch_subset['Left Load Cell (g)'] + pitch_subset['Right Load Cell (g)'])*0.0098*10**(-3)

      pitch_subset['Torque (Nm)'] = Torque

      df2 = pd.concat([df2, pitch_subset])

   df2.to_csv(path_Data + f'/{prop_type}_{prop_diameter}_VP_Stand1.csv', sep = ',', encoding = 'utf-8')
   pd.options.mode.chained_assignment = 'warn'

   

   sys.exit(0)

# %% main program
try:

   print('Ensure RPM can be read')
   RPM_test()

   print('First time running ESC? (y) or (n): ')
   inp = input()
   if inp == 'y': 
      print("Run calibration script")
      sys.exit(0)
      inp = 'n'

   if inp == 'n': 

      print("Zeroing VP System, press Enter")

      inp = input()
      Set_DC_Front_Zero()

      time.sleep(5)


      setpoints = np.arange(0, -9000-500, -500)

      for i in range(len(setpoints)):
         if DC_Front_Zero_State == 1:
            msg_flag_DC_Front_position = 0
            break

         Set_DC_Front_Setpoint(setpoints[i])

         while msg_flag_DC_Front_position == 0:
            print('waiting for inputs', end = '\r')
            
         print("\n")    
         time.sleep(2)
         msg_flag_DC_Front_position = 0

         

      print("DC Motor Zero'd")
      
      pitch_calibration_fits = pd.read_csv(path_Calib + f'/Pitch_Calibration_F.csv')
      pitch_calibration_fits = pitch_calibration_fits.to_numpy()
      pitch_calibration_fits = pitch_calibration_fits[1,:].astype(np.float64)
      pitch_fit_a = pitch_calibration_fits[1]
      pitch_fit_b = pitch_calibration_fits[2]
      pitch_fit_c = pitch_calibration_fits[3]

      def Pitch_to_DCPosition(y, a, b, c):
         return (-b + sqrt(b**2 - (4*a*(c-y))))/(2*a)




      pitch_setpoints = np.array([])

      n = int(input("Enter the number of pitch angles to test:    "))

      for i in range(n):
         v = float(input(f"Pitch angle {i+1}/{n}: "))
         pitch_setpoints = np.append(pitch_setpoints, v)

      print("Press Enter if these pitches are correct, else type 'quit'")
      print(pitch_setpoints)
      inp = input()
      if input == "quit":
         sys.exit(0)

      calculated_DC_setpoints = np.zeros_like(pitch_setpoints, dtype=float)

      for i in range(len(pitch_setpoints)):
         calculated_DC_setpoints[i] = Pitch_to_DCPosition(pitch_setpoints[i], pitch_fit_a, pitch_fit_b,
         pitch_fit_c)


      print(f"Current DC Position is:  {DC_Front_position}")

      do_measurement(calculated_DC_setpoints, pitch_setpoints)
      print('Power motor and press enter to begin test')
      inp = input()
      if inp == '':
         time.sleep(1)
         do_measurement(calculated_DC_setpoints, pitch_setpoints)

      

except (KeyboardInterrupt):


   print("\n"*8 + "User exit.\n")
   stop()
   pi.set_servo_pulsewidth(ESC, 0)
   pi.stop()
   format_save_csv(pitch_setpoints)



