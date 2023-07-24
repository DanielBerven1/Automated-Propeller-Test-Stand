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


# %% Dynamixel 

#########
# dynamixel initialization
#########
import os
os.system ("sudo pigpiod") #Launching GPIO library
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * # Uses Dynamixel SDK library

MY_DXL = 'X_SERIES'

# Control table address
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_GOAL_VELOCITY          = 104
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10    # Dynamixel moving status threshold

index = 0
dxl_goal_position = 0         # Goal position
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Set operating mode to extended position control mode
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 11, 4)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode changed to extended position control mode.")

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

def dynamixel_readPosition():
   # Read present position
   if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
      dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
   else:
      dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
      if dxl_present_position > 0x7fffffff:
         dxl_present_position = dxl_present_position - 4294967294
   if dxl_comm_result != COMM_SUCCESS:
      print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
      print("%s" % packetHandler.getRxPacketError(dxl_error))

   print(f'Present position: {dxl_present_position}')

   return dxl_present_position

def dynamixel_writeGoalPosition(goal_position):
   dxl_goal_position = goal_position
   # Write goal position
   if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
      dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position)
   else:
      dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position)
   if dxl_comm_result != COMM_SUCCESS:
      print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
      print("%s" % packetHandler.getRxPacketError(dxl_error))

def dynamixel_writeGoalPosition_hold(goal_position):
   dxl_goal_position = goal_position
   # Write goal position
   if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
      dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position)
   else:
      dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position)
   if dxl_comm_result != COMM_SUCCESS:
      print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
      print("%s" % packetHandler.getRxPacketError(dxl_error))
   while 1:
      # Read present position
      if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
         dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
      else:
         dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
         if dxl_present_position > 0x7fffffff:
               dxl_present_position = dxl_present_position - 4294967294

      if dxl_comm_result != COMM_SUCCESS:
         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
      elif dxl_error != 0:
         print("%s" % packetHandler.getRxPacketError(dxl_error))

      print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position, dxl_present_position))

      if not abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break

# Function for clearing current dynamixel position
def dynamixel_clearMultiTurnInfo():
   # Clear Multi-Turn Information
   dxl_comm_result, dxl_error = packetHandler.clearMultiTurn(portHandler, DXL_ID)
   if dxl_comm_result != COMM_SUCCESS:
      print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
      print("%s" % packetHandler.getRxPacketError(dxl_error))


# Function for disabling dynamixel torque
def dynamixel_DisableTorque():
   xl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
   if dxl_comm_result != COMM_SUCCESS:
      print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
      print("%s" % packetHandler.getRxPacketError(dxl_error))   

# Function for closing dynamixel serial port communication 
def dynamixel_End():
   portHandler.closePort()


# Function for converting desired distance (mm) to # of dynamixel turns
# 4095 dynamixel turns per revolution, lead screw has 1mm pitch 1mm lead
def convert_Dist_to_Turns(dist, zero_pos):
    turns = np.zeros_like(dist, dtype = int)
    for i in range(len(dist)):
        turns[i] = dist[i]*4095 + zero_pos
    return turns

# Function for back converting current dynamixel turns to coaxial distance
# with respect to spacing at zero position (265mm)
def convert_Turns_to_Dist(turns, dynamixel_zero_turns):
   # dist = np.zeros_like(turns, dtype = int)
    zero_dist = 265 # mm
   #  for i in range(len(turns)):
   #      dist[i] = (turns[i]-dynamixel_zero_turns)/4095 + zero_dist
    return (turns-dynamixel_zero_turns)/4095 + zero_dist

# %% Set up MQTT Communications

# Setup mqtt node and subscribed topics
client = mqtt_com.setup_test_client()

# Front motor subscription variables
msg_flag_DC_Front_position = 0  # Flag for messege recieved from ESP32 after setpoint reached
DC_Front_position = 0           # Current motor position updated by ESP32 upon recieving message
msg_flag_DC_Front_zero = 0      # Flag for message recieved from ESP32 after IR sensor reads low
DC_Front_Zero_State = 0         # Current zero status updated by ESP32 upon recieving message

# Back motor subscription variables
msg_flag_DC_Back_position = 0   # Flag for messege recieved from ESP32 after setpoint reached
DC_Back_position = 0            # Current motor position updated by ESP32 upon recieving message
msg_flag_DC_Back_zero = 0       # Flag for message recieved from ESP32 after IR sensor reads low
DC_Back_Zero_State = 0          # Current zero status updated by ESP32 upon recieving message

# Dynamixel subscription variables
msg_flag_Dynamixel_zero = 0     # Flag for message recieved from ESP32 after back motor platform hits limit switch
Dynamixel_have_been_zerod = 0   # Current zero status updated by ESP32 upon recieving message

# Front motor pitch calibration variables
pitch_calibration_fits = pd.read_csv(path_Calib + f'/Pitch_Calibration_F.csv')
pitch_calibration_fits = pitch_calibration_fits.to_numpy()
pitch_calibration_fits = pitch_calibration_fits[1,:].astype(np.float64)
Front_pitch_fit_a = pitch_calibration_fits[1]
Front_pitch_fit_b = pitch_calibration_fits[2]
Front_pitch_fit_c = pitch_calibration_fits[3]

# Back motor pitch calibration variables
pitch_calibration_fits = pd.read_csv(path_Calib + f'/Pitch_Calibration_B.csv')
pitch_calibration_fits = pitch_calibration_fits.to_numpy()
pitch_calibration_fits = pitch_calibration_fits[1,:].astype(np.float64)
Back_pitch_fit_a = pitch_calibration_fits[1]
Back_pitch_fit_b = pitch_calibration_fits[2]
Back_pitch_fit_c = pitch_calibration_fits[3]

# Equation for calculating Front DC setpoint for specified pitch from calibration
def Front_Pitch_to_DCPosition(y, a, b, c):
    return (-b + sqrt(b**2 - (4*a*(c-y))))/(2*a)

# Equation for calculating Back DC setpoint for specified pitch from calibration
def Back_Pitch_to_DCPosition(y, a, b, c):
    return (-b - sqrt(b**2 - (4*a*(c-y))))/(2*a)


# Function for publishing front zero flag
def Set_DC_Front_Zero():

    # Publish flag for ESP32 to look for IR low
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

# Function for publishing back zero flag
def Set_DC_Back_Zero():

    # Publish flag for ESP32 to look for IR low
    try:    
        # publishing setpoint as string
            msg = str("")
            pubMsg = client.publish(
                topic='rpi/BackSetDCZero',
                payload=msg.encode('utf-8'),
                qos=0,
            )
            pubMsg.wait_for_publish()
            print(pubMsg.is_published())
    except Exception as e:
        print(e)

# Function for publishing Front DC setpoint
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

# Function for publishing Back DC setpoint
def Set_DC_Back_Setpoint(setpoint):
    try:
        # publishing setpoint as string
        msg = str(setpoint)
        pubMsg = client.publish(
            topic='rpi/BackPitchSetpoint',
            payload=msg.encode('utf-8'),
            qos=0,
        )
        pubMsg.wait_for_publish()
        print(pubMsg.is_published())
    except Exception as e:
        print(e)

# Function for publishing flag for getting current DC motor data
def Get_Data():
    try:
        # publishing setpoint as string
        msg = str(" ")
        pubMsg = client.publish(
            topic='rpi/GetData',
            payload=msg.encode('utf-8'),
            qos=0,
        )
        pubMsg.wait_for_publish()
        print(pubMsg.is_published())
    except Exception as e:
        print(e)  

# Function for publishing flag for resetting Dynamixel zero identifer procedure on ESP32
def Reset_Dynamixel_Zero():
    try:
         # publishing setpoint as string
         msg = str(' ')
         pubMsg = client.publish(
            topic='rpi/ResetDynamixelZero',
            payload=msg.encode('utf-8'),
            qos=0,
         )
         pubMsg.wait_for_publish()
         print(pubMsg.is_published())
    except Exception as e:
         print(e)

# Function for publishing flag for ESP32 to look for limit switch
def Set_Dynamixel_Zero_Flag():
    try:
         # publishing setpoint as string
         msg = str(' ')
         pubMsg = client.publish(
            topic='rpi/SetDynamixelZero',
            payload=msg.encode('utf-8'),
            qos=0,
         )
         pubMsg.wait_for_publish()
         print(pubMsg.is_published())
    except Exception as e:
         print(e)    

# Function for procedure of setting Front DC motor to zero
def Zero_Front_Motor():

    global msg_flag_DC_Front_position
    global DC_Front_position
    global DC_Front_Zero_State


    print("Zeroing Front VP System, press Enter")    
    input()

    # Get current position
    Get_Data()
    
    # Wait for message return
    while msg_flag_DC_Front_position == 0:
        print('waiting for inputs', end = '\r')

    print("\n")    
    time.sleep(2) # Allow for message data to be stored

    # Set motor postion slightly away from IR sensor 
    # whereever it may be
    Set_DC_Front_Setpoint(DC_Front_position + 720)
    
    # Wait for message return
    while msg_flag_DC_Front_position == 0:
        print('waiting for inputs', end = '\r')

    print("\n")    
    time.sleep(2)   # Allow for message data to be stored
    
    # Publish flag to start ESP32 zero proecdure
    Set_DC_Front_Zero() 
    time.sleep(5)

    # Array of setpoints to move twoards IR sensor
    setpoints = np.arange(DC_Front_position, -9000-500, -500)

    # Publish and iterate
    for i in range(len(setpoints)):

        # If zero message has been recieved
        if DC_Front_Zero_State == 1:
            msg_flag_DC_Front_position = 0
            break

        

        Set_DC_Front_Setpoint(setpoints[i])

        # Wait for setpoint 
        while msg_flag_DC_Front_position == 0:
            print('waiting for inputs', end = '\r')
            
        print("\n")    
        time.sleep(2)
        msg_flag_DC_Front_position = 0

        
    DC_Front_Zero_State = 0
    print("Front DC Motor Zero'd")
    
def Zero_Back_Motor():

    global msg_flag_DC_Back_position
    global DC_Back_position
    global DC_Back_Zero_State

    print("Zeroing Back VP System, press Enter")    
    input()

    Get_Data()

    while msg_flag_DC_Back_position == 0:
        print('waiting for inputs', end = '\r')

    print("\n")    
    time.sleep(2)

    Set_DC_Back_Setpoint(DC_Back_position + 720)
    
    
    while msg_flag_DC_Back_position == 0:
        print('waiting for inputs', end = '\r')

    print("\n")    
    time.sleep(2)
    
    Set_DC_Back_Zero()
    time.sleep(5)


    setpoints = np.arange(DC_Back_position, -9000-500, -500)

    for i in range(len(setpoints)):
        if DC_Back_Zero_State == 1:
            msg_flag_DC_Back_position = 0
            break

        

        Set_DC_Back_Setpoint(setpoints[i])

        while msg_flag_DC_Back_position == 0:
            print('waiting for inputs', end = '\r')
            
        print("\n")    
        time.sleep(2)
        msg_flag_DC_Back_position = 0

        
    DC_Back_Zero_State = 0
    print("Back DC Motor Zero'd")


def Zero_Dynamixel():
    
    global Dynamixel_have_been_zerod

    print('Zeroing dynamixel')   

    Reset_Dynamixel_Zero()
    Set_Dynamixel_Zero_Flag()
    
    print('Zeroing dynamixel')
    
    time.sleep(5)

    pos = dynamixel_readPosition()

    time.sleep(3)

    while True:
        
        if Dynamixel_have_been_zerod == 1:
            dxl_present_position = dynamixel_readPosition()
            dynamixel_writeGoalPosition(dxl_present_position)
            time.sleep(0.3)
            dynamixel_clearMultiTurnInfo()
            dxl_present_position = dynamixel_readPosition()
            dynamixel_writeGoalPosition(dxl_present_position)
            dxl_zero_position = dxl_goal_position
            break
        else:
            pos -= 4095
            dynamixel_writeGoalPosition(pos)
            time.sleep(5)

    print("Dynamixel zero'd")
    return dxl_zero_position

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

    if msg.topic == "DC_Back_Position":
        message = str(msg.payload.decode("utf-8"))

        global DC_Back_position
        DC_Back_position = float(message)


        global msg_flag_DC_Back_position
        msg_flag_DC_Back_position = 1  
        
    if msg.topic == "DC_Back_Zero":
        message = str(msg.payload.decode("utf-8"))

        global DC_Back_Zero_State
        DC_Back_Zero_State = float(message)

        global msg_flag_DC_Back_zero
        msg_flag_DC_Back_zero = 1  

    if msg.topic == "Zero_Switch":
        message = str(msg.payload.decode("utf-8"))

        global Dynamixel_have_been_zerod
        Dynamixel_have_been_zerod = float(message)

        global msg_flag_Dynamixel_zero
        msg_flag_Dynamixel_zero= 1  


client.on_message = on_message 


# %% Setup utilities

# GPIO.setmode(GPIO.BOARD)

print ("Enter Propeller Type:")
prop_type = input() 

print ("Enter Prop Diameter:")
prop_diameter = input()


print("max speed (RPM): ")
max_RPM = float(input())

df = [['Data Subset Num', 'Co_axial Dist', 'ESC_1', 'Pitch_1', 'ESC_2', 'Pitch_2', 'RPM_1','Left Load Cell 1', 'Right Load Cell 1', 
        'Thrust 1', 'RPM_2','Left Load Cell 2', 'Right Load Cell 2', 'Thrust 2']]
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

Front_new_speed = False
Back_new_speed = False
Front_speed_increment = 0
Back_speed_increment = 0

new_spacing = False
spacing_increment = 0

Front_pitch_counter = 0
Back_pitch_counter = 0
Front_new_pitch = False
Back_new_pitch = False


start_loop_a = False
start_loop_b = False
start_loop_c = False
start_loop_d = False
stop_flag = False

data_subset_increment = 0


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


def stop_1(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC_1, min_value)


def stop_2(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC_2, min_value)     

# %% record measurements

def wait_for_input(Data_Subset_num, Coax_dist, Speed_1, Pitch_1, Speed_2, Pitch_2, RPM_1,RPM_2,voltages):
    global df
    
    df.append([Data_Subset_num, Coax_dist, Speed_1, Pitch_1, Speed_2, Pitch_2,  int(RPM_1+0.5), voltages[0], voltages[1], voltages[2], 
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

def do_calibrate_measurement(ads, Coax_dist, speed_1, Front_pitch_setpoint, 
      speed_2, Back_pitch_setpoint):
   # Preform taring measurements
   global left_min_voltage
   global right_min_voltage
   global first_time
   global data_subset_increment

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
      wait_for_input(data_subset_increment, Coax_dist, speed_1, Front_pitch_setpoint, speed_2, Back_pitch_setpoint, RPM_1, RPM_2, voltages)
   first_time = False

def do_measurement(Front_calculated_DC_setpoints, Front_pitch_setpoints, Back_calculated_DC_setpoints,
                        Back_pitch_setpoints, dxl_setpoints):
    ### STEP 1: Initialise ADC object using default configuration:
    # (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
    # (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
    global first_time


    global DC_Front_position
    global msg_flag_DC_Front_position
    global DC_Back_position
    global msg_flag_DC_Back_position


    global Front_new_speed
    global Back_new_speed
    global Front_speed_increment
    global Back_speed_increment

    global new_spacing
    global spacing_increment
    global dxl_zero_position
    global coaxial_dist

    global Front_pitch_counter
    global Back_pitch_counter
    global Front_new_pitch
    global Back_new_pitch

    global start_loop_a
    global start_loop_b 
    global start_loop_c 
    global start_loop_d 
    global stop_flag 

    global data_subset_increment

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
    speed_2_temp = 1160
    

    pi.set_servo_pulsewidth(ESC_1, speed)
    pi.set_servo_pulsewidth(ESC_2, speed)

    start = time.time()
    
    if (first_time == False):
        pi.set_servo_pulsewidth(ESC_1,speed_1)
        pi.set_servo_pulsewidth(ESC_2, speed_2)
        
        while True:
         ### STEP 3: Get data:





            # %%  
            # Loop a
            while start_loop_a == True:

                print("In loop a")


                raw_channels = ads.read_sequence(CH_SEQUENCE)
                voltages     = [i * ads.v_per_digit for i in raw_channels]


                RPM_1 = p_1.RPM()
                RPM_2 = p_2.RPM()





                if RPM_1 > max_RPM and abs(RPM_1 - RPM_1_last) < 500:
                    stop_1()
                    stop_2()
                    Back_new_speed = True
                    Back_speed_increment += 1

                
                if RPM_2 > max_RPM and abs(RPM_2 - RPM_2_last) < 500:
                    stop_1()
                    stop_2()
                    start_loop_a = False
                    start_loop_b = True
                    break


                

                wait_for_input(data_subset_increment, coaxial_dist[spacing_increment], speed_1, Front_pitch_setpoints[Front_pitch_counter], 
                    speed_2, Back_pitch_setpoints[Back_pitch_counter], RPM_1, RPM_2, voltages)


                # FOR INITIAL DEBUGGING  WITH NO MOTOR POWER
                ################################################
                
                # if speed_1 > 1190: 
                #    # Stop the motor via ESC
                #    # Set flag for new pitch value setpoint
                #     stop_1()
                #     stop_2()
                #     Back_new_speed = True
                #     Back_speed_increment += 1

                # if speed_2 > 1190:
                #     stop_1()
                #     stop_2()
                #     start_loop_a = False
                #     start_loop_b = True
                #     break

                ################################################



                if (time.time() - start) >= 12.0 and Back_new_speed == False:
                    speed_1 += 40
                    pi.set_servo_pulsewidth(ESC_1, speed_1)
                    start = time.time()
                    print(int(RPM_1+0.5))
                    print(int(RPM_2+0.5))
                    print(f'ESC 1:    {speed_1}')
                    print(f'ESC 2:    {speed_2}')
                
                if Back_new_speed == True:
                    time.sleep(7)
                    data_subset_increment += 1
                    do_calibrate_measurement(ads, coaxial_dist[spacing_increment], min_value, Front_pitch_setpoints[Front_pitch_counter], 
                            min_value, Back_pitch_setpoints[Back_pitch_counter])
                    time.sleep(3)

                    
                    speed_1 = 1160

                    speed_2_temp += 50
                    speed_2 = 1160 
                    
                    while speed_2 != speed_2_temp:
                        print(speed_2)
                        pi.set_servo_pulsewidth(ESC_2, speed_2)
                        speed_2+=50
                        
                        time.sleep(3)

                    pi.set_servo_pulsewidth(ESC_2, speed_2)
                    pi.set_servo_pulsewidth(ESC_1, speed_1)
                    


                    print(int(RPM_1+0.5))
                    print(int(RPM_2+0.5))
                    print(f'ESC 1:    {speed_1}')
                    print(f'ESC 2:    {speed_2}')

                    Back_new_speed = False
                    start = time.time()

                RPM_1_last = RPM_1
                RPM_2_last = RPM_2
        
            if start_loop_b == True:

                print("In loop b")


                # Increment the pitch counter
                Front_pitch_counter +=1

                if Front_pitch_counter == len(Front_pitch_setpoints):

                    stop_1()
                    stop_2()
                    time.sleep(3)

                    Zero_Front_Motor()

                    #####################

                    Front_pitch_counter = 0

                    Front_DC_setpoint = Front_calculated_DC_setpoints[Front_pitch_counter]
                    
                    time.sleep(3)
                    Set_DC_Front_Setpoint(Front_DC_setpoint)

                    while msg_flag_DC_Front_position == 0:
                        print('waiting for pitch adjustment', end='\r')
                    msg_flag_DC_Front_position = 0
                    
                    print('\n')
                    time.sleep(7.5)
                    print('message recieved')


                    start_loop_b = False
                    start_loop_c = True



                else:

                    # Grab next DC motor setpoint
                    
                    Front_DC_setpoint = Front_calculated_DC_setpoints[Front_pitch_counter]
                    print(f' Front_Pitch setpoint: {Front_pitch_setpoints[Front_pitch_counter]}')
                    print(Front_DC_setpoint)
                    time.sleep(3)
                    Set_DC_Front_Setpoint(Front_DC_setpoint)

                    while msg_flag_DC_Front_position == 0:
                        print('waiting for pitch adjustment', end='\r')
                    msg_flag_DC_Front_position = 0

                    print('\n')
                    time.sleep(7.5)
                    print('message recieved')

                    data_subset_increment += 1
                    do_calibrate_measurement(ads, coaxial_dist[spacing_increment], min_value, Front_pitch_setpoints[Front_pitch_counter], 
                            min_value, Back_pitch_setpoints[Back_pitch_counter])
                    
                    time.sleep(3)

                    start_loop_a = True
                    start_loop_b = False

                    speed_1 = 1160
                    speed_2 = 1160
                    speed_2_temp = 1160

                    pi.set_servo_pulsewidth(ESC_1,speed_1)
                    pi.set_servo_pulsewidth(ESC_2, speed_2)
                    start = time.time()
            
            if start_loop_c == True: 

                print("In loop c")


                # Increment the pitch counter
                Back_pitch_counter +=1

                if Back_pitch_counter == len(Back_pitch_setpoints):

                    stop_1()
                    stop_2()
                    time.sleep(3)

                    Zero_Back_Motor()

                    #####################

                    Back_pitch_counter = 0

                    Back_DC_setpoint = Back_calculated_DC_setpoints[Back_pitch_counter]
                    time.sleep(3)
                    Set_DC_Back_Setpoint(Back_DC_setpoint)
                    
                    while msg_flag_DC_Back_position == 0:
                            print('waiting for pitch adjustment', end='\r')
                    msg_flag_DC_Back_position = 0
                    
                    print('\n')
                    time.sleep(7.5)
                    print('message recieved')

                    while msg_flag_DC_Back_position == 0:
                        print('waiting for pitch adjustment', end='\r')
                    msg_flag_DC_Back_position = 0

                    print('\n')
                    time.sleep(7.5)
                    print('message recieved')

                    start_loop_c = False
                    start_loop_d = True

                    #####################

                    




                else:

                    # Grab next DC motor setpoint
                    Back_DC_setpoint = Back_calculated_DC_setpoints[Back_pitch_counter]
                    time.sleep(3)
                    print(f' Back_Pitch setpoint: {Back_pitch_setpoints[Back_pitch_counter]}')
                    Set_DC_Back_Setpoint(Back_DC_setpoint)

                    while msg_flag_DC_Back_position == 0:
                        print('waiting for pitch adjustment', end='\r')
                    msg_flag_DC_Back_position = 0

                    print('\n')
                    time.sleep(7.5)
                    print('message recieved')

                    data_subset_increment += 1
                    do_calibrate_measurement(ads, coaxial_dist[spacing_increment], min_value, Front_pitch_setpoints[Front_pitch_counter], 
                        min_value, Back_pitch_setpoints[Back_pitch_counter])

                    time.sleep(3)
                    
                    start_loop_a = True
                    start_loop_c = False

                    speed_1 = 1160
                    speed_2 = 1160
                    speed_2_temp = 1160

                    pi.set_servo_pulsewidth(ESC_1,speed_1)
                    pi.set_servo_pulsewidth(ESC_2, speed_2)
                    start = time.time()
            
            if start_loop_d == True:
                
                print("In loop d")


                spacing_increment += 1

                if spacing_increment == len(spacing_setpoints):

                    stop_1()
                    stop_2()
                    time.sleep(3)

                    start_loop_d = False
                    stop_flag = True
                
                else: 
                    dynamixel_writeGoalPosition_hold(dxl_setpoints[spacing_increment])
                    time.sleep(5)
                    dxl_present_position = dynamixel_readPosition()
                    
                    coaxial_dist[spacing_increment] = convert_Turns_to_Dist(dxl_present_position, dxl_zero_position)      

                    data_subset_increment += 1
                    do_calibrate_measurement(ads, coaxial_dist[spacing_increment], min_value, Front_pitch_setpoints[Front_pitch_counter], 
                        min_value, Back_pitch_setpoints[Back_pitch_counter])

                    time.sleep(3)
                    
                    start_loop_a = True
                    start_loop_d = False

                    speed_1 = 1160
                    speed_2 = 1160
                    speed_2_temp = 1160

                    pi.set_servo_pulsewidth(ESC_1,speed_1)
                    pi.set_servo_pulsewidth(ESC_2, speed_2)
                    start = time.time()
            
            if stop_flag == True:

                print("Test done")

                pi.set_servo_pulsewidth(ESC_1, 0)
                pi.set_servo_pulsewidth(ESC_2, 0)

                format_save_csv()

                dynamixel_writeGoalPosition_hold(dxl_zero_position)

                Set_DC_Front_Setpoint(0)

                while msg_flag_DC_Front_position == 0:
                        print('waiting for pitch adjustment', end='\r')
                msg_flag_DC_Front_position = 0

                print('\n')
                time.sleep(7.5)
                print('message recieved')

                Set_DC_Back_Setpoint(0)
                
                while msg_flag_DC_Back_position == 0:
                    print('waiting for pitch adjustment', end='\r')
                msg_flag_DC_Back_position = 0

                print('\n')
                time.sleep(7.5)
                print('message recieved')


                break


                # format_save_csv(coaxial_dist
       
    elif (first_time == True):
      
        Front_DC_setpoint = Front_calculated_DC_setpoints[Front_pitch_counter]
        Back_DC_setpoint = Back_calculated_DC_setpoints[Back_pitch_counter]
        
        #####################

        Set_DC_Front_Setpoint(Front_DC_setpoint)

        while msg_flag_DC_Front_position == 0:
                print('waiting for pitch adjustment', end='\r')
        msg_flag_DC_Front_position = 0
        
        print('\n')
        time.sleep(7.5)
        print('message recieved')

        #####################

        Set_DC_Back_Setpoint(Back_DC_setpoint)

        while msg_flag_DC_Back_position == 0:
                print('waiting for pitch adjustment', end='\r')
        msg_flag_DC_Back_position = 0
        
        print('\n')
        time.sleep(7.5)
        print('message recieved')

        #####################

        dxl_goal_position = dxl_setpoints[spacing_increment]
        dynamixel_writeGoalPosition_hold(dxl_goal_position)
        time.sleep(5)
        dxl_present_position = dynamixel_readPosition()
        coaxial_dist[spacing_increment] = convert_Turns_to_Dist(dxl_present_position, dxl_zero_position)

        do_calibrate_measurement(ads, coaxial_dist[spacing_increment], min_value, Front_pitch_setpoints[Front_pitch_counter], 
        min_value, Back_pitch_setpoints[Back_pitch_counter])

        start_loop_a = True
        




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
    
    increments = np.arange(0, df['Data Subset Num'].max()+1, 1)
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

      data_subset = df.loc[df['Data Subset Num'] == increments[i]]
      
      Left_mv_to_g_intcpt_1 = data_subset.iloc[1:20,7]
      Left_mv_to_g_intcpt_1 = Left_mv_to_g_intcpt_1.to_numpy()
      Left_mv_to_g_intcpt_1 = np.sum(Left_mv_to_g_intcpt_1)/len(Left_mv_to_g_intcpt_1)

      Right_mv_to_g_intcpt_1 = data_subset.iloc[1:20,8]
      Right_mv_to_g_intcpt_1 = Right_mv_to_g_intcpt_1.to_numpy()
      Right_mv_to_g_intcpt_1 = np.sum(Right_mv_to_g_intcpt_1)/len(Right_mv_to_g_intcpt_1)

      Thrust_mv_to_g_intcpt_1 = data_subset.iloc[1:20,9] 
      Thrust_mv_to_g_intcpt_1 = Thrust_mv_to_g_intcpt_1.to_numpy()
      Thrust_mv_to_g_intcpt_1 = np.sum(Thrust_mv_to_g_intcpt_1)/len(Thrust_mv_to_g_intcpt_1)
      
      
      Left_mv_to_g_intcpt_2 = data_subset.iloc[1:20,11]
      Left_mv_to_g_intcpt_2 = Left_mv_to_g_intcpt_2.to_numpy()
      Left_mv_to_g_intcpt_2 = np.sum(Left_mv_to_g_intcpt_2)/len(Left_mv_to_g_intcpt_2)

      Right_mv_to_g_intcpt_2 = data_subset.iloc[1:20,12]
      Right_mv_to_g_intcpt_2 = Right_mv_to_g_intcpt_2.to_numpy()
      Right_mv_to_g_intcpt_2 = np.sum(Right_mv_to_g_intcpt_2)/len(Right_mv_to_g_intcpt_2)

      Thrust_mv_to_g_intcpt_2 = data_subset.iloc[1:20,13] 
      Thrust_mv_to_g_intcpt_2 = Thrust_mv_to_g_intcpt_2.to_numpy()
      Thrust_mv_to_g_intcpt_2 = np.sum(Thrust_mv_to_g_intcpt_2)/len(Thrust_mv_to_g_intcpt_2)
      
      data_subset['Left Load Cell 1'] = data_subset['Left Load Cell 1'].apply(lambda x: (x - Left_mv_to_g_intcpt_1)*Left_mV_to_g_slope_1)
      data_subset['Right Load Cell 1'] = data_subset['Right Load Cell 1'].apply(lambda x: (x-Right_mv_to_g_intcpt_1)*Right_mv_to_g_slope_1)

      data_subset['Left Load Cell 1 (g)'] = data_subset['Left Load Cell 1'].apply(lambda x: (x*Left_msrd_to_theo_slope_1 + Left_msrd_to_theo_intcpt_1))
      data_subset['Right Load Cell 1 (g)'] = data_subset['Right Load Cell 1'].apply(lambda x: (x*Right_msrd_to_theo_slope_1+Right_msrd_to_theo_intcpt_1)) 

      data_subset['Thrust 1 (kgf)'] = data_subset['Thrust 1'].apply(lambda x: (x-Thrust_mv_to_g_intcpt_1)*0.001*Thrust_mV_to_g_slope_1)  

      Torque_1 = 16.25*(data_subset['Left Load Cell 1 (g)'] + data_subset['Right Load Cell 1 (g)'])*0.0098*10**(-3)

      data_subset['Torque 1 (Nm)'] = Torque_1

      data_subset['Left Load Cell 2'] = data_subset['Left Load Cell 2'].apply(lambda x: (x - Left_mv_to_g_intcpt_2)*Left_mV_to_g_slope_2)
      data_subset['Right Load Cell 2'] = data_subset['Right Load Cell 2'].apply(lambda x: (x-Right_mv_to_g_intcpt_2)*Right_mv_to_g_slope_2)

      data_subset['Left Load Cell 2 (g)'] = data_subset['Left Load Cell 2'].apply(lambda x: (x*Left_msrd_to_theo_slope_2 + Left_msrd_to_theo_intcpt_2))
      data_subset['Right Load Cell 2 (g)'] = data_subset['Right Load Cell 2'].apply(lambda x: (x*Right_msrd_to_theo_slope_2+Right_msrd_to_theo_intcpt_2)) 

      data_subset['Thrust 2 (kgf)'] = data_subset['Thrust 2'].apply(lambda x: (x-Thrust_mv_to_g_intcpt_2)*0.001*Thrust_mV_to_g_slope_2)  

      Torque_2 = 16.25*(data_subset['Left Load Cell 2 (g)'] + data_subset['Right Load Cell 2 (g)'])*0.0098*10**(-3)

      data_subset['Torque 2 (Nm)'] = Torque_2

      df2 = pd.concat([df2, data_subset])

    df2.to_csv(path_Data + f'/{prop_type}_{prop_diameter}_Coaxial_VS_VP.csv', sep = ',', encoding = 'utf-8')
    pd.options.mode.chained_assignment = 'warn'


# %% main program
try:
   print('Test RPM? (y) or any')
   inp = input()

   if inp == 'y':
        print('Ensure RPM can be read')
        RPM_test()
    

   print('First time running ESC? (y) or (n): ')
   inp = input()
   if inp == 'y': 
      print("Run ESC Calibration file")
      sys.exit()


   if inp == 'n': 

      Zero_Front_Motor()

      
      
    ##################

      Front_pitch_setpoints = np.array([])

      n = int(input("Enter the number of Front pitch angles to test:    "))

      for i in range(n):
         v = float(input(f"Pitch angle {i+1}/{n}: "))
         Front_pitch_setpoints = np.append(Front_pitch_setpoints, v)

      print("Press Enter if these Front pitches are correct, else type 'quit'")
      print(Front_pitch_setpoints)
      inp = input()
      if input == "quit":
         sys.exit(0)

      Front_calculated_DC_setpoints = np.zeros_like(Front_pitch_setpoints, dtype=float)

      for i in range(len(Front_pitch_setpoints)):
         Front_calculated_DC_setpoints[i] = Front_Pitch_to_DCPosition(Front_pitch_setpoints[i], Front_pitch_fit_a, 
         Front_pitch_fit_b, Front_pitch_fit_c)


      ##################
      Zero_Back_Motor()

      Back_pitch_setpoints = np.array([])

      n = int(input("Enter the number of Back pitch angles to test:    "))

      for i in range(n):
         v = float(input(f"Pitch angle {i+1}/{n}: "))
         Back_pitch_setpoints = np.append(Back_pitch_setpoints, v)

      print("Press Enter if these Back pitches are correct, else type 'quit'")
      print(Back_pitch_setpoints)
      inp = input()
      if input == "quit":
         sys.exit(0)

      Back_calculated_DC_setpoints = np.zeros_like(Back_pitch_setpoints, dtype=float)

      for i in range(len(Back_pitch_setpoints)):
         Back_calculated_DC_setpoints[i] = Back_Pitch_to_DCPosition(Back_pitch_setpoints[i], Back_pitch_fit_a, 
         Back_pitch_fit_b, Back_pitch_fit_c)      

      ##################


      dxl_zero_position = Zero_Dynamixel()

      spacing_setpoints = np.array([])

      n = int(input("Enter the number of spacings to test:    "))

      for i in range(n):
         v = float(input(f"Spacing {i+1}/{n}: "))
         spacing_setpoints = np.append(spacing_setpoints, v)

      print("Press Enter if these spacings are correct, else type 'quit'")
      print(spacing_setpoints)
      inp = input()
      if input == "quit":
         sys.exit(0)

      dxl_setpoints = np.zeros_like(spacing_setpoints)
      dxl_setpoints = convert_Dist_to_Turns(spacing_setpoints, dxl_zero_position)

      coaxial_dist = np.zeros_like(dxl_setpoints)



      do_measurement(Front_calculated_DC_setpoints, Front_pitch_setpoints, Back_calculated_DC_setpoints,
                        Back_pitch_setpoints, dxl_setpoints)
      print('Power motor and press enter to begin test')
      inp = input()
      if inp == '':
         time.sleep(1)
         do_measurement(Front_calculated_DC_setpoints, Front_pitch_setpoints, Back_calculated_DC_setpoints,
                        Back_pitch_setpoints, dxl_setpoints)

except (KeyboardInterrupt):
    print("\n"*8 + "User exit.\n")
    stop_1()
    stop_2()
    pi.set_servo_pulsewidth(ESC_1, 0)
    pi.set_servo_pulsewidth(ESC_2, 0)
    pi.stop()
    format_save_csv()    
