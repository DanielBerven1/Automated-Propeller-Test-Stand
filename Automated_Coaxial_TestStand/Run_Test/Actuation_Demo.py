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

    Dynamixel_have_been_zerod = 0
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



# %% main program
try:

    print("enter")
    inp=input()

    Front_pitch_setpoints = np.array([])

    n = int(input("Enter the number of Front pitch angles to test:    "))

    for i in range(n):
        v = float(input(f"Pitch angle {i+1}/{n}: "))
        Front_pitch_setpoints = np.append(Front_pitch_setpoints, v)

    print("Press Enter if these Front pitches are correct, else type 'quit'")
    print(Front_pitch_setpoints)



    Front_calculated_DC_setpoints = np.zeros_like(Front_pitch_setpoints, dtype=float)

    for i in range(len(Front_pitch_setpoints)):
        Front_calculated_DC_setpoints[i] = Front_Pitch_to_DCPosition(Front_pitch_setpoints[i], Front_pitch_fit_a, 
        Front_pitch_fit_b, Front_pitch_fit_c)


    Back_pitch_setpoints = np.array([])

    n = int(input("Enter the number of Back pitch angles to test:    "))

    for i in range(n):
        v = float(input(f"Pitch angle {i+1}/{n}: "))
        Back_pitch_setpoints = np.append(Back_pitch_setpoints, v)

    print("Press Enter if these Back pitches are correct, else type 'quit'")
    print(Back_pitch_setpoints)


    Back_calculated_DC_setpoints = np.zeros_like(Back_pitch_setpoints, dtype=float)

    for i in range(len(Back_pitch_setpoints)):
        Back_calculated_DC_setpoints[i] = Back_Pitch_to_DCPosition(Back_pitch_setpoints[i], Back_pitch_fit_a, 
        Back_pitch_fit_b, Back_pitch_fit_c)      


    spacing_setpoints = np.array([])

    n = int(input("Enter the number of spacings to test:    "))

    for i in range(n):
        v = float(input(f"Spacing {i+1}/{n}: "))
        spacing_setpoints = np.append(spacing_setpoints, v)

    print("Press Enter if these spacings are correct, else type 'quit'")
    print(spacing_setpoints)


    print("Enter to start")

    inp = input()


    Zero_Front_Motor()

    Zero_Back_Motor()

    dxl_zero_position = Zero_Dynamixel()

    dxl_setpoints = np.zeros_like(spacing_setpoints)
    dxl_setpoints = convert_Dist_to_Turns(spacing_setpoints, dxl_zero_position)

    coaxial_dist = np.zeros_like(dxl_setpoints)

    front = 1
    front_counter = 0

    back = 0
    back_counter = 0
    
    dyna = 0
    dyna_counter = 0

    Set_DC_Front_Setpoint(Front_calculated_DC_setpoints[front_counter])

    while msg_flag_DC_Front_position == 0:
        print('waiting for pitch adjustment', end='\r')

    time.sleep(3)

    Set_DC_Back_Setpoint(Front_calculated_DC_setpoints[back_counter])
                
    while msg_flag_DC_Front_position == 0:
        print('waiting for pitch adjustment', end='\r')

    time.sleep(3)

    dynamixel_writeGoalPosition_hold(dxl_setpoints[dyna_counter])
    time.sleep(2)
    
    while True:
        if front == 1:
            front_counter += 1
            if front_counter == len(Front_calculated_DC_setpoints):
                Zero_Front_Motor()
                time.sleep(3)
                front_counter = 0
                Set_DC_Front_Setpoint(Front_calculated_DC_setpoints[front_counter])

                while msg_flag_DC_Front_position == 0:
                    print('waiting for pitch adjustment', end='\r')

                time.sleep(1)
                msg_flag_DC_Front_position = 0
                print('\n')
                time.sleep(3)
                print('message recieved')

                front = 0
                back = 1

            
            else:
                print("front_publish")
                Set_DC_Front_Setpoint(Front_calculated_DC_setpoints[front_counter])

                while msg_flag_DC_Front_position == 0:
                    print('waiting for pitch adjustment', end='\r')

                time.sleep(1)
                msg_flag_DC_Front_position = 0
                print('\n')
                time.sleep(3)
                print('message recieved')

        
        if back == 1:
            back_counter += 1
            if back_counter == len(Back_calculated_DC_setpoints):
                Zero_Back_Motor()
                time.sleep(3)

                back_counter = 0
                Set_DC_Back_Setpoint(Front_calculated_DC_setpoints[back_counter])
                
                while msg_flag_DC_Front_position == 0:
                    print('waiting for pitch adjustment', end='\r')

                time.sleep(1)
                msg_flag_DC_Front_position = 0
                print('\n')
                time.sleep(3)
                print('message recieved')


                back = 0
                dyna = 1
                
            
            else:
                print("front_publish")
                Set_DC_Back_Setpoint(Back_calculated_DC_setpoints[back_counter])

                while msg_flag_DC_Back_position == 0:
                    print('waiting for pitch adjustment', end='\r')

                
                msg_flag_DC_Back_position = 0
                print('\n')
                time.sleep(3)
                print('message recieved')
                back = 0
                front = 1



        
        if dyna == 1:
            dyna_counter += 1
            if dyna_counter == len(spacing_setpoints):
                Zero_Dynamixel()
                dyna = 0
                break
                
                
            
            else:
                dynamixel_writeGoalPosition_hold(dxl_setpoints[dyna_counter])


                print('\n')
                time.sleep(3)
                print('message recieved')
                dyna = 0
                front = 1


        time.sleep(3)

    sys.exit(0)





except (KeyboardInterrupt):
    print("\n"*8 + "User exit.\n")


