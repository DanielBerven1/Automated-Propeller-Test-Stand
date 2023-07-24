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
import paho.mqtt.client as mqtt
import MQTT_RunTest_Definitions as mqtt_com


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

def dynamixel_clearMultiTurnInfo():
   # Clear Multi-Turn Information
   dxl_comm_result, dxl_error = packetHandler.clearMultiTurn(portHandler, DXL_ID)
   if dxl_comm_result != COMM_SUCCESS:
      print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
      print("%s" % packetHandler.getRxPacketError(dxl_error))



def dynamixel_DisableTorque():
   xl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
   if dxl_comm_result != COMM_SUCCESS:
      print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
      print("%s" % packetHandler.getRxPacketError(dxl_error))   

def dynamixel_End():
   portHandler.closePort()

def convert_Dist_to_Turns(dist, zero_pos):
    turns = np.zeros_like(dist, dtype = int)
    for i in range(len(dist)):
        turns[i] = dist[i]*4095 + zero_pos
    return turns


def convert_Turns_to_Dist(turns, dynamixel_zero_turns):
   # dist = np.zeros_like(turns, dtype = int)
    zero_dist = 265 # mm
   #  for i in range(len(turns)):
   #      dist[i] = (turns[i]-dynamixel_zero_turns)/4095 + zero_dist
    return (turns-dynamixel_zero_turns)/4095 + zero_dist




# %% Set up MQTT Communications

client = mqtt_com.setup_test_client()


global msg_flag_zero
msg_flag_zero = 0

global have_been_zerod
have_been_zerod = 0

def on_message(client, userdata, msg):
    # print(msg.topic + '' + str(msg.payload))



    if msg.topic == "Zero_Switch":
        message = str(msg.payload.decode("utf-8"))
        global have_been_zerod
        have_been_zerod = float(message)
        print(have_been_zerod)
        global msg_flag_zero
        msg_flag_zero= 1  


client.on_message = on_message 



# %% Setup utilities

# GPIO.setmode(GPIO.BOARD)








print ("Enter Propeller Type:")
prop_type = input() 

print ("Enter Prop Diameter:")
prop_diameter = input()


# print("max speed (RPM): ")
# max_RPM = float(input())

df = [['Coaxial Distance', 'RPM_1','Left Load Cell 1', 'Right Load Cell 1', 'Thrust 1', 'RPM_2','Left Load Cell 2', 'Right Load Cell 2', 'Thrust 2']]
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

increment_spacing = 0
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

def wait_for_input(coaxial_dist, RPM_1,RPM_2,voltages):
    global df
    
    df.append([coaxial_dist, int(RPM_1+0.5), voltages[0], voltages[1], voltages[2], 
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

def do_calibrate_measurement(ads, coaxial_dist):
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
      wait_for_input(coaxial_dist, RPM_1, RPM_2, voltages)
   first_time = False

def do_measurement(dxl_setpoints):
    ### STEP 1: Initialise ADC object using default configuration:
    # (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
    # (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
    global first_time
    global RPM_last
    global skip_counter 
    global skip_speed
    global increment_spacing
    global increment
    global dxl_zero_position
    global coaxial_dist

    
    ads = ADS1256(pi=pigpio.pi(PI_HOST))

    ### STEP 2: Gain and offset self-calibration:
    ads.cal_self()
    RPM_1_GPIO = 24
    RPM_2_GPIO = 16
   
    p_1 = reader(pi, RPM_1_GPIO)
    p_2 = reader(pi, RPM_2_GPIO)
    time.sleep(1)

    speed = 1050
    speed_1 = 1250
    speed_2 = 1250
    

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
         

         
         print(speed_1)
         print(speed_2)




            
      
         #print("RPM={}".format(int(RPM+0.5)))

         ### STEP 4: DONE. Have fun!
         #nice_output(raw_channels, voltages)

         

         wait_for_input(coaxial_dist[increment-1], RPM_1, RPM_2, voltages)


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

         if (time.time() - start) >= 12.0 and increment_spacing == 0:

            print('STOPPED')
            stop_1()
            stop_2()            
            print(int(RPM_1+0.5))
            print(int(RPM_2+0.5))
            increment_spacing = 1
         
         if increment_spacing == 1:
            time.sleep(7)

            if increment == len(dxl_setpoints):
                           pi.set_servo_pulsewidth(ESC_1, 0)
                           pi.set_servo_pulsewidth(ESC_2, 0)
                           dynamixel_writeGoalPosition(dxl_zero_position)
                           format_save_csv(coaxial_dist)

            dynamixel_writeGoalPosition_hold(dxl_setpoints[increment])
            time.sleep(5)
            dxl_present_position = dynamixel_readPosition()
            coaxial_dist[increment] = convert_Turns_to_Dist(dxl_present_position, dxl_zero_position)      

            do_calibrate_measurement(ads, coaxial_dist[increment])

            increment+=1

            

            time.sleep(3)
            print('START')
            pi.set_servo_pulsewidth(ESC_1, speed_1)
            pi.set_servo_pulsewidth(ESC_2, speed_2)
            start = time.time()
            print(int(RPM_1+0.5))
            print(int(RPM_2+0.5))

            increment_spacing=0

         
         


       
    elif (first_time == True):
      dxl_goal_position = dxl_setpoints[increment]
      dynamixel_writeGoalPosition_hold(dxl_goal_position)
      time.sleep(5)
      dxl_present_position = dynamixel_readPosition()
      coaxial_dist[increment] = convert_Turns_to_Dist(dxl_present_position, dxl_zero_position)
      do_calibrate_measurement(ads, coaxial_dist[increment])

      increment += 1



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

def format_save_csv(coaxial_dist):
    pd.options.mode.chained_assignment = None
    global df
    print("\n"*8 + "Max Speed Reached.\n")
    path_Calib = r'/home/ubuntu/src/Co_Axial_Stand/Calibration'
    path_Data = r'/home/ubuntu/src/Co_Axial_Stand/experiment'
    df = pd.DataFrame(df)
    df.columns = df.iloc[0]
    df = df.drop(0)
    
    
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

    print(coaxial_dist)
    for i in range(len(coaxial_dist)):
      print(coaxial_dist[i])
      ESC_2_coaxDist_subset = df.loc[df['Coaxial Distance'] == coaxial_dist[i]]
      
      Left_mv_to_g_intcpt_1 = ESC_2_coaxDist_subset.iloc[1:31,2]
      Left_mv_to_g_intcpt_1 = Left_mv_to_g_intcpt_1.to_numpy()
      Left_mv_to_g_intcpt_1 = np.sum(Left_mv_to_g_intcpt_1)/len(Left_mv_to_g_intcpt_1)

      Right_mv_to_g_intcpt_1 = ESC_2_coaxDist_subset.iloc[1:31,3]
      Right_mv_to_g_intcpt_1 = Right_mv_to_g_intcpt_1.to_numpy()
      Right_mv_to_g_intcpt_1 = np.sum(Right_mv_to_g_intcpt_1)/len(Right_mv_to_g_intcpt_1)

      Thrust_mv_to_g_intcpt_1 = ESC_2_coaxDist_subset.iloc[1:31,4] 
      Thrust_mv_to_g_intcpt_1 = Thrust_mv_to_g_intcpt_1.to_numpy()
      Thrust_mv_to_g_intcpt_1 = np.sum(Thrust_mv_to_g_intcpt_1)/len(Thrust_mv_to_g_intcpt_1)
      
      
      Left_mv_to_g_intcpt_2 = ESC_2_coaxDist_subset.iloc[1:31,6]
      Left_mv_to_g_intcpt_2 = Left_mv_to_g_intcpt_2.to_numpy()
      Left_mv_to_g_intcpt_2 = np.sum(Left_mv_to_g_intcpt_2)/len(Left_mv_to_g_intcpt_2)

      Right_mv_to_g_intcpt_2 = ESC_2_coaxDist_subset.iloc[1:31,7]
      Right_mv_to_g_intcpt_2 = Right_mv_to_g_intcpt_2.to_numpy()
      Right_mv_to_g_intcpt_2 = np.sum(Right_mv_to_g_intcpt_2)/len(Right_mv_to_g_intcpt_2)

      Thrust_mv_to_g_intcpt_2 = ESC_2_coaxDist_subset.iloc[1:31,8] 
      Thrust_mv_to_g_intcpt_2 = Thrust_mv_to_g_intcpt_2.to_numpy()
      Thrust_mv_to_g_intcpt_2 = np.sum(Thrust_mv_to_g_intcpt_2)/len(Thrust_mv_to_g_intcpt_2)
      
      ESC_2_coaxDist_subset['Left Load Cell 1'] = ESC_2_coaxDist_subset['Left Load Cell 1'].apply(lambda x: (x - Left_mv_to_g_intcpt_1)*Left_mV_to_g_slope_1)
      ESC_2_coaxDist_subset['Right Load Cell 1'] = ESC_2_coaxDist_subset['Right Load Cell 1'].apply(lambda x: (x-Right_mv_to_g_intcpt_1)*Right_mv_to_g_slope_1)

      ESC_2_coaxDist_subset['Left Load Cell 1'] = ESC_2_coaxDist_subset['Left Load Cell 1'].apply(lambda x: (x*Left_msrd_to_theo_slope_1 + Left_msrd_to_theo_intcpt_1))
      ESC_2_coaxDist_subset['Right Load Cell 1'] = ESC_2_coaxDist_subset['Right Load Cell 1'].apply(lambda x: (x*Right_msrd_to_theo_slope_1+Right_msrd_to_theo_intcpt_1)) 

      ESC_2_coaxDist_subset['Thrust 1'] = ESC_2_coaxDist_subset['Thrust 1'].apply(lambda x: (x-Thrust_mv_to_g_intcpt_1)*0.001*Thrust_mV_to_g_slope_1)  

      Torque_1 = 16.25*(ESC_2_coaxDist_subset['Left Load Cell 1'] + ESC_2_coaxDist_subset['Right Load Cell 1'])*0.0098*10**(-3)

      ESC_2_coaxDist_subset['Torque 1'] = Torque_1

      ESC_2_coaxDist_subset['Left Load Cell 2'] = ESC_2_coaxDist_subset['Left Load Cell 2'].apply(lambda x: (x - Left_mv_to_g_intcpt_2)*Left_mV_to_g_slope_2)
      ESC_2_coaxDist_subset['Right Load Cell 2'] = ESC_2_coaxDist_subset['Right Load Cell 2'].apply(lambda x: (x-Right_mv_to_g_intcpt_2)*Right_mv_to_g_slope_2)

      ESC_2_coaxDist_subset['Left Load Cell 2'] = ESC_2_coaxDist_subset['Left Load Cell 2'].apply(lambda x: (x*Left_msrd_to_theo_slope_2 + Left_msrd_to_theo_intcpt_2))
      ESC_2_coaxDist_subset['Right Load Cell 2'] = ESC_2_coaxDist_subset['Right Load Cell 2'].apply(lambda x: (x*Right_msrd_to_theo_slope_2+Right_msrd_to_theo_intcpt_2)) 

      ESC_2_coaxDist_subset['Thrust 2'] = ESC_2_coaxDist_subset['Thrust 2'].apply(lambda x: (x-Thrust_mv_to_g_intcpt_2)*0.001*Thrust_mV_to_g_slope_2)  

      Torque_2 = 16.25*(ESC_2_coaxDist_subset['Left Load Cell 2'] + ESC_2_coaxDist_subset['Right Load Cell 2'])*0.0098*10**(-3)

      ESC_2_coaxDist_subset['Torque 2'] = Torque_2

      df2 = pd.concat([df2, ESC_2_coaxDist_subset])

    df2.to_csv(path_Data + f'/{prop_type}_{prop_diameter}_Coaxial_VarDist.csv', sep = ',', encoding = 'utf-8')
    pd.options.mode.chained_assignment = 'warn'
    pi.set_servo_pulsewidth(ESC_1, 0)
    pi.set_servo_pulsewidth(ESC_2, 0)
    sys.exit(0)


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


      try:
         # publishing setpoint as string
         msg = str(' ')
         pubMsg = client.publish(
            topic='rpi/reset_zero',
            payload=msg.encode('utf-8'),
            qos=0,
         )
         pubMsg.wait_for_publish()
         print(pubMsg.is_published())
      except Exception as e:
         print(e)


      print('Zeroing dynamixel')

      
      pos = dynamixel_readPosition()

      try:
         # publishing setpoint as string
         msg = str(setpoint)
         pubMsg = client.publish(
            topic='rpi/get_data',
            payload=msg.encode('utf-8'),
            qos=0,
         )
         pubMsg.wait_for_publish()
         print(pubMsg.is_published())
      except Exception as e:
         print(e)

      time.sleep(3)

      while True:
         
         if have_been_zerod == 1:
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
            time.sleep(3)
      
      dxl_setpoints = np.zeros_like(spacing_setpoints)
      dxl_setpoints = convert_Dist_to_Turns(spacing_setpoints, dxl_present_position)

      print("Press Enter if these dynamixel positions are correct, else type 'quit'")
      print(dxl_setpoints)
      inp = input()
      if input == "quit":
         sys.exit(0)

      coaxial_dist = np.zeros_like(dxl_setpoints)

      do_measurement(dxl_setpoints)
      print('Power motor and press enter to begin test')
      inp = input()
      if inp == '':
         time.sleep(1)
         do_measurement(dxl_setpoints)

except (KeyboardInterrupt):

   try:
      # publishing setpoint as string
      msg = str(setpoint)
      pubMsg = client.publish(
         topic='rpi/reset_zero',
         payload=msg.encode('utf-8'),
         qos=0,
      )
      pubMsg.wait_for_publish()
      print(pubMsg.is_published())
   except Exception as e:
      print(e)

   print("\n"*8 + "User exit.\n")
   stop_1()
   stop_2()
   pi.set_servo_pulsewidth(ESC_1, 0)
   pi.set_servo_pulsewidth(ESC_2, 0)
   pi.stop()
   format_save_csv(coaxial_dist)



