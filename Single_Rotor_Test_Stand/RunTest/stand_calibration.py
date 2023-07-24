import pandas as pd
import numpy as np
import scipy as scipy
from scipy import stats
import matplotlib.pyplot as plt



#!/usr/bin/python
# -*- coding: utf-8 -*-
"""PiPyADC: Example file for class ADS1256 in module pipyadc:

ADS1256 cycling through eight input channels.

Hardware: Waveshare ADS1256 board interfaced to the Raspberry Pi 3

Ulrich Lukas 2017-03-10
"""
import sys
import os
import pigpio as io
from ADS1256_definitions import *
from ADS1256_PiGPIO import ADS1256
import time

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
#CH_SEQUENCE = (POTI, LDR, EXT2)
################################################################################

ads = ADS1256(pi=io.pi(PI_HOST))

### STEP 2: Gain and offset self-calibration:
ads.cal_self()

def do_measurement():
    ### STEP 1: Initialise ADC object using default configuration:
    # (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
    # (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)

    while True:
        ### STEP 3: Get data:
        #print(time.time())
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
        #print(time.time())
        # print(f"Left load cell: {voltages[2]}")
        # print(f"Right load cell: {voltages[4]}")
        # print(f"thrust load cell: {voltages[5]}")


print("STAND CALIBRATION")
print("Type 'skip' to skip MLP Calibration ")
inp = input()
if inp == "skip":
    pass
else: 
    print ("PART 1: MLP Calibration")


    weights = np.array([0, 10, 20, 50, 100, 200, 500])


    print(weights)
    left_MLP_measurements = np.zeros(7)
    right_MLP_measurements = np.zeros(7)


        # NO Weight
    print("Place NO weight on LEFT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[0] = voltages[2]

    print("Place NO weight on RIGHT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    right_MLP_measurements[0] = voltages[4]

        # 10g Weight
    print("Place 10g weight on LEFT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[1] = voltages[2]

    print("Place 10g weight on RIGHT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    right_MLP_measurements[1] = voltages[4]

        # 20g Weight
    print("Place 20g weight on LEFT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[2] = voltages[2]

    print("Place 20g weight on RIGHT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    right_MLP_measurements[2] = voltages[4]

        # 50g Weight
    print("Place 50g weight on LEFT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[3] = voltages[2]

    print("Place 50g weight on RIGHT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    right_MLP_measurements[3] = voltages[4]

        # 100g Weight
    print("Place 100g weight on LEFT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[4] = voltages[2]

    print("Place 100g weight on RIGHT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    right_MLP_measurements[4] = voltages[4]

        # 200g weight
    print("Place 200g weight on LEFT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[5] = voltages[2]

    print("Place 200g weight on RIGHT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    right_MLP_measurements[5] = voltages[4]

        # 500g weight
    print("Place 500g weight on LEFT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[6] = voltages[2]

    print("Place 500g weight on RIGHT load cell, then press Enter")
    inp = input()
    if inp == "":
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    right_MLP_measurements[6] = voltages[4]




    left_result = scipy.stats.linregress(weights, left_MLP_measurements, alternative='two-sided')

    print(f"Left Slope is: {left_result.slope}")
    print(f"Left Intercept is: {left_result.intercept}")
    print(f"Left R value is: {left_result.rvalue}")

    right_result = scipy.stats.linregress(weights, right_MLP_measurements, alternative='two-sided')

    print(f"Right Slope is: {right_result.slope}")
    print(f"Right Intercept is: {right_result.intercept}")
    print(f"Right R value is: {right_result.rvalue}")



######################## Replace if neccasary ################################3

left_slope = 0.0010797531279340853

right_slope = 0.0011062980813381132

########################                       ################################

print("Part 2: Torque Calibration")
print("Assemble Torque Calibration Mount, then press Enter")
inp = input()
if inp == "":
    print("Part 2.1: 200g Weight vs Varying Distance")

    right_measured_forces = np.zeros(26)
    left_measured_forces = np.zeros(26)
    right_theoretical_forces = np.zeros(26)
    left_theoretical_forces = np.zeros(26)


    length_1 = 16.25*2
    length_2 = np.array([0, 35, 45, 55, 65, 75])

    left_theoretical_force = np.zeros(6)
    right_theoretical_force = np.zeros(6)

    for i in range(len(length_2)-1):
        left_theoretical_force[i+1] = 200*(length_2[i+1] + length_1/2)/length_1
        right_theoretical_force[i+1] = 200*(length_2[i+1]-length_1/2)/length_1

    left_theoretical_forces[0:6] = left_theoretical_force
    right_theoretical_forces[0:6] = right_theoretical_force



    left_MLP_measurements = np.zeros(6)
    right_MLP_measurements = np.zeros(6)

    print("After assembled and NO weight is placed, press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_intercept = voltages[2]
    right_intercept = voltages[4]
    left_MLP_measurements[0] = left_intercept
    right_MLP_measurements[0] = right_intercept



    print("Place 200g weight at 35mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[1] = voltages[2]
    right_MLP_measurements[1] = voltages[4]

    print("Place 200g weight at 45mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[2] = voltages[2]
    right_MLP_measurements[2] = voltages[4]

    print("Place 200g weight at 55mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[3] = voltages[2]
    right_MLP_measurements[3] = voltages[4]

    print("Place 200g weight at 65mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[4] = voltages[2]
    right_MLP_measurements[4] = voltages[4]

    print("Place 200g weight at 75mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[5] = voltages[2]
    right_MLP_measurements[5] = voltages[4]
    
    
    left_MLP_measurements = (left_MLP_measurements- left_intercept)/left_slope
    right_MLP_measurements = (right_MLP_measurements- right_intercept)/right_slope 


    left_measured_forces[0:6] = left_MLP_measurements
    right_measured_forces[0:6] = right_MLP_measurements



#####################################
    print("Part 2.2: 50g Weight vs Varying Distance")


    left_theoretical_force = np.zeros(6)
    right_theoretical_force = np.zeros(6)

    for i in range(len(length_2)-1):
        left_theoretical_force[i+1] = 50*(length_2[i+1] + length_1/2)/length_1
        right_theoretical_force[i+1] = 50*(length_2[i+1]-length_1/2)/length_1

    left_theoretical_forces[6:12] = left_theoretical_force
    right_theoretical_forces[6:12] = right_theoretical_force

    left_MLP_measurements = np.zeros(6)
    right_MLP_measurements = np.zeros(6)

    print("After assembled and NO weight is placed, press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_intercept = voltages[2]
    right_intercept = voltages[4]
    left_MLP_measurements[0] = left_intercept
    right_MLP_measurements[0] = right_intercept



    print("Place 50g weight at 35mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[1] = voltages[2]
    right_MLP_measurements[1] = voltages[4]

    print("Place 50g weight at 45mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[2] = voltages[2]
    right_MLP_measurements[2] = voltages[4]

    print("Place 50g weight at 55mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[3] = voltages[2]
    right_MLP_measurements[3] = voltages[4]

    print("Place 50g weight at 65mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[4] = voltages[2]
    right_MLP_measurements[4] = voltages[4]

    print("Place 50g weight at 75mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[5] = voltages[2]
    right_MLP_measurements[5] = voltages[4]
    
    
    left_MLP_measurements = (left_MLP_measurements- left_intercept)/left_slope
    right_MLP_measurements = (right_MLP_measurements- right_intercept)/right_slope 

    left_measured_forces[6:12] = left_MLP_measurements
    right_measured_forces[6:12] = right_MLP_measurements


##################################3


    print("Part 2.3: 75mm Distance Vs Varying Weight")



    weights = np.array([0, 10, 20, 50, 100, 200, 500])

    left_theoretical_force = np.zeros(7)
    right_theoretical_force = np.zeros(7)

    for i in range(len(weights)-1):
        left_theoretical_force[i+1] = weights[i+1]*(75 + length_1/2)/length_1
        right_theoretical_force[i+1] = weights[i+1]*(75-length_1/2)/length_1

    left_theoretical_forces[12:19] = left_theoretical_force
    right_theoretical_forces[12:19] = right_theoretical_force

    left_MLP_measurements = np.zeros(7)
    right_MLP_measurements = np.zeros(7)

    print("After assembled and NO weight is placed, press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_intercept = voltages[2]
    right_intercept = voltages[4]
    left_MLP_measurements[0] = left_intercept
    right_MLP_measurements[0] = right_intercept



    print("Place 10g weight at 75mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[1] = voltages[2]
    right_MLP_measurements[1] = voltages[4]

    print("Place 20g weight at 75mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[2] = voltages[2]
    right_MLP_measurements[2] = voltages[4]

    print("Place 50g weight at 75mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[3] = voltages[2]
    right_MLP_measurements[3] = voltages[4]

    print("Place 100g weight at 75mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[4] = voltages[2]
    right_MLP_measurements[4] = voltages[4]

    print("Place 200g weight at 75mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[5] = voltages[2]
    right_MLP_measurements[5] = voltages[4]
    
    print("Place 500g weight at 75mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[6] = voltages[2]
    right_MLP_measurements[6] = voltages[4]
    
    left_MLP_measurements = (left_MLP_measurements- left_intercept)/left_slope
    right_MLP_measurements = (right_MLP_measurements- right_intercept)/right_slope 

    left_measured_forces[12:19] = left_MLP_measurements
    right_measured_forces[12:19] = right_MLP_measurements


#########################

    print("Part 2.3: 45mm Distance Vs Varying Weight")



    weights = np.array([0, 10, 20, 50, 100, 200, 500])

    left_theoretical_force = np.zeros(7)
    right_theoretical_force = np.zeros(7)

    for i in range(len(weights)-1):
        left_theoretical_force[i+1] = weights[i+1]*(45 + length_1/2)/length_1
        right_theoretical_force[i+1] = weights[i+1]*(45-length_1/2)/length_1

    left_theoretical_forces[19:26] = left_theoretical_force
    right_theoretical_forces[19:26] = right_theoretical_force

    print(left_theoretical_forces)

    left_MLP_measurements = np.zeros(7)
    right_MLP_measurements = np.zeros(7)

    print("After assembled and NO weight is placed, press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_intercept = voltages[2]
    right_intercept = voltages[4]
    left_MLP_measurements[0] = left_intercept
    right_MLP_measurements[0] = right_intercept



    print("Place 10g weight at 45mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[1] = voltages[2]
    right_MLP_measurements[1] = voltages[4]

    print("Place 20g weight at 45mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[2] = voltages[2]
    right_MLP_measurements[2] = voltages[4]

    print("Place 50g weight at 45mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[3] = voltages[2]
    right_MLP_measurements[3] = voltages[4]

    print("Place 100g weight at 45mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[4] = voltages[2]
    right_MLP_measurements[4] = voltages[4]

    print("Place 200g weight at 45mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[5] = voltages[2]
    right_MLP_measurements[5] = voltages[4]
    
    print("Place 500g weight at 45mm location, then press Enter")
    inp = input()
    if inp == "": 
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages     = [i * ads.v_per_digit for i in raw_channels]
    left_MLP_measurements[6] = voltages[2]
    right_MLP_measurements[6] = voltages[4]
    
    left_MLP_measurements = (left_MLP_measurements- left_intercept)/left_slope
    right_MLP_measurements = (right_MLP_measurements- right_intercept)/right_slope 

    left_measured_forces[19:26] = left_MLP_measurements
    right_measured_forces[19:26] = right_MLP_measurements

#############################


    print(left_measured_forces)
    print(left_theoretical_forces)

    left_result = scipy.stats.linregress(left_measured_forces, left_theoretical_forces, alternative='two-sided')

    print(f"Left Slope is: {left_result.slope}")
    print(f"Left Intercept is: {left_result.intercept}")
    print(f"Left R value is: {left_result.rvalue}")

    right_result = scipy.stats.linregress(right_measured_forces, right_theoretical_forces, alternative='two-sided')

    print(f"Right Slope is: {right_result.slope}")
    print(f"Right Intercept is: {right_result.intercept}")
    print(f"Right R value is: {right_result.rvalue}")

    left_fit = left_measured_forces*left_result.slope + left_result.intercept
    right_fit = right_measured_forces*right_result.slope + right_result.intercept





    fig_1 = plt.figure()
    ax_1 = fig_1.add_subplot()
    ax_1.set_title(r"Left MLP")
    ax_1.set_xlabel(r"Measured Forces")
    ax_1.set_ylabel(r"Theoretical Forces")
    line_1 = ax_1.scatter(left_measured_forces, left_theoretical_forces, c="blue", s= 5)
    line_2 = ax_1.plot(left_measured_forces, left_fit, c="red")
    

    fig_2 = plt.figure()
    ax_2 = fig_2.add_subplot()
    ax_2.set_title(r"Right MLP")
    ax_2.set_xlabel(r"Measured Forces")
    ax_2.set_ylabel(r"Theoretical Forces")
    line_1 = ax_2.scatter(right_measured_forces, right_theoretical_forces, c="blue", s=5)
    line_2 = ax_2.plot(right_measured_forces, right_fit, c="red")

    plt.show()

    df = [['Left_Mesrd_to_Theo_Slope', 'Left_Mesrd_to_Theo_Intcpt' , 'Right_Mesrd_to_Theo_Slope', 'Right_Mesrd_to_Theo_Intcpt', 'Left_mV_to_g_Slope', 'Right_mv_to_g_Slope']]
    df.append([1.1095250518914466, -1.0394290060635853, 1.2068502499608134, 0.041521816421550284, 0.0010797531279340853, 0.0011062980813381132])





    path = r'/home/ubuntu/src/waveshare_a2d_d2a_python/experiment'
    df = pd.DataFrame(df)

    df.to_csv(path + f'/MLP_calibrated_fits.csv', encoding = 'utf-8')

    # Left Slope is: 0.0010797531279340853
    # Left Intercept is: 0.07648392823421798
    # Left R value is: 0.9999985950225894
    # Right Slope is: 0.0011062980813381132
    # Right Intercept is: 0.036809638704094594
    # Right R value is: 0.9999908081169806


    # Left Slope is: 1.1095250518914466
    # Left Intercept is: -1.0394290060635853
    # Left R value is: 0.9997652891573956
    # Right Slope is: 1.2068502499608134
    # Right Intercept is: 0.041521816421550284
    # Right R value is: 0.9998617987830284
