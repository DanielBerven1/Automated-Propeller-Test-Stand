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

def wait_for_input(RPM):
    global df
    global setpoint
    inp = input()

    if inp == "":
        series1 = pd.DataFrame([[RPM, ADC_Value[2]*5.0/0x7fffff, ADC_Value[4]*5.0/0x7fffff]], columns=['Setpoint', 'Left Load Cell', 'Right Load Cell'])
        df = pd.concat([df,series1], ignore_index = True)
    elif inp == "n":
        setpoint += 1
   
    print('noted')


if __name__ == "__main__":

    try:
        pass
        ADC = ADS1256.ADS1256()
        DAC = DAC8532.DAC8532()
        ADC.ADS1256_init()

        DAC.DAC8532_Out_Voltage(0x30, 3)
        DAC.DAC8532_Out_Voltage(0x34, 3)

        while(1):

            #print(ADC.ADS1256_ReadChipID())
            ADC_Value = ADC.ADS1256_GetAll()
            print ("0 ADC = %lf"%(ADC_Value[0]*5.0/0x7fffff))
            print ("1 ADC = %lf"%(ADC_Value[1]*5.0/0x7fffff))
            print ("2 ADC = %lf"%(ADC_Value[2]*5.0/0x7fffff))
            print ("3 ADC = %lf"%(ADC_Value[3]*5.0/0x7fffff))
            print ("4 ADC = %lf"%(ADC_Value[4]*5.0/0x7fffff))
            print ("5 ADC = %lf"%(ADC_Value[5]*5.0/0x7fffff))
            print ("6 ADC = %lf"%(ADC_Value[6]*5.0/0x7fffff))
            print ("7 ADC = %lf"%(ADC_Value[7]*5.0/0x7fffff))

            temp = (ADC_Value[0]>>7)*5.0/0xffff
            print ("DAC :",temp)
            print ("\33[10A")
            DAC.DAC8532_Out_Voltage(DAC8532.channel_A, temp)
            DAC.DAC8532_Out_Voltage(DAC8532.channel_B, 3.3 - temp)

            # threading.Thread(target = wait_for_input, daemon=True).start()

    except :
        GPIO.cleanup()
        print ("\r\nProgram end     ")
        df.to_csv('out.csv', sep = '\t', encoding = 'utf-8')
        exit()
