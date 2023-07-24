# This program will let you test your ESC and brushless motor.
# Make sure your battery is not connected if you are going to calibrate it at first.
# Since you are testing your motor, I hope you don't have your propeller attached to it otherwise you are in trouble my friend...?
# This program is made by AGT @instructable.com. DO NOT REPUBLISH THIS PROGRAM... actually the program itself is harmful                                             pssst Its not, its safe.

import os     #importing os library so as to communicate with the system
import time   #importing time library to make Rpi wait because its too impatient 
os.system ("sudo pigpiod") #Launching GPIO library
time.sleep(1) # As i said it is too impatient and so if this delay is removed you will get an error
# import RPi.GPIO as GPIO #importing GPIO library
import pigpio


ESC=25  #Connect the ESC in this GPIO pin 

pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC, 0) 

max_value = 2000 #change this if your ESC's max value is different or leave it be
min_value = 1000  #change this if your ESC's min value is different or leave it be

# %% RPM Reader class definition

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

                
def calibrate():   #This is the auto calibration procedure of a normal ESC
    pi.set_servo_pulsewidth(ESC, 0)
    print("Disconnect the battery and press Enter")
    inp = input("type in enter")
    if inp == "":
        pi.set_servo_pulsewidth(ESC, max_value)
        print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = input("type enter")
        if inp == '':            
            pi.set_servo_pulsewidth(ESC, min_value)
            print ("Special tone")
            time.sleep(1)
            control() # You can change this to any other function you want
            
def control(): 
    print ("Starting the motor, make sure its calibrated and armed, if not restart by giving 'x'")
    time.sleep(1)

    speed = 1050

    RPM_GPIO = 24
    RUN_TIME = 10000.0
    SAMPLE_TIME = 2.0


    ### STEP 2: Gain and offset self-calibration:
    RPM_GPIO = 24
   
    p = reader(pi, RPM_GPIO)
    time.sleep(1)

    pi.set_servo_pulsewidth(ESC, speed)

    
    speed = 1050
    # change your speed if you want to.... it should be between 700 - 2000
    print ("Controls - a to decrease speed & d to increase speed")
    while True:
        
        RPM=p.RPM()
        if RPM > 2400:
            stop()          #going for the stop function
            break
        
        pi.set_servo_pulsewidth(ESC, speed)
        inp = input()
        

        if inp == "d":
            speed += 20     # incrementing the speed 
            print ("speed = %d" % speed)
            time.sleep(SAMPLE_TIME)
            RPM = p.RPM()
            print(RPM)


        elif inp == "a":
            speed -= 20     # decrementing the speed
            print ("speed = %d" % speed)
            time.sleep(SAMPLE_TIME)
            RPM = p.RPM()
            print(RPM)

        elif inp == "stop":
            stop()          #going for the stop function
            break


        else:
            print ("Invalid Input")
            
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
        control() 
        
def stop(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC, 0)
    pi.stop()

#This is the start of the program actually, to start the function it needs to be initialized before calling... stupid python.    

calibrate()


