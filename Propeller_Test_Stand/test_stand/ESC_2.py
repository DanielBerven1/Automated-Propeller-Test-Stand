import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 0.5)
p.start(0)
try:
    # print ("Connect the battery and press Enter")
    # inp = input("type Enter") 

    # if inp == '':
    #     p.start(0)
    #     time.sleep(1)

    #     p.ChangeDutyCycle(2)
    #     time.sleep(1)
    #     p.ChangeDutyCycle(0.5)
    #     time.sleep(1)

    
    print("Disconnect the battery and press Enter")
    inp = input("type in enter")
    if inp == "":
        p.ChangeFrequency(500)
        print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
        inp = input("type enter")
        if inp == '':            
            p.ChangeFrequency(1000)
            print ("Wierd eh! Special tone")
            time.sleep(7)
            print ("Wait for it ....")
            time.sleep (5)
            print ("Im working on it, DONT WORRY JUST WAIT.....")
            p.ChangeFrequency(0.5)
            time.sleep(2)
            print ("Arming ESC now...")
            p.ChangeFrequency(1000)
            time.sleep(5)
            print ("See.... uhhhhh")

    p.ChangeFrequency(950)
    while(True):
        pass


except(KeyboardInterrupt):
    p.stop()
    GPIO.cleanup()


