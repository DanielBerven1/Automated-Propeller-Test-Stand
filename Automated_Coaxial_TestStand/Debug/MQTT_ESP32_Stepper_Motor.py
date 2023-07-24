import time
import paho.mqtt.client as mqtt
import MQTT_Definitions as mqtt_com
import numpy as np


# setup client
client = mqtt_com.setup_Stepper_client()


global msg_flag_zero
msg_flag_zero = 0

global msg_flag_Stepper
msg_flag_Stepper= 0

global Current_Stepper_Position
Current_Stepper_Position = 0

global have_been_zerod
have_been_zerod = 0


def on_message(client, userdata, msg):
    # print(msg.topic + '' + str(msg.payload))

    global setup_flag
    if msg.topic == "Stepper_Position":

        message = str(msg.payload.decode("utf-8"))
        global Current_Stepper_Position
        Current_Stepper_Position = float(message)
        print(Current_Stepper_Position)
        global msg_flag_Stepper
        msg_flag_Stepper = 1   

    if msg.topic == "Zero_Switch":
        message = str(msg.payload.decode("utf-8"))
        global have_been_zerod
        have_been_zerod = float(message)
        print(have_been_zerod)
        global msg_flag_zero
        msg_flag_zero= 1  


client.on_message = on_message 




# initialize setpoint
Stepper_setpoint = 0

flag_first_time = 1


while True:

    print(msg_flag_Stepper)
    if flag_first_time == 1:
        try:
            # publishing setpoint as string
            msg = str('get_data')
            pubMsg = client.publish(
                topic='rpi/get_data',
                payload=msg.encode('utf-8'),
                qos=0,
            )
            pubMsg.wait_for_publish()
            print(pubMsg.is_published())
        except Exception as e:
            print(e)
        flag_first_time = 0
    
    
        while msg_flag_Stepper != 1:
            print('Waiting for stepper return', end='\r')
        msg_flag_Stepper = 0
        time.sleep(1)


    print(f'Current Step is {Current_Stepper_Position}')
    if have_been_zerod == 0:
        print('Status: Not zerod')
    else:
        print('Status: Zerod')

    print("Send step?")
    inp = input()
    if inp == "get data":
        try:
            # publishing setpoint as string
            msg = str('get_data')
            pubMsg = client.publish(
                topic='rpi/get_data',
                payload=msg.encode('utf-8'),
                qos=0,
            )
            pubMsg.wait_for_publish()
            print(pubMsg.is_published())
        except Exception as e:
            print(e)

    
    
        while msg_flag_Stepper != 1:
            print('Waiting for stepper return', end='\r')
        msg_flag_Stepper = 0
        time.sleep(1)


        print(f'Current Step is {Current_Stepper_Position}')
        if have_been_zerod == 0:
            print('Status: Not zerod')
        else:
            print('Status: Zerod')
    else:


        Stepper_setpoint = inp
        # Send message
        try:
            # publishing setpoint as string
            msg = str(Stepper_setpoint)
            pubMsg = client.publish(
                topic='rpi/broadcast',
                payload=msg.encode('utf-8'),
                qos=0,
            )
            pubMsg.wait_for_publish()
            print(pubMsg.is_published())
        except Exception as e:
            print(e)

        while msg_flag_Stepper != 1:
            print('Waiting for stepper return', end='\r')
        msg_flag_Stepper = 0
        time.sleep(3)


    

