import time
import paho.mqtt.client as mqtt
import MQTT_Definitions as mqtt_com

# setup client
client = mqtt_com.setup_client()

# initialize setpoint
setpoint = 0

while True:
    print("Waiting for input...")
    inp = input()


    if inp == "a":
        # Increase setpoint 
        setpoint+= 180
        # Send message
        try:
            # publishing setpoint as string
            msg = str(setpoint)
            pubMsg = client.publish(
                topic='rpi/broadcast',
                payload=msg.encode('utf-8'),
                qos=0,
            )
            pubMsg.wait_for_publish()
            print(pubMsg.is_published())
        except Exception as e:
            print(e)

    if inp == 'z':
        # Decrease setpoint
        setpoint -= 180
        # Send message
        try:
            # publishing setpoint as string
            msg = str(setpoint)
            pubMsg = client.publish(
                topic='rpi/broadcast',
                payload=msg.encode('utf-8'),
                qos=0,
            )
            pubMsg.wait_for_publish()
            print(pubMsg.is_published())
        except Exception as e:
            print(e)
    

