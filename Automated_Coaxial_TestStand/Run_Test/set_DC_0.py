import numpy as np

import paho.mqtt.client as mqtt
MQTT_TOPIC_DC_POSITION = 'DC_Position'

def on_connect(client, userdata, flags, rc):
    print('Connected with result code ' + str(rc))
    client.subscribe(MQTT_TOPIC_DC_POSITION)

def on_publish(client, userdata, mid):
    print("message published")


client = mqtt.Client("rpi_client2") #this name should be unique
client.username_pw_set('ubuntu','asimov7749')

client.on_publish = on_publish
client.on_connect = on_connect


client.connect('10.105.76.221',1883)
# start a new thread
client.loop_start()




setpoint = 0
try:
    # publishing setpoint as string
    msg = str(setpoint)
    pubMsg = client.publish(
        topic='rpi/PitchSetpoint',
        payload=msg.encode('utf-8'),
        qos=0,
    )
    pubMsg.wait_for_publish()
    print(pubMsg.is_published())
except Exception as e:
    print(e)
# Format CSV and exit script

client.disconnect