import time
import paho.mqtt.client as mqtt

pitch_angle = 0 


def setup_DC_client():
    MQTT_TOPIC_IMU = 'IMU'
    MQTT_TOPIC_POSITION = 'POSITION'

    def on_connect(client, userdata, flags, rc):
        print('Connected with result code ' + str(rc))
        client.subscribe(MQTT_TOPIC_IMU)
        client.subscribe(MQTT_TOPIC_POSITION)


    def on_publish(client, userdata, mid):
        print("message published")


    client = mqtt.Client("rpi_client2") #this name should be unique
    client.username_pw_set('ubuntu','asimov7749')

    client.on_publish = on_publish
    client.on_connect = on_connect


    client.connect('10.105.76.221',1883)
    # start a new thread
    client.loop_start()

    return client

def setup_Stepper_client():

    MQTT_TOPIC_ZERO_SWITCH = 'Zero_Switch'

    def on_connect(client, userdata, flags, rc):
        print('Connected with result code ' + str(rc))

        client.subscribe(MQTT_TOPIC_ZERO_SWITCH)


    def on_publish(client, userdata, mid):
        print("message published")


    client = mqtt.Client("rpi_client2") #this name should be unique
    client.username_pw_set('ubuntu','asimov7749')

    client.on_publish = on_publish
    client.on_connect = on_connect


    client.connect('10.105.76.221',1883)
    # start a new thread
    client.loop_start()

    return client