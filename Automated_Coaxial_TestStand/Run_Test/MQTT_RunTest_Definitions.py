import time
import paho.mqtt.client as mqtt

pitch_angle = 0 


def setup_test_client():
    MQTT_TOPIC_DC_POSITION = 'DC_Position'
    MQTT_TOPIC_DC_ZERO = 'DC_Zero'
    MQTT_TOPIC_ZERO_SWITCH = 'Zero_Switch'


    def on_connect(client, userdata, flags, rc):
        print('Connected with result code ' + str(rc))
        client.subscribe(MQTT_TOPIC_DC_POSITION)
        client.subscribe(MQTT_TOPIC_DC_ZERO)
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

    
