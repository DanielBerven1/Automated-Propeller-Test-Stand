

MQTT_TOPIC_IMU = 'IMU'


def on_connect(client, userdata, flags, rc):
    print(‘Connected with result code ‘ + str(rc))
    client.subscribe(MQTT_TOPIC_IMU)


def on_message(client, userdata, msg):
    print(msg.topic + '' + str(msg.payload))
    message = str(msg.payload)
    pitch_angle = float(message)
    
    

 

def main():
    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    mqtt_client.connect(MQTT_ADDRESS, 1883)
    mqtt_client.loop_forever()

 

if __name__ == ‘__main__’:
    print(‘MQTT to InfluxDB bridge’)
    main()