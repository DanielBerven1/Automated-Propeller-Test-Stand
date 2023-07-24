import pigpio
pi = pigpio.pi()
pi.set_servo_pulsewidth(20,0)
pi.set_servo_pulsewidth(25,0)
pi.stop()