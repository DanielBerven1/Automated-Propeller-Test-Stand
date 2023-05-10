import pigpio

pi = pigpio.pi()

while True:
    pi.write(17, 1)