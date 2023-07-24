from pigpio import pi
x_pi = pi('')

h = x_pi.i2c_open(1, 118)
x_pi.i2c_close(h)
print(h)
if h > 30:
    x = h - 1
    while x >= 0:
        x_pi.i2c_close(x)
        x = x - 1
    print(str(h) + ' pigpio handles closed')
x_pi.stop()