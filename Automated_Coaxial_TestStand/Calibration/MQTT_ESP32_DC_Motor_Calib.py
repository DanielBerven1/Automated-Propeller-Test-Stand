#############################################
# Subscribed Topics:    IMU_1
#                       IMU_2
#                       DC_Position
#                       DC_Zero

# Published Topics      DCSetpoint
#                       SetDCZero
#############################################


import time
import paho.mqtt.client as mqtt
import MQTT_Definitions as mqtt_com
import numpy as np
import pandas as pd

import scipy as scipy
from scipy import stats

from scipy.optimize import curve_fit

import matplotlib.pyplot as plt
import matplotlib as mpl

print("Which motor is being calibrated? (F) or (B)")
stand = input()

path = r'/home/ubuntu/src/Co_Axial_Stand/Calibration'

plt.rcParams['text.usetex'] = True
mpl.rcParams.update({'pgf.preamble': r'\usepackage{amsmath}'})
plt.style.use('my_style.txt')
fig_x=6.5; fig_y=3


# setup client
client = mqtt_com.setup_DC_client()

global msg_flag_DC_position
msg_flag_DC_position = 0

global msg_flag_angle_1
msg_flag_angle_1 = 0

global msg_flag_angle_2
msg_flag_angle_2 = 0

global msg_flag_DC_Zero
msg_flag_DC_zero = 0

global setup_flag
setup_flag = 1   

global pitch_angle_1
pitch_angle_1 = 0
global pitch_angle_2
pitch_angle_2 = 0 
global DC_position
DC_position = 0 
global DC_Zero_State
DC_Zero_State = 0

def on_message(client, userdata, msg):
    # print(msg.topic + '' + str(msg.payload))

    global setup_flag
    if msg.topic == "IMU1":

        message = str(msg.payload.decode("utf-8"))
        global pitch_angle_1
        pitch_angle_1 = float(message)
        # print(pitch_angle)
        global msg_flag_angle_1
        msg_flag_angle_1 = 1   
    
    if msg.topic == "IMU2":

        message = str(msg.payload.decode("utf-8"))
        global pitch_angle_2
        pitch_angle_2 = float(message)
        # print(pitch_angle)
        global msg_flag_angle_2
        msg_flag_angle_2 = 1   


    if msg.topic == "DC_Position":
        message = str(msg.payload.decode("utf-8"))
        global DC_position
        DC_position = float(message)
        # print(position)
        global msg_flag_DC_position
        msg_flag_DC_position = 1  
    
    if msg.topic == "DC_Zero":
        message = str(msg.payload.decode("utf-8"))
        global DC_Zero_State
        DC_Zero_State = float(message)
        global msg_flag_DC_Zero
        msg_flag_DC_Zero = 1  



    if setup_flag == 1:
        setup_flag = 0

client.on_message = on_message 

try:    
# publishing setpoint as string
    msg = str("")
    pubMsg = client.publish(
        topic='rpi/GetData',
        payload=msg.encode('utf-8'),
        qos=0,
    )
    pubMsg.wait_for_publish()
    print(pubMsg.is_published())
except Exception as e:
    print(e)

time.sleep(2)

while msg_flag_angle_1 == 0 and msg_flag_angle_2 == 0 and msg_flag_DC_position == 0 and setup_flag == 1:
    print('waiting for initial positions', end='\r')

msg_flag_DC_position = 0
msg_flag_angle_1 = 0
msg_flag_angle_2 = 0


print(f'Current DC Motor Position is: {DC_position}')
print(f'Current IMU_1 reading is {pitch_angle_1}')
print(f'Current IMU_2 reading is {pitch_angle_2}')
if DC_Zero_State == 1:
    print("DC is zero'd")
else: 
    print("DC is not zero'd")
print('------------')

########################################################


print("Zeroing VP System, press Enter")

inp = input()
try:    
# publishing setpoint as string
    msg = str("")
    pubMsg = client.publish(
        topic='rpi/SetDCZero',
        payload=msg.encode('utf-8'),
        qos=0,
    )
    pubMsg.wait_for_publish()
    print(pubMsg.is_published())
except Exception as e:
    print(e)

time.sleep(2)




setpoints = np.arange(0, -9000-500, -500)

for i in range(len(setpoints)):
    if DC_Zero_State == 1:
            break
    try:
        # publishing setpoint as string
        msg = str(setpoints[i])
        pubMsg = client.publish(
            topic='rpi/DCSetpoint',
            payload=msg.encode('utf-8'),
            qos=0,
        )
        pubMsg.wait_for_publish()
        print(pubMsg.is_published())
    except Exception as e:
        print(e)

    while msg_flag_DC_position == 0 and msg_flag_angle_1 == 0 and msg_flag_angle_2 == 0:
        print('waiting for inputs', end = '\r')
    print("\n")
    time.sleep(4)
    msg_flag_DC_position = 0
    msg_flag_angle_1 = 0
    msg_flag_angle_2 = 0
    msg_flag_DC_Zero = 0

    

print("DC Motor Zero'd")

time.sleep(2)
min_pitch_1 = pitch_angle_1
min_pitch_2 = pitch_angle_2
min_pitch_avg = (min_pitch_1+min_pitch_2)/2
min_DC_position = DC_position

print("Zero'd values:")
print(min_pitch_1)
print(min_pitch_2)
print(min_DC_position)

df = [['Min_DC_Position', 'Min_Angle', 'Min_Angle_1', 'Min_Angle_2']]
df.append([min_DC_position, (min_pitch_1+min_pitch_2)/2, min_pitch_1, min_pitch_2])

df = pd.DataFrame(df)

df.to_csv(path + f'/VP_Zero_{stand}.csv', encoding = 'utf-8')

#################################################

print("Calibration of VP System, Press Enter")

inp = input()

zero_setpoint = 0 
end_setpoint = 6000
increment_setpoint = 180

setpoints = np.arange(zero_setpoint, end_setpoint, increment_setpoint)

print(setpoints)

print('SAFETY CHECK: This okay? (y) or (any)')
inp = input()

if inp == 'y':
    pass
else:
    quit()

motor_positions = np.zeros_like(setpoints, dtype=float)
IMU_angles_1 = np.zeros_like(setpoints, dtype=float)
IMU_angles_2 = np.zeros_like(setpoints, dtype=float)


i = 0
first_time = True
while True:

    if first_time == True: 
        setpoint = setpoints[i] 
        try:
            # publishing setpoint as string
            msg = str(setpoint)
            pubMsg = client.publish(
                topic='rpi/DCSetpoint',
                payload=msg.encode('utf-8'),
                qos=0,
            )
            pubMsg.wait_for_publish()
            print(pubMsg.is_published())
        except Exception as e:
            print(e)
        first_time = False
        time.sleep(3)

    
    
    if msg_flag_DC_position == 1 and msg_flag_angle_1 == 1 and msg_flag_angle_2 == 1:
        
        print('message recieved')
        time.sleep(1)
        IMU_angles_1[i] = pitch_angle_1
        IMU_angles_2[i] = pitch_angle_2
        motor_positions[i] = DC_position 
        i += 1

        msg_flag_DC_position = 0
        msg_flag_angle_1 = 0
        msg_flag_angle_2 = 0

        if i == len(setpoints):
            break

        setpoint = setpoints[i]
        try:
            # publishing setpoint as string
            msg = str(setpoint)
            pubMsg = client.publish(
                topic='rpi/DCSetpoint',
                payload=msg.encode('utf-8'),
                qos=0,
            )
            pubMsg.wait_for_publish()
            print(pubMsg.is_published())
        except Exception as e:
            print(e)
        time.sleep(3)

        
#######################################################

print("Returning to DC start up position")

setpoint = 0
try:
    # publishing setpoint as string
    msg = str(setpoint)
    pubMsg = client.publish(
        topic='rpi/DCSetpoint',
        payload=msg.encode('utf-8'),
        qos=0,
    )
    pubMsg.wait_for_publish()
    print(pubMsg.is_published())
except Exception as e:
    print(e)



while msg_flag_DC_position != 1 and msg_flag_angle_1 != 1 and msg_flag_angle_2 != 1:
    print('waiting to return home...', end='\r')

msg_flag_DC_position = 0
msg_flag_angle_1 = 0
msg_flag_angle_2 = 0
print("\n")

time.sleep(1)

print(DC_position)
print(f'Zero angle_1 error{min_pitch_1 - pitch_angle_1}')
print(f'Zero angle_2 error{min_pitch_2 - pitch_angle_2}')




############################################################


# result_1 = scipy.stats.linregress(motor_positions, IMU_angles_1, alternative = 'two-sided')
# result_2 = scipy.stats.linregress(motor_positions, IMU_angles_2, alternative='two-sided')

# fit_IMU_angles_1 = result_1.slope*motor_positions + result_2.intercept
# fit_IMU_angles_2 = result_2.slope*motor_positions + result_2.intercept


def func(x, a, b, c):
    return a*x**2 + b*x + c

popt_1, pconv_1 = curve_fit(func, motor_positions, IMU_angles_1)
popt_2, pconv_2 = curve_fit(func, motor_positions, IMU_angles_2)

fit_IMU_angles_1 = func(motor_positions, popt_1[0], popt_1[1], popt_1[2])
fit_IMU_angles_2 = func(motor_positions, popt_2[0], popt_2[1], popt_2[2])


############################################################

# Figure and axes setup
fig_1 = plt.figure(figsize=(fig_x,fig_y))
ax_1 = fig_1.add_subplot()
ax_1.set_title("Pitch Calibration")
ax_1.set_xlabel(r"DC Motor Position (deg)", fontsize = 12)
ax_1.set_ylabel(r"Pitch Angle (deg)", fontsize = 12)        

# Scatter plot of tested points
line_1 = ax_1.scatter(motor_positions, IMU_angles_1, c='blue')
line_2 = ax_1.scatter(motor_positions, IMU_angles_2, c='red')

# Line plot with calibration fits
line_3 = ax_1.plot(motor_positions, fit_IMU_angles_1, c="blue", linestyle ='--')
line_4 = ax_1.plot(motor_positions, fit_IMU_angles_2, c="red", linestyle = '--')

# # Calibration fit value formatting for figure
# formatted_result_slope_1 = '{:.3E}'.format(result_1.slope)
# formatted_result_intercept_1 = round(result_1.intercept, 4)
# formatted_result_rvalue_1 = round(result_1.rvalue, 4)

# formatted_result_slope_2 = '{:.3E}'.format(result_2.slope)
# formatted_result_intercept_2 = round(result_2.intercept, 4)
# formatted_result_rvalue_2 = round(result_2.rvalue, 4)


# Calibration fit value formatting for figure
formatted_result_a_1 = '{:.3E}'.format(popt_1[0])
formatted_result_b_1 = '{:.3E}'.format(popt_1[1])
formatted_result_c_1 = round(popt_1[2],2)

formatted_result_a_2 = '{:.3E}'.format(popt_2[0])
formatted_result_b_2 = '{:.3E}'.format(popt_2[1])
formatted_result_c_2 = round(popt_2[2],2)




# # Add arrow annotations
# ax_1.annotate(
#         f'm_1={formatted_result_slope_1} \nb_1={formatted_result_intercept_1} \nR_1={formatted_result_rvalue_1}',
#         xy=(motor_positions[4], fit_IMU_angles_1[4]), xycoords='data',
#         xytext=(-25, 30), textcoords='offset points',
#         arrowprops=dict(arrowstyle="->"))

# ax_1.annotate(
#         f'm_2={formatted_result_slope_2} \nb_2={formatted_result_intercept_2} \nR_2={formatted_result_rvalue_2}',
#         xy=(motor_positions[4], fit_IMU_angles_2[4]), xycoords='data',
#         xytext=(25, -30), textcoords='offset points',
#         arrowprops=dict(arrowstyle="->"))

# Add arrow annotations
ax_1.annotate(
        f'a_1={formatted_result_a_1} \nb_1={formatted_result_b_1} \nc_1={formatted_result_c_1}',
        xy=(motor_positions[4], fit_IMU_angles_1[4]), xycoords='data',
        xytext=(-25, -30), textcoords='offset points',
        arrowprops=dict(arrowstyle="->"))

ax_1.annotate(
        f'a_2={formatted_result_a_2} \nb_2={formatted_result_b_2} \nc_2={formatted_result_c_2}',
        xy=(motor_positions[4], fit_IMU_angles_2[4]), xycoords='data',
        xytext=(60, -60), textcoords='offset points',
        arrowprops=dict(arrowstyle="->"))


ax_1.set_xlim(np.min(motor_positions), None)
plt.xticks(fontsize = 10)
plt.yticks(fontsize = 10)
plt.savefig(f'Calibration/Pitch_Calibration_{stand}', dpi='figure', format='png',
        bbox_inches="tight", pad_inches=0.1,
        facecolor='auto', edgecolor='auto')

# plt.legend([line_3, line_4],['Prop 1','Prop 2'])

plt.show()

# df = [['Pitch_Slope', 'Pitch_Intercept', 'Pitch_Rvalue']]
# df.append([(result_1.slope+result_2.slope)/2, (result_1.intercept+result_2.intercept)/2, (result_1.rvalue+result_2.rvalue)/2])

df = [['a', 'b', 'c']]
df.append([(popt_1[0] + popt_2[0])/2, (popt_1[1] + popt_2[1])/2, (popt_1[2] + popt_2[2])/2])

df = pd.DataFrame(df)

df.to_csv(path + f'/Pitch_Calibration_{stand}.csv', encoding = 'utf-8')



    
    

 



 

