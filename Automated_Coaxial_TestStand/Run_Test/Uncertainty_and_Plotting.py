
from cProfile import label
from cmath import nan
from turtle import color


import numpy as np
import pandas as pd
from math import pi, sqrt
import csv
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib as mpl
from matplotlib.ticker import LinearLocator
import statistics as st
from scipy import stats


plt.rcParams['text.usetex'] = True
mpl.rcParams.update({'pgf.preamble': r'\usepackage{amsmath}'})
plt.style.use('my_style.txt')
fig_x=6.5; fig_y=3

path_csv = r'/home/ubuntu/src/Co_Axial_Stand/experiment/csv'
path_plot = r'/home/ubuntu/src/Co_Axial_Stand/experiment/Plots'




def uncertainty_formatter(file_name, max_speed):
    
    # Read in user input data file
    data = pd.read_csv(path_csv + f'/{file_name}.csv')
    
    
    ####### Original sequence ########### 
    pitch = data["Pitch"].to_numpy()
    speed = data['RPM'].to_numpy()
    thrust = data['Thrust (kgf)'].to_numpy()
    left_load_cell = data['Left Load Cell (g)'].to_numpy()
    right_load_cell = data['Right Load Cell (g)'].to_numpy()
    torque = data['Torque (Nm)'].to_numpy()
    
    ###### Drop zero RPM data ##########
    index = np.where(speed == 0)
    index = index[0]

    pitch = np.delete(pitch, index, 0)
    speed = np.delete(speed, index, 0)
    thrust = np.delete(thrust, index, 0)
    left_load_cell = np.delete(left_load_cell, index, 0)
    right_load_cell = np.delete(right_load_cell, index, 0)
    torque = np.delete(torque, index, 0)

    
    ###### Drop RPM's above maximum speed tested ########
    index = np.where(speed > (max_speed-50))
    index = index[0]

    pitch = np.delete(pitch, index, 0)
    speed = np.delete(speed, index, 0)
    thrust = np.delete(thrust, index, 0)
    left_load_cell = np.delete(left_load_cell, index, 0)
    right_load_cell = np.delete(right_load_cell, index, 0)
    torque = np.delete(torque, index, 0)

    ####### Calculate speed transistion indicies ########

    # diffference array from successive speed measurements
    diff = np.zeros_like(speed)
    for i in range(len(speed)-1):
        diff[i] = speed[i+1] - speed[i]

    # find indicies where successive difference has a maginitude greater than 100
    speed_index = np.where(np.abs(diff) > 100)
    speed_index = speed_index[0]

    # array for finding where rpm magnitdues are changing by 100 successivly
    # These are periods where ESC value has been incremented but the motor has not
    # reached steady state
    drop = np.zeros_like(speed_index)
    for i in range(len(speed_index)-1):
        drop[i] = speed_index[i+1] - speed_index[i]
        
    # if drop == 1, than the previous measurent is 100 less then current
    drop_index = np.where(drop == 1)
    drop_index = drop_index[0]

    # Convert drop index back to the speed index
    index = speed_index[drop_index+1]   

    # Remove unsteady measurment regions
    pitch = np.delete(pitch, index, 0)
    speed = np.delete(speed, index, 0)
    thrust = np.delete(thrust, index, 0)
    left_load_cell = np.delete(left_load_cell, index, 0)
    right_load_cell = np.delete(right_load_cell, index, 0)
    torque = np.delete(torque, index, 0)

    # recalculate speed transistion indicies with new data set
    for i in range(len(speed)-1):
        diff[i] = speed[i+1] - speed[i]

    # Find transistions with magnitude greater than 500
    speed_index = np.where(np.abs(diff) > 500)
    index = speed_index[0]

    # Delete these samples because they are leftover unsteady periods
    pitch = np.delete(pitch, index, 0)
    speed = np.delete(speed, index, 0)
    thrust = np.delete(thrust, index, 0)
    left_load_cell = np.delete(left_load_cell, index, 0)
    right_load_cell = np.delete(right_load_cell, index, 0)
    torque = np.delete(torque, index, 0)

    # Recalculate transistion indicies 
    diff = np.zeros_like(speed)
    for i in range(len(speed)-1):
        diff[i] = speed[i+1] - speed[i]

    # Find transistions with magintide greather than 100
    speed_index = np.where(np.abs(diff) > 100)
    index = speed_index[0]

    # Add the last index manually because it has no transistion 
    index = np.append(index, len(speed))

    ############ Average and Uncertainty Calculations ###############
    
    # Create matrix to store averages and uncertainties
    uncertainty = np.zeros((len(index),7))
    
    # initial transistion from 0 RPM to initial motor speed 
    
    # mean pitch, should be the pitch tested, if not, there is a transistion index error
    mean_pitch = np.mean(pitch[0:index[0]])

    t = stats.t.ppf(0.95, len(speed[0:index[0]])) # t-distribution for 95% based on number of samples (transistion index)

    # mean speed and random uncertainty
    mean_speed = np.mean(speed[0:index[0]])
    stdev_speed = t*st.stdev(speed[0:index[0]])/sqrt(index[0])

    # mean thrust and random uncertainty
    mean_thrust = np.mean(thrust[0:index[0]])
    stdev_thrust = t*st.stdev(thrust[0:index[0]])/sqrt(index[0])

    # mean left load cell value and random uncerainty
    mean_left_load_cell = np.mean(left_load_cell[0:index[0]])
    stdev_left_load_cell = t*st.stdev(left_load_cell[0:index[0]])/sqrt(index[0])

    # mean right load cell value and random uncertainty
    mean_right_load_cell = np.mean(right_load_cell[0:index[0]])
    stdev_right_load_cell = t*st.stdev(right_load_cell[0:index[0]])/sqrt(index[0])

    # propagated mean and random uncerainty for torque 
    mean_torque = 16.25*(mean_left_load_cell + mean_right_load_cell)*0.0098*10**(-3)
    stdev_torque = t*sqrt((16.25*(mean_right_load_cell)*0.0098*10**(-3)*(stdev_left_load_cell*0.0098))**2
                        + (16.25*(mean_left_load_cell)*0.0098*10**(-3)*(stdev_right_load_cell*0.0098))**2)

    # mean_torque = np.mean(torque[0:index[0]])
    # stdev_torque = t*st.stdev(torque[0:index[0]])/sqrt(index[0])

    # Store first transistion in uncertainty matrix
    uncertainty[0, :] = [mean_pitch, mean_speed, stdev_speed, mean_thrust, stdev_thrust, mean_torque, stdev_torque]

    # Repeat process for rest of transistions 
    for i in range(len(index)-1):
        
        t = stats.t.ppf(0.95, len(speed[index[i]+1:index[i+1]]))

        mean_pitch = np.mean(pitch[index[i]+1:index[i+1]])      
        
        mean_speed = np.mean(speed[index[i]+1:index[i+1]])
        stdev_speed = t*st.stdev(speed[index[i]+1:index[i+1]])/sqrt(index[i+1]-index[i]+1)
        
        mean_thrust = np.mean(thrust[index[i]+1:index[i+1]])
        stdev_thrust = t*st.stdev(thrust[index[i]+1:index[i+1]])/sqrt(index[i+1]-index[i]+1)
        
        mean_left_load_cell = np.mean(left_load_cell[index[i]+1:index[i+1]])
        stdev_left_load_cell = st.stdev(left_load_cell[index[i]+1:index[i+1]])/sqrt(index[i+1]-index[i]+1)
        
        mean_right_load_cell = np.mean(right_load_cell[index[i]+1:index[i+1]])
        stdev_right_load_cell = st.stdev(right_load_cell[index[i]+1:index[i+1]])/sqrt(index[i+1]-index[i]+1)

        mean_torque = 16.25*(mean_left_load_cell + mean_right_load_cell)*0.0098*10**(-3)
        stdev_torque = t*sqrt((16.25*(mean_right_load_cell)*0.0098*10**(-3)*(stdev_left_load_cell*0.0098))**2 + (16.25*(mean_left_load_cell)*0.0098*10**(-3)*(stdev_right_load_cell*0.0098))**2)

        # mean_torque = np.mean(torque[index[i]+1:index[i+1]])
        # stdev_torque = t*st.stdev(torque[index[i]+1:index[i+1]])/sqrt(index[i+1]-index[i]+1)
    
        uncertainty[i+1, :] = [mean_pitch, mean_speed, stdev_speed, mean_thrust, stdev_thrust, mean_torque, stdev_torque]
        

    # Store values in dataframe and save as csv
    df=pd.DataFrame(data=uncertainty, columns= ['Pitch (deg)','Motor Speed (RPM)','Motor Speed stdv','Thrust (kgf)','Thrust stdv','Torque (Nm)','Torque stdv'])
    df.to_csv(path_csv + f'/{file_name}_averaged.csv', sep=',',encoding='utf-8')

def VP_prop_data_plotter(file_name):
    
    # Read in user input data file
    data = pd.read_csv(path_csv + f'/{file_name}_averaged.csv')

    # Original data sequence
    pitch = data['Pitch (deg)'].to_numpy()
    speed = data['Motor Speed (RPM)'].to_numpy()
    speed_stdv = data['Motor Speed stdv'].to_numpy()
    thrust = data['Thrust (kgf)'].to_numpy()
    thrust_stdv = data['Thrust stdv'].to_numpy()
    torque = data['Torque (Nm)'].to_numpy()
    torque_stdv = data['Torque stdv'].to_numpy()

    ##### Determine pitch transistions #####
    pitch_diff = np.zeros_like(pitch)
    for i in range(len(pitch_diff)-1):
        pitch_diff[i+1] = pitch[i+1] - pitch[i]

    index = np.where(pitch_diff != 0)
    index = index[0]

    ##### Determine the pitches tested #####
    tested_pitches = np.zeros(len(index)+1)
    tested_pitches[0] = pitch[index[0]-1]
    for i in range(len(index)):
        tested_pitches[i+1] = pitch[index[i]]
    
    ##### Sort in decsencing order ###### (pitches tested in ascending order)
    tested_pitches = -np.sort(-tested_pitches)

    fig = plt.figure(figsize=(fig_x,fig_y))
    ax = fig.add_subplot()
    ax.set_xlabel(r"Motor Speed (RPM)", fontsize = 12)
    ax.set_ylabel(r"Thrust $(kgf)$", fontsize = 12)

    elw = 0.5

    color = cm.rainbow(np.linspace(0, 1, len(tested_pitches)))

    for i in range(len(tested_pitches)):
        index = np.where(pitch == tested_pitches[i])
        
        plot_speed = speed[index]
        plot_speed_uncert = speed_stdv[index]
        plot_thrust = thrust[index]
        plot_thrust_uncert = thrust_stdv[index]
        
        ax.errorbar(plot_speed, plot_thrust, plot_thrust_uncert, plot_speed_uncert,  
                    elinewidth=elw, markersize= 1, marker='v', linewidth = 0.3, c=color[i], linestyle = (0, (5, 10)), label = f'{tested_pitches[i]} $^{{\\circ}}$')
        
    plt.legend()  

    plt.savefig(path_plot + f'/{file_name}_Thrust.png', dpi='figure', format='png',
        bbox_inches="tight", pad_inches=0.1,
            facecolor='auto', edgecolor='auto')


    
    


    fig = plt.figure(figsize=(fig_x,fig_y))
    ax = fig.add_subplot()
    ax.set_xlabel(r"Motor Speed (RPM)", fontsize = 12)
    ax.set_ylabel(r"Torque $(Nm)$", fontsize = 12)

    for i in range(len(tested_pitches)):
        index = np.where(pitch == tested_pitches[i])
        
        plot_speed = speed[index]
        plot_speed_uncert = speed_stdv[index]
        plot_torque = torque[index]
        plot_torque_uncert = torque_stdv[index]
        
        ax.errorbar(plot_speed, plot_torque, plot_torque_uncert, plot_speed_uncert,  
                    elinewidth=elw, markersize= 1, marker='v', linewidth = 0.3, c=color[i], linestyle = (0, (5, 10)), label = f'{tested_pitches[i]} $^{{\\circ}}$')
        
    plt.legend()  

    plt.savefig(path_plot + f'/{file_name}_Torque.png', dpi='figure', format='png',
            bbox_inches="tight", pad_inches=0.1,
            facecolor='auto', edgecolor='auto')

    plt.show()

def VP_Surface_Plotting(file_name):

    # Read in original data
    data = pd.read_csv(path_csv + f'/{file_name}_averaged.csv')
    data = data.drop('Unnamed: 0', axis=1)
    pitch = data["Pitch (deg)"].to_numpy()
    speed = data['Motor Speed (RPM)'].to_numpy()
    thrust = data['Thrust (kgf)'].to_numpy()
    torque = data['Torque (Nm)'].to_numpy()

    data_new = data.to_numpy()



    ####### Determine pitches tested ########

    pitch_diff = np.zeros_like(pitch)
    for i in range(len(pitch_diff)-1):
        pitch_diff[i+1] = pitch[i+1] - pitch[i]

    index = np.where(pitch_diff != 0)
    index = index[0]

    tested_pitches = np.zeros(len(index)+1)

    tested_pitches[0] = pitch[index[0]-1]
    for i in range(len(index)):
        tested_pitches[i+1] = pitch[index[i]]
        


    #########  Make # of samples the same for each pitch  ############

    num_samples = np.zeros_like(tested_pitches)
    for i in range(len(tested_pitches)):
        
        index = np.where(pitch == tested_pitches[i])
        index = index[0]
        
        num_samples[i] = len(index)
        
    min_samples_tested = np.min(num_samples)
    x = np.where(num_samples > min_samples_tested)
    x = x[0]
    index_to_delete = np.zeros(len(x), dtype=int)

    j=0
    for i in range(len(tested_pitches)):
        
        if num_samples[i] > min_samples_tested:
            index = np.where(pitch == tested_pitches[i])
            index = index[0]
        
            index_to_delete[j] = index[0]
            
            j +=1

    # if a pitch set has a higher number of RPMs tested,
    # remove the minimum
    data_new = np.delete(data_new, index_to_delete, 0)
    pitch = data_new[:,0]
    speed = data_new[:,1]
    thrust = data_new[:,3]
    torque = data_new[:,5]

    min_samples_tested = int(min_samples_tested)

    PITCH = np.zeros((min_samples_tested, len(tested_pitches)))
    SPEED = np.zeros_like(PITCH)
    THRUST = np.zeros_like(PITCH)
    TORQUE = np.zeros_like(PITCH)

    for i in range(len(tested_pitches)):
        PITCH[:,i] = tested_pitches[i]
        SPEED[:,i] = speed[i*min_samples_tested:(i+1)*min_samples_tested]
        THRUST[:,i] = thrust[i*min_samples_tested:(i+1)*min_samples_tested]
        TORQUE[:,i] = torque[i*min_samples_tested:(i+1)*min_samples_tested]


    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    surf = ax.plot_surface(PITCH,SPEED, THRUST, cmap=cm.coolwarm,
                        linewidth=0, antialiased=False)
    # Customize the z axis.
    #ax.set_zlim(-1.01, 1.01)
    ax.zaxis.set_major_locator(LinearLocator(10))
    # A StrMethodFormatter is used automatically
    ax.zaxis.set_major_formatter('{x:.02f}')
    # Add a color bar which maps values to colors.
    ax.set_xlabel(r"Angle of attack (deg)")
    ax.set_ylabel(r"Motor Speed (RPM)")
    ax.set_zlabel(r'Thrust (kgf)')
    fig.colorbar(surf, shrink=0.5, aspect=5, label = 'Thrust ($kgf$)')

    plt.savefig(path_plot + f'/{file_name}_Thrust_Surface.png', dpi='figure', format='png',
    bbox_inches="tight", pad_inches=0.1,
    facecolor='auto', edgecolor='auto')


        
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    surf = ax.plot_surface(PITCH,SPEED, TORQUE, cmap=cm.coolwarm,
                        linewidth=0, antialiased=False)
    # Customize the z axis.
    #ax.set_zlim(-1.01, 1.01)
    ax.zaxis.set_major_locator(LinearLocator(5))
    # A StrMethodFormatter is used automatically
    ax.zaxis.set_major_formatter('{x:.03f}')
    # Add a color bar which maps values to colors.
    ax.set_xlabel(r"Angle of attack (deg)")
    ax.set_ylabel(r"Motor Speed (RPM)")
    ax.set_zlabel(r'Torque $Nm$')
    fig.colorbar(surf, shrink=0.5, aspect=5, label = 'Torque ($Nm$)')

    plt.savefig(path_plot + f'/{file_name}_Torque_Surface.png', dpi='figure', format='png',
        bbox_inches="tight", pad_inches=0.1,
        facecolor='auto', edgecolor='auto')



    plt.show()
