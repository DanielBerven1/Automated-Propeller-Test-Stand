import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


plt.rcParams['text.usetex'] = True
mpl.rcParams.update({'pgf.preamble': r'\usepackage{amsmath}'})
plt.style.use('my_style.txt')
fig_x=6.5; fig_y=3





def MLP_Calib_Plotting(num, weights, left_MLP_measurements, right_MLP_measurements, left_result_slope, 
    left_result_intercept, left_result_rvalue, right_result_slope, 
    right_result_intercept, right_result_rvalue):

        # generate points for line plot
        min_left = np.min(left_MLP_measurements)
        max_left = np.max(left_MLP_measurements)
        points_left = np.arange(min_left, max_left+0.1*(max_left-min_left), 0.1*(max_left-min_left))

        min_right = np.min(right_MLP_measurements)
        max_right = np.max(right_MLP_measurements)
        points_right = np.arange(min_right, max_right+0.1*(max_right-min_right), 0.1*(max_right-min_right))

        points_left_weights = left_result_slope*points_left + left_result_intercept
        points_right_weights = right_result_slope*points_right + right_result_intercept


        # Figure and axes setup
        fig_1 = plt.figure(figsize=(fig_x,fig_y))
        ax_1 = fig_1.add_subplot()
        ax_1.set_title("MLP Compression Calibration")
        ax_1.set_xlabel(r"MLP Voltage (V)", fontsize = 12)
        ax_1.set_ylabel(r"Weight $(g)$", fontsize = 12)
        
        # Scatter plot of tested points
        line_1 = ax_1.scatter(left_MLP_measurements, weights, c='blue')
        line_2 = ax_1.scatter(right_MLP_measurements, weights, c='red')

        # Line plot with calibration fits
        line_3 = ax_1.plot(points_left, points_left_weights, c="blue", linestyle ='--')
        line_4 = ax_1.plot(points_right, points_right_weights, c='red', linestyle = '--')

        # Calibration fit value formatting for figure
        left_result_slope = '{:.3E}'.format(left_result_slope)
        left_result_intercept = round(left_result_intercept, 4)
        left_result_rvalue = round(left_result_rvalue, 4)

        right_result_slope = '{:.3E}'.format(right_result_slope)
        right_result_intercept = round(right_result_intercept, 4)
        right_result_rvalue = round(right_result_rvalue, 4)

        # Add arrow annotations
        ax_1.annotate(
                f'm_left={left_result_slope} \nb_left={left_result_intercept} \nR_left={left_result_rvalue}',
                xy=(points_left[4], points_left_weights[4]), xycoords='data',
                xytext=(-25, 30), textcoords='offset points',
                arrowprops=dict(arrowstyle="->"))
        
        ax_1.annotate(
                f'm_right={right_result_slope} \nb_right={right_result_intercept} \nR_right={right_result_rvalue}',
                xy=(points_right[4], points_right_weights[4]), xycoords='data',
                xytext=(25, -30), textcoords='offset points',
                arrowprops=dict(arrowstyle="->"))



        ax_1.legend([line_1, line_2],
                ['MLP_Left', 'MLP_Right'], loc='lower right')
        if min_left < min_right:
                ax_1.set_xlim(min_left, None)
        else:
                ax_1.set_xlim(min_right, None)
        ax_1.set_ylim(-50, None)
        plt.xticks(fontsize = 10)
        plt.yticks(fontsize = 10)
        plt.savefig(f'Calibration/Stand{num}_MLP_Compression_Calibration', dpi='figure', format='png',
                bbox_inches="tight", pad_inches=0.1,
                facecolor='auto', edgecolor='auto')
        plt.show()


def Thrust_Calib_Plotting(num, thrust_beam_measurements, weights, thrust_result_slope, thrust_result_intercept, thrust_result_rvalue):


        thrust_fit = thrust_beam_measurements*thrust_result_slope + thrust_result_intercept

        # Figure and axes setup
        fig_1 = plt.figure(figsize=(fig_x,fig_y))
        ax_1 = fig_1.add_subplot()
        ax_1.set_title("Thrust Beam Calibration")
        ax_1.set_xlabel(r"Voltage (V)", fontsize = 12)
        ax_1.set_ylabel(r"Weight $(g)$", fontsize = 12)        
  
        # Scatter plot of tested points
        line_1 = ax_1.scatter(thrust_beam_measurements, weights, c='blue')

        # Line plot with calibration fits
        line_2 = ax_1.plot(thrust_beam_measurements, thrust_fit, c="blue", linestyle ='--')

        # Calibration fit value formatting for figure
        thrust_result_slope = '{:.3E}'.format(thrust_result_slope)
        thrust_result_intercept = round(thrust_result_intercept, 4)
        thrust_result_rvalue = round(thrust_result_rvalue, 4)
        
        # Add arrow annotations
        ax_1.annotate(
                f'm_left={thrust_result_slope} \nb_left={thrust_result_intercept} \nR_left={thrust_result_rvalue}',
                xy=(thrust_beam_measurements[4], weights[4]), xycoords='data',
                xytext=(-25, 30), textcoords='offset points',
                arrowprops=dict(arrowstyle="->"))

        ax_1.set_xlim(np.min(thrust_beam_measurements), None)
        ax_1.set_ylim(-50, None)
        plt.xticks(fontsize = 10)
        plt.yticks(fontsize = 10)
        plt.savefig(f'Calibration/Stand{num}_Thrust_Beam_Calibration', dpi='figure', format='png',
                bbox_inches="tight", pad_inches=0.1,
                facecolor='auto', edgecolor='auto')
        plt.show()





def Measured_to_Theoretical_Plots(num, left_measured_forces, left_theoretical_forces, right_measured_forces, right_theoretical_forces, 
        left_result_slope, left_result_intercept, left_result_rvalue, right_result_slope, right_result_intercept, right_result_rvalue):
        
        ########

        left_fit = left_measured_forces*left_result_slope + left_result_intercept
        right_fit = right_measured_forces*right_result_slope + right_result_intercept

        ######

        fig_1 = plt.figure(figsize=(fig_x,fig_y))
        ax_1 = fig_1.add_subplot()
        ax_1.set_title(r"Torque Force Calibration")
        ax_1.set_xlabel(r"Measured Forces (g)", fontsize=12)
        ax_1.set_ylabel(r"Theoretical Forces (g)", fontsize=12)

        line_1 = ax_1.scatter(left_measured_forces, left_theoretical_forces, c="blue")
        ax_1.plot(left_measured_forces, left_fit, c="blue")
        line_2 = ax_1.scatter(right_measured_forces, right_theoretical_forces, c="red")
        ax_1.plot(right_measured_forces, right_fit, c="red")
        
        #####

        # Calibration fit value formatting for figure
        left_result_slope = round(left_result_slope, 4)
        left_result_intercept = round(left_result_intercept, 4)
        left_result_rvalue = round(left_result_rvalue, 4)

        right_result_slope = round(right_result_slope, 4)
        right_result_intercept = round(right_result_intercept, 4)
        right_result_rvalue = round(right_result_rvalue, 4)

        if num==1:

                # Add arrow annotations
                ax_1.annotate(
                        f'm_left={left_result_slope} \nb_left={left_result_intercept} \nR_left={left_result_rvalue}',
                        xy=(left_measured_forces[4], left_fit[4]), xycoords='data',
                        xytext=(-25, 60), textcoords='offset points',
                        arrowprops=dict(arrowstyle="->"))

                ax_1.annotate(
                        f'm_right={right_result_slope} \nb_right={right_result_intercept} \nR_right={right_result_rvalue}',
                        xy=(right_measured_forces[4], right_fit[4]), xycoords='data',
                        xytext=(-50, 30), textcoords='offset points',
                        arrowprops=dict(arrowstyle="->"))

        else: 

                 # Add arrow annotations
                ax_1.annotate(
                        f'm_left={left_result_slope} \nb_left={left_result_intercept} \nR_left={left_result_rvalue}',
                        xy=(left_measured_forces[4], left_fit[4]), xycoords='data',
                        xytext=(-25, 60), textcoords='offset points',
                        arrowprops=dict(arrowstyle="->"))

                ax_1.annotate(
                        f'm_right={right_result_slope} \nb_right={right_result_intercept} \nR_right={right_result_rvalue}',
                        xy=(right_measured_forces[4], right_fit[4]), xycoords='data',
                        xytext=(50, 60), textcoords='offset points',
                        arrowprops=dict(arrowstyle="->"))               

        #######

        ax_1.legend([line_1, line_2],
                ['MLP_Left', 'MLP_Right'], loc='lower right')

        ax_1.set_ylim(0, None)
        plt.xticks(fontsize = 10)
        plt.yticks(fontsize = 10)

        plt.savefig(f'Calibration/Stand{num}_Torque_Force_Calibration', dpi='figure', format='png',
        bbox_inches="tight", pad_inches=0.1,
        facecolor='auto', edgecolor='auto')
        plt.show()