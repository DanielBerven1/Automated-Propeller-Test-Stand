import numpy as np
from math import sqrt
import pandas as pd

path = r'/home/ubuntu/src/Co_Axial_Stand/Calibration'

pitch_calibration_fits = pd.read_csv(path + f'/Pitch_Calibration.csv')
pitch_calibration_fits = pitch_calibration_fits.to_numpy()
pitch_calibration_fits = pitch_calibration_fits[1,:].astype(np.float64)
pitch_fit_a = pitch_calibration_fits[1]
pitch_fit_b = pitch_calibration_fits[2]
pitch_fit_c = pitch_calibration_fits[3]

def Pitch_to_DCPosition(y, a, b, c):
    return (-b + sqrt(b**2 - (4*a*(c-y))))/(2*a)


step = 1.5

pitch_setpoints_BtF = np.arange(-15, 15 + step, step)
pitch_setpoints_FtB = np.arange(15-step, -15, -step)

pitch_setpoints = np.concatenate((pitch_setpoints_BtF,pitch_setpoints_FtB), axis = 0)

calculated_DC_setpoints = np.zeros_like(pitch_setpoints, dtype=float)

for i in range(len(pitch_setpoints)):
    calculated_DC_setpoints[i] = Pitch_to_DCPosition(pitch_setpoints[i], pitch_fit_a, pitch_fit_b,
    pitch_fit_c)

print(calculated_DC_setpoints)