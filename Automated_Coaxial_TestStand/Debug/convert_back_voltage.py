import numpy as np
import pandas as pd

data = pd.read_csv("/home/ubuntu/src/waveshare_a2d_d2a_python/optimization_data/zeva_75_14in_cw.csv")

MLP_calibrated_fits = pd.read_csv('/home/ubuntu/src/waveshare_a2d_d2a_python/experiment/MLP_calibrated_fits.csv')
Left_msrd_to_theo_slope = float(MLP_calibrated_fits.iloc[1,1])
Left_msrd_to_theo_intcpt = float(MLP_calibrated_fits.iloc[1,2])
Right_msrd_to_theo_slope = float(MLP_calibrated_fits.iloc[1,3])
Right_msrd_to_theo_intcpt = float(MLP_calibrated_fits.iloc[1,4])
Left_mV_to_g_slope = float(MLP_calibrated_fits.iloc[1,5])
Right_mv_to_g_slope = float(MLP_calibrated_fits.iloc[1,6])
Thrust_mV_to_g_slope = 0.0004365

Left_mv_to_g_intcpt = df.iloc[1:31,1]
Left_mv_to_g_intcpt = Left_mv_to_g_intcpt.to_numpy()
Left_mv_to_g_intcpt = np.sum(Left_mv_to_g_intcpt)/len(Left_mv_to_g_intcpt)

Right_mv_to_g_intcpt = df.iloc[1:31,2]
Right_mv_to_g_intcpt = Right_mv_to_g_intcpt.to_numpy()
Right_mv_to_g_intcpt = np.sum(Right_mv_to_g_intcpt)/len(Right_mv_to_g_intcpt)

Thrust_mv_to_g_intcpt = df.iloc[1:31,3] 
Thrust_mv_to_g_intcpt = Thrust_mv_to_g_intcpt.to_numpy()
Thrust_mv_to_g_intcpt = np.sum(Thrust_mv_to_g_intcpt)/len(Thrust_mv_to_g_intcpt)

df['Left Load Cell'] = df['Left Load Cell'].apply(lambda x: (x - Left_mv_to_g_intcpt)/Left_mV_to_g_slope )
df['Right Load Cell'] = df['Right Load Cell'].apply(lambda x: (x-Right_mv_to_g_intcpt)/Right_mv_to_g_slope)

df['Left Load Cell'] = df['Left Load Cell'].apply(lambda x: (x*Left_msrd_to_theo_slope + Left_msrd_to_theo_intcpt))
df['Right Load Cell'] = df['Right Load Cell'].apply(lambda x: (x*Right_msrd_to_theo_slope+Right_msrd_to_theo_intcpt)) 

df['Thrust'] = df['Thrust'].apply(lambda x: (x-Thrust_mv_to_g_intcpt)*0.001/Thrust_mV_to_g_slope)  

Torque = 16.75*(df['Left Load Cell'] + df['Right Load Cell'])*0.0098*10**(-3)