import Uncertainty_and_Plotting as up

print ("Enter Propeller Type:")
prop_type = input() 

print ("Enter Prop Diameter:")
prop_diameter = input()

print ("cw or ccw:")
prop_direction = input()

print("Max RPM: ")
max_RPM = float(input())


up.uncertainty_formatter(f'{prop_type}_{prop_diameter}_{prop_direction}', max_RPM)

up.VP_prop_data_plotter(f'{prop_type}_{prop_diameter}_{prop_direction}')

up.VP_Surface_Plotting(f'{prop_type}_{prop_diameter}_{prop_direction}')

