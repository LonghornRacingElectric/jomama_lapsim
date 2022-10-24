import numpy as np
import pandas as pd
import engine
import engine.magic_moment_method.vehicle_params as vehicles
import engine.magic_moment_method.state_solver.aerodynamics as Aero
import engine.magic_moment_method.state_solver.logger as Fuck_this_Logger
import math
pd.options.mode.chained_assignment = None

# TODO: Move this file into sweeping, gonna have to mess with module imports

# Test Parameters #
#######################################
TEST_MIN = 0
TEST_MAX = 5
divisions = 20

K = 0.06215178719838  # lift-induced drag constant
Cl0 = 0  # lift at 0 induced drag
Cd0 = 0.4414249328442602  # parasitic drag coefficien
refA = 1.2 # reference area, m^2
Cl = (np.linspace(TEST_MIN, TEST_MAX, divisions))
Cd_A = refA * (K * (Cl - Cl0) ** 2 + Cd0)
Cl_A = refA * Cl

#######################################

#GGV Mesh Parameters
sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
            "steered_angle" : (-18 * np.pi / 180, 18 * np.pi / 180),
            "velocity" : (3, 30),
            "torque_request": (-1, 1),
            "is_left_diff_bias": (True, False)}
mesh_size = 13 #Anything below 5 is troublesome, time to generate increases to the forth power of mesh size

#TODO: Get most up to date track (I believe Robert is working on it)
endurance_track = engine.Track("racing_lines/en_mi_2019.csv", 1247.74)
autocross_track = engine.Track("racing_lines/ax_mi_2019.csv", 50.008)
skidpad_times = 5.0497
accel_times = 4.004
Drag_kWh = 0.6611

easy_driver = engine.Racecar(vehicles.Concept2023(motor_directory="engine/magic_moment_method/vehicle_params/Eff228.csv"))
#logger = Fuck_this_Logger.Logger()
#aero = Aero.Aerodynamics(vehicles.Concept2023(motor_directory="engine/magic_moment_method/vehicle_params/Eff228.csv"),logger)

#TODO: Increase results to capture all even times and points in their specific columns, not the entire df
results_df = pd.DataFrame(columns=["ClA","CdA","endurance_points","autocross_points","skidpad_points","accel_points",
        "endurance_time","autocross_time","skidpad_time","accel_time", "drag_energy (kWh)", "Mass Delta (kg)", 
        "Max Long Accel (g)","Max braking Accel (g)", "Max Lat Accel (g)"])

if __name__ == '__main__':
    for i in range(divisions):
        #TODO: Look into altering CL distribution to inflence CoP location
        easy_driver.params.ClA_tot = Cl_A[i]
        easy_driver.params.CdA_tot = Cd_A[i]
        Mass_Delta = 15.770 * 0.45 * (Drag_kWh - 2)
        print(Drag_kWh)
        print(Mass_Delta)

        if Cl_A[i] == 0:
            Initial_mass = easy_driver.params.mass_sprung
            easy_driver.params.mass_sprung += Mass_Delta -18  #Subtracting Mass of Aero Components

        else:
            easy_driver.params.mass_sprung = Initial_mass + Mass_Delta

        print("Testing CL_A: " + str(Cl_A[i]) + " and Cd_A: " + str(Cd_A[i]) )
        easy_driver.regenerate_GGV(sweep_range, mesh_size)
        print("GGV Generated!")
        results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,skidpad_times, accel_times).run()

        df_endurance = results[0]
        df_endurance = df_endurance[["dist",'vel','pos','ax','ay']]
        Total_Laps = math.ceil(22000/df_endurance["pos"].max())

        print(times)
        print(points)

        df_endurance_breaking = df_endurance[df_endurance['ax'] < 0]
        df_endurance_accel = df_endurance[df_endurance['ax'] >= 0]

        results_df.loc[i] = [Cl_A[i], Cd_A[i], points[0], points[1], points[2], points[3], times[0] * Total_Laps,
                     times[1], times[2], times[3], Drag_kWh, easy_driver.params.mass_sprung - Initial_mass, df_endurance_breaking['ax'].max()/9.81
                     ,df_endurance_breaking['ax'].max()/9.81, df_endurance['ay'].max()/9.81]
        
        df_endurance['Drag_J'] = easy_driver.params.CdA_tot * df_endurance['vel'] ** 2 * 1.153 / 2 * df_endurance['dist']
        Drag_kWh = (df_endurance["Drag_J"].sum() / (times[0] * 1000) * (times[0] * Total_Laps)/3600)

    results_df.to_csv("results/aero_sweep-easy_driver.csv")
