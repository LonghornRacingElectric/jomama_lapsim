import numpy as np
import pandas as pd
import engine
import engine.magic_moment_method.vehicle_params as vehicles

# TODO: Move this file into sweeping, gonna have to mess with module imports

# Test Parameters #
#######################################
TEST_MIN = 0
TEST_MAX = 4
divisions = 3

K = 0.06215178719838  # lift-induced drag constant
Cl0 = 0  # lift at 0 induced drag
Cd0 = 0.4414249328442602  # parasitic drag coefficien
refA = 1.2 # reference area, m^2
Cl = (np.linspace(TEST_MIN, TEST_MAX, divisions))
Cd_A = refA * (K * (Cl - Cl0) ** 2 + Cd0)
Cl_A = refA * Cl

# TODO: Upgrade this to reflect the new fit from Andrew Zhang, basically Cl will increase energy consumption (kWh) due to drag
#           then the extra kWh will correlate to more mass (Andrews equation). Challenge is to correlate drag and kWh.
#               LapSim should be able to do so given the torque request. If you have questions about it contact Igor Souza
Aero_mass = 0.3745 * (Cl_A ** 2) + 0.0612 * Cl_A + 0.7783
#######################################

#GGV Mesh Parameters
sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
            "steered_angle" : (-18 * np.pi / 180, 18 * np.pi / 180),
            "velocity" : (3, 30),
            "torque_request": (-1, 1),
            "is_left_diff_bias": (True, False)}
mesh_size = 9 #Anything below 5 is troublesome, time to generate increases to the forth power of mesh size

#TODO: Get most up to date track (I believe Robert is working on it)
endurance_track = engine.Track("racing_lines/en_mi_2019.csv", 1247.74)
autocross_track = engine.Track("racing_lines/ax_mi_2019.csv", 50.008)
skidpad_times = 5.0497
accel_times = 4.004

easy_driver = engine.Racecar(vehicles.Concept2023(motor_directory="engine/magic_moment_method/vehicle_params/Eff228.csv"))

#TODO: Increase results to capture all even times and points in their specific columns, not the entire df
results_df = pd.DataFrame(columns=["ClA","CdA","endurance_points","autocross_points","skidpad_points","accel_points",
        "endurance_time","autocross_time","skidpad_time","accel_time", "endurance_results"])

if __name__ == '__main__':
    for i in range(divisions):
        #TODO: Implement mass changes to the vehicle mass
        #TODO: Look into altering CL distribution to inflence CoP location
        easy_driver.params.ClA_tot = Cl_A[i]
        easy_driver.params.CdA_tot = Cd_A[i]
        print("Testing CL_A: " + str(Cl_A[i]) + " and Cd_A: " + str(Cd_A[i]) )
        easy_driver.regenerate_GGV(sweep_range, mesh_size)
        print("GGV Generated!")
        results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,skidpad_times, accel_times).run()
        #results[0].to_csv("results/endurance_michigan_2019-easy_driver.csv")
        #results[1].to_csv("results/autocross_michigan_2019-easy_driver.csv")
        print(times)
        print(points)
        results_df.loc[i] = [Cl_A, Cd_A, points[0], points[1], points[2], points[3], times[0], times[1], times[2], times[3],results[0]]
    results_df.to_csv("results/aero_sweep-easy_driver.csv")
