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

#TODO: Apply correct equations from pareto front analysis

K = .06  # lift-induced drag constant
Cl0 = 0  # lift at 0 induced drag
Cd0 = .44  # parasitic drag coefficient
Cl_A = (np.linspace(TEST_MIN, TEST_MAX, divisions))
Cd_A = 1.2 * (K * (Cl_A - Cl0) ** 2 + Cd0)
Cl_A *= 1.2

# TODO: Upgrade this to reflect the new fit from Andrew Zhang, basically Cl will increase energy consumption (kWh) due to drag
#           then the extra kWh will correlate to more mass (Andrews equation). Challenge is to correlate drag and kWh.
#               LapSim should be able to do so given the torque request. If you have questions about it contact Igor Souza
Aero_mass = 0.3745 * (Cl_A*1.2) ** 2 + 0.0612 * Cl_A + 0.7783  
#######################################

#GGV Mesh Parameters
sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
            "steered_angle" : (-18 * np.pi / 180, 18 * np.pi / 180),
            "velocity" : (3, 30),
            "torque_request": (-1, 1),
            "is_left_diff_bias": (True, False)}
mesh_size = 7 #Anything below 5 is troublesome, time to generate increases to the forth power of mesh size

#TODO: Get most up to date track (I believe Robert is working on it)
endurance_track = engine.Track("endurance_michigan_2019", 1681.963)
autocross_track = engine.Track("autocross_michigan_2019", 63.236)
skidpad_times = 5.0497
accel_times = 4.004

easy_driver = engine.Racecar(vehicles.Concept2023() )

#TODO: Increase results to capture all even times and points in their specific columns, not the entire df
results_df = pd.DataFrame(columns=["ClA","CdA","points","endurance_time"])

if __name__ == '__main__':    
    for i in range(divisions):
        #TODO: Implement mass changes to the vehicle mass
        #TODO: Look into altering CL distribution to inflence CoP location
        easy_driver.params.ClA_tot = Cl_A[i]
        easy_driver.params.CdA_tot = Cd_A[i]
        print("Testing CL_A: " + str(Cl_A[i]) + " and Cd_A: " + str(Cd_A[i]) )
        easy_driver.regenerate_GGV(sweep_range, mesh_size)

        results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,skidpad_times, accel_times).run()
        #results[0].to_csv("results/endurance_michigan_2019-easy_driver.csv")
        #results[1].to_csv("results/autocross_michigan_2019-easy_driver.csv")
        print(times)
        print(points)
        results_df.loc[i] = [Cl_A, Cd_A, points, times[0]]
    results_df.to_csv("results/aero_sweep-easy_driver.csv")
