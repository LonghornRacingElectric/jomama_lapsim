import numpy as np
import pandas as pd
import engine
import engine.magic_moment_method.vehicle_params as vehicles


# TODO: Move this file into sweeping, gonna have to 
# Test Parameters #
#######################################
TEST_MIN = 0
TEST_MAX = 4
divisions = 3
section = 'aero'
key = 'Cl_A'

K = .06  # lift-induced drag constant
Cl0 = 0  # lift at 0 induced drag
Cd0 = .44  # parasitic drag coefficient
Cl_A = (np.linspace(TEST_MIN, TEST_MAX, divisions))
Cd_A = 1.2 * (K * (Cl_A - Cl0) ** 2 + Cd0)
Cl_A *= 1.2
Aero_mass = 0.3745 * (Cl_A*1.2) ** 2 + 0.0612 * Cl_A + 0.7783  # Drag Mass increase estimation from Excel fit
#######################################

sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
            "steered_angle" : (-18 * np.pi / 180, 18 * np.pi / 180),
            "velocity" : (3, 30),
            "torque_request": (-1, 1),
            "is_left_diff_bias": (True, False)}
mesh_size = 7
endurance_track = engine.Track("endurance_michigan_2019", 1681.963)
autocross_track = engine.Track("autocross_michigan_2019", 63.236)
skidpad_times = 5.0497
accel_times = 4.004

easy_driver = engine.Racecar(vehicles.Concept2023() )

results_df = pd.DataFrame(columns=["ClA","CdA","points","endurance_time"])

if __name__ == '__main__':    
    for i in range(divisions):
        easy_driver.params.ClA_tot = Cl_A[i]
        easy_driver.params.CdA_tot = Cd_A[i]
        print("Testing CL_A: " + str(Cl_A[i]) + " and Cd_A: " + str(Cd_A[i]) )
        easy_driver.regenerate_GGV(sweep_range, mesh_size)

        results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,skidpad_times, accel_times).run()
        results[0].to_csv("results/endurance_michigan_2019-easy_driver.csv")
        results[1].to_csv("results/autocross_michigan_2019-easy_driver.csv")
        print(times)
        print(points)
        results_df.loc[i] = [Cl_A, Cd_A, points, times[0]]
    results_df.to_csv("results/aero_sweep-easy_driver.csv")
