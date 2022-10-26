import numpy as np
import pandas as pd
import engine
import engine.magic_moment_method.vehicle_params as vehicles
import engine.magic_moment_method.state_solver.aerodynamics as Aero
import engine.magic_moment_method.state_solver.logger as Fuck_this_Logger
import math
import random
pd.options.mode.chained_assignment = None

# TODO: Move this file into sweeping, gonna have to mess with module imports

# Test Parameters #
#######################################

Cl_Sweep = True
TEST_MIN = 0
TEST_MAX = 5
divisions = 2

K = 0.06215178719838  # lift-induced drag constant
Cl0 = 0  # lift at 0 induced drag
Cd0 = 0.4414249328442602  # parasitic drag coefficien
refA = 1.2 # reference area, m^2
Cl = (np.linspace(TEST_MIN, TEST_MAX, divisions))
Cd_A = refA * (K * (Cl - Cl0) ** 2 + Cd0)
Cl_A = refA * Cl

CoP_Target = np.linspace(40,60,divisions)
Tries = 2000
variance = 350
error = 0.25

#######################################

#GGV Mesh Parameters
sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
            "steered_angle" : (-18 * np.pi / 180, 18 * np.pi / 180),
            "velocity" : (3, 30),
            "torque_request": (-1, 1),
            "is_left_diff_bias": (True, False)}
mesh_size = 5 #Anything below 5 is troublesome, time to generate increases to the forth power of mesh size

endurance_track = engine.Track("racing_lines/en_mi_2019.csv", 1247.74)
autocross_track = engine.Track("racing_lines/ax_mi_2019.csv", 50.008)
skidpad_times = 5.0497
accel_times = 4.004
Drag_kWh = 0.6611
result_DF = []
result_DG = []
average_DFR = []
average_DFF = []
average_DFU = []
average_DGR = []
average_DGF = []
average_DGU = []

#TODO: Have 

easy_driver = engine.Racecar(vehicles.Concept2023(motor_directory="engine/magic_moment_method/vehicle_params/Eff228.csv"))
#logger = Fuck_this_Logger.Logger()
#aero = Aero.Aerodynamics(vehicles.Concept2023(motor_directory="engine/magic_moment_method/vehicle_params/Eff228.csv"),logger)

Total_Downforce = easy_driver.params.CdA_tot * 15 ** 2 * 1.153 / 2
Total_Drag = easy_driver.params.CdA_tot * 15 ** 2 * 1.153 / 2
Initial_mass = easy_driver.params.mass_sprung

if __name__ == '__main__':
    for i in range(divisions):
        Mass_Delta = 15.770 * 0.45 * (Drag_kWh - 2)

        if Cl_Sweep == True:
            results_df = pd.DataFrame(columns=["ClA","CdA","endurance_points","autocross_points","skidpad_points","accel_points",
                    "endurance_time","autocross_time","skidpad_time","accel_time", "drag_energy (kWh)", "Mass Delta (kg)", 
                    "Max Long Accel (g)","Max braking Accel (g)", "Max Lat Accel (g)"])

            Mass_Delta = 15.770 * 0.45 * (Drag_kWh - 2)
            easy_driver.params.ClA_tot = Cl_A[i]
            easy_driver.params.CdA_tot = Cd_A[i]

            if Cl_A[i] == 0:
                    Initial_mass = easy_driver.params.mass_sprung
                    easy_driver.params.mass_sprung += Mass_Delta - 16  #Subtracting Mass of Aero Components

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
                                times[1], times[2], times[3], Drag_kWh, easy_driver.params.mass_sprung - Initial_mass, df_endurance_accel['ax'].max()/9.81
                                ,df_endurance_breaking['ax'].min()/9.81, df_endurance['ay'].max()/9.81]

            df_endurance['Drag_J'] = easy_driver.params.CdA_tot * df_endurance['vel'] ** 2 * 1.153 / 2 * df_endurance['dist']
            Drag_kWh = (df_endurance["Drag_J"].sum() / (times[0] * 1000) * (times[0] * Total_Laps)/3600)

        else:
            results_df = pd.DataFrame(columns=["endurance_points","autocross_points","skidpad_points","accel_points",
                    "endurance_time","autocross_time","skidpad_time","accel_time", "cop_bias(%)", "cla_dist","cda_dist"])
            u = 0
            #CoP Sweep
            while u!= Tries:

                #Initial distribution based on data from different teams
                DF_Distribution = easy_driver.params.ClA_dist * 1000
                DG_Distribution = easy_driver.params.CdA_dist * 1000

                #Shuffles the order at which the distributions vary, this way we can assure the process is completely random
                first_list = [0, 1, 2]
                random.shuffle(first_list)

                #Assigns a new distribution value within variance to each list, last one is organized in a way the sum is equal to 100%
                DF_Distribution[first_list[0]] = abs(np.random.randint(DF_Distribution[first_list[0]] - variance, DF_Distribution[first_list[0]] + variance))
                DF_Distribution[first_list[1]] = abs(np.random.randint(DF_Distribution[first_list[1]] - variance, DF_Distribution[first_list[1]] + variance))
                DF_Distribution[first_list[2]] += (-sum(DF_Distribution) + 1000)
                DG_Distribution[first_list[0]] = abs(np.random.randint(DG_Distribution[first_list[0]] - variance, DG_Distribution[first_list[0]] + variance))
                DG_Distribution[first_list[1]] = abs(np.random.randint(DG_Distribution[first_list[1]] - variance, DG_Distribution[first_list[1]] + variance))
                DG_Distribution[first_list[2]] += (-sum(DG_Distribution) + 1000)

                #Calculates the total moment around the CoP of the full car
                Total = - easy_driver.params.CoP[1][2]*DG_Distribution[1]*Total_Drag + easy_driver.params.CoP[0][0]*DF_Distribution[0]*Total_Downforce - \
                        easy_driver.params.CoP[0][2]*DG_Distribution[0]*Total_Drag - easy_driver.params.CoP[2][2]*DG_Distribution[2]*Total_Drag - \
                        easy_driver.params.CoP[2][0]*DF_Distribution[2]*Total_Downforce - easy_driver.params.CoP[1][0]*DF_Distribution[1]*Total_Downforce

                CoP_x = abs((Total* 0.001/((Total_Downforce ** 2 + Total_Drag ** 2) ** 0.5) *
                        math.cos(math.atan(Total_Drag/Total_Downforce)))/(easy_driver.params.wheelbase*easy_driver.params.cg_bias)*100)

                #print(CoP_x)
                #Checks if the moments balance within a % range
                if CoP_Target[i]-error < CoP_x < CoP_Target[i]+error:

                #For each sucessful combination saves the result in a list creating a matrix
                    result_DF.append(DF_Distribution)
                    result_DG.append(DG_Distribution)
                    u+=1
                    if u/Tries*100 == 20 or u/Tries*100 == 40 or u/Tries*100 == 60 or u/Tries*100 == 80 or u/Tries*100 == 100:
                        print('CoP Simulation Progress {:.2f}%'.format(u/Tries*100))

            New_ClA_Dist = sum(result_DF) / len(result_DF)
            New_CdA_Dist = sum(result_DG) / len(result_DG)
            print(New_ClA_Dist)

            easy_driver.params.ClA_dist = New_ClA_Dist/1000
            easy_driver.params.CdA_dist = New_CdA_Dist/1000

            easy_driver.regenerate_GGV(sweep_range, mesh_size)
            print("GGV Generated!")
            results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,skidpad_times, accel_times).run()

            df_endurance = results[0]
            df_endurance = df_endurance[["dist",'vel','pos','ax','ay']]
            Total_Laps = math.ceil(22000/df_endurance["pos"].max())

            print(times)
            print(points)

            results_df.loc[i] = [points[0], points[1], points[2], points[3], times[0] * Total_Laps,
                                times[1], times[2], times[3], CoP_Target[i], New_ClA_Dist, New_CdA_Dist]
                
    if Cl_Sweep == True:
        results_df.to_csv("results/Cl_sweep-Concept2023.csv")
    else:
        results_df.to_csv("results/CoP_sweep-Concept2023.csv")