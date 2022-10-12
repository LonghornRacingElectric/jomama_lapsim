import engine
from engine import racecar
import engine.magic_moment_method.vehicle_params as vehicles
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    easy_driver = engine.Racecar(vehicles.Concept2023() ) #, "engine/magic_moment_method/analysis/GGV.csv")

    endurance_track = engine.Track("endurance_michigan_2019", 1681.963)
    autocross_track = engine.Track("autocross_michigan_2019", 63.236)
    skidpad_times = 5.0497
    accel_times = 4.004

    ### Example: Sweeping! ###
    # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
    sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
            "steered_angle" : (-18 * np.pi / 180, 18 * np.pi / 180),
            "velocity" : (3, 30),
            "torque_request": (-1, 1),
            "is_left_diff_bias": (True, False)}
    mesh_size = 11
    
    # masses = np.array([400, 500, 600]) - 22*4 * .454
    # results_df = pd.DataFrame(columns=["mass", "points", "endurance_time"])
    # for index, mass in enumerate(masses):
    #     easy_driver.params.mass_sprung = mass
    #     easy_driver.regenerate_GGV(sweep_range, mesh_size)

    #     # If a new racing line is needed (i.e. trackwidth changes), enable these lines
    #     #endurance_track.regenerate_racing_line(easy_driver.params.front_trackwidth)
    #     #autocross_track.regenerate_racing_line(easy_driver.params.front_trackwidth)

    #     _, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,
    #                         skidpad_times, accel_times).run()
    #     print(times)
    #     results_df.loc[index] = [mass, points, times[0]]
    # results_df.to_csv("results/mass_sweep-easy_driver.csv")


    ### Example: One-off simulations ###
    easy_driver.regenerate_GGV(sweep_range, mesh_size)
    #easy_driver.save_ggv("engine/magic_moment_method/analysis/GGV.csv")
    results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,
                            skidpad_times, accel_times).run()
    results[0].to_csv("results/endurance_michigan_2019-easy_driver.csv")
    results[1].to_csv("results/autocross_michigan_2019-easy_driver.csv")
    print(times)
    print(points)
    # for radius in [5, 10, 20, 30, 40, 60]:
    #     print(easy_driver.max_vel_corner(radius))
    # print(easy_driver.deccel(25, 5))
    # print(easy_driver.accel(25, 5))
    # accel_track = engine.Track("Acceleration", best_time = accel_times)
    # acceleration_sim = engine.Simulation(easy_driver, accel_track)
    # results, time = acceleration_sim.run()
    # print(time)

if __name__ == "__main__":
    main()