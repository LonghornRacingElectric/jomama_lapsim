import engine
import engine.magic_moment_method.vehicle_params as vehicles
import numpy as np
import pandas as pd

def main():
    # NOTE: this requires a GGV to be pre generated (second input)
    easy_driver = engine.Racecar(vehicles.Concept2023(motor_directory="engine/magic_moment_method/vehicle_params/Eff228.csv"))
    endurance_track = engine.Track("racing_lines/en_mi_2019.csv", 1247.74)
    autocross_track = engine.Track("racing_lines/ax_mi_2019.csv", 50.008)
    skidpad_times = 5.0497
    accel_times = 4.004

    # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
    sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
            "steered_angle" : (-18 * np.pi / 180, 18 * np.pi / 180),
            "velocity" : (3, 30),
            "torque_request": (-1, 1),
            "is_left_diff_bias": (True, False)}
    mesh_size = 11 # NOTE: MAKE SURE THIS IS ODD


    ##########################
    ### Example: Sweeping! ###

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


    ####################################
    ### Example: One-off simulations ###

    easy_driver.ggv = pd.read_csv("results/GGV.csv")
    #easy_driver.regenerate_GGV(sweep_range, mesh_size)
    #easy_driver.save_ggv("results/GGV.csv")
    #endurance_track.regenerate_racing_line(easy_driver.params.front_trackwidth)
    #autocross_track.regenerate_racing_line(easy_driver.params.front_trackwidth)
    results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,
                           skidpad_times, accel_times).run()
    results[0].to_csv("results/endurance_michigan_2019-easy_driver.csv")
    results[1].to_csv("results/autocross_michigan_2019-easy_driver.csv")
    print(times)
    print(points)

if __name__ == "__main__":
    main()