import engine
import engine.magic_moment_method.vehicle_params as vehicles
import numpy as np
import pandas as pd

def main():
    # NOTE: this requires a GGV to be pre generated (second input)
    easy_driver = engine.Racecar(vehicles.Concept2023(motor_directory="engine/magic_moment_method/vehicle_params/Eff228.csv"))
    #easy_driver.ggv = pd.read_csv("results/GGV.csv")
    endurance_track = engine.Track("racing_lines/en_mi_2019.csv", 1247.74, track_type="endurance")
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

    #masses = np.array([400, 450, 500, 550, 600]) - 22*4 * .454
    power_limit = [80000, 70000, 60000]
    results_df = pd.DataFrame(columns=["power_limit", "points", "times", "endurance_battery_capacity"])
    for index, p_l in enumerate(power_limit):
        easy_driver.params.power_limit = p_l
        easy_driver.regenerate_GGV(sweep_range, mesh_size)

        # If a new racing line is needed (i.e. trackwidth changes), enable these lines
        #endurance_track.regenerate_racing_line(easy_driver.params.front_trackwidth)
        #autocross_track.regenerate_racing_line(easy_driver.params.front_trackwidth)

        p_l_results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,
                            skidpad_times, accel_times).run()
        energy = sum(p_l_results[0]["delta_t"] * p_l_results[0]["power_in"])
        print(times)
        print(points)
        results_df.loc[index] = [p_l, points, times, energy]
        p_l_results[0].to_csv(f"results/p_l_{str(p_l)}-endurance-easy_driver.csv")
    results_df.to_csv("results/p_l_sweep-easy_driver.csv")


    ####################################
    ### Example: One-off simulations ###

    # easy_driver.ggv = pd.read_csv("results/GGV.csv")
    # #easy_driver.regenerate_GGV(sweep_range, mesh_size)
    # #easy_driver.save_ggv("results/GGV.csv")
    # #endurance_track.regenerate_racing_line(easy_driver.params.front_trackwidth)
    # #autocross_track.regenerate_racing_line(easy_driver.params.front_trackwidth)
    # comp = engine.Competition(easy_driver, endurance_track, autocross_track,
    #                        skidpad_times, accel_times)
    # results, points, times = comp.run()
    # results[0].to_csv("results/endurance_michigan_2019-easy_driver.csv")
    # results[1].to_csv("results/autocross_michigan_2019-easy_driver.csv")
    # comp.autocross_sim.forward_sim_results.to_csv("results/forward-autocross_michigan_2019-easy_driver.csv")
    # comp.autocross_sim.reverse_sim_results.to_csv("results/reverse-autocross_michigan_2019-easy_driver.csv")
    # print(times)
    # print(points)

if __name__ == "__main__":
    main()