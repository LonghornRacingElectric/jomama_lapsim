import time
import engine
import engine.magic_moment_method.vehicle_params as vehicles
import numpy as np
import pandas as pd

def main():
    # NOTE: this requires a GGV to be pre generated (second input)
    lapsim_racecar = engine.Racecar(vehicles.Concept2023(motor_directory="engine/magic_moment_method/vehicle_params/Eff228.csv"))
    #lapsim_racecar.ggv = pd.read_csv("results/GGV.csv")
    endurance_track = engine.Track("racing_lines/en_mi_2019.csv", 1247.74, track_type="endurance")
    autocross_track = engine.Track("racing_lines/ax_mi_2019.csv", 50.008)
    skidpad_times = 5.0497
    accel_times = 3.7

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
    # power_limit = [80000, 70000, 60000]
    # results_df = pd.DataFrame(columns=["power_limit", "points", "times", "endurance_battery_capacity"])
    # for index, p_l in enumerate(power_limit):
    #     lapsim_racecar.params.power_limit = p_l
    #     lapsim_racecar.regenerate_GGV(sweep_range, mesh_size)

    #     # If a new racing line is needed (i.e. trackwidth changes), enable these lines
    #     #endurance_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
    #     #autocross_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)

    #     p_l_results, points, times = engine.Competition(lapsim_racecar, endurance_track, autocross_track,
    #                         skidpad_times, accel_times).run()
    #     energy = sum(p_l_results[0]["delta_t"] * p_l_results[0]["power_into_inverter"])
    #     print(times)
    #     print(points)
    #     results_df.loc[index] = [p_l, points, times, energy]
    #     p_l_results[0].to_csv(f"results/p_l_{str(p_l)}-endurance-concept_2023.csv")
    # results_df.to_csv("results/p_l_sweep-concept_2023.csv")


    ####################################
    ### Example: One-off simulations ###

    start_time = time.time()
    print()

    lapsim_racecar.ggv = pd.read_csv("results/GGV.csv")
    lapsim_racecar.prepare_GGV()
    #lapsim_racecar.recreate_initial_guesses(sweep_range, mesh_size)
    #lapsim_racecar.regenerate_GGV(sweep_range, mesh_size)
    #lapsim_racecar.save_ggv("results/GGV.csv")
    #endurance_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
    #autocross_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
    comp = engine.Competition(lapsim_racecar, endurance_track, autocross_track,
                           skidpad_times, accel_times)
    results, points, times = comp.run()

    results[0].to_csv("results/endurance_michigan_2019-concept_2023.csv")
    results[1].to_csv("results/autocross_michigan_2019-concept_2023.csv")
    results[3].to_csv("results/acceleration-concept_2023.csv")
    # comp.autocross_sim.forward_sim_results.to_csv("results/forward-autocross_michigan_2019-concept_2023.csv")
    # comp.autocross_sim.reverse_sim_results.to_csv("results/reverse-autocross_michigan_2019-concept_2023.csv")
    
    print()
    for event_name, event_time, event_points, event_results in zip(("Endurance", "Autocross", "Skidpad", "Acceleration"), times, points, results):
        event_str = event_name.ljust(18)
        time_str = " ".join(("time:", str(round(event_time, 2)), "s")).ljust(18)
        points_str = " ".join(("points:", str(round(event_points, 2)))).ljust(18)
        energy_str = ""
        if not event_results.empty:
            energy_str = " ".join(("energy:", str(round(sum(event_results["delta_t"] * event_results["power_into_inverter"]) * 2.77778e-7, 4)), "kWh"))
        print(event_str, time_str, points_str, energy_str)

    elapsed_time = time.time() - start_time
    print("\nresults in", round(elapsed_time, 2), "s")


if __name__ == "__main__":
    main()